#include "sweep.hpp"
#include "impl/boost_io.hpp"

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometry.hpp>

#include <cassert>
#include <cmath>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace cover {

namespace interpol {

start_strategy::start_strategy(const Eigen::Isometry2d& _start,
                               const Eigen::Isometry2d&) noexcept :
    start_(_start) {}

const Eigen::Isometry2d&
start_strategy::operator()(const double) const noexcept {
  return start_;
}

linear_strategy::linear_strategy(const Eigen::Isometry2d& _start,
                                 const Eigen::Isometry2d& _end) noexcept :
    t_start_(_start.translation()),
    t_diff_(_end.translation() - t_start_),
    r_start_(_start.rotation()),
    r_end_(_end.rotation()) {}

Eigen::Isometry2d
linear_strategy::operator()(const double _t) const {
  assert(_t >= 0 && _t <= 1 && "Invalid interpolation parameter");
  return Eigen::Isometry2d{Eigen::Translation2d(t_start_ + _t * t_diff_) *
                           r_start_.slerp(_t, r_end_)};
}

}  // namespace interpol

namespace stepping {

size_t
one_step_strategy::get_steps(const Eigen::Isometry2d&,
                             const Eigen::Isometry2d&) const noexcept {
  return 1;
}

max_step_strategy::max_step_strategy(const double _max_trans_step,
                                     const double _max_angular_step) noexcept :
    t_step_(_max_trans_step), a_step_(_max_angular_step) {}

static double
compute_shortest_angle(const Eigen::Rotation2Dd& _r0,
                       const Eigen::Rotation2Dd& _r1) {
  // Compute the angle difference
  const auto diff =
      std::abs(_r0.smallestPositiveAngle() - _r1.smallestPositiveAngle());

  return diff < M_PI ? diff : 2 * M_PI - diff;
}

size_t
max_step_strategy::get_steps(const Eigen::Isometry2d& _start,
                             const Eigen::Isometry2d& _end) const noexcept {
  // Check bounds
  size_t n_t_steps = 1, n_a_steps = 1;
  if (t_step_ > 0) {
    n_t_steps = static_cast<size_t>(std::ceil(
        (_end.translation() - _start.translation()).norm() / t_step_));
  }
  if (a_step_ > 0) {
    n_a_steps = static_cast<size_t>(
        std::ceil(compute_shortest_angle(Eigen::Rotation2Dd{_start.rotation()},
                                         Eigen::Rotation2Dd{_end.rotation()}) /
                  a_step_));
  }
  return std::max(n_t_steps, n_a_steps);
}

}  // namespace stepping

namespace bg = boost::geometry;
using bg_multi_polygon = bg::model::multi_polygon<bg_polygon>;

namespace detail {

void
append(const bg_polygon& _next, bg_polygon& _curr) {
  if (bg::is_empty(_curr))
    _curr = _next;
  else if (!bg::is_empty(_next)) {
    std::vector<bg_polygon> tmp;
    bg::union_(_curr, _next, tmp);
    if (tmp.size() == 1)
      _curr = std::move(tmp.front());
    else
      throw std::runtime_error("Failed to combine polygons");
  }
}

bg_polygon
linear_sweep(const bg_polygon& _curr, const bg_polygon& _next) {
  assert(_curr.outer().size() == _next.outer().size() && "Mismatched polygons");

  auto output = _curr;

  const size_t n_points = _curr.outer().size();

  // Number of points in circular strategies.
  constexpr size_t n_circle_points = 36;

  // Boilerplate strategy definitions.
  const bg::strategy::buffer::distance_symmetric<double> ds(1e-3);
  const bg::strategy::buffer::side_straight ss;
  const bg::strategy::buffer::join_miter js;
  const bg::strategy::buffer::end_flat es;
  const bg::strategy::buffer::point_circle cs(n_circle_points);

  // Iterate over all points in the (closed) polygon.
  for (size_t ii = 1; ii != n_points; ++ii) {
    bg_multi_polygon output_collection, buffered_segment;
    // Create a rectangle.
    bg_polygon segment{{_curr.outer().at(ii - 1), _curr.outer().at(ii),
                        _next.outer().at(ii), _next.outer().at(ii - 1)}};
    bg::correct(segment);

    // If the rectangle is self-intersecting, split it into two triangles.
    // Then buffer the rectangle to avoid numerical issues.
    if (!bg::is_valid(segment)) {
      const auto splitted = detail::split_rectangle(segment);
      bg::buffer(splitted, buffered_segment, ds, ss, js, es, cs);
    }
    else {
      bg::buffer(segment, buffered_segment, ds, ss, js, es, cs);
    }

    assert(bg::is_valid(buffered_segment) && "Invalid segment");
    for (const auto& poly : buffered_segment)
      detail::append(poly, output);
  }
  return output;
}

polygon_vec
from_boost_with_holes(const bg_polygon& _polygon) {
  polygon_vec p_out{detail::from_boost<bg_point>(_polygon.outer())};
  {
    const auto& inners = _polygon.inners();
    for (const auto& inner : inners) {
      const double area = bg::area(inner);
      // The area is negative for inner rings...
      if (std::abs(area) > 1e-3)
        p_out.emplace_back(detail::from_boost<bg_point>(inner));
    }
  }
  return p_out;
}

}  // namespace detail
}  // namespace cover
