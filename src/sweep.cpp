#include "sweep.hpp"
#include "impl/boost_io.hpp"

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometry.hpp>

#include <cassert>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace cover {

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
