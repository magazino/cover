#include <cover/costmap.hpp>
#include <cover/expand.hpp>
#include <cover/footprints.hpp>
#include <cover/impl/boost_io.hpp>
#include <cover/impl/costmap.hpp>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometry.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cover {

namespace bg = boost::geometry;

// Define the major boost::geometry types.
using bg_point = bg::model::d2::point_xy<double>;
using bg_polygon = bg::model::polygon<bg_point>;
using bg_multi_polygon = bg::model::multi_polygon<bg_polygon>;
using bg_segment = bg::model::referring_segment<const bg_point>;

// Short-cut for the costmap.
using costmap_2d::Costmap2D;

double
get_radius(const polygon& _polygon) {
  // Convert to boost
  const bg_polygon poly = detail::to_boost<bg_point>(_polygon);

  // We might also throw...
  if (poly.outer().empty())
    return 0;

  // Get the center.
  bg_point center;
  bg::centroid(poly, center);

  // Find the closest distance to the centroid
  double dist = std::numeric_limits<double>::max();
  for (auto ii = poly.outer().begin(), end = std::prev(poly.outer().end());
       ii != end; ++ii) {
    bg_segment segment(*ii, *std::next(ii));
    dist = std::min(dist, bg::distance(segment, center));
  }

  return dist;
}

continuous_footprint::continuous_footprint(const double _radius,
                                           const polygon& _polygon) {
  // An empty polygon is a noop.
  if (_polygon.cols() == 0)
    return;

  // Check the input.
  if (_radius <= 0)
    throw std::invalid_argument("Radius must be positive");

  // Convert the polygon to boost
  const bg_polygon polygon = detail::to_boost<bg_point>(_polygon);

  bg_multi_polygon eroded, dilated, diffs;
  // The erode-factor is slightly larger than -1 (for numerical reasons).
  constexpr double erode_factor = 1e-6 - 1.;
  const bg::strategy::buffer::distance_symmetric<double> erode_distance(
      _radius * erode_factor),
      dilate_distance(_radius);

  // Number of points in circular strategies.
  constexpr size_t n_points = 36;

  // Boilerplate strategy definitions.
  const bg::strategy::buffer::side_straight ss;
  const bg::strategy::buffer::join_round js(n_points);
  const bg::strategy::buffer::end_round es(n_points);
  const bg::strategy::buffer::point_circle cs(n_points);

  // Perform a morphological closure.
  bg::buffer(polygon, eroded, erode_distance, ss, js, es, cs);
  if (!eroded.empty()) {
    // Note: if the eroded output is degenerate (such that it does not define a
    // polygon anymore) the bg::buffer will not work properly.
    using bg_line_string = bg::model::linestring<bg_point>;
    const bg_line_string l_string(eroded.front().outer().begin(),
                                  eroded.front().outer().end());
    bg::buffer(l_string, dilated, dilate_distance, ss, js, es, cs);
  }

  // Get the difference between the inflated and the original.
  bg::difference(polygon, dilated, diffs);  // NOLINT

  // Convert back to eigen.
  outlines = detail::from_boost(eroded);
  areas = detail::from_boost(diffs);
}

continuous_footprint::continuous_footprint(const polygon& _polygon) :
    continuous_footprint(get_radius(_polygon), _polygon) {}

bool
continuous_footprint::is_free(const Eigen::Isometry2d& _pose,
                              const Costmap2D& _map) const {
  // Convert the pose into the map-frame.
  const Eigen::Isometry2d pose =
      Eigen::Translation2d(-_map.getOriginX(), -_map.getOriginY()) * _pose;
  const Eigen::Array2i bounds(_map.getSizeInCellsX(), _map.getSizeInCellsY());
  const double resolution = _map.getResolution();

  auto is_inside = [&](const discrete_polygon& _polygon) {
    // minCoeff and maxCoeff will not work for an empty polygon.
    if (_polygon.cols() == 0)
      return true;

    // Check the lower bound.
    const cell min_cell = _polygon.rowwise().minCoeff();
    if ((min_cell.array() < 0).any())
      return false;

    // Check the upper bound.
    const cell max_cell = _polygon.rowwise().maxCoeff();
    if ((max_cell.array() >= bounds).any())
      return false;

    return true;
  };

  // Iterate over the outlines.
  for (const auto& outline : outlines) {
    // Discretize the outline
    const discrete_polygon disc(to_discrete(resolution, pose * outline));
    // Check the bounds.
    if (!is_inside(disc))
      return false;

    // Create the generator.
    outline_generator gen(disc);

    if (std::any_of(gen.begin(), gen.end(), [&](const cell& _cell) {
          const auto cost = _map.getCost(_cell.x(), _cell.y());
          return cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
                 cost == costmap_2d::LETHAL_OBSTACLE;
        }))
      return false;
  }

  // Iterate over the areas.
  for (const auto& area : areas) {
    // Discretize the outline
    const discrete_polygon disc(to_discrete(resolution, pose * area));

    // Check the bounds.
    if (!is_inside(disc))
      return false;

    // Create the generator.
    area_generator gen(to_outline(disc));

    if (std::any_of(gen.begin(), gen.end(), [&](const cell& _cell) {
          const auto cost = _map.getCost(_cell.x(), _cell.y());
          return cost == costmap_2d::LETHAL_OBSTACLE;
        }))
      return false;
  }

  return true;
}

void
generator_footprint::init(const double _resolution,
                          const continuous_footprint& _continuous) {
  const auto& c_areas = _continuous.get_areas();
  const auto& c_outlines = _continuous.get_outlines();

  areas.reserve(c_areas.size());
  outlines.reserve(c_outlines.size());

  for (const auto& area : c_areas)
    areas.emplace_back(_resolution, area);

  for (const auto& outline : c_outlines)
    outlines.emplace_back(_resolution, outline);
}

template <typename Generator, typename Check>
static bool
is_free_impl(const std::vector<Generator>& _gens, const Check& _check) {
  for (const auto& gen : _gens) {
    if (std::any_of(gen.begin(), gen.end(), _check))
      return false;
  }
  return true;
}

bool
generator_footprint::is_free(const cell& _offset,
                             const costmap_2d::Costmap2D& _map) const {
  using namespace detail;
  const auto oc = make_with_offset(is_lethal_or_inflated{_map}, _offset);
  const auto ac = make_with_offset(is_lethal{_map}, _offset);
  return is_free_impl(outlines, oc) && is_free_impl(areas, ac);
}

void
discrete_footprint::init(const generator_footprint& _generators) {
  // Expand everything
  outlines = expand_multiple(_generators.get_outlines());
  areas = expand_multiple(_generators.get_areas());
}

bool
discrete_footprint::is_free(const cell& _offset,
                            const costmap_2d::Costmap2D& _map) const {
  using namespace detail;
  const auto oc = make_with_offset(is_lethal_or_inflated{_map}, _offset);
  const auto ac = make_with_offset(is_lethal{_map}, _offset);
  return is_free_impl(outlines, oc) && is_free_impl(areas, ac);
}

}  // namespace cover
