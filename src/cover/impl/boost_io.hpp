#pragma once

#include <cover/base.hpp>

#include <Eigen/Core>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometry.hpp>

#include <cassert>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

// File contains helpers for the boost::geometry interop.

namespace cover {
namespace detail {

/**
 * @brief Converts the cover::polygon to boost::geometry::polygon.
 *
 * @tparam Point The 2d-point type.
 * @param _polygon The input polygon.
 */
template <typename Point>
boost::geometry::model::polygon<Point>
to_boost(const Eigen::Ref<const cover::polygon>& _polygon) noexcept {
  const int n_cols = _polygon.cols();

  boost::geometry::model::polygon<Point> out;
  out.outer().reserve(n_cols);

  // Convert to boost.
  for (int ii = 0; ii != n_cols; ++ii) {
    out.outer().emplace_back(_polygon.col(ii).x(), _polygon.col(ii).y());
  }

  // Try to correct the polygon.
  boost::geometry::correct(out);
  return out;
}

/**
 * @brief Converts a boost::geometry::ring to cover::polygon.
 *
 * @tparam Point The 2d-point type.
 * @param _ring The ring.
 */
template <typename Point>
cover::polygon
from_boost(const typename boost::geometry::model::polygon<Point>::ring_type&
               _ring) noexcept {
  cover::polygon out(2, _ring.size());

  int counter = -1;
  for (const auto& p : _ring)
    out.col(++counter) << p.template get<0>(), p.template get<1>();

  return out;
}

/**
 * @brief Converts the boost::geometry::polygon to cover::polygon.
 *
 * @tparam Point The 2d-point type.
 * @param _polygon The polygon.
 * @throw std::runtime_error if the _polygon has inner rings.
 */
template <typename Point>
cover::polygon
from_boost(const boost::geometry::model::polygon<Point>& _polygon) {
  if (!_polygon.inners().empty())
    throw std::runtime_error("Inner rings are not supported");

  return from_boost<Point>(_polygon.outer());
}

/**
 * @brief Converts the boost::geometry::multi_polygon to cover::polygon_vec.
 *
 * @tparam Point The 2d-point type.
 * @param _in The multi-polygon.
 */
template <typename Point>
cover::polygon_vec
from_boost(const boost::geometry::model::multi_polygon<
           boost::geometry::model::polygon<Point>>& _in) {
  namespace bg = boost::geometry;
  auto is_valid = [](const bg::model::polygon<Point>& _poly) {
    // If we have a inner polygon, it's likely degenerate.
    return !bg::is_empty(_poly) && _poly.inners().empty();
  };

  // Count the valid polygons
  size_t n_valid = std::count_if(_in.begin(), _in.end(), is_valid);

  // Create the right polygon_vec
  polygon_vec out;
  out.reserve(n_valid);

  for (const auto& poly : _in) {
    if (!is_valid(poly))
      continue;
    out.emplace_back(from_boost(poly));
  }

  return out;
}

/**
 * @brief Function will try to split a self-intersecting rectangle into two
 * triangles.
 *
 * @tparam Point The 2d-point type.
 * @param _rectangle The rectangle.
 * @return true, if successfull.
 */
template <typename Point>
boost::geometry::model::multi_polygon<boost::geometry::model::polygon<Point>>
split_rectangle(
    const boost::geometry::model::polygon<Point>& _rectangle) noexcept {
  namespace bg = boost::geometry;
  using polygon_type = bg::model::polygon<Point>;
  using multi_polygon_type = bg::model::multi_polygon<polygon_type>;

  if (_rectangle.outer().size() != 5)
    return {};

  for (size_t ii = 0; ii != 2; ++ii) {
    std::vector<Point> inter_ps;
    auto& outer = _rectangle.outer();
    using segment_type = bg::model::referring_segment<const Point>;
    // The zero is just there for "consistency".
    const segment_type s0(outer.at(ii + 0), outer.at(ii + 1));
    const segment_type s1(outer.at(ii + 2), outer.at(ii + 3));

    // Check if the segments intersect.
    if (bg::intersection(s0, s1, inter_ps) && !inter_ps.empty()) {
      const Point& x = inter_ps.front();
      multi_polygon_type out;
      if (ii == 0) {
        out = {{{x, outer.at(0), outer.at(3), x}},
               {{x, outer.at(1), outer.at(2), x}}};
      }
      else {
        out = {{{x, outer.at(0), outer.at(1), x}},
               {{x, outer.at(3), outer.at(2), x}}};
      }
      bg::correct(out);

      // If we still cannot generate a valid geometry, we continue.
      if (!bg::is_valid(out)) {
        continue;
      }
      return out;
    }
  }
  return {};
}

}  // namespace detail
}  // namespace cover
