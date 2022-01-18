// Copyright 2022 Magazino GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

}  // namespace detail
}  // namespace cover
