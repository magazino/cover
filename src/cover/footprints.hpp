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
#include <cover/generators.hpp>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Geometry>

#include <type_traits>
#include <utility>
#include <vector>

namespace cover {

/// @defgroup Footprints Footprints
///
/// The group contains classes and functions for splitting the footprint into
/// sub-areas. This can be exploited for faster collision checks.
///
/// The splitting makes only sense for costmaps which inflate the obstacles by
/// the _rad. Within the ros-navigation stack this can be done with the
/// [costmap_2d::InflationLayer](http://wiki.ros.org/costmap_2d/layered#Inflation_Layer).
///
/// The basic idea is to find sub-regions of a polygon which are at least _rad
/// away from the polygon's boundary. One can verify that these regions are
/// collision free by checking only the cells on the topological skeleton for
/// inflation costs. These sub-regions are called 'outlines' below. All
/// remaining sub-regions must be checked fully (we must iterate over all cells
/// inclosed by them). The are called 'areas' below.
///
/// The outlines can be computed through a
/// [morphological opening](https://en.wikipedia.org/wiki/Opening_(morphology))
/// of the polygon with the distance set to _rad. The areas (second type) can
/// be computed by taking the difference between the processed polygon and the
/// original. This library uses
/// [boost::geometry](https://www.boost.org/doc/libs/1_77_0/libs/geometry/doc/html/index.html)
/// as backend for the computations.
///
/// Hints for the user: The splitting of the footprint is relatively expensive
/// (factor 100 compared to the brute-force checking of the cells). The
/// splitting makes therefore only sense if we want to check the same footprint
/// recurrently and can cache the split footprint. This is for example the case
/// if want to generate a plan for a mobile robot. If you want to check if the
/// footprint is free only once or cannot cache the split footprint, then you
/// should fall back to the "traditional checks".
///
/// @see http://wiki.ros.org/costmap_2d/layered#Inflation_Layer
/// @see https://en.wikipedia.org/wiki/Opening_(morphology)
/// @see https://www.boost.org/doc/libs/1_77_0/libs/geometry/doc/html/index.html

/// @{

namespace detail {

/**
 * @brief The base class of a footprint.
 *
 * The footprint has two members: outlines and areas. We check only the outline
 * of everything defined in outlines and the entire area of everything defined
 * in areas.
 *
 * @tparam Outline The data type of the outlines.
 * @tparam Area The data type of the areas.
 */
template <typename Outlines, typename Areas = Outlines>
struct base_footprint {
  using outlines_type = Outlines;
  using areas_type = Areas;

  base_footprint() = default;
  base_footprint(const outlines_type& _outlines, const areas_type& _areas) :
      outlines(_outlines), areas(_areas) {}

  const outlines_type&
  get_outlines() const noexcept {
    return outlines;
  }

  const areas_type&
  get_areas() const noexcept {
    return areas;
  }

protected:
  outlines_type outlines;  ///< The outlines.
  areas_type areas;        ///< The areas.
};

}  // namespace detail

/// @brief Generates the inflation radius from a polygon.
///
/// The resulting value is the smallest distance between the center of weight
/// the edges defined by the outline. The output is similar to
/// costmap_2d::calculateMinAndMaxDistances.
///
/// @param _poly The outline of a polygon.
///
/// @snippet test/footprints.cpp get_radius
/// @warning This method works only for convex polygons.
double
get_radius(const polygon& _poly);

/**
 * @brief A split footprint.
 *
 * This class splits the metric input in the sub-regions which can be checked
 * only by their outline and sub-regions which must be checked by their enclosed
 * area. Use this class if you have a footprint and want check for different
 * poses if they are collision free.
 *
 * The snippet shows the basic usage of the class:
 * @snippet test/footprints.cpp continuous_footprint
 */
struct continuous_footprint : public detail::base_footprint<polygon_vec> {
  // Define the base type.
  using base_type = detail::base_footprint<polygon_vec>;

  // Below the constructors.
  using base_type::base_footprint;

  /// @brief Constructs the continuous footprint by auto-computing the radius.
  /// @param _outline The polygon outline.
  /// @see get_radius
  /// @warning Does only work for convex polygons.
  explicit continuous_footprint(const polygon& _outline);

  /// @brief Constructs the continuous footprint using the given radius.
  /// @param _radius The morphological radius.
  /// @param _outline The polygon outline.
  /// @throw std::invalid_argument if the _radius is not positive.
  continuous_footprint(double _radius, const polygon& _outline);

  /// @brief Returns true if the footprint is free.
  /// @param _pose The pose (position of the footprint).
  /// @param _map The costmap.
  bool
  is_free(const Eigen::Isometry2d& _pose,
          const costmap_2d::Costmap2D& _map) const;
};

/**
 * @brief A split footprint with generators.
 *
 * This class can be constructed from the continuous_footprint and maintains a
 * list of outline_generator for every sub-region which should be checked by its
 * outline and a list of area_generator for every sub-region which must be
 * checked entirely. However, it's likely more efficient to convert this class
 * further to a discrete_footprint.
 *
 * @see continuous_footprint
 * @see discrete_footprint
 */
struct generator_footprint
    : public detail::base_footprint<std::vector<outline_generator>,
                                    std::vector<area_generator>> {
  // Define the base type
  using base_type = detail::base_footprint<std::vector<outline_generator>,
                                           std::vector<area_generator>>;

  // Below the constructors.
  using base_type::base_footprint;

  /// @brief Will first construct a continuous_footprint and use it for
  /// construct a generator_footprint.
  ///
  /// The code below shows how a generator_footprint might be constructed:
  /// @snippet test/footprints.cpp generator_footprint_constructor
  template <typename... Args,
            typename std::enable_if<
                std::is_constructible<continuous_footprint, Args&&...>::value,
                bool>::type = true>
  generator_footprint(double _resolution, Args&&... args) {
    // Assemble the continuous_footprint and call init.
    init(_resolution, continuous_footprint{std::forward<Args>(args)...});
  }

  /// @brief Checks if the footprint is free at the given pose.
  /// @param _offset The offset.
  /// @param _map The costmap.
  bool
  is_free(const cell& _offset, const costmap_2d::Costmap2D& _map) const;

private:
  void
  init(double _resolution, const continuous_footprint& _footprint);
};

/**
 * @brief A split footprint with cells.
 *
 * This class can be constructed from the generator_footprint (it will expand
 * the generators into cell-arrays). While this version is likely faster than
 * the generator_footprint, it requires more memory. Pick the right one for your
 * use-case.
 *
 * @see continuous_footprint
 * @see generator_footprint
 */
struct discrete_footprint : public detail::base_footprint<discrete_polygon> {
  using base_type = detail::base_footprint<discrete_polygon>;
  using base_type::base_footprint;

  /// @brief Will first construct a generator_footprint and use it for
  /// construct a discrete_footprint.
  ///
  /// The code below shows how a discrete_footprint might be constructed:
  /// @snippet test/footprints.cpp discrete_footprint_constructor
  template <typename... Args,
            typename std::enable_if<
                std::is_constructible<generator_footprint, Args&&...>::value,
                bool>::type = true>
  discrete_footprint(Args&&... args) {
    init(generator_footprint{std::forward<Args>(args)...});
  }

  bool
  is_free(const cell& _offset, const costmap_2d::Costmap2D& _map) const;

private:
  void
  init(const generator_footprint& _generator);
};

template <typename Outlines, typename Areas>
bool
is_free(const detail::base_footprint<Outlines, Areas>& _footprint,
        const cell& _offset, const costmap_2d::Costmap2D& _map) {
  return _footprint.is_free(_offset, _map);
}

inline bool
is_free(const continuous_footprint& _footprint, const Eigen::Isometry2d& _pose,
        const costmap_2d::Costmap2D& _map) {
  return _footprint.is_free(_pose, _map);
}

}  // namespace cover
