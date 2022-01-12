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
#include <cover/expand.hpp>
#include <cover/generators.hpp>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Geometry>

#include <cassert>
#include <type_traits>
#include <vector>

namespace cover {

namespace detail {

/// @defgroup CostmapAccessors Costmap Accessors
/// Classes for interfacing the cell with the costmap_2d::Costmap2D.
/// @{

/// @brief Checks if a cell is inside the costmap.
/// @snippet test/costmap.cpp is_inside
struct is_inside {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  is_inside() : upper_(Eigen::Array2i::Zero()) {}

  /**
   * @brief Constructor
   *
   * @param _map The costmap.
   */
  explicit is_inside(const costmap_2d::Costmap2D& _map) :
      upper_(_map.getSizeInCellsX(), _map.getSizeInCellsY()) {}

  bool
  operator()(const cell& _cell) const noexcept {
    const Eigen::Array2i a_cell = _cell.array();
    return (a_cell >= 0).all() && (a_cell < upper_).all();
  }

private:
  const Eigen::Array2i upper_;  ///< The upper bound of the map.
};

/// @brief Base-class for accessing the costmap.
struct _costmap_functor {
  /**
   * @param _map The map on which we will perform the checks.
   *
   * Note: The map instance must outlive this instance.
   */
  explicit _costmap_functor(const costmap_2d::Costmap2D& _map) noexcept :
      is_inside_(_map), map_(_map) {}

  const is_inside is_inside_;         ///< Bounds check.
  const costmap_2d::Costmap2D& map_;  ///< The costmap to check.
};

/// @brief Checks if a cell is lethal.
/// Given the cost-values of @snippet test/costmap.cpp get_cell_cost
/// The class returns @snippet test/costmap.cpp is_lethal
struct is_lethal : private _costmap_functor {
  // Use the c'tor.
  using _costmap_functor::_costmap_functor;

  bool
  operator()(const cell& _cell) const {
    if (!is_inside_.operator()(_cell))
      return true;
    return map_.getCost(_cell.x(), _cell.y()) == costmap_2d::LETHAL_OBSTACLE;
  }
};

/// @brief Checks if the cell is either lethal or inflated.
/// Given the cost-values of @snippet test/costmap.cpp get_cell_cost
/// The class returns @snippet test/costmap.cpp is_lethal_or_inflated
struct is_lethal_or_inflated : private _costmap_functor {
  // Use the c'tor.
  using _costmap_functor::_costmap_functor;

  bool
  operator()(const cell& _cell) const {
    if (!is_inside_.operator()(_cell))
      return true;

    const auto cost = map_.getCost(_cell.x(), _cell.y());
    return cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
           cost == costmap_2d::LETHAL_OBSTACLE;
  }
};

/// @brief Returns the cost of the cell.
/// @snippet test/costmap.cpp get_cell_cost
struct get_cell_cost : private _costmap_functor {
  // Use the c'tor.
  using _costmap_functor::_costmap_functor;

  /// @brief Returns the cost of a given cell.
  /// @warning The user must make sure that the cell is inside the map.
  unsigned char
  operator()(const cell& _cell) const {
    assert(is_inside_.operator()(_cell) && "Invalid cell");
    return map_.getCost(_cell.x(), _cell.y());
  }
};

struct costmap_model_like_getter : private _costmap_functor {
  // Use the c'tor.
  using _costmap_functor::_costmap_functor;

  /// @brief Implements the remapping as in ros-navigation's costmap-model.
  double
  operator()(const cell& _cell) const;
};

template <typename Base>
struct with_offset : private Base {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  with_offset(const Base& _base, const cell& _offset) :
      Base(_base), offset_(_offset) {}

  typename std::result_of<Base(const cell&)>::type
  operator()(const cell& _cell) const {
    return Base::operator()(_cell + offset_);
  }

private:
  const cell offset_;  ///< The offset which will be applied to a cell.
};

template <typename Base>
with_offset<Base>
make_with_offset(const Base& _base, const cell& _cell) {
  return with_offset<Base>{_base, _cell};
}

/**@}*/

}  // namespace detail

/// @brief Converts the ros-msg to a Eigen::Vector.
/// @snippet test/costmap.cpp make_polygon
polygon
make_polygon(const std::vector<geometry_msgs::Point>& _msg) noexcept;

/// @defgroup CostmapChecks Costmap Checks
/// Group of functions to check if a set of cells is free. The functions below
/// offer similar functionality as ros-navigation's
/// [base_local_planner::footprint_helper.h](https://github.com/ros-planning/navigation/blob/noetic-devel/base_local_planner/include/base_local_planner/footprint_helper.h)
/// and
/// [base_local_planner::costmap_model.h](https://github.com/ros-planning/navigation/blob/noetic-devel/base_local_planner/include/base_local_planner/costmap_model.h).

/// @{

/// @brief Returns true if all cells are inside the map and have not the value
/// costmap_2d::LETHAL_OBSTACLE.
///
/// The function call is equivalent to creating the generator and a is_lethal
/// instance and checking if none of the cells is lethal.
///
/// @tparam Generator The generator type (e.x. cover::ray_generator)
/// @param _gen The generator.
/// @param _map The costmap.
///
/// The code shows the basic semantics of the function:
/// @snippet test/costmap.cpp is_free_generator
///
/// @see cover::detail::is_lethal
/// @see Generators
template <typename Generator>
bool
is_free(const Generator& _gen, const costmap_2d::Costmap2D& _map);

/// @copybrief is_free(const Generator&, const costmap_2d::Costmap2D&)
///
/// The function will add the _offset to every cell before calling the same
/// logic as cover::is_free(const Generator&, const costmap_2d::Costmap2D&).
///
/// @tparam Generator The generator type (e.x. cover::ray_generator)
/// @param _gen The generator.
/// @param _offset The offset.
/// @param _map The costmap.
///
/// The following snippet shows an example:
/// @snippet test/costmap.cpp is_free_with_offset
///
/// @see cover::detail::with_offset
/// @see is_free(const Generator&, const costmap_2d::Costmap2D&)
template <typename Generator>
bool
is_free(const Generator& _gen, const cell& _offset,
        const costmap_2d::Costmap2D& _map);

/**
 * @copybrief is_free(const Generator&, const costmap_2d::Costmap2D&)
 *
 * @param _dp The discrete polygon.
 * @param _map The costmap.
 */
bool
is_free(const discrete_polygon& _dp, const costmap_2d::Costmap2D& _map);

/**
 * @copybrief is_free(const Generator&, const costmap_2d::Costmap2D&)
 *
 * The function will add the _offset to every cell and then apply
 * is_free(const discrete_polygon& _dp, const costmap_2d::Costmap2D& _map)
 *
 * @param _dp The discrete polygon.
 * @param _offset The offset.
 * @param _map The costmap.
 *
 * @see is_free(const discrete_polygon& _dp, const costmap_2d::Costmap2D& _map)
 */
bool
is_free(const discrete_polygon& _dp, const cell& _offset,
        const costmap_2d::Costmap2D& _map);

/**
 * @copybrief is_free(const Generator&, const costmap_2d::Costmap2D&)
 *
 * @param _polygon The polygon in the world-frame of the map.
 * @param _map The costmap.
 */
bool
is_free(const polygon& _polygon, const costmap_2d::Costmap2D& _map);

/**
 * @copybrief is_free(const Generator&, const costmap_2d::Costmap2D&)
 *
 * @param _polygon The polygon.
 * @param _pose The pose of the polygon in the world-frame of the map.
 * @param _map The costmap.
 */
bool
is_free(const polygon& _polygon, const Eigen::Isometry2d& _pose,
        const costmap_2d::Costmap2D& _map);

/// @brief Returns the sum of the costs of all cells within the _gen.
///
/// The function follows the interface of
/// [base_local_planner::CostmapModel::footprintCost](https://github.com/ros-planning/navigation/blob/noetic-devel/base_local_planner/include/base_local_planner/costmap_model.h).
/// It will return:
/// - 1 If at least one cell has the value costmap_2d::LETHAL_OBSTACLE.
/// - 2 If at least one cell has the value costmap_2d::NO_INFORMATION.
/// - 3 If at least one cell is outside of the bounds.
/// The return value here however, depends on the query order of the cells. If
/// no cell fulfills any of the three conditions, the function returns the sum
/// of the costs of all cells.
///
/// @tparam Generator The generator type.
/// @param _gen The generator.
/// @param _map The costmap.
///
/// An example when all cells are valid:
/// @snippet test/costmap.cpp accumulate_cost_generator
///
/// Here an example when at least one cell is outside of the map:
/// @snippet test/costmap.cpp accumulate_cost_generator_out_of_bounds
///
/// @see cover::detail::is_inside
/// @see cover::detail::costmap_model_like_getter
template <typename Generator>
double
accumulate_cost(const Generator& _gen, const costmap_2d::Costmap2D& _map);

/// @copybrief accumulate_cost(const Generator&, const costmap_2d::Costmap2D&)
///
//// The function will apply the _offset to every cell before calling the same
/// logic as
/// cover::accumulate_cost(const Generator&, const costmap_2d::Costmap2D&).
///
/// @tparam Generator The generator type.
/// @param _gen The generator.
/// @param _offset The offset.
/// @param _map The costmap.
///
/// The following code shows an example use-case:
/// @snippet test/costmap.cpp accumulate_cost_with_offset
///
/// @see cover::detail::with_offset
/// @see accumulate_cost(const Generator&, const costmap_2d::Costmap2D&)
template <typename Generator>
double
accumulate_cost(const Generator& _gen, const cell& _offset,
                const costmap_2d::Costmap2D& _map);

/// @}

}  // namespace cover

#include "impl/costmap.hpp"