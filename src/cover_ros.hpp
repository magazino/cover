#pragma once

#include "cover.hpp"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace cover {

/// @brief se2 pose [x, y, theta].T
using se2 = Eigen::Vector3d;

/**
 * @brief splits the polygon into dense areas and the inscribed ring.
 *
 * @param _msg the outer points of a footprint.
 *
 * @throw if the _msg is ill-formed
 *
 * Internally the function calls cover::split.
 * The inscribed radius uses the default ros-navigation function
 * costmap_2d::calculateMinAndMaxDistances.
 *
 * The input _msg must have at least three distinct points.
 * If the defined polygon is not closed, we will close it for you.
 */
footprint
make_footprint(const std::vector<geometry_msgs::Point>& _msg);

bool
check_area(costmap_2d::Costmap2D& _map, const polygon& _p, const se2& _pose);

/**
 * @brief checks if the pose is collision-free
 *
 * @param _map ros-costmap. note: we wont alter the map, but its ros...
 * @param _footprint footprint structure (see make_footprint)
 * @param _pose the metrical pose of the footprint
 * @param _type the type of checks to perform
 *
 * @return true, if the pose is collision-free
 *
 * @throw if the pose if out of map
 *
 * If you specify the check_type::ALL or check_type::RING, the costmap must
 * include inflation information from the costmap_2d::InflationLayer.
 */
bool
check_pose_throw(costmap_2d::Costmap2D& _map, const footprint& _footprint,
                 const se2& _pose, check_type _type);

/**
 * @brief same as check_pose_throw, but returns false instead of throwing
 */
bool
check_pose(costmap_2d::Costmap2D& _map, const footprint& _footprint,
           const se2& _pose, check_type _type) noexcept;

}  // namespace cover