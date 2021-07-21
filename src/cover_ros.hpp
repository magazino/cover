#pragma once

#include "cover.hpp"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace cover {

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
 * @param _map ros-costmap. note: we wont alter the map, but it's ros...
 * @param _footprint footprint structure (see make_footprint)
 * @param _pose the pose of the footprint in global coordinate frame
 * @param _type the type of checks to perform
 *
 * @return true, if the pose is collision-free
 *
 * @throw if the pose if out of map
 *
 * If you specify check_type::DENSE, we will **only** check the dense part.
 * If you specify check_type::RING, we will **only** check the inflated-ring.
 * If you specify check_type::ALL, both we will check both types.
 *
 * Note: In case of passing check_type::ALL or check_type::RING, the costmap
 * **must** include inflation information from the costmap_2d::InflationLayer.
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

/**
 * @brief A throwing version of check_discrete_pose.
 * @copydetails check_discrete_pose
 * @throw std::runtime_error, if the footprint is not fully inside of the
 * costmap.
 */
bool
check_discrete_pose_throw(const costmap_2d::Costmap2D& _map,
                          const discrete_footprint& _footprint,
                          const Eigen::Vector2i _origin, check_type _type);

/**
 * @brief Checks if the discrete footprint translated to the _origin is free.
 *
 * @param _map The costmap.
 * @param _footprint The (oriented and) descrete footprint.
 * @param _origin The origin of the footprint within the costmap.
 * @param _type The type of checks to perform.
 *
 * @return true, if the footprint is free and fully within the costmap.
 */
bool
check_discrete_pose(const costmap_2d::Costmap2D& _map,
                    const discrete_footprint& _footprint,
                    const Eigen::Vector2i _origin, check_type _type);

}  // namespace cover