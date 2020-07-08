#pragma once

#include "cover.hpp"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace cover {

// our lazy definition of a se2 pose
using se2 = Eigen::Vector3d;

footprint
make_footprint(const std::vector<geometry_msgs::Point>& _msg);

bool
check_pose_throw(costmap_2d::Costmap2D& _map, const footprint& _footprint,
                 const se2& _pose, check_type _type);

bool
check_pose(costmap_2d::Costmap2D& _map, const footprint& _footprint,
           const se2& _pose, check_type _type) noexcept;

}  // namespace cover