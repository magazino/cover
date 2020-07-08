#include "cover_ros.hpp"

#include <Eigen/Dense>
#include <angles/angles.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>
#include <ros/console.h>

#include <algorithm>

#define COVER_ERROR(args) ROS_ERROR_STREAM("cover: " << args)

namespace cover {

footprint
make_footprint(const std::vector<geometry_msgs::Point>& _msg) {
  // transform to polygon
  polygon pl(2, _msg.size());
  for (size_t ii = 0; ii != _msg.size(); ++ii)
    pl.col(ii) << _msg[ii].x, _msg[ii].y;

  // get the robot inflation radius
  double min, max;
  costmap_2d::calculateMinAndMaxDistances(_msg, min, max);

  // will throw if the footprint is ill-formed
  return split(pl, min - 1e-3);
}

using cm_polygon = std::vector<costmap_2d::MapLocation>;

inline Eigen::Affine2d
to_affine(const se2& _pose) noexcept {
  return Eigen::Translation2d(_pose.segment(0, 2)) *
         Eigen::Rotation2Dd(_pose.z());
}

struct coordinate_eq {
  inline bool
  operator()(const costmap_2d::MapLocation& _l,
             const costmap_2d::MapLocation& _r) const noexcept {
    return _l.x == _r.x && _l.y == _r.y;
  }
};

cm_polygon
dense_outline(costmap_2d::Costmap2D& _map, const polygon& _p) {
  // now check every cell on the transformed ring if its free
  cm_polygon sparse(_p.cols());
  for (int cc = 0; cc != _p.cols(); ++cc) {
    if (!_map.worldToMap(_p(0, cc), _p(1, cc), sparse[cc].x, sparse[cc].y))
      throw std::out_of_range("ring outside of the map");
  }

  // make it unique, since our internal resolution might be more granular
  const auto last = std::unique(sparse.begin(), sparse.end(), coordinate_eq{});
  sparse.erase(last, sparse.end());

  // get the ray-traced ('dense') outline
  cm_polygon dense;
  _map.polygonOutlineCells(sparse, dense);

  return dense;
}

struct cost_below {
  cost_below(const costmap_2d::Costmap2D& _m, uint8_t _t) noexcept :
      map_(_m), threshold_(_t) {}

  inline bool
  operator()(const costmap_2d::MapLocation& _ml) const noexcept {
    return map_.getCost(_ml.x, _ml.y) < threshold_;
  }

private:
  const costmap_2d::Costmap2D& map_;
  const uint8_t threshold_;
};

bool
check_ring(costmap_2d::Costmap2D& _map, const polygon& _ring,
           const se2& _pose) {
  // rotate the polygon to the right pose
  const polygon sparse = to_affine(_pose) * _ring;
  const auto dense = dense_outline(_map, sparse);

  cost_below cb(_map, costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
  return std::all_of(dense.begin(), dense.end(), cb);
}

bool
check_dense_area(costmap_2d::Costmap2D& _map, const polygon& _p,
                 const se2& _pose) {
  // rotate the polygon to the right pose
  const polygon sparse = to_affine(_pose) * _p;
  const auto dense = dense_outline(_map, sparse);

  cm_polygon area;
  _map.convexFillCells(dense, area);

  cost_below cb(_map, costmap_2d::LETHAL_OBSTACLE);
  return all_of(area.begin(), area.end(), cb);
}

bool
check_dense(costmap_2d::Costmap2D& _map, const polygon_vec& _pl,
            const se2& _pose) {
  auto checker = [&](const polygon& _p) {
    return check_dense_area(_map, _p, _pose);
  };

  return std::all_of(_pl.begin(), _pl.end(), checker);
}

bool
check_pose_throw(costmap_2d::Costmap2D& _map, const footprint& _footprint,
                 const se2& _pose, check_type _type) {
  if (_type == check_type::RING || _type == check_type::ALL)
    if (!check_ring(_map, _footprint.ring, _pose))
      return false;

  if (_type == check_type::DENSE || _type == check_type::ALL)
    if (!check_dense(_map, _footprint.dense, _pose))
      return false;

  return true;
}

bool
check_pose(costmap_2d::Costmap2D& _map, const footprint& _footprint,
           const se2& _pose, check_type _type) noexcept {
  try {
    return check_pose_throw(_map, _footprint, _pose, _type);
  }
  catch (std::exception& _ex) {
    COVER_ERROR(_ex.what());
    return false;
  }
}

}  // namespace cover