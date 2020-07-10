#include "cover_ros.hpp"

#include <Eigen/Dense>
#include <angles/angles.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>
#include <ros/console.h>

#include <algorithm>
#include <iterator>
#include <unordered_map>
#include <vector>

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

// too long to write
using cm_location = costmap_2d::MapLocation;
using cm_polygon = std::vector<cm_location>;
using cm_map = costmap_2d::Costmap2D;

inline Eigen::Affine2d
to_affine(const se2& _pose) noexcept {
  return Eigen::Translation2d(_pose.segment(0, 2)) *
         Eigen::Rotation2Dd(_pose.z());
}

struct coordinate_eq {
  inline bool
  operator()(const cm_location& _l, const cm_location& _r) const noexcept {
    return _l.x == _r.x && _l.y == _r.y;
  }
};

cm_polygon
sparse_outline(const cm_map& _map, const polygon& _p) {
  // rasterise the input _p by the map's resolution
  cm_polygon sparse(_p.cols());
  for (int cc = 0; cc != _p.cols(); ++cc) {
    if (!_map.worldToMap(_p(0, cc), _p(1, cc), sparse[cc].x, sparse[cc].y))
      throw std::out_of_range("ring outside of the map");
  }

  // make it unique, since our internal resolution might be more granular
  const auto last = std::unique(sparse.begin(), sparse.end(), coordinate_eq{});
  sparse.erase(last, sparse.end());

  return sparse;
}

inline cm_polygon
dense_outline(cm_map& _map, const cm_polygon& _p) {
  // get the ray-traced ('dense') outline
  cm_polygon dense;
  _map.polygonOutlineCells(_p, dense);

  return dense;
}

inline cm_polygon
dense_outline(cm_map& _map, const polygon& _p) {
  // get the 'sparse' outline and 'densify' it
  const auto sparse = sparse_outline(_map, _p);
  return dense_outline(_map, sparse);
}

inline cm_polygon
dense_outline(cm_map& _map, const polygon& _p, const se2& _pose) {
  // rotate the polygon to the right pose
  const polygon sparse = to_affine(_pose) * _p;
  return dense_outline(_map, sparse);
}

struct cost_below {
  cost_below(const cm_map& _m, uint8_t _t) noexcept :
      map_(_m), threshold_(_t) {}

  inline bool
  operator()(const cm_location& _ml) const noexcept {
    return map_.getCost(_ml.x, _ml.y) < threshold_;
  }

private:
  const cm_map& map_;
  const uint8_t threshold_;
};

bool
check_ring(cm_map& _map, const polygon& _ring, const se2& _pose) {
  const auto dense = dense_outline(_map, _ring, _pose);
  cost_below cb(_map, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  return std::all_of(dense.begin(), dense.end(), cb);
}

inline int
signed_diff(const cm_location& _l, const cm_location& _r) noexcept {
  // note: the variables in cm_locations are unsigned, so we need the cast
  return static_cast<int>(_l.y) - static_cast<int>(_r.y);
}

inline bool
is_cusp(const cm_location& _l, const cm_location& _m,
        const cm_location& _r) noexcept {
  return signed_diff(_l, _m) * signed_diff(_m, _r) < 0;
}

bool
check_area(cm_map& _map, const polygon& _p, const se2& _pose) {
  // get the dense outline of the polygon
  const auto outline = dense_outline(_map, _p, _pose);

  // check the outline - we need this step, since we won't check the outline
  // cells in the area check below
  cost_below cb{_map, costmap_2d::LETHAL_OBSTACLE};
  if (!std::all_of(outline.begin(), outline.end(), cb))
    return false;

  // now we do a sort-free line-fill-algorithm for area checking.
  // we don't sort by y-values, but just throw them into a hash-map.
  // algorithm has 2 steps:
  // 1 - generate the line_scan; (y-lines) of cooridnate pairs
  // 2 - check pairwise the values
  using x_list = std::vector<cm_location>;
  using y_hash = std::unordered_map<unsigned int, x_list>;

  y_hash line_scan;

  // better safe then sorry
  if (outline.size() < 3)
    throw std::out_of_range("invalid size");

  // below step 1: generate the line_scan structure.
  // we have to be extra-carefully at the ends, since the outline is a closed
  // polygon. here we check the first element
  auto end = std::prev(outline.end());
  while (end != outline.begin() && end->y == outline.begin()->y)
    --end;

  if (end == outline.begin())
    throw std::runtime_error("no area defined");

  auto start = std::next(outline.begin());
  while (start != end && start->y == outline.begin()->y)
    ++start;

  if (start == end)
    throw std::runtime_error("no area defined");

  // the actual 'wrapped' check [end ..., begin(), ...start]
  if (!is_cusp(*end, outline.front(), *start))
    line_scan[outline.front().y].emplace_back(outline.front());

  // use a two pointer iteration to add 'non-horizontal' elements in [start end)
  for (auto l = outline.begin(), m = start; m != end;) {
    auto r = std::next(m);
    // skip the horizontal part...
    while (r != end && m->y == r->y) {
      ++r;
      ++m;
    }

    // skips cusps in y-direction...
    if (!is_cusp(*l, *m, *r))
      line_scan[m->y].emplace_back(*m);

    // update the variables
    l = m;
    m = r;
  }

  // check end - here we short-cut the algorithm
  if (!line_scan[end->y].empty() && line_scan[end->y].size() % 2 == 1)
    line_scan[end->y].emplace_back(*end);

  // below step 2: now check for every y line the x-pairs
  for (const auto& line_pairs : line_scan) {
    const auto& x_line = line_pairs.second;
    // sanity check if we are good to go
    if (x_line.size() % 2 != 0)
      throw std::logic_error("line-scan algorithm failed");

    // now check the line between the pairs
    // note: skip the border-cells, since we checked them above
    for (auto start = x_line.begin(); start != x_line.end(); start += 2) {
      // end is defined, since do have 2-steps-increments
      const auto end = std::next(start);

      // we have to be carefull here, since x_max is unsigned
      auto x_max = std::max(start->x, end->x);
      if(x_max != 0)
        --x_max;

      for (auto x_min = std::min(start->x, end->x) + 1; x_min <= x_max; ++x_min)
        if (_map.getCost(x_min, start->y) == costmap_2d::LETHAL_OBSTACLE)
          return false;
    }
  }

  return true;
}

bool
check_dense(cm_map& _map, const polygon_vec& _pl, const se2& _pose) {
  auto checker = [&](const polygon& _p) {
    return check_area(_map, _p, _pose);
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