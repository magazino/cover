#include <cover_ros.hpp>
#include <gtest/gtest.h>

#include <random>

#include <base_local_planner/footprint_helper.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

using namespace cover;

polygon
make_circle(size_t s, double _rad) noexcept {
  polygon out(2, s);
  for (size_t ii = 0; ii != s; ++ii) {
    const auto angle = M_PI * 2 * ii / s;
    out.col(ii) << std::cos(angle), std::sin(angle);
  }
  // scale
  out *= _rad;
  return out;
}

constexpr double rad = 0.2;

struct out_of_range_fixture : public ::testing::TestWithParam<se2> {
  footprint fp;
  costmap_2d::Costmap2D map;
  se2 pose;

  out_of_range_fixture() : map(40, 20, 0.05, 1, 2), pose(GetParam()) {
    // create a simple footprint
    fp = split(make_circle(8, rad), rad - 1e-3);
  }
};

// place the poses at the 4 edges of the map
INSTANTIATE_TEST_CASE_P(/**/, out_of_range_fixture,
                        testing::Values(se2{1, 2, 3}, se2{3, 2, 0},
                                        se2{3, 3, 0}, se2{1, 3, 0}));

TEST_P(out_of_range_fixture, generic) {
  // verify that we reject poses out of range
  EXPECT_ANY_THROW(check_pose_throw(map, fp, pose, check_type::ALL));
}

// simple counter check
using inside_map_fixture = out_of_range_fixture;

// place the poses at the 4 edges of the map
INSTANTIATE_TEST_CASE_P(/**/, inside_map_fixture,
                        testing::Values(se2{1 + rad, 2 + rad, 3},
                                        se2{3 - rad, 2 + rad, 0},
                                        se2{3 - rad, 3 - rad, 0},
                                        se2{1 + rad, 3 - rad, 0}));

TEST_P(inside_map_fixture, generic) {
  // verify that we reject poses out of range
  EXPECT_NO_THROW(check_pose_throw(map, fp, pose, check_type::ALL));
}

TEST(outline, triangle) {
  costmap_2d::Costmap2D map(30, 30, 0.1, -1.5, -1.5);
  std::vector<geometry_msgs::Point> msg(4);
  msg[0].x = 1;
  msg[0].y = 1;
  msg[1].x = -1;
  msg[1].y = 1;
  msg[2].x = -1;
  msg[2].y = -1;
  msg[3].x = 1;
  msg[3].y = -1;
  Eigen::Vector3d center{0, 0, 1};
  map.mapToWorld(15, 15, center.x(), center.y());
  // paint the outline into the map
  base_local_planner::FootprintHelper fh;
  const auto cells = fh.getFootprintCells(center.cast<float>(), msg, map, true);

  for (const auto& c : cells) {
    map.setCost(c.x, c.y,  1);
  }

  const auto fp = make_footprint(msg);
  for (const auto& d : fp.dense) {
    std::cout << d.transpose() << std::endl << std::endl;
    // convert to msg
    std::vector<geometry_msgs::Point> dm(d.cols());
    for (int cc = 0; cc != d.cols(); ++cc) {
      dm[cc].x = d(0, cc);
      dm[cc].y = d(1, cc);
    }
    const auto dc = fh.getFootprintCells(center.cast<float>(), dm, map, true);

    for (const auto& c : dc) {
      map.setCost(c.x, c.y, map.getCost(c.x, c.y) + 2);
      // map.setCost(c.x, c.y, 3);
    }
    // break;
  }

  // print the map
  for (int ii = 0; ii != 30; ++ii) {
    for (int jj = 0; jj != 30; ++jj) {
      std::cout << static_cast<int>(map.getCost(ii, jj)) << ",";
    }
    std::cout << std::endl;
  }
}
