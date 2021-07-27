// --- Internal Includes ---
#include <cover_ros.hpp>
#include "cover_test_utils.hpp"

// --- ROS Includes ---
#include <base_local_planner/footprint_helper.h>

// --- Test Includes ---
#include <gtest/gtest.h>

// --- Standard Includes ---
#include <cmath>
#include <tuple>
#include <vector>

using namespace cover;

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

using d_vector = std::vector<double>;

struct check_area_fixture
    : public testing::TestWithParam<std::tuple<d_vector, double>> {
  costmap_2d::Costmap2D map;
  base_local_planner::FootprintHelper fh;
  polygon p;
  double yaw;

  check_area_fixture() :
      map(30, 30, 0.1, -1.5, -1.5),
      p(make_footprint(std::get<0>(GetParam()))),
      yaw(std::get<1>(GetParam())) {}
};

// we will use a triangle, rectangle and a weird shape
// additionally we will rotate the shapes slightly
INSTANTIATE_TEST_CASE_P(
    /**/, check_area_fixture,
    testing::Combine(testing::Values(d_vector{1, -1, -1, 0, 1, -1},
                                     d_vector{1, 1, -1, -1, -1, 1, 1, -1},
                                     d_vector{1, 1, 0, -1, -1, -1, 1, 0.5, 1,
                                              -1}),
                     testing::Values(-2., -1., 0., 1., 2.)));

TEST_P(check_area_fixture, generic) {
  se2 center{0, 0, yaw};
  map.mapToWorld(15, 15, center.x(), center.y());
  const auto msg = to_msgs(p);
  const auto cells = fh.getFootprintCells(center.cast<float>(), msg, map, true);

  // iterate over all cells
  for (const auto& c : cells) {
    // mark one cell as occupied
    map.setCost(c.x, c.y, costmap_2d::LETHAL_OBSTACLE);

    // check that we are correct
    EXPECT_FALSE(check_area(map, p, center));

    // clear the cell
    map.setCost(c.x, c.y, 0);
  }

  // now mark all cells but the footprint as occupied
  map.setDefaultValue(costmap_2d::LETHAL_OBSTACLE);
  for (const auto& c : cells)
    map.setCost(c.x, c.y, 0);

  EXPECT_TRUE(check_area(map, p, center));
}

TEST(check_area, unit_circle) {
  // test is same as above but with a unit circle
  // todo refactor into one function
  costmap_2d::Costmap2D map(30, 30, 0.1, -1.5, -1.5);
  base_local_planner::FootprintHelper fh;
  const polygon circle = make_circle(8, 1);
  const se2 center{0, 0, 0};

  const auto msg = to_msgs(circle);
  const auto cells = fh.getFootprintCells(center.cast<float>(), msg, map, true);

  // iterate over all cells
  for (const auto& c : cells) {
    // mark one cell as occupied
    map.setCost(c.x, c.y, costmap_2d::LETHAL_OBSTACLE);

    // check that we are correct
    EXPECT_FALSE(check_area(map, circle, center));

    // clear the cell
    map.setCost(c.x, c.y, 0);
  }

  // now mark all cells but the footprint as occupied
  map.setDefaultValue(costmap_2d::LETHAL_OBSTACLE);
  for (const auto& c : cells)
    map.setCost(c.x, c.y, 0);

  EXPECT_TRUE(check_area(map, circle, center));
}
