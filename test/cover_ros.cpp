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
