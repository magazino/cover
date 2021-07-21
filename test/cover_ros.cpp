#include "cover_test_utils.hpp"

#include <cover_ros.hpp>
#include <gtest/gtest.h>

#include <base_local_planner/footprint_helper.h>

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
      p(2, std::get<0>(GetParam()).size() / 2),
      yaw(std::get<1>(GetParam())) {
    int r = 0;
    int c = 0;
    for (const auto& v : std::get<0>(GetParam())) {
      p(r, c++) = v;
      if (c == p.cols()) {
        ++r;
        c = 0;
      }
    }
  }
};

// we will use a triangle, rectangle and a weird shape
// additionally we will rotate the shapes slightly
const auto values = testing::Combine(
    testing::Values(d_vector{1, -1, -1, 0, 1, -1},
                    d_vector{1, 1, -1, -1, -1, 1, 1, -1},
                    d_vector{1, 1, 0, -1, -1, -1, 1, 0.5, 1, -1}),
    testing::Values(-2., -1., 0., 1., 2.));

INSTANTIATE_TEST_CASE_P(
    /**/, check_area_fixture, values);

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

struct fill_area_fixture : public check_area_fixture {
  using pos_t = base_local_planner::Position2DInt;
  using pos_vec_t = std::vector<pos_t>;

  fill_area_fixture() : check_area_fixture() {}

  /**
   * @brief Computes the fill cells using base local planner
   *
   * @param _polygon The outer polygon representing the footprint
   * @return pos_vec_t Fill cells computed by base local planner
   */
  pos_vec_t
  compute_blp_cells(const discrete_polygon& _polygon) {
    const auto n = static_cast<size_t>(_polygon.cols() + 1);
    pos_vec_t fp(n);

    // Copy over the data
    for (size_t idx = 0; idx < n - 1; ++idx) {
      fp[idx].x = _polygon(0, idx);
      fp[idx].y = _polygon(1, idx);
    }
    // Manually close the footprint as it is required for getFillCells
    fp.back().x = _polygon(0, 0);
    fp.back().y = _polygon(1, 0);

    fh.getFillCells(fp);

    return fp;
  }

  /**
   * @brief Coverts discrete polygon to vector
   *
   * @param _polygon Underlying polygon
   * @return pos_vec_t Polygon converted to vector
   */
  static pos_vec_t
  to_pos_vec(const discrete_polygon& _polygon) {
    const auto n_cells = static_cast<size_t>(_polygon.cols());

    pos_vec_t cells(n_cells);
    for (size_t idx = 0; idx < n_cells; ++idx) {
      cells[idx].x = _polygon(0, idx);
      cells[idx].y = _polygon(1, idx);
    }

    return cells;
  }

  static bool
  compare_points(const pos_t& _pt0, const pos_t& _pt1) {
    if (_pt0.x != _pt1.x) {
      return _pt0.x < _pt1.x;
    }
    return _pt0.y < _pt1.y;
  }

  static bool
  are_points_equal(const pos_t& _pt0, const pos_t& _pt1) {
    return (_pt0.x == _pt1.x) && (_pt0.y == _pt1.y);
  }

  /**
   * @brief Sorts and removes duplicate points from the vector in-place
   *
   * @param _vec 2D points containing the cell coordinates
   */
  static void
  uniquify(pos_vec_t& _vec) {
    std::sort(_vec.begin(), _vec.end(), compare_points);
    _vec.erase(std::unique(_vec.begin(), _vec.end()), _vec.end());
  }
};

INSTANTIATE_TEST_CASE_P(
    /**/, fill_area_fixture, values);

/**
 * @brief We check whether the area function of cover library returns the
 same
 * cells as the base_local_planner counterpart
 */
TEST_P(fill_area_fixture, area_cells) {
  // Compute the discretized footprint
  auto disc_fp = dense_outline(p, map.getResolution());
  disc_fp.colwise() -= disc_fp.rowwise().minCoeff();

  // Compute the fill cells using cover library
  auto cover_cells = to_pos_vec(area(disc_fp));
  uniquify(cover_cells);

  // Similarly, compute the cells using base local planner
  auto blp_cells = compute_blp_cells(disc_fp);
  uniquify(blp_cells);

  // Finally check if the results are the same
  EXPECT_EQ(cover_cells.size(), blp_cells.size());

  // We can do this comparison as we sorted the points while removing duplicates
  size_t n_mismatches = 0;
  for (size_t idx = 0; idx < cover_cells.size(); ++idx) {
    // Add to counter if points are not equal
    n_mismatches += !are_points_equal(cover_cells[idx], blp_cells[idx]);
  }

  EXPECT_EQ(n_mismatches, 0);
}
