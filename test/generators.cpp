#include "generators.hpp"
#include "cover_test_utils.hpp"

// --- ROS Includes ---
#include <base_local_planner/footprint_helper.h>
#include <costmap_2d/costmap_2d.h>

// --- Gtest Includes ---
#include <gtest/gtest.h>

#include <array>
#include <unordered_set>
#include <vector>

using namespace cover;

////////////////////////////////////////////////////////////////////////////////
// RayGenerator Tests
////////////////////////////////////////////////////////////////////////////////

TEST(ray_generator, empty) {
  ray_generator gen(cell::Zero(), cell::Zero());
  ASSERT_TRUE(gen.empty());

  // Check the iteration.
  for (const auto& c : gen) {
    FAIL();
    // Avoid compiler warnings
    ASSERT_EQ(c, cell::Zero());
  }
}

TEST(ray_generator, simple) {
  /// [ray_generator]
  // Define a ray-generator
  cover::ray_generator ray({-5, 0}, {5, 0});

  // Check the size
  ASSERT_EQ(ray.size(), 5 + 5);
  ASSERT_FALSE(ray.empty());

  // Iterate over all cells
  cover::cell expected = *ray.begin();
  for (const auto& c : ray) {
    ASSERT_EQ(c, expected);
    ++expected.x();
  }
  /// [ray_generator]
}

struct to_line_fixture : public testing::TestWithParam<std::array<int, 4>> {};

static to_line_fixture::ParamType to_line_params[] = {
    {0, 0, 100, 30},     {0, 0, 100, 100},    {30, 100, 0, 0},
    {0, 100, 100, 30},   {10, -11, -10, 30},  {0, 0, -10, -30},
    {-10, -30, 100, 30}, {10, -30, -100, 30}, {0, 2, 2, 2}};

INSTANTIATE_TEST_CASE_P(
    /**/, to_line_fixture, testing::ValuesIn(to_line_params));

TEST_P(to_line_fixture, compare_ros) {
  const auto p = GetParam();
  base_local_planner::FootprintHelper fh;
  using blp_vector = std::vector<base_local_planner::Position2DInt>;

  // run the code
  const auto res = to_line(cell{p[0], p[1]}, cell{p[2], p[3]});

  // we should not be empty
  ASSERT_NE(res.cols(), 0);

  // we use the base_local_planner as reference - run them also
  blp_vector expected;
  fh.getLineCells(p[0], p[2], p[1], p[3], expected);

  ASSERT_EQ(res.cols(), expected.size() - 1);

  // now compare the result
  for (int ii = 0; ii != res.cols(); ++ii) {
    ASSERT_EQ(res.col(ii).x(), expected[ii].x);
    ASSERT_EQ(res.col(ii).y(), expected[ii].y);
  }
}

////////////////////////////////////////////////////////////////////////////////
// OutlineGenerator Tests
////////////////////////////////////////////////////////////////////////////////

TEST(outline_generator, empty) {
  outline_generator gen({});
  ASSERT_TRUE(gen.empty());

  // Check the iteration.
  for (const auto& c : gen) {
    FAIL();
    ASSERT_EQ(c, cell::Zero());
  }
}

TEST(outline_generator, one) {
  // If all points are the same, the outline will just be one point.
  const discrete_polygon outline = discrete_polygon::Constant(2, 10, 1);
  const outline_generator gen(outline);

  ASSERT_EQ(gen.size(), 1);
  ASSERT_EQ(*gen.begin(), cell(1, 1));
}

TEST(outline_generator, two) {
  /// [outline_generator_two]
  // If we have only two distinct points, the outline is the ray, but with the
  // end-point.
  discrete_polygon outline = discrete_polygon::Constant(2, 10, 1);
  outline.col(9) << 2, 3;
  const ray_generator ray(outline.col(0), outline.col(9));
  const outline_generator gen(outline);

  // The outline generator includes the end-points.
  ASSERT_EQ(gen.size(), ray.size() + 1);
  auto o_iter = gen.begin();
  auto r_iter = ray.begin();
  for (; r_iter != ray.end(); ++r_iter, ++o_iter)
    ASSERT_EQ(*r_iter, *o_iter);

  // The last element is the end-point.
  ASSERT_EQ(*o_iter, cell(2, 3));
  /// [outline_generator_two]
}

TEST(outline_generator, box) {
  /// [outline_generator_box]
  // Create a sparse box
  cover::discrete_polygon sparse(2, 4);
  constexpr int rows = 10;
  constexpr int cols = 20;
  sparse.col(0) << 0, 0;
  sparse.col(1) << rows, 0;
  sparse.col(2) << rows, cols;
  sparse.col(3) << 0, cols;

  cover::outline_generator gen{sparse};
  // The outline has the size of (rows + cols) * 2
  ASSERT_EQ(gen.size(), (rows + cols) * 2);

  // Now check the cells
  auto iter = gen.begin();

  // The first flank
  for (int ii = 0; ii != rows; ++ii, ++iter) {
    ASSERT_NE(iter, gen.end());
    ASSERT_EQ(iter->x(), ii);
    ASSERT_EQ(iter->y(), 0);
  }

  // The second flank
  for (int ii = 0; ii != cols; ++ii, ++iter) {
    ASSERT_NE(iter, gen.end());
    ASSERT_EQ(iter->x(), rows);
    ASSERT_EQ(iter->y(), ii);
  }

  // The third flank
  for (int ii = 0; ii != rows; ++ii, ++iter) {
    ASSERT_NE(iter, gen.end());
    ASSERT_EQ(iter->x(), rows - ii);
    ASSERT_EQ(iter->y(), cols);
  }

  // The fourth flank
  for (int ii = 0; ii != cols; ++ii, ++iter) {
    ASSERT_NE(iter, gen.end());
    ASSERT_EQ(iter->x(), 0);
    ASSERT_EQ(iter->y(), cols - ii);
  }

  ASSERT_EQ(iter, gen.end());

  /// [outline_generator_box]
}

struct outline_fixture : public testing::TestWithParam<std::vector<double>> {};
INSTANTIATE_TEST_CASE_P(/**/, outline_fixture, testing::ValuesIn(footprints));

TEST_P(outline_fixture, compare_ros) {
  // Compares the behavior to the ros lib

  // Create a polygon and shift it into the positive space.
  polygon footprint = make_footprint(GetParam());
  footprint.colwise() -= footprint.rowwise().minCoeff();
  footprint.array() += 0.5;

  // Run the cover implementation.
  const discrete_polygon outline = to_outline(0.05, footprint);

  // Run the ros implementation.
  std::vector<costmap_2d::MapLocation> sparse(outline.cols()), expected;
  for (int cc = 0, cols = outline.cols(); cc != cols; ++cc) {
    sparse.at(cc).x = outline(0, cc);
    sparse.at(cc).y = outline(1, cc);
  }
  costmap_2d::Costmap2D map(500, 500, 0.05, 0, 0);
  map.polygonOutlineCells(sparse, expected);

  // Compare the results (not that ros's version might contain duplicates)
  std::unordered_set<size_t> index_cover, index_ros;
  for (int cc = 0, cols = outline.cols(); cc != cols; ++cc) {
    index_cover.insert(map.getIndex(outline(0, cc), outline(1, cc)));
  }

  for (const auto& cc : expected) {
    index_ros.insert(map.getIndex(cc.x, cc.y));
  }

  ASSERT_EQ(index_cover, index_ros);
}

////////////////////////////////////////////////////////////////////////////////
// AreaGenerator Tests
////////////////////////////////////////////////////////////////////////////////

TEST(area_generator, empty) {
  area_generator gen({});
  ASSERT_TRUE(gen.empty());

  for (const auto& c : gen) {
    FAIL();
    ASSERT_EQ(c, cell::Zero());
  }
}

TEST(area_generator, box) {
  /// [area_generator]
  cover::discrete_polygon sparse(2, 4);
  constexpr int rows = 10;
  constexpr int cols = 20;
  sparse.col(0) << 0, 0;
  sparse.col(1) << rows, 0;
  sparse.col(2) << rows, cols;
  sparse.col(3) << 0, cols;

  // Create the area_generator
  cover::area_generator gen(cover::to_outline(sparse));

  // The size of the box is rows * cols. The boundary is included!
  ASSERT_EQ(gen.size(), (rows + 1) * (cols + 1));

  // The lambda converts the cell to a 1d index.
  auto to_index = [&](const cell& _cell) {
    return _cell.x() + _cell.y() * (rows + 1);
  };

  int index = 0;
  for (const auto& c : gen) {
    ASSERT_EQ(to_index(c), index++);
  }

  /// [area_generator]
}
