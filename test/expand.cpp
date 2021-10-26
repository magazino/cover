#include "expand.hpp"
#include "cover_test_utils.hpp"

#include <gtest/gtest.h>

#include <costmap_2d/costmap_2d.h>

// --- OpenCV Includes ---
#include <opencv2/opencv.hpp>

#include <tuple>
#include <unordered_set>
#include <vector>

using namespace cover;

struct triangle_fixture : public testing::Test {
  cover::polygon triangle;
  const double resolution = 0.1;
  triangle_fixture() : triangle(2, 3) { triangle << 1, 2, 3, 4, 5, 6; }
};

TEST_F(triangle_fixture, outline_generator) {
  /// [expand_outline_generator]
  // Create a outline_generator and expand it.
  cover::outline_generator gen(resolution, triangle);
  cover::discrete_polygon outline = cover::expand(gen);

  ASSERT_EQ(gen.size(), outline.cols());

  auto begin = gen.begin();
  for (int ii = 0; ii != outline.cols(); ++ii, ++begin) {
    ASSERT_EQ(*begin, outline.col(ii));
  }
  /// [expand_outline_generator]
}

TEST(expand_multiple, area_generator) {
  /// [expand_multiple_expand_multiple]
  std::vector<cover::ray_generator> gens;
  size_t count = 0;
  for (int ii = 0; ii != 10; ++ii) {
    gens.emplace_back(cover::cell{ii, ii + 1}, cover::cell{ii + 2, ii + 3});
    count += gens.back().size();
  }

  cover::discrete_polygon outline = cover::expand_multiple(gens);
  ASSERT_EQ(outline.cols(), count);
  /// [expand_multiple_expand_multiple]
}

TEST(to_line, generic) {
  /// [to_line]
  // Create a ray_generator and expand it.
  const cover::discrete_polygon line =
      cover::to_line(cover::cell{0, 0}, cover::cell{10, 0});
  ASSERT_EQ(line.cols(), 10);
  /// [to_line]
}

TEST_F(triangle_fixture, to_outline) {
  /// [to_outline]
  const double resolution = 0.1;

  // The to_outline interface can be used ways.
  // The normal constructor
  const cover::discrete_polygon outline1 =
      cover::to_outline(resolution, triangle);

  // The explicit constructor
  cover::discrete_polygon outline2 =
      cover::to_outline(cover::to_discrete(resolution, triangle));

  // The move constructor.
  const cover::discrete_polygon outline3 =
      cover::to_outline(cover::outline_generator{resolution, triangle});

  // The copy constructor
  const cover::outline_generator gen{resolution, triangle};
  const cover::discrete_polygon outline4 = to_outline(gen);

  ASSERT_EQ(outline1, outline2);
  ASSERT_EQ(outline1, outline3);
  ASSERT_EQ(outline1, outline4);
  /// [to_outline]
}

TEST_F(triangle_fixture, to_area) {
  /// [to_area]
  // Create a triangle
  const double resolution = 0.1;

  // The to_area interface can be used ways.
  // The normal constructor
  const cover::discrete_polygon area1 = cover::to_area(resolution, triangle);

  // The explicit constructor. Here we need to pass the outline to the
  // constructor.
  cover::discrete_polygon area2 =
      cover::to_area(cover::to_outline(resolution, triangle));

  // The move constructor.
  const cover::discrete_polygon area3 =
      cover::to_area(cover::area_generator{resolution, triangle});

  // The copy constructor
  const cover::area_generator gen{resolution, triangle};
  const cover::discrete_polygon area4 = to_area(gen);

  ASSERT_EQ(area1, area2);
  ASSERT_EQ(area1, area3);
  ASSERT_EQ(area1, area4);
  /// [to_area]
}

struct fill_area_fixture
    : public testing::TestWithParam<std::tuple<std::vector<double>, double>> {
  const polygon p;
  const double resolution;

  fill_area_fixture() :
      p(make_footprint(std::get<0>(GetParam()))),
      resolution(std::get<1>(GetParam())) {}
};

// Relatively simple footprints where a single scan line itersects cells in
// the same order as the outline transversal
static std::vector<double> simple_footprints[] = {
    {1, -1, -1, 0, 1, -1},
    {1, 1, -1, -1, -1, 1, 1, -1},
    {1, 1, 0, -1, -1, -1, 1, 0.5, 1, -1},
    footprints[4],
    footprints[5]};

INSTANTIATE_TEST_CASE_P(
    /**/, fill_area_fixture,
    testing::Combine(testing::ValuesIn(simple_footprints),
                     testing::Values(0.1, 0.05, 0.025, 0.01)));

/**
 * @brief We check whether the to_area function of cover library returns the
 * same cells as the ros counterpart
 */
TEST_P(fill_area_fixture, compare_ros) {
  // Compute the footprint
  discrete_polygon outline = to_outline(resolution, p);
  outline.colwise() -= outline.rowwise().minCoeff();

  // Compute the to_area cells using cover library.
  const discrete_polygon area_cells = to_area(outline);

  // Compute the to_area with the ros implementation.
  std::vector<costmap_2d::MapLocation> sparse(outline.cols()), expected;
  for (int cc = 0, cols = outline.cols(); cc != cols; ++cc) {
    sparse.at(cc).x = outline(0, cc);
    sparse.at(cc).y = outline(1, cc);
  }

  costmap_2d::Costmap2D map(500, 500, 1, 0, 0);
  map.convexFillCells(sparse, expected);

  std::unordered_set<size_t> index_cover, index_ros;
  for (int cc = 0, cols = area_cells.cols(); cc != cols; ++cc) {
    index_cover.insert(map.getIndex(area_cells(0, cc), area_cells(1, cc)));
  }

  for (const auto& cc : expected) {
    index_ros.insert(map.getIndex(cc.x, cc.y));
  }

  ASSERT_EQ(index_cover, index_ros);
}

/**
 * @brief Base local planner does not support non-convex shapes so we cannot use
 * it as a reference to check the validity of our code. Therefore, we use OpenCV
 * instead
 */
using fill_non_convex_area_fixture = fill_area_fixture;

INSTANTIATE_TEST_CASE_P(
    /**/, fill_non_convex_area_fixture,
    testing::Combine(testing::ValuesIn(footprints),
                     testing::Values(0.2, 0.1, 0.05, 0.025)));

TEST_P(fill_non_convex_area_fixture, compare_opencv) {
  // Compute the footprint
  discrete_polygon outline = to_outline(resolution, p);
  outline.colwise() -= outline.rowwise().minCoeff();

  // Compute the fill cells using cover library
  const discrete_polygon area_cells = to_area(outline);

  // Compute the fill cells using OpenCV
  using cv_points_vec = std::vector<cv::Point>;
  cv_points_vec dense(outline.cols()), expected;
  for (int idx = 0, cols = outline.cols(); idx != cols; ++idx) {
    dense.at(idx).x = outline(0, idx);
    dense.at(idx).y = outline(1, idx);
  }

  cv::Mat img(500, 500, CV_8U, cv::Scalar(0));
  cv::fillPoly(img, std::vector<cv_points_vec>{dense}, cv::Scalar(1));
  cv::findNonZero(img, expected);

  auto get_index = [&](int _x, int _y) { return _x + _y * img.cols; };

  // Compare the results
  std::unordered_set<size_t> index_cover, index_cv;
  for (int cc = 0, cols = area_cells.cols(); cc != cols; ++cc) {
    index_cover.insert(get_index(area_cells(0, cc), area_cells(1, cc)));
  }

  for (const auto& cc : expected) {
    index_cv.insert(get_index(cc.x, cc.y));
  }

  ASSERT_EQ(index_cover, index_cv);
}