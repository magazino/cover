// --- Internal Includes ---
#include <cover.hpp>
#include "cover_test_utils.hpp"

// --- ROS Includes ---
#include <base_local_planner/footprint_helper.h>

// --- Test Includes ---
#include <gtest/gtest.h>

// --- OpenCV Includes ---
#include <opencv2/opencv.hpp>

// --- Standard Includes ---
#include <algorithm>
#include <vector>

using namespace cover;

struct empty_polygon_fixture : public testing::Test {
  const polygon p;
};

TEST_F(empty_polygon_fixture, empty_no_throw) {
  // we don't throw if the user passed an empty polygon
  ASSERT_NO_THROW(split(p, 1));
}

TEST_F(empty_polygon_fixture, empty_result) {
  // we return an empty footprint
  const auto res = split(p, 1);
  EXPECT_TRUE(res.dense.empty());
  EXPECT_EQ(res.ring.cols(), 0);
}

struct invalid_polygon_fixture : public testing::TestWithParam<int> {
  int cols;
  invalid_polygon_fixture() : cols(GetParam()) {}
};

INSTANTIATE_TEST_CASE_P(/**/, invalid_polygon_fixture, testing::Values(1, 2));

TEST_P(invalid_polygon_fixture, invalid_polygon) {
  // we require at least 3 distinct points (1 and 2 are not enough)
  const polygon p = polygon::Random(2, cols);
  EXPECT_ANY_THROW(split(p, 1));
}

TEST_F(empty_polygon_fixture, invalid_radius) {
  // negative radii dont make sense
  EXPECT_ANY_THROW(split(p, -1));
}

TEST_F(empty_polygon_fixture, dense_outline) {
  // Empty outline should not throw
  discrete_polygon result;
  const auto result_gatherer = [&]() { result = dense_outline(p, 0.05); };

  EXPECT_NO_THROW(result_gatherer());
  EXPECT_EQ(result.size(), 0);
}

TEST(split, tree_points) {
  // just show that we accept polygons with at least 3 distinct points...
  const polygon random = polygon::Random(2, 3);
  polygon zeros = polygon::Zero(2, 3);

  // points are not distinct
  EXPECT_ANY_THROW(split(zeros, 1));

  // this is better...
  EXPECT_NO_THROW(split(random, 1));
}

// below regression tests

TEST(split, triangle) {
  // define a triangle
  const auto r = 1.;
  const auto a = r / std::tan(30. / 180 * M_PI);
  const auto h = r / std::sin(30. / 180 * M_PI);
  polygon triangle(2, 3);
  triangle << 0, a * 2, a, 0, 0, h + r;

  // run the splits: note the small delta for numerical stability
  const auto res = split(triangle, r - 1e-3);

  // the ring shall not be empty
  EXPECT_NE(res.ring.cols(), 0);

  // but all points should be very close to the middle
  const auto mean = res.ring.rowwise().mean();
  EXPECT_NEAR(mean.x(), a, 1e-2);
  EXPECT_NEAR(mean.y(), r, 1e-2);

  // 3 since we have a triangle...
  EXPECT_EQ(res.dense.size(), 3);
}

TEST(split, rectangle) {
  // define the rectangle
  const auto r = 1.;
  const auto l = r * 2;
  polygon rectangle(2, 4);
  rectangle << 0, l, l, 0, 0, 0, l, l;

  const auto res = split(rectangle, r - 1e-3);

  // the ring shall not be empty
  EXPECT_NE(res.ring.cols(), 0);

  // but all points should be very close to the middle
  const auto mean = res.ring.rowwise().mean();
  EXPECT_NEAR(mean.x(), r, 1e-2);
  EXPECT_NEAR(mean.y(), r, 1e-2);

  // 4 since we have a rectangle...
  EXPECT_EQ(res.dense.size(), 4);
}

TEST(split, circle) {
  // define a unit circle
  polygon circle(2, 16);
  for (int cc = 0; cc != circle.cols(); ++cc) {
    const auto angle = M_PI * 2 * cc / circle.cols();
    circle.col(cc) << std::cos(angle), std::sin(angle);
  }

  const auto res = split(circle, 1 - 1e-3);

  // but all points should be very close to the middle
  const auto mean = res.ring.rowwise().mean();
  EXPECT_NEAR(mean.x(), 0, 1e-2);
  EXPECT_NEAR(mean.y(), 0, 1e-2);

  // the circle should be covered by our approximation
  EXPECT_TRUE(res.dense.empty());
}

TEST(split, real_life_toru) {
  // take a real-life toru footprint
  polygon footprint(2, 32);
  int cc = 0;
  footprint.col(cc++) << 0.84608, 0.00000;
  footprint.col(cc++) << 0.84218, 0.00597;
  footprint.col(cc++) << 0.84082, 0.03039;
  footprint.col(cc++) << 0.83842, 0.04994;
  footprint.col(cc++) << 0.81381, 0.13629;
  footprint.col(cc++) << 0.76799, 0.21197;
  footprint.col(cc++) << 0.72050, 0.26080;
  footprint.col(cc++) << 0.66835, 0.29757;
  footprint.col(cc++) << 0.59975, 0.32630;
  footprint.col(cc++) << 0.51924, 0.34051;
  footprint.col(cc++) << -0.20630, 0.34051;
  footprint.col(cc++) << -0.29326, 0.32391;
  footprint.col(cc++) << -0.38673, 0.27609;
  footprint.col(cc++) << -0.45629, 0.19506;
  footprint.col(cc++) << -0.51063, 0.10001;
  footprint.col(cc++) << -0.52585, 0.03203;
  footprint.col(cc++) << -0.52756, 0.00000;
  footprint.col(cc++) << -0.52585, -0.03203;
  footprint.col(cc++) << -0.51063, -0.10001;
  footprint.col(cc++) << -0.45629, -0.19506;
  footprint.col(cc++) << -0.38673, -0.27609;
  footprint.col(cc++) << -0.29326, -0.32391;
  footprint.col(cc++) << -0.20630, -0.34051;
  footprint.col(cc++) << 0.51924, -0.34051;
  footprint.col(cc++) << 0.59975, -0.32630;
  footprint.col(cc++) << 0.66835, -0.29757;
  footprint.col(cc++) << 0.72050, -0.26080;
  footprint.col(cc++) << 0.76799, -0.21197;
  footprint.col(cc++) << 0.81381, -0.13629;
  footprint.col(cc++) << 0.83842, -0.04994;
  footprint.col(cc++) << 0.84082, -0.03039;
  footprint.col(cc++) << 0.84218, -0.00597;

  const auto res = split(footprint, 0.34);

  // the inner part must be valid
  EXPECT_NE(res.ring.size(), 0);
}

// below raytrace tests
// parameter
struct raytrace_param {
  int x0, y0, x1, y1;
};

// fixture
struct raytrace_fixture : public testing::TestWithParam<raytrace_param> {};

INSTANTIATE_TEST_CASE_P(/**/, raytrace_fixture,
                        testing::Values(raytrace_param{0, 0, 100, 30},
                                        raytrace_param{0, 0, 100, 100},
                                        raytrace_param{30, 100, 0, 0},
                                        raytrace_param{0, 100, 100, 30},
                                        raytrace_param{10, -11, -10, 30},
                                        raytrace_param{0, 0, -10, -30},
                                        raytrace_param{-10, -30, 100, 30},
                                        raytrace_param{10, -30, -100, 30},
                                        raytrace_param{0, 2, 2, 2}));

TEST_P(raytrace_fixture, generic) {
  const auto p = GetParam();
  base_local_planner::FootprintHelper fh;
  using blp_vector = std::vector<base_local_planner::Position2DInt>;

  // run the code
  const auto res = raytrace(cell{p.x0, p.y0}, cell{p.x1, p.y1});

  // we should not be empty
  ASSERT_NE(res.cols(), 0);

  // we use the base_local_planner as reference - run them also
  blp_vector expected;
  fh.getLineCells(p.x0, p.x1, p.y0, p.y1, expected);

  ASSERT_EQ(res.cols(), expected.size() - 1);

  // now compare the result
  for (int ii = 0; ii != res.cols(); ++ii) {
    ASSERT_EQ(res.col(ii).x(), expected[ii].x);
    ASSERT_EQ(res.col(ii).y(), expected[ii].y);
  }
}

using d_vector = std::vector<double>;

struct fill_area_fixture
    : public testing::TestWithParam<std::tuple<d_vector, double>> {
  using pos_t = base_local_planner::Position2DInt;
  using pos_vec_t = std::vector<pos_t>;

  base_local_planner::FootprintHelper fh;
  polygon p;
  double resolution;

  fill_area_fixture() :
      p(make_footprint(std::get<0>(GetParam()))),
      resolution(std::get<1>(GetParam())) {}

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
   * @brief Coverts discrete polygon to vector of cells
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

  /**
   * @brief Computes the number of mismatches between 2 sorted vectors of
   * points. The vectors size must be equal for valid results
   *
   * @param _vec0 The first vector
   * @param _vec1 The second vector
   * @return size_t Number of mismathces between the first and second vector
   */
  static size_t
  compute_mismatches(const pos_vec_t& _vec0, const pos_vec_t& _vec1) {
    // We can do this comparison as we sorted the points while removing
    // duplicates
    size_t n_mismatches = 0;
    for (size_t idx = 0; idx < _vec0.size(); ++idx) {
      // Add to counter if points are not equal
      n_mismatches += !are_points_equal(_vec0[idx], _vec1[idx]);
    }

    return n_mismatches;
  }
};

// Relatively simple footprints where a single scan line itersects cells in
// the same order as the outline transversal
const auto simple_footprints = testing::Values(
    d_vector{1, -1, -1, 0, 1, -1}, d_vector{1, 1, -1, -1, -1, 1, 1, -1},
    d_vector{1, 1, 0, -1, -1, -1, 1, 0.5, 1, -1});
const auto resolutions = testing::Values(0.5, 0.25, 0.1, 0.05, 0.025, 0.01);

INSTANTIATE_TEST_CASE_P(
    /**/, fill_area_fixture, testing::Combine(simple_footprints, resolutions));

/**
 * @brief We check whether the area function of cover library returns the same
 * cells as the base_local_planner counterpart
 */
TEST_P(fill_area_fixture, area_cells) {
  // Compute the discretized footprint
  auto disc_fp = dense_outline(p, resolution);
  disc_fp.colwise() -= disc_fp.rowwise().minCoeff();

  // Compute the fill cells using cover library
  auto cover_cells = to_pos_vec(area(disc_fp));
  uniquify(cover_cells);

  // Similarly, compute the cells using base local planner
  auto blp_cells = compute_blp_cells(disc_fp);
  uniquify(blp_cells);

  // Finally check if the results are the same
  EXPECT_EQ(cover_cells.size(), blp_cells.size());
  EXPECT_EQ(compute_mismatches(cover_cells, blp_cells), 0);
}

/**
 * @brief Base local planner does not support non-convex shapes so we cannot use
 * it as a reference to check the validity of our code. Therefore, we use OpenCV
 * instead
 */
struct fill_non_convex_area_fixture : public fill_area_fixture {
  using cv_vec_t = std::vector<cv::Point>;

  using fill_area_fixture::to_pos_vec;

  fill_non_convex_area_fixture() : fill_area_fixture() {}

  /**
   * @brief Creates an empty cv image based on the polygon dimensions
   *
   * @param _polygon The underlying footprint
   * @return cv::Mat Empty image that can strictly contain the footprint
   */
  static cv::Mat
  create_empty_cv_image(const discrete_polygon& _polygon) {
    // Compute the size of the OpenCV image
    const auto min_idx = _polygon.rowwise().minCoeff();
    const auto max_idx = _polygon.rowwise().maxCoeff();
    const auto cols = max_idx.x() - min_idx.x() + 1;
    const auto rows = max_idx.y() - min_idx.y() + 1;

    return {rows, cols, CV_8U, cv::Scalar(0)};
  }

  /**
   * @brief Convert discrete polygon to OpenCV point vector
   *
   * @param _polygon The underlying footprint
   * @return cv_vec_t OpenCV compatible points
   */
  static cv_vec_t
  to_cv_vec(const discrete_polygon& _polygon) {
    const auto n_cells = static_cast<size_t>(_polygon.cols());

    cv_vec_t cv_cells(n_cells);
    for (size_t idx = 0; idx < n_cells; ++idx) {
      cv_cells[idx].x = _polygon(0, idx);
      cv_cells[idx].y = _polygon(1, idx);
    }

    return cv_cells;
  }

  /**
   * @brief Uses OpenCV to fill the given polygon
   *
   * @param _polygon The underlying footprint
   * @return cv::Mat A binary mask representing filled cells as 1 while empty
   * cells as 0
   */
  static cv::Mat
  compute_cv_fill_image(const discrete_polygon& _polygon) {
    auto img = create_empty_cv_image(_polygon);

    // Convert discrete points to OpenCV compatible types
    const auto cells = to_cv_vec(_polygon);

    // Fill the cells using OpenCV
    cv::fillPoly(img, std::vector<cv_vec_t>{cells}, cv::Scalar(1));

    return img;
  }

  /**
   * @brief Extracts the filled cells from the binary mask (_img)
   *
   * @param _img Binary mask representing filled polygon. Any non-zero value is
   * considered filled
   * @return pos_vec_t Extracted filled cells from the binary image
   */
  static pos_vec_t
  to_pos_vec(const cv::Mat& _img) {
    cv_vec_t nz_cells;
    cv::findNonZero(_img, nz_cells);

    // Convert to common type
    pos_vec_t cells(nz_cells.size());
    std::transform(nz_cells.begin(), nz_cells.end(), cells.begin(),
                   [](const cv::Point& _pt) {
                     pos_t cell;
                     cell.x = _pt.x;
                     cell.y = _pt.y;

                     return cell;
                   });

    return cells;
  }
};

const auto non_convex_footprints = testing::Values(
    d_vector{-2.5, -1, 1, 2.5, 1, -1, 1, -0.5, 1.5, -1, 0.5, -1.5},  // S-Shape
    d_vector{-2, -2, -1, -1, 1, 1, 2, 2, 2, -2, -2, -1, -1, -2, -2,
             2},  // Closed W-Shape with flat interior cusp
    d_vector{2,  2, 0, -2, -2, -1.5, -1.5, 0,    1.5, 1.5,
             -2, 2, 0, 2,  -2, -2,   1,    -0.5, 1,   -2},  // M-Shape
    d_vector{0,   0.05, 0,    -0.2, 0,    0.5, 0,    -1.3, 0,    3.4,
             0,   6.8,  0,    -2.6, 0,    1,   0,    -0.4, 0,    0.15,
             0,   0.05, 0.1,  0,    -0.3, 0,   0.8,  0,    -2.1, 0,
             5.5, 0,    -4.2, 0,    1.6,  0,   -0.6, 0,    0.2,  0.05}
    // Fibonacci spiral
);

INSTANTIATE_TEST_CASE_P(
    /**/, fill_non_convex_area_fixture,
    testing::Combine(non_convex_footprints, resolutions));

TEST_P(fill_non_convex_area_fixture, non_convex_area_cells) {
  // Compute the discretized footprint
  auto disc_fp = dense_outline(p, resolution);
  disc_fp.colwise() -= disc_fp.rowwise().minCoeff();

  // Compute the fill cells using cover library
  auto cover_cells = to_pos_vec(area(disc_fp));
  uniquify(cover_cells);

  // Compute the fill cells using OpenCV
  auto cv_cells = to_pos_vec(compute_cv_fill_image(disc_fp));
  uniquify(cv_cells);

  // Finally check if the results are the same
  EXPECT_EQ(cover_cells.size(), cv_cells.size());
  EXPECT_EQ(compute_mismatches(cover_cells, cv_cells), 0);
}
