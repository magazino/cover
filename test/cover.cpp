#include <cover.hpp>

#include <gtest/gtest.h>

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
