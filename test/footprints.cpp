// --- Internal Includes ---
#include <cover/footprints.hpp>
#include "cover_test_utils.hpp"

// --- Ros-includes ---
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

// --- Test Includes ---
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace cover;

TEST(get_radius, empty) { ASSERT_EQ(get_radius({}), 0); }

TEST(get_radius, box) {
  /// [get_radius]
  // Create a box-polygon.
  polygon p(2, 4);
  p.row(0) << -1, -1, 1, 1;  // The x-values.
  p.row(1) << -2, 2, 2, -2;  // The y-values.

  ASSERT_FLOAT_EQ(get_radius(p), 1.);

  // The operation is invaliant to transformations.
  for (int ii = 0; ii != 100; ++ii) {
    const double rad = ii / 50. * M_PI;
    const Eigen::Isometry2d transform =
        Eigen::Translation2d(ii, -ii) * Eigen::Rotation2Dd(rad);

    const polygon transformed = transform * p;
    ASSERT_FLOAT_EQ(get_radius(transformed), 1.);
  }
  /// [get_radius]
}

TEST(continuous_footprint, empty) {
  // we don't throw if the user passed an empty polygon
  const polygon p;
  ASSERT_NO_THROW(continuous_footprint(1, p));

  const continuous_footprint res(1, p);
  ASSERT_TRUE(res.get_areas().empty());
  ASSERT_TRUE(res.get_outlines().empty());
}

struct invalid_polygon_fixture : public testing::TestWithParam<int> {
  int cols;
  invalid_polygon_fixture() : cols(GetParam()) {}
};

INSTANTIATE_TEST_CASE_P(/**/, invalid_polygon_fixture, testing::Values(1, 2));

TEST_P(invalid_polygon_fixture, invalid_polygon) {
  // we require at least 3 distinct points (1 and 2 are not enough)
  const polygon p = polygon::Random(2, cols);

  const continuous_footprint res(1, p);
  ASSERT_TRUE(res.get_areas().empty());
  ASSERT_TRUE(res.get_outlines().empty());
}

TEST(continuous_footprint, all_points_same) {
  // As above: we don't have three distinct points.
  const polygon p = polygon::Zero(2, 10);

  const continuous_footprint res(1, p);
  ASSERT_TRUE(res.get_areas().empty());
  ASSERT_TRUE(res.get_outlines().empty());
}

TEST(continuous_footprint, invalid_radius) {
  // negative radii dont make sense
  const polygon p(2, 2);
  EXPECT_ANY_THROW(continuous_footprint(-1, p));
}

TEST(continuous_footprint, too_large_radius) {
  // If the radius is too large, we will just return the original polygon as an
  // area. The outline should be empty.

  // Note: boost will return a closed polygon. In order to compare the output
  // and the input, we close the input.
  polygon p(2, 4);
  p.row(0) << 0, 0.5, -0.5, 0;
  p.row(1) << 1, 0, 0, 1;

  const continuous_footprint res(2, p);
  ASSERT_TRUE(res.get_outlines().empty());
  ASSERT_EQ(res.get_areas().size(), 1);

  ASSERT_FLOAT_EQ((p - res.get_areas().front()).array().abs().sum(), 0.);
}

TEST(continuous_footprint, tree_points) {
  // just show that we accept polygons with at least 3 distinct points...
  const polygon p = polygon::Random(2, 3);

  const continuous_footprint res(p);
  ASSERT_FALSE(res.get_areas().empty());
  ASSERT_FALSE(res.get_outlines().empty());
}

// below regression tests

TEST(continuous_footprint, triangle) {
  /// [continuous_footprint_triangle]
  // Define a triangle
  const auto r = 1.;
  const auto a = r / std::tan(30. / 180 * M_PI);
  const auto h = r / std::sin(30. / 180 * M_PI);
  cover::polygon triangle(2, 3);
  triangle << 0, a * 2, a, 0, 0, h + r;

  // Create the continuous footprint
  const continuous_footprint res(triangle);
  ASSERT_FALSE(res.get_areas().empty());
  ASSERT_FALSE(res.get_outlines().empty());

  // The center of the outline is our mean.
  const cover::point mean = res.get_outlines().front().rowwise().mean();
  EXPECT_NEAR(mean.x(), a, 1e-2);
  EXPECT_NEAR(mean.y(), r, 1e-2);
  /// [continuous_footprint_triangle]
}

TEST(continuous_footprint, rectangle) {
  // define the rectangle
  const auto r = 1.;
  const auto l = r * 2;
  polygon rectangle(2, 4);
  rectangle << 0, l, l, 0, 0, 0, l, l;

  const continuous_footprint res(rectangle);
  ASSERT_FALSE(res.get_areas().empty());
  ASSERT_FALSE(res.get_outlines().empty());

  // but all points should be very close to the middle
  const point mean = res.get_outlines().front().rowwise().mean();
  EXPECT_NEAR(mean.x(), r, 1e-2);
  EXPECT_NEAR(mean.y(), r, 1e-2);
}

TEST(continuous_footprint, circle) {
  // define a unit circle
  polygon circle(2, 16);
  for (int cc = 0; cc != circle.cols(); ++cc) {
    const double angle = M_PI * 2 * cc / circle.cols();
    circle.col(cc) << std::cos(angle), std::sin(angle);
  }

  const continuous_footprint res(circle);
  ASSERT_TRUE(res.get_areas().empty());
  ASSERT_FALSE(res.get_outlines().empty());

  // but all points should be very close to the middle
  const point mean = res.get_outlines().front().rowwise().mean();
  EXPECT_NEAR(mean.x(), 0, 1e-2);
  EXPECT_NEAR(mean.y(), 0, 1e-2);
}

/// @brief Fixture which will initialize the inflation layer.
struct inflation_layer : public costmap_2d::InflationLayer {
  void
  onInitialize() override {
    current_ = true;
    enabled_ = true;
    // This must be set, otherwise the InflationLayer will not do anything.
    inflation_radius_ = 1;

    matchSize();
  }
};

/// @brief The fixture for running our regression test.
struct continuous_footprint_fixture
    : public testing::TestWithParam<std::vector<double>> {
  costmap_2d::LayeredCostmap master;               ///< Master costmap.
  boost::shared_ptr<costmap_2d::Layer> inflation;  ///< Inflation layer.
  polygon footprint;                               ///< Robot footprint.

  continuous_footprint_fixture() : master("map", false, false) {
    footprint = make_footprint(GetParam());
    inflation.reset(new inflation_layer{});

    // Add the inflation first to the master and then initialize the inflation.
    master.addPlugin(inflation);
    inflation->initialize(&master, "foo", nullptr);

    // Both methods will call the registered
    master.resizeMap(200, 200, 0.05, -5, -5);
    master.setFootprint(to_msgs(footprint));
  }
};

INSTANTIATE_TEST_CASE_P(/**/, continuous_footprint_fixture,
                        testing::Values(footprints[4], footprints[5]));

TEST_P(continuous_footprint_fixture, real_footprints) {
  // This test uses the inflation layer to simulate the real use use-case...

  const continuous_footprint res(footprint);
  ASSERT_FALSE(res.get_outlines().empty());
  ASSERT_FALSE(res.get_areas().empty());

  auto& map = *master.getCostmap();

  // Create an area_generator so we can mark cells inside the fooprint.
  const point origin(map.getOriginX(), map.getOriginY());
  const area_generator gen(map.getResolution(), footprint.colwise() - origin);

  // Iterate over all cells
  size_t failures = 0;
  for (const auto& c : gen) {
    ASSERT_TRUE(res.is_free(Eigen::Isometry2d::Identity(), map));

    // Mark cell as occupied.
    ASSERT_TRUE((c.array() >= 0).all());
    ASSERT_TRUE((c.array() < 200).all());
    map.setCost(c.x(), c.y(), costmap_2d::LETHAL_OBSTACLE);

    // Update the entire map
    inflation->updateCosts(map, 0, 0, 200, 200);

    // Note this should be assert-false but due to discretization issues we have
    // always one cell left out.
    if (res.is_free(Eigen::Isometry2d::Identity(), map)) {
      ++failures;
    }

    // Reset the map
    std::fill_n(map.getCharMap(), 200 * 200, 0);
  }

  ASSERT_LE(failures, 2);
}

struct is_free_fixture : public testing::Test {
  costmap_2d::Costmap2D map;
  cover::polygon box;
  is_free_fixture() : map(100, 200, 0.1, -5, -10), box(2, 4) {
    // Create a box
    box.row(0) << 0, 1, 1, 0;
    box.row(1) << 0, 0, 2, 2;

    // Center the box around zero
    box.colwise() -= cover::point(0.5, 1);
  }
};

TEST_F(is_free_fixture, continuous_footprint) {
  /// [continuous_footprint]
  // We have a box of the dimension (1x2) and a costmap of the size (10x20).
  // Assume that the box represents the outline
  cover::continuous_footprint fp(cover::polygon_vec{box}, cover::polygon_vec{});

  // Without any obstacle the box is free
  ASSERT_TRUE(fp.is_free(Eigen::Isometry2d::Identity(), map));

  // We now mark one cell inside the outline as lethal. This has no effect since
  // we just check the outline.
  map.setCost(50, 100, costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(fp.is_free(Eigen::Isometry2d::Identity(), map));

  // Lets place an obstacle on the outline
  unsigned int x, y;
  map.worldToMap(box(0, 0), box(1, 0), x, y);
  map.setCost(x, y, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_FALSE(fp.is_free(Eigen::Isometry2d::Identity(), map));

  // Now we use the area
  fp = continuous_footprint(cover::polygon_vec{}, cover::polygon_vec{box});

  // Since we have already a lethal obstacle we expect that the area is not
  // free.
  ASSERT_FALSE(fp.is_free(Eigen::Isometry2d::Identity(), map));

  // If we remove the obstacle it will be free again.
  map.setCost(50, 100, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_TRUE(fp.is_free(Eigen::Isometry2d::Identity(), map));
  /// [continuous_footprint]
}

TEST_F(is_free_fixture, continuous_footprint_transformation) {
  // Create the footprint.
  const cover::continuous_footprint fp(0.1, box);

  // The rotations will not move the footprint out of map bounds.
  for (int ii = 0; ii != 10; ++ii) {
    const double angle = ii / 5 * M_PI;
    ASSERT_TRUE(fp.is_free(Eigen::Isometry2d{Eigen::Rotation2Dd(angle)}, map));
  }

  // But if we translate it outside of map bounds, we will fail.
  const double x_diff = map.getSizeInMetersX() / 2;
  const double y_diff = map.getSizeInMetersY() / 2;
  ASSERT_FALSE(
      fp.is_free(Eigen::Isometry2d{Eigen::Translation2d{x_diff, y_diff}}, map));
}

TEST_F(is_free_fixture, generator_footprint_constructor) {
  /// [generator_footprint_constructor]
  // Below some examples how to construct the generator_footprint.
  // The construction from a existing continuous_footprint
  constexpr double resolution = 0.05;
  constexpr double radius = 0.1;

  continuous_footprint helper{box};
  generator_footprint fp{resolution, helper};

  // With inplace construction of the continuous_footprint
  fp = generator_footprint{resolution, continuous_footprint{box}};

  // With the arguments used for the continuous_footprint:
  fp = generator_footprint{resolution, box};
  fp = generator_footprint{resolution, radius, box};

  /// [generator_footprint_constructor]
}

TEST_F(is_free_fixture, discrete_footprint_constructor) {
  /// [discrete_footprint_constructor]
  // Below some examples how to construct the generator_footprint.
  // The construction from a existing continuous_footprint
  constexpr double resolution = 0.05;
  constexpr double radius = 0.1;

  continuous_footprint c_fp{box};
  generator_footprint g_fp{resolution, c_fp};

  // Pass an existing generator_footprint
  discrete_footprint d_fp{g_fp};

  // Pass a newly created generator_footprint
  d_fp = discrete_footprint{resolution, continuous_footprint{box}};

  // Construct with a resolution and a box
  d_fp = discrete_footprint{resolution, box};

  // Construct with a resolution, a box and a radius
  d_fp = discrete_footprint{resolution, radius, box};
  /// [discrete_footprint_constructor]
}

TEST_P(continuous_footprint_fixture, discrete_footprints) {
  // This test uses the inflation layer to simulate the real use use-case...
  auto& map = *master.getCostmap();

  // Create the discrete footprint
  const discrete_footprint res(map.getResolution(), footprint);

  // Both types should not be empty.
  ASSERT_NE(res.get_outlines().size(), 0);
  ASSERT_NE(res.get_areas().size(), 0);

  // Create the area generator for marking our cells inside the footprint.
  const area_generator gen(map.getResolution(), footprint);
  const cell offset(100, 100);

  // Iterate over all cells
  size_t failures = 0;
  for (auto c : gen) {
    // Move the cell to the middle.
    c += offset;
    ASSERT_TRUE(res.is_free(offset, map));

    // Mark cell as occupied.
    ASSERT_TRUE((c.array() >= 0).all());
    ASSERT_TRUE((c.array() < 200).all());
    map.setCost(c.x(), c.y(), costmap_2d::LETHAL_OBSTACLE);

    // Inflate the obstacle.
    inflation->updateCosts(map, 0, 0, 200, 200);

    // Note this should be assert-false but due to discretization issues we have
    // always one cell left out.
    if (res.is_free(offset, map)) {
      ++failures;
    }

    // Reset the map
    std::fill_n(map.getCharMap(), 200 * 200, 0);
  }
  ASSERT_LE(failures, 2);
}
