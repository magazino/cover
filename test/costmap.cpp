#include <cover/costmap.hpp>
#include <gtest/gtest.h>

TEST(make_polygon, empty) {
  // Create an empty polygon
  const cover::polygon p = cover::make_polygon({});

  // The result is also empty.
  ASSERT_EQ(p.size(), 0);
}

TEST(make_polygon, generic) {
  /// [make_polygon]
  // Create an input.
  std::vector<geometry_msgs::Point> msg;
  msg.resize(10);
  for (size_t ii = 0; ii != msg.size(); ++ii) {
    msg.at(ii).x = ii;
    msg.at(ii).y = msg.size() + ii;
  }

  // Convert it to the polygon.
  const cover::polygon p = cover::make_polygon(msg);
  ASSERT_EQ(p.cols(), msg.size());  // A column for every point.

  // Compare the values.
  for (int ii = 0; ii != p.cols(); ++ii) {
    ASSERT_DOUBLE_EQ(p.col(ii).x(), msg.at(ii).x);
    ASSERT_DOUBLE_EQ(p.col(ii).y(), msg.at(ii).y);
  }

  /// [make_polygon]
}

TEST(is_inside, generic) {
  /// [is_inside]
  // Create a costmap.
  costmap_2d::Costmap2D map(10, 20, 1, 0, 0);

  // Create a checker.
  cover::detail::is_inside checker(map);

  ASSERT_TRUE(checker({0, 0}));     // The lower left corner
  ASSERT_TRUE(checker({9, 19}));    // The upper right corner
  ASSERT_FALSE(checker({10, 19}));  // x-value too large
  ASSERT_FALSE(checker({9, 20}));   // y-value too large
  ASSERT_FALSE(checker({0, -1}));   // x-value too small
  ASSERT_FALSE(checker({-1, 0}));   // y-value too small

  /// [is_inside]
}

struct costmap_fixture : testing::Test {
  costmap_2d::Costmap2D map;

  costmap_fixture() {
    // Create a costmap.
    map = costmap_2d::Costmap2D(10, 20, 1, 0, 0);
    // Set the costs.
    map.setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);
    map.setCost(5, 6, costmap_2d::NO_INFORMATION);
    map.setCost(5, 7, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }
};

TEST_F(costmap_fixture, is_lethal) {
  /// [is_lethal]
  // Create a checker.
  cover::detail::is_lethal checker(map);

  ASSERT_TRUE(checker({5, 5}));    // The LETHAL_OBSTACLE cell.
  ASSERT_TRUE(checker({10, 19}));  // out-of-bounds is lethal.
  ASSERT_FALSE(checker({0, 0}));   // FREE_SPACE is ok.
  ASSERT_FALSE(checker({5, 6}));   // NO_INFORMATION is ok.
  ASSERT_FALSE(checker({5, 7}));   // INSCRIBED_INFLATED_OBSTACLE is also ok.
  /// [is_lethal]
}

TEST_F(costmap_fixture, is_lethal_or_inflated) {
  /// [is_lethal_or_inflated]

  // Create a checker.
  cover::detail::is_lethal_or_inflated checker(map);

  ASSERT_TRUE(checker({5, 5}));    // The LETHAL_OBSTACLE cell.
  ASSERT_TRUE(checker({5, 7}));    // The INSCRIBED_INFLATED_OBSTACLE cell.
  ASSERT_TRUE(checker({10, 19}));  // out-of-bounds is lethal.
  ASSERT_FALSE(checker({0, 0}));   // FREE_SPACE is ok.
  ASSERT_FALSE(checker({5, 6}));   // NO_INFORMATION is ok.
  /// [is_lethal_or_inflated]
}

TEST_F(costmap_fixture, get_cell_cost) {
  /// [get_cell_cost]

  // Create a cost_getter.
  cover::detail::get_cell_cost cost_at(map);

  ASSERT_EQ(cost_at({5, 5}), costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(cost_at({5, 7}), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(cost_at({0, 0}), costmap_2d::FREE_SPACE);
  ASSERT_EQ(cost_at({5, 6}), costmap_2d::NO_INFORMATION);
  /// [get_cell_cost]
}

TEST_F(costmap_fixture, is_free_generator) {
  /// [is_free_generator]
  // The lethal cell is (5, 5)
  {
    // Create a ray not passing through this cell.
    // Note: this could also be an outline_generator or area_generator.
    cover::ray_generator ray({4, 0}, {4, 9});
    ASSERT_TRUE(cover::is_free(ray, map));
  }

  {
    // Create a ray through the lethal cell.
    // Note: this could also be an outline_generator or area_generator.
    cover::ray_generator ray({5, 0}, {5, 9});
    ASSERT_FALSE(cover::is_free(ray, map));
  }
  /// [is_free_generator]
}

TEST_F(costmap_fixture, is_free_with_offset) {
  /// [is_free_with_offset]
  // Create a checker outside of the map-bounds (10, 20)
  cover::ray_generator ray({-4, -4}, {-4, 4});

  // The ray is outside of the map.
  ASSERT_FALSE(cover::is_free(ray, map));

  // The ray is inside if we move it by its origin.
  ASSERT_TRUE(cover::is_free(ray, -*ray.begin(), map));
  /// [is_free_with_offset]
}

TEST_F(costmap_fixture, accumulate_cost_generator) {
  /// [accumulate_cost_generator]
  // Set the cells to constant value.
  const int n_cells = map.getSizeInCellsY() * map.getSizeInCellsX();
  constexpr unsigned char value = 10;
  for (int index = 0; index != n_cells; ++index) {
    map.getCharMap()[index] = value;
  }
  // Create an area_generator to cover the whole map.
  cover::discrete_polygon sparse(2, 4);
  sparse.col(0) << 0, 0;
  sparse.col(1) << map.getSizeInCellsX() - 1, 0;
  sparse.col(2) << map.getSizeInCellsX() - 1, map.getSizeInCellsY() - 1;
  sparse.col(3) << 0, map.getSizeInMetersY();

  cover::area_generator gen(cover::to_outline(sparse));

  // Now get the sum of all cells:
  const auto sum_of_costs = value * n_cells;
  ASSERT_EQ(cover::accumulate_cost(gen, map), sum_of_costs);
  /// [accumulate_cost_generator]
}

TEST_F(costmap_fixture, accumulate_cost_with_offset) {
  /// [accumulate_cost_with_offset]
  // Create a polygon with the map-bounds.
  // Create an area_generator to cover the whole map.
  cover::discrete_polygon sparse(2, 4);
  sparse.col(0) << 0, 0;
  sparse.col(1) << map.getSizeInCellsX() - 1, 0;
  sparse.col(2) << map.getSizeInCellsX() - 1, map.getSizeInCellsY() - 1;
  sparse.col(3) << 0, map.getSizeInMetersY();

  // Transpose it.
  const cover::cell offset(4, 4);
  sparse.colwise() -= offset;

  cover::area_generator gen(cover::to_outline(sparse));
  // Without the offset the cost will indicate out-of-bounds.
  ASSERT_EQ(cover::accumulate_cost(gen, map), -3);

  // With the offset we will stay inside the map.
  ASSERT_NE(cover::accumulate_cost(gen, offset, map), -3);
  /// [accumulate_cost_with_offset]
}

TEST_F(costmap_fixture, out_of_bounds) {
  /// [accumulate_cost_generator_out_of_bounds]
  // Create a ray generator where the first cell is outside of the map.
  cover::ray_generator ray({-1, -1}, {5, 5});
  cover::detail::is_inside check{map};
  ASSERT_FALSE(check(*ray.begin()));

  // Now call the accumulate_sum function
  ASSERT_EQ(cover::accumulate_cost(ray, map), -3);
  /// [accumulate_cost_generator_out_of_bounds]
}
