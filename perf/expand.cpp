#include <benchmark/benchmark.h>

#include <base_local_planner/footprint_helper.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_2d_msgs/Point2D.h>
#include <nav_2d_msgs/Polygon2D.h>
#include <nav_grid/nav_grid_info.h>
#include <nav_grid_iterators/polygon_fill.h>
#include <nav_grid_iterators/polygon_outline.h>

#include <cover.hpp>
#include "cover_test_utils.hpp"

#include <algorithm>
#include <vector>

// benchmark our expansions vs costmap's

using namespace cover;

static void
to_line_costmap(benchmark::State& state) {
  base_local_planner::FootprintHelper fh;

  for (auto _ : state) {
    std::vector<base_local_planner::Position2DInt> cells;
    fh.getLineCells(0, 100, 0, 30, cells);
  }
}

static void
to_line_cover(benchmark::State& state) {
  for (auto _ : state) {
    benchmark::DoNotOptimize(to_line(cell(0, 0), cell(100, 30)));
  }
}

// Register the function as a benchmark
BENCHMARK(to_line_cover);
BENCHMARK(to_line_costmap);

struct costmap_fixture : benchmark::Fixture {
  costmap_2d::Costmap2D map;
  polygon footprint;

  costmap_fixture() : map(200, 200, 0.05, -100 * 0.05, -100 * 0.05) {}

  void
  SetUp(const benchmark::State& _state) {
    // Load the footprint
    footprint = make_footprint(footprints.at(_state.range(0)));
  }
};

BENCHMARK_DEFINE_F(costmap_fixture, to_outline_cover)
(benchmark::State& _state) {
  // Get the specific footprint.
  const auto sparse = to_discrete(map.getResolution(), footprint);
  for (auto _ : _state) {
    outline_generator g(sparse);
    for (const auto& c : g)
      benchmark::DoNotOptimize(c.x());
  }
}

BENCHMARK_DEFINE_F(costmap_fixture, to_outline_costmap)
(benchmark::State& _state) {
  // Get the specific footprint.
  base_local_planner::FootprintHelper fh;
  const auto outline = to_msgs(footprint);
  for (auto _ : _state) {
    const auto cells = fh.getFootprintCells({0, 0, 0}, outline, map, false);
    for (const auto& c : cells)
      benchmark::DoNotOptimize(c.x);
  }
}

// Converts ros-msgs to nav_2d_msgs
static nav_2d_msgs::Polygon2D
to_nav_2d_polygon(const std::vector<geometry_msgs::Point>& _msg) {
  nav_2d_msgs::Polygon2D polygon;
  polygon.points.resize(_msg.size());

  std::transform(_msg.begin(), _msg.end(), polygon.points.begin(),
                 [](const geometry_msgs::Point& _point) {
                   nav_2d_msgs::Point2D point;
                   point.x = _point.x;
                   point.y = _point.y;
                   return point;
                 });
  return polygon;
}

BENCHMARK_DEFINE_F(costmap_fixture, to_outline_nav_grid_iterators)
(benchmark::State& _state) {
  nav_grid::NavGridInfo info;
  info.width = info.height = map.getSizeInCellsX();
  info.resolution = map.getResolution();
  info.origin_x = info.origin_y = map.getOriginX();

  // Generate the footprint
  const auto outline = to_msgs(footprint);
  const nav_2d_msgs::Polygon2D polygon = to_nav_2d_polygon(outline);

  for (auto _ : _state) {
    nav_grid_iterators::PolygonOutline iter(&info, polygon);

    for (const auto& c : iter)
      benchmark::DoNotOptimize(c.x);
  }
}

// Register the function as a benchmark
BENCHMARK_REGISTER_F(costmap_fixture, to_outline_cover)->DenseRange(0, 5);
BENCHMARK_REGISTER_F(costmap_fixture, to_outline_costmap)->DenseRange(0, 5);
BENCHMARK_REGISTER_F(costmap_fixture, to_outline_nav_grid_iterators)
    ->DenseRange(0, 5);

BENCHMARK_DEFINE_F(costmap_fixture, to_area_cover)(benchmark::State& _state) {
  // Get the specific footprint.
  const auto outline = to_outline(map.getResolution(), footprint);
  for (auto _ : _state) {
    area_generator g(outline);
    for (const auto& c : g)
      benchmark::DoNotOptimize(c.x());
  }
}

BENCHMARK_DEFINE_F(costmap_fixture, to_area_costmap)(benchmark::State& _state) {
  // Get the specific footprint.
  base_local_planner::FootprintHelper fh;
  const auto outline = to_msgs(footprint);

  for (auto _ : _state) {
    const auto cells = fh.getFootprintCells({0, 0, 0}, outline, map, true);
    for (const auto& c : cells)
      benchmark::DoNotOptimize(c.x);
  }
}

BENCHMARK_DEFINE_F(costmap_fixture, to_area_nav_grid_iterators)
(benchmark::State& _state) {
  nav_grid::NavGridInfo info;
  info.width = info.height = map.getSizeInCellsX();
  info.resolution = map.getResolution();
  info.origin_x = info.origin_y = map.getOriginX();

  // Generate the footprint
  const auto outline = to_msgs(footprint);
  const nav_2d_msgs::Polygon2D polygon = to_nav_2d_polygon(outline);

  for (auto _ : _state) {
    nav_grid_iterators::PolygonFill iter(&info, polygon);

    for (const auto& c : iter)
      benchmark::DoNotOptimize(c.x);
  }
}

// The range will be used as the index in the footprints vector above.
BENCHMARK_REGISTER_F(costmap_fixture, to_area_cover)->DenseRange(0, 5);
BENCHMARK_REGISTER_F(costmap_fixture, to_area_costmap)
    ->DenseRange(4, 5);  // concave footprints are not supported.
BENCHMARK_REGISTER_F(costmap_fixture, to_area_nav_grid_iterators)
    ->DenseRange(0, 5);

BENCHMARK_MAIN();
