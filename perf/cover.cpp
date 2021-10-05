#include <benchmark/benchmark.h>

#include <cover.hpp>
#include <base_local_planner/footprint_helper.h>
#include <costmap_2d/costmap_2d.h>
#include "cover_test_utils.hpp"

#include <vector>

// benchmark our raytrace vs costmap's

using namespace cover;

static void
raytrace_costmap(benchmark::State& state) {
  base_local_planner::FootprintHelper fh;

  for (auto _ : state) {
    std::vector<base_local_planner::Position2DInt> cells;
    fh.getLineCells(0, 100, 0, 30, cells);
  }
}

static void
raytrace_cover(benchmark::State& state) {
  for (auto _ : state) {
    const auto cells = raytrace(cell(0, 0), cell(100, 30));
  }
}

// Register the function as a benchmark
BENCHMARK(raytrace_cover);
BENCHMARK(raytrace_costmap);

static std::vector<std::vector<double>> footprints = {
    // S-shape
    {-2.5, -1, 1, 2.5, 1, -1, 1, -0.5, 1.5, -1, 0.5, -1.5},
    // W-shape
    {-2, -2, -1, -1, 1, 1, 2, 2, 2, -2, -2, -1, -1, -2, -2, 2},
    // M-shape
    {2,  2, 0, -2, -2, -1.5, -1.5, 0,    1.5, 1.5,
     -2, 2, 0, 2,  -2, -2,   1,    -0.5, 1,   -2},
    // Spiral
    {0,   0.05, 0,    -0.2, 0,    0.5, 0,    -1.3, 0,    3.4,
     0,   6.8,  0,    -2.6, 0,    1,   0,    -0.4, 0,    0.15,
     0,   0.05, 0.1,  0,    -0.3, 0,   0.8,  0,    -2.1, 0,
     5.5, 0,    -4.2, 0,    1.6,  0,   -0.6, 0,    0.2,  0.05},
    // SOTO
    {
        0.41,  1.19,  1.19,  1.19,  1.19,  0.41,  -0.35, -1.13, -1.16,
        -1.22, -1.27, -1.33, -1.33, -1.33, -1.33, -1.33, -1.33, -1.27,
        -1.22, -1.16, -1.13, -0.35, -0.65, -0.65, -0.21, 0.21,  0.65,
        0.65,  0.65,  0.65,  0.58,  0.58,  0.58,  0.58,  0.35,  0.11,
        -0.11, -0.35, -0.58, -0.58, -0.58, -0.58, -0.65, -0.65,
    },
    // TORU
    {0.84,  0.84,  0.84,  0.83,  0.81,  0.76,  0.72,  0.66,  0.59,  0.51,
     -0.20, -0.29, -0.38, -0.45, -0.51, -0.52, -0.52, -0.52, -0.51, -0.45,
     -0.38, -0.29, -0.20, 0.51,  0.59,  0.66,  0.72,  0.76,  0.81,  0.83,
     0.84,  0.84,  0.00,  0.00,  0.03,  0.04,  0.13,  0.21,  0.26,  0.29,
     0.32,  0.34,  0.34,  0.32,  0.27,  0.19,  0.10,  0.03,  0.00,  -0.03,
     -0.10, -0.19, -0.27, -0.32, -0.34, -0.34, -0.32, -0.29, -0.26, -0.21,
     -0.13, -0.04, -0.03, -0.00}};

static void
generate_area_cover(benchmark::State& _state) {
  // Get the specific footprint.
  const auto& data = footprints.at(_state.range(0));
  const auto outline = dense_outline(make_footprint(data), 0.05);

  for (auto _ : _state) {
    area_generator g(outline);
    for (const auto& c : g)
      benchmark::DoNotOptimize(c.x());
  }
}

static void
generate_area_costmap(benchmark::State& _state) {
  // Get the specific footprint.
  const auto& data = footprints.at(_state.range(0));
  base_local_planner::FootprintHelper fh;
  costmap_2d::Costmap2D map(500, 500, 0.05, -12.5, -12.5);
  const auto outline = to_msgs(make_footprint(data));

  for (auto _ : _state) {
    const auto cells = fh.getFootprintCells({0, 0, 0}, outline, map, true);
    for (const auto& c : cells)
      benchmark::DoNotOptimize(c.x);
  }
}

// The range will be used as the index in the footprints vector above.
BENCHMARK(generate_area_cover)->DenseRange(0, 5);
BENCHMARK(generate_area_costmap)
    ->DenseRange(4, 5);  // concave footprints are not supported.
