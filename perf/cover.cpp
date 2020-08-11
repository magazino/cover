#include <benchmark/benchmark.h>

#include <cover.hpp>
#include <base_local_planner/footprint_helper.h>

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
