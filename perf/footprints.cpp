#include <footprints.hpp>
#include "cover_test_utils.hpp"

#include <costmap_2d/costmap_2d.h>

#include <benchmark/benchmark.h>

using namespace cover;

static void
continuous_footprint_generator(benchmark::State& _state) {
  // Get the specific footprint.
  const auto& data = footprints.at(_state.range(0));

  const polygon footprint = make_footprint(data);
  const double radius = footprint.row(1).array().abs().maxCoeff() - 1e-3;

  for (auto _ : _state) {
    benchmark::DoNotOptimize(continuous_footprint(radius, footprint));
  }
}

// Benchmark just real footprints.
BENCHMARK(continuous_footprint_generator)->DenseRange(4, 5);

/// @brief Fixture for running is_free benchmarks
struct is_free_fixture : public benchmark::Fixture {
  const size_t size;
  costmap_2d::Costmap2D map;
  polygon footprint;

  is_free_fixture() : size(1000), map(size, size, 0.02, -10, -10) {}
  // Common code to create a footprint
  void
  SetUp(const benchmark::State& _state) override {
    const auto& data = footprints.at(_state.range(0));
    // Rotate the footprint by 30 deg so the lines are not aligned
    const Eigen::Matrix2d rot = Eigen::Rotation2Dd(30. / 180. * M_PI).matrix();
    footprint = rot * make_footprint(data);
  }
};

BENCHMARK_DEFINE_F(is_free_fixture, area_generator)(benchmark::State& _state) {
  const double res = map.getResolution();
  for (auto _ : _state) {
    // Create a area_generator
    area_generator gen(res, footprint);
    benchmark::DoNotOptimize(is_free(gen, cell{100, 100}, map));
  }
}

BENCHMARK_DEFINE_F(is_free_fixture, discrete_polygon)
(benchmark::State& _state) {
  // Create a discrete polygon
  const discrete_polygon dp = to_area(map.getResolution(), footprint);
  for (auto _ : _state) {
    benchmark::DoNotOptimize(is_free(dp, cell{100, 100}, map));
  }
}

BENCHMARK_DEFINE_F(is_free_fixture, continuous_footprint)
(benchmark::State& _state) {
  // Create a continuous_footprint
  continuous_footprint fp{footprint};
  for (auto _ : _state) {
    benchmark::DoNotOptimize(fp.is_free(Eigen::Isometry2d::Identity(), map));
  }
}

BENCHMARK_DEFINE_F(is_free_fixture, generator_footprint)
(benchmark::State& _state) {
  // Create a generator_footprint
  generator_footprint fp{map.getResolution(), footprint};
  for (auto _ : _state) {
    benchmark::DoNotOptimize(fp.is_free(cell{100, 100}, map));
  }
}

BENCHMARK_DEFINE_F(is_free_fixture, discrete_footprint)
(benchmark::State& _state) {
  // Create a discrete_footprint
  discrete_footprint fp{map.getResolution(), footprint};
  for (auto _ : _state) {
    benchmark::DoNotOptimize(fp.is_free(cell{100, 100}, map));
  }
}

// Register the benchmarks in our fixture.
BENCHMARK_REGISTER_F(is_free_fixture, area_generator)->DenseRange(4, 5);
BENCHMARK_REGISTER_F(is_free_fixture, discrete_polygon)->DenseRange(4, 5);
BENCHMARK_REGISTER_F(is_free_fixture, continuous_footprint)->DenseRange(4, 5);
BENCHMARK_REGISTER_F(is_free_fixture, generator_footprint)->DenseRange(4, 5);
BENCHMARK_REGISTER_F(is_free_fixture, discrete_footprint)->DenseRange(4, 5);
