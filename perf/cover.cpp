#include <benchmark/benchmark.h>

#include <cover_ros.hpp>
#include <base_local_planner/footprint_helper.h>

using namespace cover;

polygon
make_circle(size_t s, double _rad) noexcept {
  polygon out(2, s);
  for (size_t ii = 0; ii != s; ++ii) {
    const auto angle = M_PI * 2 * ii / s;
    out.col(ii) << std::cos(angle), std::sin(angle);
  }
  // scale
  out *= _rad;
  return out;
}

std::vector<geometry_msgs::Point>
to_msgs(const polygon& _p) noexcept {
  std::vector<geometry_msgs::Point> msg(_p.cols());
  for (int cc = 0; cc != _p.cols(); ++cc) {
    msg[cc].x = _p(0, cc);
    msg[cc].y = _p(1, cc);
  }
  return msg;
}

static void
footprint_helper_unit_circle(benchmark::State& state) {
  costmap_2d::Costmap2D map(30, 30, 0.1, -1.5, -1.5);
  base_local_planner::FootprintHelper fh;
  const auto msg = to_msgs(make_circle(16, 1));
  const Eigen::Vector3f center(0, 0, 0);

  for (auto _ : state) {
    const auto cells = fh.getFootprintCells(center, msg, map, true);
    for (const auto& c : cells)
      map.getCost(c.x, c.y);
  }
}

// Register the function as a benchmark
BENCHMARK(footprint_helper_unit_circle);

static void
cover_unit_circle(benchmark::State& state) {
  costmap_2d::Costmap2D map(30, 30, 0.1, -1.5, -1.5);
  const auto fh = make_circle(16, 1);
  const Eigen::Vector3d center(0, 0, 0);

  for (auto _ : state) {
    check_area(map, fh, center);
  }
}

BENCHMARK(cover_unit_circle);

static void
footprint_helper_polygon(benchmark::State& state) {
  costmap_2d::Costmap2D map(50, 50, 0.1, -2.5, -2.5);
  base_local_planner::FootprintHelper fh;
  polygon p(2, 12);
  p << 1, 1, 0, -1, -1, 0.2, -1, 1, 0.5, 1, -1, -0.8;
  const auto msg = to_msgs(p);
  const Eigen::Vector3f center(0, 0, 1);

  for (auto _ : state) {
    const auto cells = fh.getFootprintCells(center, msg, map, true);
    for (const auto& c : cells)
      map.getCost(c.x, c.y);
  }
}

BENCHMARK(footprint_helper_polygon);

static void
cover_polygon(benchmark::State& state) {
  costmap_2d::Costmap2D map(50, 50, 0.1, -2.5, -2.5);
  polygon p(2, 12);
  p << 1, 1, 0, -1, -1, 0.2, -1, 1, 0.5, 1, -1, -0.8;
  const Eigen::Vector3d center(0, 0, 1);

  for (auto _ : state) {
    check_area(map, p, center);
  }
}

BENCHMARK(cover_polygon);

BENCHMARK_MAIN();