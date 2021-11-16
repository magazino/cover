#pragma once

#include "base.hpp"
#include "impl/boost_io.hpp"

#include <Eigen/Geometry>

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <cassert>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cover {

// Below are some interpolation strategies. They all receive two poses in the
// ctor and provide a operator to retreive the pose. The parameter t must be in
// the interval [0, 1].

/// @brief Interpolation strategy returning just the start-pose.
struct StartStrategy {
  StartStrategy(const Eigen::Isometry2d& _start,
                [[maybe_unused]] const Eigen::Isometry2d& _end) noexcept :
      start_(_start) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const Eigen::Isometry2d&
  operator()([[maybe_unused]] const double _t) const noexcept {
    assert(_t >= 0 && _t <= 1 && "Invalid interpolation parameter");
    return start_;
  }

private:
  Eigen::Isometry2d start_;  ///< The start pose.
};

/// @brief Spline-based interpolation strategy.
struct SplineStrategy {
  SplineStrategy(const Eigen::Isometry2d& _start,
                 const Eigen::Isometry2d& _end) noexcept :
      start_(_start), end_(_end) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const Eigen::Isometry2d&
  operator()(const double _t) {
    if (_t < 0 || _t > 1)
      throw std::runtime_error("T out of bounds");

    // todo  implement me
    return curr_;
  }

private:
  Eigen::Isometry2d start_, end_, curr_;  ///< Start and end poses.
};

// Define the major boost::geometry types.
using bg_point = boost::geometry::model::d2::point_xy<double>;
using bg_polygon = boost::geometry::model::polygon<bg_point>;

namespace detail {

/// @brief Appends the _next ot the _curr (using union_).
/// @throw std::runtime_error if geometries aren't compatible.
void
append(const bg_polygon& _next, bg_polygon& _curr);

/**
 * @brief Applies linear sweep between curr and next.
 *
 * @param _curr The current element (already part of output).
 * @param _next The next element.
 */
bg_polygon
linear_sweep(const bg_polygon& _curr, const bg_polygon& _next);

/**
 * @brief Converts the boost::polygon while preserving the holes.
 *
 * @param _polygon The polygon.
 *
 * The first element of the output will be the outer ring, all others will be
 * inners.
 */
polygon_vec
from_boost_with_holes(const bg_polygon& _polygon);

// Below some helper structs to define the number of poses for a interpolation
// strategy.

template <typename InterpolStrategy>
struct step_traits {
  static constexpr size_t max = std::numeric_limits<size_t>::max();
};

template <typename InterpolStrategy>
constexpr size_t step_traits<InterpolStrategy>::max;

template <>
struct step_traits<StartStrategy> {
  static constexpr size_t max = 1;
};

constexpr size_t step_traits<StartStrategy>::max;

}  // namespace detail

/**
 * @brief Sweeps the polygon between the given poses.
 *
 * @tparam InterpolStrategy The interpolation strategy type.
 * @param _polygon The polygon.
 * @param _poses The poses.
 * @param n_steps Number of steps between each pose (should be >= 1).
 *
 * The interpolation strategy will be applied between the poses. For
 * StartStrategy use n_steps == 1, since it will always return just the
 * start-pose.
 */
template <typename InterpolStrategy>
polygon_vec
sweep(const polygon& _polygon, const std::vector<Eigen::Isometry2d>& _poses,
      size_t n_steps) {
  // Simple case.
  if (_polygon.cols() < 3 || _poses.empty() || n_steps == 0) {
    return {};
  }
  // Do not use intermediate poses for simple strategies.
  n_steps = std::min(n_steps, detail::step_traits<InterpolStrategy>::max);

  // Seed the body.
  bg_polygon curr = detail::to_boost<bg_point>(_poses.front() * _polygon);
  bg_polygon output = curr;

  const auto p_end = _poses.end();
  const double t_step = 1. / n_steps;

  for (auto p_begin = std::next(_poses.begin()); p_begin != p_end; ++p_begin) {
    const InterpolStrategy is{*std::prev(p_begin), *p_end};
    double t = t_step;
    for (size_t p = 0; p != n_steps; ++p, t += t_step) {
      const bg_polygon next = detail::to_boost<bg_point>(is(t) * _polygon);

      // Apply the sweep and append it to the output.
      const bg_polygon swept = detail::linear_sweep(curr, next);
      detail::append(swept, output);

      // Reuse the next polygon.
      curr = std::move(next);
    }
  }

  // Convert to cover
  return detail::from_boost_with_holes(output);
}

polygon_vec
sweep(const polygon& _polygon, const std::vector<Eigen::Isometry2d>& _poses) {
  return sweep<StartStrategy>(_polygon, _poses, 1);
}

}  // namespace cover
