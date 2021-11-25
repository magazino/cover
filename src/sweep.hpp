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
namespace interpolation {

/// @brief Interpolation strategy returning just the start-pose.
struct StartStrategy {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StartStrategy(const Eigen::Isometry2d& _start,
                const Eigen::Isometry2d& _end) noexcept;

  const Eigen::Isometry2d&
  operator()(const double _t) const noexcept;

private:
  Eigen::Isometry2d start_;  ///< The start pose.

};  // namespace interpolation

/// @brief Interpolation strategy that linearly interpolates translation and
/// rotation
struct LinearStrategy {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LinearStrategy(const Eigen::Isometry2d& _start,
                 const Eigen::Isometry2d& _end) noexcept;

  Eigen::Isometry2d
  operator()(const double _t) const;

private:
  // Translation
  const Eigen::Vector2d t_start_;
  const Eigen::Vector2d t_diff_;

  // Rotation
  const Eigen::Rotation2Dd r_start_;
  const Eigen::Rotation2Dd r_end_;
};

}  // namespace interpolation

// Below are some stepping strategies
namespace stepping {

/// @brief Do not step in between intermediate poses
struct SingleStepStrategy {
  size_t
  get_steps(const Eigen::Isometry2d& _start,
            const Eigen::Isometry2d& _end) const;
};

/// @brief Translational and rotational constrained strategy. The translational
/// step is in world coordinates for e.g meters while the angular step is in
/// radians
struct TransRotStrategy {
  TransRotStrategy(const double _max_trans_step,
                   const double _max_angular_step) noexcept;

  size_t
  get_steps(const Eigen::Isometry2d& _start,
            const Eigen::Isometry2d& _end) const;

private:
  static double
  compute_shortest_angle(const Eigen::Rotation2Dd& _r0,
                         const Eigen::Rotation2Dd& _r1);

  const double t_step_;  ///> Translational step
  const double a_step_;  ///> Angular step
};

}  // namespace stepping

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

}  // namespace detail

/**
 * @brief Sweeps the polygon between the given poses.
 *
 * @tparam InterpolStrategy The interpolation strategy type.
 * @param _polygon The polygon.
 * @param _poses The poses.
 * @param _stepping_strategy Strategy for computing the steps between 2 poses.
 *
 * The interpolation strategy will be applied between the poses. For
 * StartStrategy use SingleStepStrategy, since it will always return just the
 * start-pose.
 */
template <typename InterpolStrategy, typename SteppingStrategy>
polygon_vec
sweep(const polygon& _polygon, const std::vector<Eigen::Isometry2d>& _poses,
      const SteppingStrategy& _stepping_strategy) {
  // Simple case.
  if (_polygon.cols() < 3 || _poses.empty()) {
    return {};
  }

  // Seed the body.
  bg_polygon curr = detail::to_boost<bg_point>(_poses.front() * _polygon);
  bg_polygon output = curr;

  const auto p_end = _poses.end();

  for (auto p_begin = std::next(_poses.begin()); p_begin != p_end; ++p_begin) {
    // Strategies setup
    const InterpolStrategy is{*std::prev(p_begin), *p_begin};
    const size_t n_steps = std::max(
        1ul, _stepping_strategy.get_steps(*std::prev(p_begin), *p_begin));
    const double t_step = 1. / n_steps;

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
  return sweep<interpolation::StartStrategy>(_polygon, _poses,
                                             stepping::SingleStepStrategy{});
}

}  // namespace cover
