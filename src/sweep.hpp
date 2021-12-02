#pragma once

#include "base.hpp"
#include "impl/boost_io.hpp"

#include <Eigen/Geometry>

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <cmath>
#include <iterator>
#include <utility>
#include <vector>

namespace cover {

/// @defgroup Sweeping Sweeping
/// Generate a swept area from a polygon and a set of poses.
/// @{

// Below are some interpolation strategies. They all receive two poses in the
// ctor and provide a operator to retrieve the pose. The parameter t must be in
// the interval [0, 1].
namespace interpol {

/// @brief Interpolation strategy returning just the start-pose.
/// @note Use this strategy in conjunction with stepping::one_step_strategy,
/// since it will not produce any intermediate poses.
struct start_strategy {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  start_strategy(const Eigen::Isometry2d& _start,
                 const Eigen::Isometry2d& _end) noexcept;

  const Eigen::Isometry2d&
  operator()(const double _t) const noexcept;

private:
  Eigen::Isometry2d start_;  ///< The start pose.
};

/// @brief Interpolation strategy that linearly interpolates translation and
/// rotation
struct linear_strategy {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  linear_strategy(const Eigen::Isometry2d& _start,
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

/// @brief Factory, which will generate the strategy from two poses.
/// Add a specialization if your strategy requires more arguments. See e.x.
/// @snippet test/sweep.cpp custom_factory
template <typename InterpolStrategy>
struct strategy_factory {
  using strategy_type = InterpolStrategy;

  strategy_type
  make(const Eigen::Isometry2d& _start, const Eigen::Isometry2d& _end) const {
    return strategy_type{_start, _end};
  }
};

}  // namespace interpol

// Below are some stepping strategies
namespace stepping {

/// @brief Do not step in between intermediate poses
struct one_step_strategy {
  size_t
  get_steps(const Eigen::Isometry2d& _start,
            const Eigen::Isometry2d& _end) const noexcept;
};

/// @brief Translational and rotational constrained strategy. The translational
/// step is in world coordinates for e.g meters while the angular step is in
/// radians
struct max_step_strategy {
  max_step_strategy(const double _max_trans_step,
                    const double _max_angular_step) noexcept;

  size_t
  get_steps(const Eigen::Isometry2d& _start,
            const Eigen::Isometry2d& _end) const noexcept;

private:
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
 * @tparam SteppingStrategy The stepping strategy type.
 * @param _polygon The polygon.
 * @param _poses The poses.
 * @param _interpol_factory Factory for producing the interpolation strategy.
 * @param _stepping_strategy Strategy for computing the steps between 2 poses.
 *
 * The function generates a swept-area from a polygon and a set of poses. The
 * stepping_strategy defines how many intermediate steps between two consecutive
 * poses are used. The interpol_factory constructs an interpolator from two
 * consecutive poses.
 *
 * @see interpol::start_strategy or interpol::linear_strategy for interpolation
 * examples and stepping::one_step_strategy and stepping::max_step_strategy for
 * stepping examples.
 */
template <typename InterpolStrategy, typename SteppingStrategy>
polygon_vec
sweep(const polygon& _polygon, const std::vector<Eigen::Isometry2d>& _poses,
      const interpol::strategy_factory<InterpolStrategy>& _interpol_factory,
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
    const auto is = _interpol_factory.make(*std::prev(p_begin), *p_begin);
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

/**
 * @brief Sweeps the polygon between the poses.
 *
 * @param _polygon The polygon.
 * @param _poses The poses.
 *
 * Will use the interpol::start_strategy and stepping::one_step_strategy for
 * computing the swept area.
 */
polygon_vec
sweep(const polygon& _polygon, const std::vector<Eigen::Isometry2d>& _poses) {
  return sweep(_polygon, _poses,
               interpol::strategy_factory<interpol::start_strategy>{},
               stepping::one_step_strategy{});
}

/// @}

}  // namespace cover
