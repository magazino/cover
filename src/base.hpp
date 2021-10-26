#pragma once

#include <Eigen/Core>

#include <stdexcept>
#include <vector>

namespace cover {

// below our interface format
using point = Eigen::Vector2d;
using polygon = Eigen::Matrix<double, 2ul, Eigen::Dynamic>;
using polygon_vec = std::vector<polygon>;

using cell = Eigen::Vector2i;
using discrete_polygon = Eigen::Matrix<int, 2ul, Eigen::Dynamic>;
using discrete_polygon_vec = std::vector<discrete_polygon>;

// Below basic conversions between both.

/// @brief Converts the discrete input to continuous.
template <int Size>
Eigen::Matrix<double, 2, Size>
to_continuous(const double _resolution,
              const Eigen::Matrix<int, 2, Size>& _discrete) noexcept {
  return _discrete.template cast<double>() * _resolution;
}

/// @copybrief to_continuous(const double, const Eigen::Matrix<int, 2, Size>&)
polygon_vec
to_continuous(double _resolution,
              const discrete_polygon_vec& _discrete) noexcept;

/// @brief Converts the continuous input to discrete.
/// @throw std::invalid_argument if the resolution is not positive.
template <int Size>
Eigen::Matrix<int, 2, Size>
to_discrete(const double _resolution,
            const Eigen::Matrix<double, 2, Size>& _continuous) {
  if (_resolution <= 0)
    throw std::invalid_argument("The resolution must be positive");
  return (_continuous * (1. / _resolution)).template cast<int>();
}

/// @copydoc to_discrete(const double, const Eigen::Matrix<double, 2, Size>&)
discrete_polygon_vec
to_discrete(double _resolution, const polygon_vec& _continuous);

}  // namespace cover
