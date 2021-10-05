#pragma once

// --- Internal Includes ---
#include <cover_ros.hpp>

// --- ROS Includes ---
#include <geometry_msgs/Point.h>

// --- Standard Includes ---
#include <cmath>
#include <vector>

namespace cover {

/**
 * @brief Given a linearized vector of doubles which contains all x-coordinates
 * followed by y-coordinates, constructs an Eigen counter-part of it
 *
 * @param coordinates Vector of doubles containing x-coordinates followed by
 * y-coordinates
 * @return polygon The polygon(footprint) as Eigen matrix
 */
inline polygon
make_footprint(std::vector<double> _coordinates) {
  // We define a custom type here as Eigen by default uses column major storage.
  // This is also what the cover library uses. To copy the data over properly,
  // we must transpose the matrix as _coordinates contains all x-values followed
  // by all y-values and not x-y pairs
  using polygon_tranpose_t =
      Eigen::Matrix<polygon::Scalar, polygon::ColsAtCompileTime,
                    polygon::RowsAtCompileTime>;

  return Eigen::Map<polygon_tranpose_t>(_coordinates.data(),
                                        _coordinates.size() / 2, 2)
      .transpose();
}

inline polygon
make_circle(size_t _s, double _radius) noexcept {
  polygon out(2, _s);
  const auto d_s = static_cast<double>(_s);
  for (int ii = 0; ii != out.cols(); ++ii) {
    const auto angle = M_PI * 2 * ii / d_s;
    out.col(ii) << std::cos(angle), std::sin(angle);
  }
  // scale
  out *= _radius;
  return out;
}

inline std::vector<geometry_msgs::Point>
to_msgs(const polygon& _p) noexcept {
  std::vector<geometry_msgs::Point> msg(_p.cols());
  for (int cc = 0; cc != _p.cols(); ++cc) {
    msg[cc].x = _p(0, cc);
    msg[cc].y = _p(1, cc);
  }
  return msg;
}

}  // namespace cover
