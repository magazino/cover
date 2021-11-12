#pragma once

// --- Internal Includes ---
#include <cover.hpp>

// --- ROS Includes ---
#include <geometry_msgs/Point.h>

// --- Standard Includes ---
#include <cmath>
#include <vector>

namespace cover {

// The coordinates of polygons, where all x coordinates are followed by the y
// coordinates.
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
     -0.13, -0.04, -0.03, -0.00},
    // L-shape
    {0.51, -0.51, -0.41, -0.31, -0.31, 0.51, 0.11, -0.11, -0.81, -0.76, -0.26,
     -0.11}};

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

/**
 * @brief Computes the bounds of the cells
 *
 * @param[in] _polygon Discretized polygon
 * @param[out] _rows The number of rows
 * @param[out] _cols The number of columns
 */
inline void
compute_size(const discrete_polygon& _polygon, int& _rows, int& _cols) {
  _cols = 0;
  _rows = 0;

  if (_polygon.cols()) {
    const cell extent = _polygon.rowwise().maxCoeff();
    _rows = extent.y() + 1;
    _cols = extent.x() + 1;
  }
}

}  // namespace cover
