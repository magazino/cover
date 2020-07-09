#pragma once

#include <Eigen/Core>

#include <vector>

namespace cover {

// below our interface format
using point = Eigen::Vector2d;
using polygon = Eigen::Matrix<double, 2ul, Eigen::Dynamic>;
using polygon_vec = std::vector<polygon>;

enum class check_type { DENSE, RING, ALL };

/**
 * @brief main POD
 *
 * The footprint consists out of 2 types of data - dense and ring.
 *
 * The footprint is in collsion if
 *  * points **on** the ring are within the inflated cost area
 *  * points **inside** the dense areas are within lethal cost area
 */
struct footprint {
  polygon_vec dense;
  polygon ring;
};

footprint
split(const polygon& _p, const double& _rad);

}  // namespace cover