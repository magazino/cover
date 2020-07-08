#pragma once

#include <Eigen/Core>
// #include <Eigen/StdVector>

#include <vector>
// #include <list>

namespace cover {

// avoid repeating yourself
#define EIGEN_STL_VECTOR(type)                                                 \
  using type##_vec = std::vector<type, Eigen::aligned_allocator<type> >;

// below our interface format
using point = Eigen::Vector2d;
using polygon = Eigen::Matrix<double, 2ul, Eigen::Dynamic>;
using polygon_vec = std::vector<polygon>;

// EIGEN_STL_VECTOR(polygon);

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