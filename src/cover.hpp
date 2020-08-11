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
 *  * points **on** the ring have inflated cost or higher
 *  * points **inside** the dense areas have lethal cost
 */
struct footprint {
  polygon_vec dense;
  polygon ring;
};

/**
 * @brief function will split the given polygon with a given radius into
 * the footprint structure.
 *
 * @param _p polygon with at least three distinct points
 * @param _rad inflation radius for calculating the ring
 *
 * @throw if the input is ill-formed
 */
footprint
split(const polygon& _p, const double& _rad);

using cell = Eigen::Vector2i;
using cell_vec = std::vector<cell>;

cell_vec
discretise(const polygon& _p, double _res);

// below some machinery to densify the outline

/**
 * @brief raytraces between [+begin, _end)
 *
 * Note: contrary to the ros-implementation _end is not part of the output
 * array. In this manner, we are closer to the std-style. This makes the
 * densifying step of a polygon easier, since we don't have duplicates.
 *
 * @param _begin inclusive start of the ray
 * @param _end exclusive end of the ray
 */
cell_vec
raytrace(const cell& _begin, const cell& _end) noexcept;

/**
 * @brief returns the dense representation of the outline
 *
 * Will basically call pair-wise the raytrace function above and stich the
 * result together.
 *
 * @param _sparse polygon of an arbitrary size
 */
cell_vec
densify(const cell_vec& _sparse) noexcept;

struct discrete_footprint {
  cell_vec dense, inscribed;
};

discrete_footprint
discretise(const footprint& _fp);

}  // namespace cover