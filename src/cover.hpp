#pragma once

#include <Eigen/Dense>

#include <vector>

namespace cover {

////////////////////////////////////////////////////////////////////////////////
// CONTINUOUS-METHODS
////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////
// DISCRETE-METHODS
////////////////////////////////////////////////////////////////////////////////

using cell = Eigen::Vector2i;
using discrete_polygon = Eigen::Matrix<int, 2ul, Eigen::Dynamic>;
using discrete_polygon_vec = std::vector<discrete_polygon>;

/**
 * @brief converts every metric point of input to a cell.
 *
 * Note: size of the output will be the size of the input parameter _p
 *
 * @param _p a metric polygon
 * @param _res the resolution (size of a cell)
 */
discrete_polygon
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
discrete_polygon
raytrace(const cell& _begin, const cell& _end) noexcept;

/**
 * @brief returns the dense representation of the outline
 *
 * Will basically call pair-wise the raytrace function above and stich the
 * result together.
 *
 * @param _sparse polygon of an arbitrary size
 */
discrete_polygon
densify(const discrete_polygon& _sparse) noexcept;

/**
 * @brief Generates a dense outline of the given polygon.
 *
 * @param _p The polygon.
 * @param _res The resolution.
 */
discrete_polygon
dense_outline(const polygon& _p, double _res);

/**
 * @brief Creates an area from the outline.
 *
 * @param _outline A closed outline.
 */
discrete_polygon
area(const discrete_polygon& _outline);

struct discrete_footprint {
  discrete_polygon_vec dense;
  discrete_polygon ring;
};

/// @brief se2 pose [x, y, theta].T
using se2 = Eigen::Vector3d;

inline Eigen::Affine2d
to_affine(const se2& _pose) noexcept {
  return Eigen::Translation2d(_pose.segment(0, 2)) *
         Eigen::Rotation2Dd(_pose.z());
}

discrete_footprint
discretise(const footprint& _fp, const se2& _pose);

}  // namespace cover