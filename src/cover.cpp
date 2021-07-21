#include "cover.hpp"

#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateArraySequence.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/util/PolygonExtracter.h>

#include <ros/console.h>

#include <cmath>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

constexpr char mod_name[] = "cover: ";

#define COVER_DEBUG(args) ROS_DEBUG_STREAM(mod_name << args)
#define COVER_INFO(args) ROS_INFO_STREAM(mod_name << args)
#define COVER_WARN(args) ROS_WARN_STREAM(mod_name << args)

// lets be lazy and define useful aliases as <NAME>Ptr
#define MAKE_UNIQUE_PTR(name) using name##Ptr = std::unique_ptr<name>

namespace cover {

// define short-cut for the geos-namespace
namespace gg = geos::geom;

// define easier access to the main classes
using gg::CoordinateArraySequence;
using gg::CoordinateSequence;
using gg::Geometry;
using gg::LinearRing;
using gg::Polygon;

// define some pointers for simpler handling
MAKE_UNIQUE_PTR(Geometry);
MAKE_UNIQUE_PTR(LinearRing);
MAKE_UNIQUE_PTR(Polygon);
MAKE_UNIQUE_PTR(CoordinateSequence);
MAKE_UNIQUE_PTR(CoordinateArraySequence);

polygon
to_eigen(const gg::CoordinateSequence& _s) noexcept {
  polygon out(2, _s.size());
  for (size_t ii = 0; ii != _s.size(); ++ii)
    out.col(ii) = Eigen::Vector2d{_s.getAt(ii).x, _s.getAt(ii).y};

  return out;
}

inline polygon
to_eigen(const gg::Polygon& _p) noexcept {
  return to_eigen(*_p.getExteriorRing()->getCoordinates());
}

std::string
to_string(const GeometryPtr& _inflated,
          const gg::Polygon::ConstVect& _dense) noexcept {
  // function compbines everything into a MULTIPOLYGON string
  // legnth of "POLYGON "
  constexpr size_t start = 8;

  std::stringstream ss;
  ss << "MULTIPOLYGON (";
  try {
    ss << _inflated->toString().substr(start);
    for (const auto& polygon_ptr : _dense)
      ss << ", " << polygon_ptr->toString().substr(start);
  }
  catch (std::out_of_range& ex) {
    COVER_WARN("failed to generate wkt: " << ex.what());
  }

  ss << ")";
  return ss.str();
}

footprint
to_geos(const polygon& _p, double _d) {
  if (_d <= 0)
    throw std::invalid_argument("radius must be positive");

  COVER_INFO("setting up geometry...");

  // convert to CoordinateSequence
  CoordinateArraySequencePtr sequence(new gg::CoordinateArraySequence());
  for (int cc = 0; cc != _p.cols(); ++cc)
    sequence->add(gg::Coordinate{_p(0, cc), _p(1, cc)});

  // check if we have to close the polygon
  if (!sequence->isEmpty() && _p.col(0) != _p.col(_p.cols() - 1)) {
    COVER_INFO("closing polygon geometry");
    sequence->add(sequence->front());
  }

  static auto factory = gg::GeometryFactory::create();

  // linear ring will take the ownership now
  // throws if invalid
  LinearRingPtr ring{factory->createLinearRing(sequence.release())};

  // polygon takes ownership of everything
  PolygonPtr poly(factory->createPolygon(ring.release(), nullptr));

  if (!poly->isValid())
    throw std::runtime_error("failed to construct a valid polygon");

  COVER_INFO("erode/inflating with the radius " << _d);

  // erode the geometry
  GeometryPtr eroded{poly->buffer(-_d, 16)};
  if (eroded->isEmpty()) {
    COVER_WARN("distance " << _d << " too big");
  }

  // inflate it back and get the diff
  GeometryPtr inflated{eroded->buffer(_d)};
  GeometryPtr diff{poly->difference(inflated.get())};

  // convert the geometry to polygons
  // the ownership lies at the geometry object
  gg::Polygon::ConstVect polygons;
  gg::util::PolygonExtracter::getPolygons(*diff, polygons);

  // convert back to output format
  footprint out;
  out.ring = to_eigen(*eroded->getCoordinates());
  out.dense.reserve(polygons.size());
  for (const auto& polygon_ptr : polygons) {
    // newer versions of geos append an empty polygon
    if (polygon_ptr->isEmpty())
      continue;

    out.dense.emplace_back(to_eigen(*polygon_ptr));

    // check the polygon quality
    if (polygon_ptr->getNumInteriorRing() != 0)
      throw std::runtime_error("interior rings must be empty");
  }

  // finally print the wkt-representation
  COVER_DEBUG(to_string(inflated, polygons));

  return out;
}

footprint
split(const polygon& _p, const double& _rad) {
  // todo if _p is smaller then 3, then just return the line?
  return to_geos(_p, _rad);
}

double
_get_factor(const double& _res) {
  if (_res <= 0)
    throw std::runtime_error("resolution must be greater then zero");
  return 1. / _res;
}

discrete_polygon
discretise(const polygon& _polygon, double _res) {
  // get the factor - will throw for bad resolution
  const auto factor = _get_factor(_res);

  // convert the polygon to cells
  return (_polygon * factor).cast<int>();
}

discrete_polygon
raytrace(const cell& _begin, const cell& _end) noexcept {
  // adjusted from ros - speed-up with eigen's magic
  const Eigen::Array2i delta_raw = _end - _begin;
  const cell delta = delta_raw.abs();

  // auxilary stuff
  cell::Index max_row, min_row;
  const int den = delta.maxCoeff(&max_row);
  const int add = delta.minCoeff(&min_row);
  const int size = den;
  int num = den / 2;

  // the minor is zero at max
  cell inc_minor = delta_raw.sign();
  cell inc_major = inc_minor;
  inc_minor[max_row] = 0;

  // Corner case where both are equal, we need to manually flip the axis
  if (min_row == max_row) {
    // Works as we have 2 axis. Change axis from 0 -> 1 or 1 -> 0
    min_row = !min_row;
  }
  inc_major[min_row] = 0;

  // the running vars
  cell curr = _begin;
  discrete_polygon ray(2, size);

  // mind the smaller sign
  for (int ii = 0; ii < size; ++ii) {
    ray.col(ii) = curr;

    num += add;
    if (num >= den) {
      num -= den;
      curr += inc_minor;
    }
    curr += inc_major;
  }

  return ray;
}

inline int
ray_size(const cell& _begin, const cell& _end) noexcept {
  return (_end - _begin).array().abs().maxCoeff();
}

discrete_polygon
densify(const discrete_polygon& _sparse) noexcept {
  const auto sparse_cols = _sparse.cols();

  // trivial case check
  if (sparse_cols < 2)
    return _sparse;

  // first calculate the final size of the array
  // allocate memory for the ray_sizes (rs)
  std::vector<int> rs(sparse_cols, 0);

  // double pointer iteration to get the size of every ray
  // note: with the check above sparse_cols cannot be negative
  for (int ii = 0; ii != sparse_cols - 1; ++ii)
    rs[ii] = ray_size(_sparse.col(ii), _sparse.col(ii + 1));

  // add the last one
  if (sparse_cols > 2)
    rs.back() = ray_size(_sparse.col(sparse_cols - 1), _sparse.col(0));

  // now allocate enough memory dense and densify
  // the last element of rs is the final number of dense-points
  discrete_polygon dense(2, std::accumulate(rs.begin(), rs.end(), 0));

  // double pointer iteration to get the ray
  // note: with the check above sparse_cols cannot be negative
  int start_col = 0;
  for (int ii = 0; ii != sparse_cols - 1; ++ii) {
    dense.block(0, start_col, 2, rs[ii]) =
        raytrace(_sparse.col(ii), _sparse.col(ii + 1));
    start_col += rs[ii];
  }

  // close the last one
  if (sparse_cols > 2) {
    dense.block(0, start_col, 2, rs.back()) =
        raytrace(_sparse.col(sparse_cols - 1), _sparse.col(0));
  }

  return dense;
}

discrete_polygon
dense_outline(const polygon& _p, double _res) {
  const discrete_polygon discrete = discretise(_p, _res);
  return densify(discrete);
}

inline int
signed_y_diff(const cell& _l, const cell& _r) noexcept {
  // note: the variables in cm_locations are unsigned, so we need the cast
  return _l.y() - _r.y();
}

inline bool
is_cusp(const cell& _l, const cell& _m, const cell& _r) noexcept {
  return signed_y_diff(_l, _m) * signed_y_diff(_m, _r) < 0;
}

discrete_polygon
area(const discrete_polygon& _outline) {
  // Now we do a sort-free line-fill-algorithm for area checking.
  // We don't sort by y-values, but just throw them into a hash-map.
  // algorithm has 2 steps:
  // 1 - Generate the scan_line; (y-lines) of cooridnate pairs
  // 2 - Check pairwise the values
  using x_list = std::vector<int>;
  using y_hash = std::unordered_map<int, x_list>;

  y_hash scan_line;

  // Better safe than sorry
  if (_outline.cols() < 3)
    throw std::out_of_range("Invalid size");

  const auto n_pts = static_cast<size_t>(_outline.cols());

  // Below step 1: Generate the scan_line structure.
  // We have to be extra-careful at the ends, since the outline is a closed
  // polygon. here we check the first element

  // Find the end point where y changes
  size_t end = n_pts - 1;
  while (end != 0 && _outline(1, end) == _outline(1, 0)) {
    --end;
  }

  // Similarly, find the start point where y chanegs
  size_t start = 0;
  while (start != end && _outline(1, start) == _outline(1, 0)) {
    ++start;
  }

  // Return base cells if no area is defined
  if (start >= end) {
    return _outline;
  }

  // Add to the scan line if the current point is not a cusp
  if (!is_cusp(_outline.col(end), _outline.col(0), _outline.col(start))) {
    // We create a mapping of y-coordinates to multiple x-coordinates
    scan_line[_outline(1, 0)].push_back(_outline(0, 0));
  }

  // Use a two pointer iteration to add 'non-horizontal' elements to the scan
  // line in [start, end)
  for (size_t prev = 0, curr = start; curr != end;) {
    auto next = curr + 1;
    // Skip the horizontal part...
    while (next != end && _outline(1, curr) == _outline(1, next)) {
      ++curr;
      ++next;
    }

    // Skips cusps in y-direction...
    if (!is_cusp(_outline.col(prev), _outline.col(curr), _outline.col(next))) {
      // Add the x-coordinate of the curr point to the y-hashmap
      scan_line[_outline(1, curr)].push_back(_outline(0, curr));
    }

    // update the variables
    prev = curr;
    curr = next;
  }

  // Check end - here we short-cut the algorithm and just check if the final
  // scan_line has odd number of elements
  {
    const auto& end_pt = _outline.col(end);

    if (!scan_line[end_pt.y()].empty() && scan_line[end_pt.y()].size() % 2 == 1)
      scan_line[end_pt.y()].push_back(end_pt.x());
  }

  // Size computation for number of area cells
  size_t n_cells = static_cast<size_t>(_outline.cols());

  // Accumulate the inner cells size
  for (const auto& line_pairs : scan_line) {
    // Contains the list of x coordinates
    const auto& x_line = line_pairs.second;

    // sanity check if we are good to go
    if (x_line.size() % 2 != 0)
      throw std::logic_error("line-scan algorithm failed");

    for (auto itt = x_line.begin(); itt != x_line.end(); itt += 2) {
      // Can happen if 2 points of the polygon got discretized to the same cell
      const auto diff = std::abs(*itt - *(itt + 1));
      n_cells += static_cast<size_t>(std::max(diff - 1, 0));
    }
  }

  // Below step 2: now check for every y line the x-pairs
  discrete_polygon area(2, n_cells);

  // Block copy the outline
  area.block(0, 0, 2, n_pts) = _outline;

  // Counter to determine how many points we have already added to the area
  size_t counter = n_pts;

  for (const auto& line_pairs : scan_line) {
    // Contains the list of x coordinates
    const auto curr_y = line_pairs.first;
    const auto& x_line = line_pairs.second;

    // Go over line pairs and fill in the inner cells
    for (auto itt = x_line.begin(); itt != x_line.end(); itt += 2) {
      auto curr_x = *itt;
      const auto end_x = *(itt + 1);
      const auto dx = end_x > curr_x ? 1 : -1;

      // Can happen if 2 points of the polygon got discretized to the same cell
      if (curr_x == end_x) {
        continue;
      }

      // Skip first point as we already added it above
      curr_x += dx;

      while (curr_x != end_x) {
        area.col(counter++) = cell{curr_x, curr_y};
        curr_x += dx;
      }
    }
  }
  return area;
}

discrete_polygon
discretise_area(const polygon& _p, double _res) {
  // get the dense outline of the polygon
  const discrete_polygon outline = dense_outline(_p, _res);
  return area(outline);
}

discrete_footprint
discretise(const footprint& _fp, double _res) {
  discrete_footprint discrete;
  discrete.ring = dense_outline(_fp.ring, _res);
  discrete.dense.reserve(_fp.dense.size());
  for (const auto& area : _fp.dense)
    discrete.dense.emplace_back(discretise_area(area, _res));

  return discrete;
}

}  // namespace cover
