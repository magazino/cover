#include "cover.hpp"

#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateArraySequence.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/util/PolygonExtracter.h>

#include <ros/console.h>

#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

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
using gg::CoordinateSequence;
using gg::CoordinateArraySequence;
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
    if(polygon_ptr->isEmpty())
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

cell_vec
discretise(const polygon& _polygon, double _res) {
  const auto size = _polygon.cols();
  cell_vec discrete(size);

  // get the factor - will throw for bad resolution
  const auto factor = _get_factor(_res);

  // convert the polygon to cells
  for (int ii = 0; ii != size; ++ii)
    discrete[ii] = (_polygon.col(ii) * factor).cast<int>();

  return discrete;
}

cell_vec
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
  inc_major[min_row] = 0;

  // the running vars
  cell curr = _begin;
  cell_vec ray;
  ray.reserve(size);

  // mind the smaller sign
  for (int ii = 0; ii < size; ++ii) {
    ray.push_back(curr);

    num += add;
    if (num >= den) {
      num -= den;
      curr += inc_minor;
    }
    curr += inc_major;
  }

  return ray;
}

cell_vec
densify(const cell_vec& _sparse) noexcept {
  // trivial case check
  if (_sparse.size() < 2)
    return _sparse;

  cell_vec dense;
  // do the double pointer iteration
  // note: with the check above size cannot underflow
  for (int ii = 0; ii != _sparse.size() - 1; ++ii) {
    const auto curr = raytrace(_sparse.at(ii), _sparse.at(ii + 1));
    dense.insert(dense.end(), curr.begin(), curr.end());
  }

  // close the last one
  if (_sparse.size() > 2) {
    const auto last = raytrace(_sparse.back(), _sparse.front());
    dense.insert(dense.end(), last.begin(), last.end());
  }

  return dense;
}

}  // namespace cover
