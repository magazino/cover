#include "cover.hpp"

#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateArraySequence.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/util/PolygonExtracter.h>

#include <ros/console.h>

constexpr char mod_name[] = "cover: ";

#define COVER_INFO(args) ROS_INFO_STREAM(mod_name << args)
#define COVER_WARN(args) ROS_WARN_STREAM(mod_name << args)

// lets be lazy and define useful aliases as <NAME>Ptr
#define MAKE_UNIQUE_PTR(name) using name##Ptr = std::unique_ptr<name>;

namespace geos {
namespace geom {

// define some pointers for simpler handling
MAKE_UNIQUE_PTR(Geometry);
MAKE_UNIQUE_PTR(LinearRing);
MAKE_UNIQUE_PTR(Polygon);
MAKE_UNIQUE_PTR(CoordinateSequence);
// something special...
using GeometryVecPtr = std::unique_ptr<Geometry::NonConstVect>;

}  // namespace geom
}  // namespace geos

namespace cover {

namespace gg = geos::geom;

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

// allow millimeter tolerance
constexpr double tolerance = 1e-3;

footprint
to_geos(const polygon& _p, double _d) {
  if (_d <= 0)
    throw std::invalid_argument("radius must be positive");

  COVER_INFO("setting up geometry...");

  // convert to CoordinateSequence
  gg::CoordinateSequencePtr sequence(new gg::CoordinateArraySequence());
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
  gg::LinearRingPtr ring{factory->createLinearRing(sequence.release())};
  gg::GeometryVecPtr holes{new gg::Geometry::NonConstVect()};

  // polygon takes ownership of everything
  gg::PolygonPtr poly(factory->createPolygon(ring.release(), holes.release()));

  if (!poly->isValid())
    throw std::runtime_error("failed to construct a valid polygon");

  COVER_INFO("erode/inflating with the radius " << _d);

  // erode the geometry
  gg::GeometryPtr eroded{poly->buffer(-_d, 16)};
  if (eroded->isEmpty()) {
    COVER_WARN("distance " << _d << " too big");
  }

  // inflate it back and get the diff
  gg::GeometryPtr inflated{eroded->buffer(_d)};
  gg::GeometryPtr diff{poly->difference(inflated.get())};

  // convert the geometry to polygons
  // the ownership lies at the geometry object
  gg::Polygon::ConstVect polygons;
  gg::util::PolygonExtracter::getPolygons(*diff, polygons);

  // convert back to output format
  footprint out;
  out.ring = to_eigen(*eroded->getCoordinates());
  out.dense.reserve(polygons.size());
  for (const auto& polygon_ptr : polygons) {
    out.dense.emplace_back(to_eigen(*polygon_ptr));

    // check the polygon quality
    if (polygon_ptr->getNumInteriorRing() != 0)
      throw std::runtime_error("interior rings must be empty");
  }

  return out;
}

footprint
split(const polygon& _p, const double& _rad) {
  // todo if _p is smaller then 3, then just return the line?
  return to_geos(_p, _rad);
}

}  // namespace cover
