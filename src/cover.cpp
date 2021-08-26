#include "cover.hpp"

#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateArraySequence.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/util/PolygonExtracter.h>

#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
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

/// @brief A Range defined by min and max values.
class range {
public:
  range(const int _min, const int _max) {
    // We sort the indices to make sure that the order is correct
    std::tie(min_, max_) = std::minmax(_min, _max);
  }

  const int&
  min() const {
    return min_;
  }

  const int&
  max() const {
    return max_;
  }

private:
  int min_;
  int max_;
};

// define scan line types
using x_list = std::vector<range>;
using y_hash = std::unordered_map<int, x_list>;

class area_constructor {
public:
  /**
   * @brief Constructs the fill area from the given closed outline
   *
   * @param _outline The closed outline to fill. The outline is included in the
   * fill cells
   *
   * @throw std::out_of_range If outline is empty
   * @throw std::logic_error If the outline is not closed properly
   */
  area_constructor(const discrete_polygon& _outline) {
    // Now we do a sort-free line-fill-algorithm for area checking.
    // We don't sort by y-values, but just throw them into a hash-map.
    // Algorithm has 2 steps:
    // 1 - Generate the scan_line; (y-lines) of cooridnate pairs
    // 2 - Check pairwise the values

    // Better safe than sorry
    if (_outline.cols() == 0)
      throw std::out_of_range("Outline cannot be empty");

    const auto n_pts = static_cast<size_t>(_outline.cols());

    // Below step 1: Generate the scan_line structure

    // Find the end point where y changes
    size_t end = n_pts - 1;
    while (end != 0 && _outline(1, end) == _outline(1, 0)) {
      --end;
    }

    // If end is 0, it means that all points were at the same height. In such a
    // case, we simply return the min and max at the given y coordinate
    if (end == 0) {
      int min_x = _outline.row(0).minCoeff();
      int max_x = _outline.row(0).maxCoeff();

      hash_map_[_outline(1, 0)].push_back({min_x, min_x});
      hash_map_[_outline(1, 0)].push_back({max_x, max_x});
    }

    // Helper to wrap points around in case index reaches the size
    const auto wrap_around = [n_pts](const size_t _idx) {
      return _idx == n_pts ? 0 : _idx;
    };

    for (size_t curr = 0, last = end; curr <= end; ++curr) {
      auto next = wrap_around(curr + 1);

      // Skip curr if y does not change between curr and next
      if (_outline(1, curr) == _outline(1, next)) {
        continue;
      }

      // Add current range to the scan line
      const auto start = wrap_around(last + 1);
      hash_map_[_outline(1, curr)].push_back(
          {_outline(0, start), _outline(0, curr)});

      // If current range is a cusp, add it again
      if (is_cusp(_outline.col(last), _outline.col(curr), _outline.col(next))) {
        hash_map_[_outline(1, curr)].push_back(
            {_outline(0, start), _outline(0, curr)});
      }

      last = curr;
    }

    // Now, sort the x-line pairs to account for non-convex shapes
    std::for_each(hash_map_.begin(), hash_map_.end(), [](auto& line_pairs) {
      std::sort(line_pairs.second.begin(), line_pairs.second.end(),
                [](const range& _r0, const range& _r1) {
                  return _r0.min() < _r1.min();
                });
    });

    // Finally, compute the size
    size_ = verify_and_compute_size(hash_map_);
  }

  class iterator {
    // Define internal types
    using line_iterator_t = y_hash::const_iterator;
    using range_iterator_t = y_hash::value_type::second_type::const_iterator;

  public:
    // Define the iterator category as per the stl standard
    using iterator_category = std::input_iterator_tag;
    using value_type = cell;

    iterator(line_iterator_t _line_itt, line_iterator_t _end_itt) :
        line_itt_(std::move(_line_itt)),
        end_itt_(std::move(_end_itt)),
        curr_x_() {
      update();
    }

    iterator&
    operator++() {
      increment();
      return *this;
    }

    bool
    operator!=(const iterator& other) const {
      return line_itt_ != other.line_itt_;
    }

    value_type
    operator*() const {
      return {curr_x_, line_itt_->first};
    }

  private:
    /**
     * @brief Increments the internal iterators such that we return the next
     * cell of the area when dereferenced
     */
    void
    increment() {
      // In case we are still iterating over the current range
      if (curr_x_ < (curr_range_ + 1)->max()) {
        ++curr_x_;
      }
      // Otherwise, we check if there is another valid range pair
      else if ((curr_range_ + 2) != line_itt_->second.end()) {
        curr_range_ += 2;
        curr_x_ = curr_range_->min();
      }
      // If both the above conditions are not met, it means that we have gone
      // over all the area cells at the current y. We then move to the next y
      // value
      else {
        ++line_itt_;
        update();
      }
    }

    /**
     * @brief Updates the internal state variables if there are lines remaining
     */
    void
    update() {
      if (line_itt_ != end_itt_) {
        curr_range_ = line_itt_->second.begin();
        curr_x_ = curr_range_->min();
      }
    }

    // Info about the hashmap
    line_iterator_t line_itt_;
    line_iterator_t end_itt_;

    // Internal state variables
    range_iterator_t curr_range_;
    int curr_x_;
  };

  size_t
  size() const {
    return size_;
  }

  iterator
  begin() const {
    return {hash_map_.begin(), hash_map_.end()};
  }

  iterator
  end() const {
    return {hash_map_.end(), hash_map_.end()};
  }

private:
  /**
   * @brief Computes the total number of cells after filling the area and
   * verifies that the pairs are even in number for every y
   *
   * @param _hash_map The hash map for which cells count needs to be computed
   * @return Total number of cells
   *
   * @throw std::logic_error If the scan line have odd number of x-pairs at any
   * given y
   */
  static size_t
  verify_and_compute_size(const y_hash& _hash_map) {
    size_t n_cells = 0;

    // Accumulate the inner cells size
    for (const auto& line_pairs : _hash_map) {
      // Contains the list of x coordinates
      const auto& x_line = line_pairs.second;

      // sanity check if we are good to go
      if (x_line.size() % 2 != 0)
        throw std::logic_error("line-scan algorithm failed");

      for (auto itt = x_line.begin(); itt != x_line.end(); itt += 2) {
        n_cells += static_cast<size_t>((*(itt + 1)).max() - (*itt).min() + 1);
      }
    }

    return n_cells;
  }

  y_hash hash_map_;
  size_t size_;
};

discrete_polygon
area(const discrete_polygon& _outline) {
  const auto area_ctor = area_constructor(_outline);

  // Fetch the area cells from the area constructor
  discrete_polygon area_cells(2, area_ctor.size());
  size_t counter = 0;

  for (const auto& fill_cell : area_ctor) {
    area_cells.col(counter++) = fill_cell;
  }

  return area_cells;
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
