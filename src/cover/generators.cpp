#include <cover/base.hpp>
#include <cover/expand.hpp>
#include <cover/generators.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace cover {

ray_generator::ray_generator(const cell& _begin, const cell& _end) :
    begin_(_begin) {
  // Adjusted from ros - speed-up with eigen's magic
  const Eigen::Array2i delta_raw = _end - _begin;
  const cell delta = delta_raw.abs();

  // Auxiliary stuff
  cell::Index max_row, min_row;
  size_ = delta.maxCoeff(&max_row);
  add_ = delta.minCoeff(&min_row);

  // The minor is zero at max and vice versa
  inc_minor_ = delta_raw.sign();
  inc_major_ = inc_minor_;
  inc_minor_[max_row] = 0;

  // Corner case where both are equal, we need to manually flip the axis
  if (min_row == max_row) {
    // Works as we have 2 axis. Change axis from 0 -> 1 or 1 -> 0
    min_row = !min_row;
  }
  inc_major_[min_row] = 0;
}

outline_generator::outline_generator(const discrete_polygon& _sparse) noexcept :
    size_(0) {
  // Reserve space
  const auto n_pts = static_cast<size_t>(_sparse.cols());
  outline_.reserve(n_pts);

  if (n_pts == 0) {
    return;
  }

  // Create ray generator for each point pair
  for (size_t idx = 0; idx != n_pts - 1; ++idx) {
    add(_sparse.col(idx), _sparse.col(idx + 1));
  }

  // If there were more than 2 non-degenerate points, close the outline.
  // Otherwise add a dummy-segment with the last point.
  if (outline_.size() > 1) {
    add(_sparse.col(n_pts - 1), _sparse.col(0));
  }
  else {
    // We create a dummy point by offsetting by 1 which will not be included (as
    // it is the end point)
    const cell dummy = _sparse.col(n_pts - 1).array() + 1;
    add(_sparse.col(n_pts - 1), dummy);
  }
}

outline_generator::outline_generator(const double _resolution,
                                     const polygon& _polygon) :
    outline_generator{to_discrete(_resolution, _polygon)} {}

void
outline_generator::add(const cell& _begin, const cell& _end) noexcept {
  // If not degenerate
  if (_begin != _end) {
    outline_.emplace_back(_begin, _end);
    size_ += outline_.back().size();
  }
}

static int
signed_y_diff(const cell& _l, const cell& _r) noexcept {
  return _l.y() - _r.y();
}

static bool
is_cusp(const cell& _l, const cell& _m, const cell& _r) noexcept {
  return signed_y_diff(_l, _m) * signed_y_diff(_m, _r) < 0;
}

area_generator::range::range(const int _y, const int _min,
                             const int _max) noexcept :
    y_(_y) {
  // We sort the indices to make sure that the order is correct
  std::tie(min_, max_) = std::minmax(_min, _max);
}

area_generator::area_generator(const discrete_polygon& _outline) :
    area_generator{&_outline, 1} {}

area_generator::area_generator(const double _resolution,
                               const polygon& _polygon) :
    area_generator{to_outline(_resolution, _polygon)} {}

area_generator::area_generator(const discrete_polygon_vec& _outlines) :
    area_generator{_outlines.data(), _outlines.size()} {}

static discrete_polygon_vec
polygons_to_outlines(const double _resolution, const polygon_vec& _polygons) {
  discrete_polygon_vec outlines;
  outlines.reserve(_polygons.size());

  for (const auto& polygon : _polygons) {
    outlines.emplace_back(to_outline(_resolution, polygon));
  }
  return outlines;
}

area_generator::area_generator(const double _resolution,
                               const polygon_vec& _polygons) :
    area_generator(polygons_to_outlines(_resolution, _polygons)) {}

area_generator::area_generator(const discrete_polygon* _outlines,
                               const size_t _n_outlines) :
    size_(0) {
  // We do a sort-free line-fill-algorithm for area checking. We don't sort
  // by y-values, but just throw them into a hash-map. Algorithm has 2 steps:
  // 1 - Generate the scan_line; (y-lines) of coordinate pairs
  // 2 - Check pairwise the values

  // For convenience
  const auto outline_begin = _outlines;
  const auto outline_end = outline_begin + _n_outlines;

  // Better safe than sorry
  const int n_cols =
      std::accumulate(outline_begin, outline_end, 0,
                      [](const int _cols, const discrete_polygon& _outline) {
                        return _cols + _outline.cols();
                      });
  if (n_cols == 0)
    return;

  // Get the y-bounds of the areas.
  int y_begin = std::numeric_limits<int>::max();
  int y_end = std::numeric_limits<int>::min();
  for (auto ptr = outline_begin; ptr != outline_end; ++ptr) {
    const auto& outline = *ptr;

    if (outline.cols() == 0) {
      continue;
    }
    // Compute the limits
    y_begin = std::min(y_begin, outline.row(1).minCoeff());
    y_end = std::max(y_end, outline.row(1).maxCoeff());
  }
  assert(y_begin <= y_end && "Wrong bounds in y");

  using y_hash = std::vector<x_list>;
  y_hash y_map;

  y_map.resize(y_end - y_begin + 1);
  for (auto& m : y_map)
    m.reserve(10);

  // Now go over each outline and apply the line fill algorithm
  for (auto ptr = outline_begin; ptr != outline_end; ++ptr) {
    const auto& outline = *ptr;

    // Get the size.
    const auto n_pts = static_cast<size_t>(outline.cols());

    // Below step 1: Generate the scan_line structure

    // Find the end point where y changes
    size_t end = n_pts - 1;
    while (end != 0 && outline(1, end) == outline(1, 0)) {
      --end;
    }

    // If end is 0, it means that all points are at the same height. In such a
    // case, we simply return the min and max at the given y coordinate
    if (end == 0) {
      const int min_x = outline.row(0).minCoeff();
      const int max_x = outline.row(0).maxCoeff();
      const int curr_y = outline(1, 0) - y_begin;

      y_map.at(curr_y).emplace_back(outline(1, 0), min_x, min_x);
      y_map.at(curr_y).emplace_back(outline(1, 0), max_x, max_x);
    }

    // Helper to wrap points around in case index reaches the size
    const auto wrap_around = [n_pts](const size_t _idx) {
      return _idx == n_pts ? 0 : _idx;
    };

    for (size_t curr = 0, last = end; curr <= end; ++curr) {
      const auto next = wrap_around(curr + 1);
      const auto curr_y = outline(1, curr) - y_begin;

      // Skip curr if y does not change between curr and next
      if (curr_y == outline(1, next) - y_begin) {
        continue;
      }

      // Add current range to the scan line
      const auto start = wrap_around(last + 1);
      y_map.at(curr_y).emplace_back(outline(1, curr), outline(0, start),
                                    outline(0, curr));

      // If current range is a cusp, add it again
      if (is_cusp(outline.col(last), outline.col(curr), outline.col(next))) {
        y_map.at(curr_y).emplace_back(outline(1, curr), outline(0, start),
                                      outline(0, curr));
      }

      last = curr;
    }
  }

  // Now, sort the x-line pairs to account for non-convex shapes
  std::for_each(y_map.begin(), y_map.end(), [](x_list& ranges) {
    std::sort(ranges.begin(), ranges.end(),
              [](const range& _r0, const range& _r1) {
                return _r0.min() < _r1.min();
              });
  });

  // Merge consecutive ranges
  size_t x_list_size = 0;
  for (auto& ranges : y_map) {
    // Verify that we have an even number of ranges
    if (ranges.size() % 2 != 0) {
      throw std::logic_error("Even number of line-pairs expected");
    }
    x_list_size += ranges.size() / 2;

    // Merge the ranges.
    for (auto ii = ranges.begin(); ii != ranges.end(); ii += 2)
      ii->set_max(std::next(ii)->max());
  }
  // Reserve the space for all ranges
  x_list_.reserve(x_list_size);
  for (auto& ranges : y_map) {
    for (auto ii = ranges.begin(); ii != ranges.end(); ii += 2)
      x_list_.emplace_back(std::move(*ii));
  }

  assert(x_list_.size() == x_list_size && "Mismatched sizes");

  // Accumulate the size within the given line.
  for (const auto& range : x_list_)
    size_ += static_cast<size_t>(range.max() - range.min() + 1);
}

}  // namespace cover
