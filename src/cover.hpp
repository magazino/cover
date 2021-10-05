#pragma once

#include <Eigen/Dense>

#include <cassert>
#include <iterator>
#include <stdexcept>
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

class ray_generator {
public:
  /**
   * @brief Raytraces between the 2 given cells using the Bresenham algorithm.
   * Similar to STL, end is not included
   *
   * @param _begin Start cell
   * @param _end End cell
   */
  ray_generator(const cell& _begin, const cell& _end);

  class iterator {
    /**
     * @brief The iterator should be only created from the ray_generator
     */
    friend ray_generator;

    /**
     * @brief Contructs the line iterator from a ray generator
     *
     * @param _ray_gen Pointer to ray generator object
     */
    explicit iterator(const ray_generator* const _ray_gen) noexcept :
        ray_gen_(_ray_gen),
        val_(ray_gen_->begin_),
        counter_(0),
        num_(ray_gen_->size_ / 2) {}

    /**
     * @brief Constructs the end of line iterator
     *
     * @param _end Total cell count of the line
     */
    explicit iterator(const size_t _size) noexcept :
        ray_gen_(nullptr), counter_(_size) {}

  public:
    // Define the iterator category as per the STL standard
    using iterator_category = std::input_iterator_tag;
    using value_type = cell;

    iterator&
    operator++() {
      num_ += ray_gen_->add_;
      if (num_ >= static_cast<int>(ray_gen_->size_)) {
        num_ -= static_cast<int>(ray_gen_->size_);
        val_ += ray_gen_->inc_minor_;
      }
      val_ += ray_gen_->inc_major_;

      ++counter_;
      return *this;
    }

    bool
    operator==(const iterator& _other) const noexcept {
      return counter_ == _other.counter_;
    }

    bool
    operator!=(const iterator& _other) const noexcept {
      return counter_ != _other.counter_;
    }

    const value_type&
    operator*() const noexcept {
      return val_;
    }

  private:
    // Pointer to ray generator object
    const ray_generator* const ray_gen_;

    // Internal state variables
    cell val_;
    size_t counter_;
    int num_;
  };

  size_t
  size() const noexcept {
    return size_;
  }

  bool
  empty() const noexcept {
    return size_ == 0;
  }

  iterator
  begin() const noexcept {
    return iterator{this};
  }

  iterator
  end() const noexcept {
    return iterator(size_);
  }

private:
  // Start cell
  const cell begin_;

  // Increments for major and minor axis
  cell inc_minor_, inc_major_;

  // Size and addition coefficient
  size_t size_, add_;
};

class area_generator {
  /// @brief A Range defined by min and max values and an y-value.
  class range {
  public:
    range(int _y, int _min, int _max) noexcept;

    int
    y() const noexcept {
      return y_;
    }

    int
    min() const noexcept {
      return min_;
    }

    int
    max() const noexcept {
      return max_;
    }

    void
    set_max(const int _max) noexcept {
      assert(min_ <= _max && "Max cannot be smaller than min");
      max_ = _max;
    }

    void
    set_min(const int _min) noexcept {
      assert(_min <= max_ && "Min cannot be larger than max");
      min_ = _min;
    }

  private:
    int y_;
    int min_;
    int max_;
  };

  // Define scan line types. This is basically run-length-encoding.
  using x_list = std::vector<range>;

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
  explicit area_generator(const discrete_polygon& _outline);

  class iterator {
    // The iterator should be only created from the area_generator.
    friend area_generator;

    /// @brief The dummy constructor.
    explicit iterator(x_list::const_iterator _begin) noexcept :
        x_list_(_begin) {}

    /// @brief Value-based constructor.
    /// @param _begin The first iterator of the x-list.
    /// @param _end The last iterator of the x-list.
    iterator(x_list::const_iterator _begin, x_list::const_iterator _end) :
        x_list_(_begin), x_end_(_end) {
      if (x_list_ != x_end_)
        val_ << x_list_->min(), x_list_->y();
    }

  public:
    // Define the iterator category as per the stl standard
    using iterator_category = std::input_iterator_tag;
    using value_type = cell;

    iterator&
    operator++() {
      if (++val_.x() > x_list_->max() && ++x_list_ != x_end_) {
        val_ << x_list_->min(), x_list_->y();
      }
      return *this;
    }

    bool
    operator==(const iterator& _other) const noexcept {
      return x_list_ == _other.x_list_;
    }

    bool
    operator!=(const iterator& other) const noexcept {
      return x_list_ != other.x_list_;
    }

    const value_type&
    operator*() const noexcept {
      return val_;
    }

  private:
    // Info about the x-list.
    x_list::const_iterator x_list_, x_end_;

    // Internal state variables
    cell val_;
  };

  size_t
  size() const noexcept {
    return size_;
  }

  bool
  empty() const noexcept {
    return size_ == 0;
  }

  iterator
  begin() const noexcept {
    return iterator{x_list_.cbegin(), x_list_.cend()};
  }

  iterator
  end() const noexcept {
    return iterator(x_list_.cend());
  }

private:
  x_list x_list_;
  size_t size_;
};

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