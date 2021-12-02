#pragma once

#include <cover/base.hpp>

#include <Eigen/Core>

#include <cassert>
#include <cstddef>
#include <iterator>
#include <vector>

namespace cover {

namespace detail {

/// @brief The base class for the ray, outline and area iterators.
///
/// This allows us to use std::find_if and its derivatives with our custom
/// iterators.
///
/// @warning We don't have a ptr_diff type, so we set it to void.
struct base_iterator {
  using iterator_category = std::input_iterator_tag;
  using value_type = cell;
  using difference_type = void;
  using pointer = value_type*;
  using reference = value_type&;
};

}  // namespace detail

/// @defgroup Generators Generators
///
/// The group offers three types of generators: cover::ray_generator,
/// cover::outline_generator and cover::area_generator. The generators allow the
/// user to iterate over a set of cells without allocating these cells.
///
/// All generators offer a stl-like interface, allowing the user to access the
/// cells using range-based for-loops. The defined iterators also support the
/// usage of typical algorithms like std::find_if or std::foreach. Algorithms
/// which require a difference_type of the iterator are not supported.
///
/// Hints for the user: If the discrete
/// representation of a line or to_area should be used only once, the generators
/// should be preferred. If the discrete representation can be cached and
/// reused, the user may consider to expand the generators into a
/// cover::discrete_polygon.

/// @{

/**
 * @brief A ray generator.
 * The following code shows the basic use of the class:
 * @snippet test/generators.cpp ray_generator
 */
class ray_generator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief to_lines between the 2 given cells using the Bresenham algorithm.
   * Similar to STL, end is not included.
   *
   * @param _begin Start cell.
   * @param _end End cell.
   */
  ray_generator(const cell& _begin, const cell& _end);

  class iterator : public detail::base_iterator {
    /**
     * @brief The iterator should be only created from the ray_generator.
     */
    friend ray_generator;

    /**
     * @brief Constructs the line iterator from a ray generator.
     *
     * @param _ray_gen Pointer to ray generator object.
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
        ray_gen_(nullptr), counter_(_size), num_(0) {}

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Allow iterator to be only default constructable outside this
     * class. Should only be used for empty/invalid iterators.
     */
    iterator() : iterator(0ul) {}

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

    const value_type*
    operator->() const noexcept {
      return &val_;
    }

  private:
    // Pointer to ray generator object
    const ray_generator* ray_gen_;

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

/// @brief Class for generating a dense closed outline.
///
/// The class takes a sparse outline and generates the cells on this outline. It
/// is similar to a concatenated cover::ray_generator. If the outline has
/// - just one point, the generator will include this point.
/// - just two points (aka. one ray), the generator will include the endpoint.
/// - otherwise, the generator will return the rays between the consecutive
/// points and between the last and first points.
///
/// The following code shows a simple example of the outline generator with only
/// two points: @snippet test/generators.cpp outline_generator_two
///
/// A more complex example is the outline of a box:
/// @snippet test/generators.cpp outline_generator_box
///
/// @see cover::ray_generator.
class outline_generator {
  using ray_generator_vec = std::vector<ray_generator>;

public:
  /**
   * @brief Creates the outline generator.
   *
   * @param _sparse An eigen matrix containing sparse discretized 2D points.
   */
  explicit outline_generator(const discrete_polygon& _sparse) noexcept;
  outline_generator(double _resolution, const polygon& _polygon);

  class iterator : public detail::base_iterator {
    /**
     * @brief The iterator should be only created from the outline_generator.
     */
    friend outline_generator;

    /**
     * @brief Constructs the outline iterator.
     *
     * @param _begin The start line of the polygon.
     * @param _end The end line of the polygon.
     */
    iterator(ray_generator_vec::const_iterator _begin,
             ray_generator_vec::const_iterator _end) :
        outer_end_(_end), outer_itt_(_begin), counter_(0) {
      if (outer_itt_ != outer_end_) {
        inner_itt_ = outer_itt_->begin();
      }
    }

    /**
     * @brief Constructs the end of outline iterator.
     *
     * @param _size Total cell count of the outline.
     */
    explicit iterator(const size_t _size) noexcept : counter_(_size) {}

  public:
    iterator&
    operator++() {
      if (++inner_itt_ == outer_itt_->end() && ++outer_itt_ != outer_end_) {
        inner_itt_ = outer_itt_->begin();
      }

      ++counter_;
      return *this;
    }

    bool
    operator==(const iterator& _other) const noexcept {
      return counter_ == _other.counter_;
    }

    bool
    operator!=(const iterator& other) const noexcept {
      return counter_ != other.counter_;
    }

    const value_type&
    operator*() const noexcept {
      return *inner_itt_;
    }

    const value_type*
    operator->() const noexcept {
      return inner_itt_.operator->();
    }

  private:
    // Iterators
    const ray_generator_vec::const_iterator outer_end_;
    ray_generator_vec::const_iterator outer_itt_;
    ray_generator_vec::value_type::iterator inner_itt_;

    // Internal state variables. Note - we could also use outer_itt for
    // comparison, but the result is slightly slower.
    size_t counter_;
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
    return iterator{outline_.begin(), outline_.end()};
  }

  iterator
  end() const noexcept {
    return iterator(size_);
  }

private:
  /**
   * @brief Add ray generator to the outline if ray is not degenerate.
   *
   * @param _begin The start point of the ray.
   * @param _end The end point of the ray.
   */
  void
  add(const cell& _begin, const cell& _end) noexcept;

  ray_generator_vec outline_;
  size_t size_;
};

/// @brief Will generate all cells enclosed by the given outline.
///
/// The class takes a dense outline of a polygon and generates the cells
/// enclosed by the outline. It works with convex and concave polygons.
///
/// The following code shows a basic example:
/// @snippet test/generators.cpp area_generator
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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructs the area_generator from the given closed outline.
   *
   * @param _outline The closed outline of the area.
   *
   * @throw std::out_of_range If outline is empty
   * @throw std::logic_error If the outline is not closed properly
   */
  explicit area_generator(const discrete_polygon& _outline);
  area_generator(double _resolution, const polygon& _polygon);

  /**
   * @brief Constructs the area_generator from the given closed outlines.
   *
   * @param _outlines The closed outlines representing the underlying geometry.
   * By nesting outlines inside each other, it is possible to represent holes
   * and areas inside holes...(generalized even-odd rule).
   *
   * @throw std::logic_error If the outline is not closed properly
   */
  explicit area_generator(const discrete_polygon_vec& _outlines);
  area_generator(double _resolution, const polygon_vec& _polygons);

  class iterator : public detail::base_iterator {
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

    const value_type*
    operator->() const noexcept {
      return &val_;
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
  area_generator(const discrete_polygon* _outlines, size_t _n_outlines);

  x_list x_list_;
  size_t size_;
};

/// @}

}  // namespace cover
