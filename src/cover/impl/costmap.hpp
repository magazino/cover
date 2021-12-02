#pragma once

#include <costmap_2d/costmap_2d.h>

#include <algorithm>

namespace cover {

template <typename Generator>
bool
is_free(const Generator& _gen, const costmap_2d::Costmap2D& _map) {
  return std::none_of(_gen.begin(), _gen.end(), detail::is_lethal{_map});
}

template <typename Generator>
bool
is_free(const Generator& _gen, const cell& _offset,
        const costmap_2d::Costmap2D& _map) {
  const auto check = detail::make_with_offset(detail::is_lethal{_map}, _offset);
  return std::none_of(_gen.begin(), _gen.end(), check);
}

namespace detail {

template <typename Check>
bool
is_free_impl(const discrete_polygon& _dp, const Check& _check) {
  const int cols = _dp.cols();
  for (int ii = 0; ii != cols; ++ii) {
    if (_check(_dp.col(ii)))
      return false;
  }
  return true;
}

template <typename Generator, typename Getter>
double
accumulate_cost(const Generator& _gen, const Getter& _getter) {
  double sum = 0;
  for (const auto& c : _gen) {
    // Get the next cost.
    const auto cost = _getter(c);
    // We abort if the cost is negative.
    if (cost < 0)
      return cost;
    sum += cost;
  }
  return sum;
}

}  // namespace detail

template <typename Generator>
double
accumulate_cost(const Generator& _gen, const costmap_2d::Costmap2D& _map) {
  const detail::costmap_model_like_getter getter{_map};
  return detail::accumulate_cost(_gen, getter);
}

template <typename Generator>
double
accumulate_cost(const Generator& _gen, const cell& _offset,
                const costmap_2d::Costmap2D& _map) {
  const auto getter = detail::make_with_offset(
      detail::costmap_model_like_getter{_map}, _offset);
  return detail::accumulate_cost(_gen, getter);
}

}  // namespace cover
