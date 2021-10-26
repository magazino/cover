#include "costmap.hpp"

#include <cstddef>

namespace cover {

namespace detail {

namespace {

// Constants for the costs.
constexpr double LETHAL_OBSTACLE = -1;
constexpr double NO_INFORMATION = -2;
constexpr double OUT_OF_MAP = -3;

}  // namespace

double
costmap_model_like_getter::operator()(const cell& _cell) const {
  if (!is_inside_(_cell)) {
    return OUT_OF_MAP;
  }
  const auto cost = map_.getCost(_cell.x(), _cell.y());
  if (cost == costmap_2d::LETHAL_OBSTACLE)
    return LETHAL_OBSTACLE;
  else if (cost == costmap_2d::NO_INFORMATION)
    return NO_INFORMATION;
  else
    return cost;
}
}  // namespace detail

polygon
make_polygon(const std::vector<geometry_msgs::Point>& _msg) noexcept {
  // transform to polygon
  polygon pl(2, _msg.size());
  for (size_t ii = 0; ii != _msg.size(); ++ii)
    pl.col(ii) << _msg[ii].x, _msg[ii].y;

  return pl;
}

bool
is_free(const discrete_polygon& _dp, const costmap_2d::Costmap2D& _map) {
  return is_free_impl(_dp, detail::is_lethal{_map});
}

bool
is_free(const discrete_polygon& _dp, const cell& _offset,
        const costmap_2d::Costmap2D& _map) {
  const auto check = detail::make_with_offset(detail::is_lethal{_map}, _offset);
  return is_free_impl(_dp, check);
}

}  // namespace cover
