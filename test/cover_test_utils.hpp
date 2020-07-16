#pragma once

#include <cover_ros.hpp>

#include <geometry_msgs/Point.h>

#include <cmath>
#include <vector>

namespace cover {

static polygon
make_circle(size_t _s, double _radius) noexcept {
  polygon out(2, _s);
  const auto d_s = static_cast<double>(_s);
  for (int ii = 0; ii != out.cols(); ++ii) {
    const auto angle = M_PI * 2 * ii / d_s;
    out.col(ii) << std::cos(angle), std::sin(angle);
  }
  // scale
  out *= _radius;
  return out;
}

static std::vector<geometry_msgs::Point>
to_msgs(const polygon& _p) noexcept {
  std::vector<geometry_msgs::Point> msg(_p.cols());
  for (int cc = 0; cc != _p.cols(); ++cc) {
    msg[cc].x = _p(0, cc);
    msg[cc].y = _p(1, cc);
  }
  return msg;
}

}  // namespace cover