#include <cover/base.hpp>

#include <algorithm>

namespace cover {

polygon_vec
to_continuous(const double _resolution,
              const discrete_polygon_vec& _discrete) noexcept {
  polygon_vec continuous(_discrete.size());
  std::transform(_discrete.begin(), _discrete.end(), continuous.begin(),
                 [&](const discrete_polygon& __discrete) {
                   return to_continuous(_resolution, __discrete);
                 });
  return continuous;
}

discrete_polygon_vec
to_discrete(const double _resolution, const polygon_vec& _continuous) {
  discrete_polygon_vec discrete(_continuous.size());
  std::transform(_continuous.begin(), _continuous.end(), discrete.begin(),
                 [&](const polygon& __continuous) {
                   return to_discrete(_resolution, __continuous);
                 });
  return discrete;
}

}  // namespace cover
