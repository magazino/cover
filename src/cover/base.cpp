// Copyright 2022 Magazino GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
