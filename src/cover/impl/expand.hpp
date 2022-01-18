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

#pragma once

#include <cassert>
#include <numeric>
#include <vector>

namespace cover {

template <typename Generator>
discrete_polygon
expand(const Generator& _gen) noexcept {
  // Allocate the space
  discrete_polygon expanded(2, _gen.size());
  size_t counter = 0;

  // Expand the generator
  for (const auto& curr_cell : _gen) {
    expanded.col(counter++) = curr_cell;
  }

  // Sanity check
  assert(counter == _gen.size() && "Mismatched size");

  return expanded;
}

template <typename Generator>
discrete_polygon
expand_multiple(const std::vector<Generator>& _gens) {
  // Compute the size
  const size_t n_cells = std::accumulate(
      _gens.begin(), _gens.end(), 0,
      [](size_t _sum, const Generator& _gen) { return _sum + _gen.size(); });

  // Allocate the space
  discrete_polygon out(2, n_cells);

  // Copy the values to the destination.
  size_t counter = 0;
  for (const auto& gen : _gens)
    for (const auto& c : gen) {
      out.col(counter++) = c;
    }

  assert(counter == n_cells && "Mismatched size");
  return out;
}

}  // namespace cover
