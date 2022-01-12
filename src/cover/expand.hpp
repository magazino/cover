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

#include <cover/base.hpp>
#include <cover/generators.hpp>

#include <utility>

namespace cover {

/// @defgroup Expansions Expansions
/// The group provides methods to expand the Generators to a discrete_polygon.

/// @{

/// @brief Expands all cells of a generator into a cover::discrete_polygon.
///
/// @tparam Generator The generator type (e.x. cover::ray_generator).
/// @param _gen The generator.
///
/// The following code shows the basic functionality of the function:
/// @snippet test/expand.cpp expand_outline_generator
///
/// @see Generators
template <typename Generator>
discrete_polygon
expand(const Generator& _gen) noexcept;

/// @brief Expands multiple generators into a cover::discrete_polygon.
///
/// @tparam Generator The generator type (e.x. cover::ray_generator).
/// @param _gens A vector of generators.
///
/// The following code shows the basic functionality of the function:
/// @snippet test/expand.cpp expand_multiple_expand_multiple
///
/// @see cover::expand
/// @see Generators
template <typename Generator>
discrete_polygon
expand_multiple(const std::vector<Generator>& _gens);

// Below helpers to construct a generator and expand it inplace.

/// @brief Creates a ray_generator and expands it.
/// Example usage: @snippet test/expand.cpp to_line
template <typename... Args>
discrete_polygon
to_line(Args&&... args) {
  return expand(ray_generator{std::forward<Args>(args)...});
}

/// @brief Creates a outline_generator and expands it.
/// Example usage: @snippet test/expand.cpp to_outline
template <typename... Args>
discrete_polygon
to_outline(Args&&... args) {
  return expand(outline_generator{std::forward<Args>(args)...});
}

/// @brief Creates a area_generator and expands it.
/// Example usage: @snippet test/expand.cpp to_area
template <typename... Args>
discrete_polygon
to_area(Args&&... args) {
  return expand(area_generator{std::forward<Args>(args)...});
}

/// @}

}  // namespace cover

#include "impl/expand.hpp"
