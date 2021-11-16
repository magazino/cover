#include "impl/boost_io.hpp"
#include <gtest/gtest.h>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace cover;
namespace bg = boost::geometry;
using bg_point = bg::model::d2::point_xy<double>;
using bg_polygon = bg::model::polygon<bg_point>;

TEST(correct_rectangle, first) {
  bg_polygon p{
      {bg_point{0, 0}, bg_point{1, 0}, bg_point{0, 1}, bg_point{1, 1}}};
  bg::correct(p);
  ASSERT_FALSE(bg::is_valid(p));
  ASSERT_EQ(p.outer().size(), 5);
  
  const auto result = detail::split_rectangle(p);

  ASSERT_FALSE(result.empty());
  ASSERT_TRUE(bg::is_valid(result));
}

TEST(correct_rectangle, second) {
  bg_polygon p{
      {bg_point{0, 0}, bg_point{1, 1}, bg_point{0, 1}, bg_point{1, 0}}};
  bg::correct(p);
  ASSERT_FALSE(bg::is_valid(p));

  const auto result = detail::split_rectangle(p);

  ASSERT_FALSE(result.empty());
  ASSERT_TRUE(bg::is_valid(result));
}