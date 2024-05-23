#include <catch2/catch_all.hpp>
#include "turtlelib/wall_range.hpp"
#include <iostream>
#include <cmath>

#define TOLERANCE 1e-5

namespace turtlelib
{
TEST_CASE("Test if a ray and line segment intersects", "[can_intersect]")
{
  Wall test1 {
    {-1.0, 1.0},
    {-1.0, -1.0}
  };

  Wall test2 {
    {-1.0, -1.0},
    {-1.0, 1.0}
  };

  std::cout << "Starting test" << std::endl;

  REQUIRE(can_intersect(Point2D{0.0, 0.0}, -PI + 0.001, test1));
  REQUIRE_FALSE(can_intersect(Point2D{0.0, 0.0}, PI / 2.0, test1));

  REQUIRE(can_intersect(Point2D{0.0, 0.0}, -PI + 0.001, test2));
  REQUIRE_FALSE(can_intersect(Point2D{0.0, 0.0}, PI / 2.0, test2));

  std::cout << "End test" << std::endl;
}

TEST_CASE("Test the distance", "[find_distance]")
{
  Wall test1 {
    {-1.0, 1.0},
    {-1.0, -1.0}
  };

  Wall test2 {
    {0.0, 1.0},
    {1.0, 0.0}
  };

  if (can_intersect(Point2D{0.0, 0.0}, -PI, test1)) {
    const auto d1 = find_wall_distance(Point2D{0.0, 0.0}, -PI, test1);
    REQUIRE_THAT(d1, Catch::Matchers::WithinAbs(1.0, TOLERANCE));
  }

  if (can_intersect(Point2D{0.0, 0.0}, PI / 4.0, test2)) {
    const auto d2 = find_wall_distance(Point2D{0.0, 0.0}, PI / 4.0, test2);
    REQUIRE_THAT(d2, Catch::Matchers::WithinAbs(sqrt(2.0) / 2.0, TOLERANCE));
  }
}
}
