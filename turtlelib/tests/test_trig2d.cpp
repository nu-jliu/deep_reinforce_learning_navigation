#include <catch2/catch_all.hpp>

#include "turtlelib/trig2d.hpp"

using Catch::Matchers::WithinAbs;

#define TOLERANCE 1e-5

namespace turtlelib
{
TEST_CASE("Test can intersect", "[can_intersect]")
{
  Obstacle obs1{2.0, 0.0, 1.0};

  double theta1 = PI / 6.0;
  double theta2 = PI;

  REQUIRE(can_intersect(theta1, obs1));
  REQUIRE_FALSE(can_intersect(theta2, obs1));
}

TEST_CASE("Test find distance", "[distance]") /// Allen Liu
{
  Obstacle obs1{2.0, 0.0, 1.0};
  double theta1 = PI / 6.0;

  REQUIRE_THAT(find_distance(theta1, obs1), WithinAbs(sqrt(3.0), TOLERANCE));

  Obstacle obs2{1.0, 1.0, 1.0};
  double theta2 = PI / 4.0;

  REQUIRE_THAT(find_distance(theta2, obs2), WithinAbs(1.0, TOLERANCE));
}
}
