#include <catch2/catch_all.hpp>

#include "turtlelib/trig2d.hpp"

using Catch::Matchers::WithinAbs;

#define TOLERANCE 1e-5

namespace turtlelib
{
TEST_CASE("Test cross product", "[cross2d]")
{
  Vector2D v1{1.3, -3.1};
  Vector2D v2{-5.2, -3.0};

  double v12 = cross2d(v1, v2);
  REQUIRE_THAT(v12, WithinAbs(-20.02, TOLERANCE));

  Vector2D v3{9.2, 4.6};
  Vector2D v4{-3.4, -6.7};

  double v34 = cross2d(v3, v4);
  REQUIRE_THAT(v34, WithinAbs(-46.0, TOLERANCE));
}

TEST_CASE("Test dot product", "[dot2d]")
{
  Vector2D v1{1.2, 4.5};
  Vector2D v2{-1.1, -4.3};

  double v12 = dot2d(v1, v2);
  REQUIRE_THAT(v12, WithinAbs(-20.67, TOLERANCE));

  Vector2D v3{-2.3, -5.6};
  Vector2D v4{-1.1, -4.3};

  double v34 = dot2d(v3, v4);
  REQUIRE_THAT(v34, WithinAbs(26.61, TOLERANCE));
}

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
  double theta2 = PI / 2.0;

  REQUIRE_THAT(find_distance(theta2, obs2), WithinAbs(1.0, TOLERANCE));
}
}
