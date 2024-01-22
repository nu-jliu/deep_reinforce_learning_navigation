#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_all.hpp>

#define TOLERANCE 1e-12

using namespace turtlelib;
using Catch::Matchers::WithinAbs;


TEST_CASE("Almost Equal", "[almost_equal]") // Allen Liu
{
  REQUIRE(almost_equal(2.0, 2.0 + 1e-15));
  REQUIRE(almost_equal(-5.3, -5.3 - 1e-13));
  REQUIRE_FALSE(almost_equal(1.1, 1.2));
  REQUIRE_FALSE(almost_equal(1.2, 1.2 + 1e-5));
}

TEST_CASE("Degree to radian", "[deg2rad]") // Allen Liu
{
  REQUIRE_THAT(deg2rad(360), WithinAbs(2.0 * turtlelib::PI, TOLERANCE));
  REQUIRE_THAT(deg2rad(180), WithinAbs(turtlelib::PI, TOLERANCE));
  REQUIRE_THAT(deg2rad(135), WithinAbs(turtlelib::PI * 0.75, TOLERANCE));
  REQUIRE_THAT(deg2rad(90), WithinAbs(turtlelib::PI / 2.0, TOLERANCE));
  REQUIRE_THAT(deg2rad(45), WithinAbs(turtlelib::PI / 4.0, TOLERANCE));
}

TEST_CASE("Radian to degree", "[rad2deg]") // Allen Liu
{
  REQUIRE_THAT(rad2deg(PI), WithinAbs(180.0, TOLERANCE));
  REQUIRE_THAT(rad2deg(PI / 2.0), WithinAbs(90.0, TOLERANCE));
  REQUIRE_THAT(rad2deg(PI * 2.0), WithinAbs(360.0, TOLERANCE));
}

TEST_CASE("Normalize angle", "[normalize_angle]") // Allen Liu
{
  REQUIRE_THAT(normalize_angle(PI), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(normalize_angle(-PI), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(normalize_angle(0), WithinAbs(0, TOLERANCE));
  REQUIRE_THAT(normalize_angle(-PI / 4.0), WithinAbs(-PI / 4.0, TOLERANCE));
  REQUIRE_THAT(normalize_angle(3.0 / 2.0 * PI), WithinAbs(-PI / 2.0, TOLERANCE));
  REQUIRE_THAT(normalize_angle(-5.0 * PI / 2.0), WithinAbs(-PI / 2.0, TOLERANCE));
}

TEST_CASE("Point2D stream", "[point2d]") // Allen Liu
{
  struct Point2D test;
  test.x = 1.2;
  test.y = 2.5;
  std::stringstream ss;
  ss << test;
  REQUIRE(ss.str() == "[1.2 2.5]");

  struct Point2D test2;
  test2.x = 1.0;
  test2.y = 2.0;
  std::stringstream ss2;
  ss2 << test2;
  REQUIRE(ss2.str() == "[1 2]");
}

TEST_CASE("Point2D read", "[point2d]") // Allen Liu
{
  struct Point2D test;
  struct Point2D sample;
  sample.x = 1.2;
  sample.y = 2.5;
  std::stringstream ss;

  ss << sample;
  ss >> test;
  REQUIRE_THAT(test.x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(test.y, WithinAbs(2.5, TOLERANCE));
}

TEST_CASE("Point subs", "[vector]") // Allen Liu
{
  struct Point2D p1;
  struct Point2D p2;
  struct Point2D p3;

  p1.x = 1.2;
  p1.y = 2.4;
  p2.x = 4.5;
  p2.y = 5.6;
  p3.x = -1.3;
  p3.y = -2.4;

  struct Vector2D v12 = p2 - p1;
  struct Vector2D v23 = p3 - p2;
  struct Vector2D v13 = p3 - p1;

  REQUIRE_THAT(v12.x, WithinAbs(3.3, TOLERANCE));
  REQUIRE_THAT(v12.y, WithinAbs(3.2, TOLERANCE));
  REQUIRE_THAT(v23.x, WithinAbs(-5.8, TOLERANCE));
  REQUIRE_THAT(v23.y, WithinAbs(-8.0, TOLERANCE));
  REQUIRE_THAT(v13.x, WithinAbs(-2.5, TOLERANCE));
  REQUIRE_THAT(v13.y, WithinAbs(-4.8, TOLERANCE));
}

TEST_CASE("Point adds", "[vector]") // Allen Liu
{
  struct Point2D p;

  p.x = 1.2;
  p.y = 3.4;

  struct Vector2D v1;
  struct Vector2D v2;
  struct Vector2D v3;

  v1.x = 1.2;
  v1.y = -2.4;
  v2.x = 1.4;
  v2.y = 1.8;
  v3.x = -2.2;
  v3.y = -5.6;

  struct Point2D p1 = p + v1;
  struct Point2D p2 = p + v2;
  struct Point2D p3 = p + v3;

  REQUIRE_THAT(p1.x, WithinAbs(2.4, TOLERANCE));
  REQUIRE_THAT(p1.y, WithinAbs(1.0, TOLERANCE));
  REQUIRE_THAT(p2.x, WithinAbs(2.6, TOLERANCE));
  REQUIRE_THAT(p2.y, WithinAbs(5.2, TOLERANCE));
  REQUIRE_THAT(p3.x, WithinAbs(-1.0, TOLERANCE));
  REQUIRE_THAT(p3.y, WithinAbs(-2.2, TOLERANCE));
}

TEST_CASE("Vector2D stream", "[vector2d]") // Allen Liu
{
  struct Vector2D test;
  test.x = 1.2;
  test.y = 2.5;
  std::stringstream ss;
  ss << test;
  REQUIRE(ss.str() == "[1.2 2.5]");

  struct Vector2D test2;
  test2.x = 1.0;
  test2.y = 2.0;
  std::stringstream ss2;
  ss2 << test2;
  REQUIRE(ss2.str() == "[1 2]");
}

TEST_CASE("Vector2D read", "[vector2d]") // Allen Liu
{
  struct Vector2D test;
  struct Vector2D sample = {1.2, 2.5};
  std::stringstream ss;

  ss << sample;
  ss >> test;
  REQUIRE_THAT(test.x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(test.y, WithinAbs(2.5, TOLERANCE));
}
