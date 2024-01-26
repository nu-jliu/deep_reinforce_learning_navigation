///`
/// \file test_geometry2d.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief Test 2D geometry library functions
/// \version 0.1
/// \date 2024-01-25
///
/// \copyright Copyright (c) 2024
///
///
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
  Point2D test;
  test.x = 1.2;
  test.y = 2.5;
  std::stringstream ss;
  ss << test;
  REQUIRE(ss.str() == "[1.2 2.5]");

  Point2D test2;
  test2.x = 1.0;
  test2.y = 2.0;
  std::stringstream ss2;
  ss2 << test2;
  REQUIRE(ss2.str() == "[1 2]");
}

TEST_CASE("Point2D read", "[point2d]") // Allen Liu
{
  Point2D test;
  Point2D sample;
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
  Point2D p1;
  Point2D p2;
  Point2D p3;

  p1.x = 1.2;
  p1.y = 2.4;
  p2.x = 4.5;
  p2.y = 5.6;
  p3.x = -1.3;
  p3.y = -2.4;

  Vector2D v12 = p2 - p1;
  Vector2D v23 = p3 - p2;
  Vector2D v13 = p3 - p1;

  REQUIRE_THAT(v12.x, WithinAbs(3.3, TOLERANCE));
  REQUIRE_THAT(v12.y, WithinAbs(3.2, TOLERANCE));
  REQUIRE_THAT(v23.x, WithinAbs(-5.8, TOLERANCE));
  REQUIRE_THAT(v23.y, WithinAbs(-8.0, TOLERANCE));
  REQUIRE_THAT(v13.x, WithinAbs(-2.5, TOLERANCE));
  REQUIRE_THAT(v13.y, WithinAbs(-4.8, TOLERANCE));
}

TEST_CASE("Point adds", "[vector]") // Allen Liu
{
  Point2D p;

  p.x = 1.2;
  p.y = 3.4;

  Vector2D v1;
  Vector2D v2;
  Vector2D v3;

  v1.x = 1.2;
  v1.y = -2.4;
  v2.x = 1.4;
  v2.y = 1.8;
  v3.x = -2.2;
  v3.y = -5.6;

  Point2D p1 = p + v1;
  Point2D p2 = p + v2;
  Point2D p3 = p + v3;

  REQUIRE_THAT(p1.x, WithinAbs(2.4, TOLERANCE));
  REQUIRE_THAT(p1.y, WithinAbs(1.0, TOLERANCE));
  REQUIRE_THAT(p2.x, WithinAbs(2.6, TOLERANCE));
  REQUIRE_THAT(p2.y, WithinAbs(5.2, TOLERANCE));
  REQUIRE_THAT(p3.x, WithinAbs(-1.0, TOLERANCE));
  REQUIRE_THAT(p3.y, WithinAbs(-2.2, TOLERANCE));
}

TEST_CASE("Vector2D stream", "[vector2d]") // Allen Liu
{
  Vector2D test;
  test.x = 1.2;
  test.y = 2.5;
  std::stringstream ss;
  ss << test;
  REQUIRE(ss.str() == "[1.2 2.5]");

  Vector2D test2;
  test2.x = 1.0;
  test2.y = 2.0;
  std::stringstream ss2;
  ss2 << test2;
  REQUIRE(ss2.str() == "[1 2]");
}

TEST_CASE("Vector2D read", "[vector2d]") // Allen Liu
{
  Vector2D test;
  Vector2D sample = {1.2, 2.5};
  std::stringstream ss;

  ss << sample;
  ss >> test;
  REQUIRE_THAT(test.x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(test.y, WithinAbs(2.5, TOLERANCE));
}

TEST_CASE("Vector2D +", "[vector2d]") // Allen Liu
{
  Vector2D v1{1.2, 2.3};
  Vector2D v2{3.4, 4.5};

  Vector2D v12 = v1 + v2;

  REQUIRE_THAT(v12.x, WithinAbs(4.6, TOLERANCE));
  REQUIRE_THAT(v12.y, WithinAbs(6.8, TOLERANCE));

  Vector2D v3{3.4, -5.6};
  Vector2D v4{-1.3, 3.3};

  Vector2D v34 = v3 + v4;
  REQUIRE_THAT(v34.x, WithinAbs(2.1, TOLERANCE));
  REQUIRE_THAT(v34.y, WithinAbs(-2.3, TOLERANCE));
}

TEST_CASE("Vector2D +=", "[vector2d]")
{
  Vector2D v1{1.2, 3.4};
  Vector2D v2{3.5, 4.6};

  v1 += v2;

  REQUIRE_THAT(v1.x, WithinAbs(4.7, TOLERANCE));
  REQUIRE_THAT(v1.y, WithinAbs(8.0, TOLERANCE));

  Vector2D v3{1.2, -3.4};
  Vector2D v4{-5.6, 4.6};

  v3 += v4;

  REQUIRE_THAT(v3.x, WithinAbs(-4.4, TOLERANCE));
  REQUIRE_THAT(v3.y, WithinAbs(1.2, TOLERANCE));
}

TEST_CASE("Vector2D -", "[vector2d]") // Allen Liu
{
  Vector2D v{1.2, 4.5};

  Vector2D v1{2.4, 0.5};
  Vector2D test1 = v - v1;

  REQUIRE_THAT(test1.x, WithinAbs(-1.2, TOLERANCE));
  REQUIRE_THAT(test1.y, WithinAbs(4.0, TOLERANCE));

  Vector2D v2{3.5, -6.7};
  Vector2D test2 = v - v2;

  REQUIRE_THAT(test2.x, WithinAbs(-2.3, TOLERANCE));
  REQUIRE_THAT(test2.y, WithinAbs(11.2, TOLERANCE));
}

TEST_CASE("Vector2D -=", "[vector2d]") // Allen Liu
{
  Vector2D v1{1.2, 4.5};
  Vector2D v2{3.4, 6.7};

  v1 -= v2;

  REQUIRE_THAT(v1.x, WithinAbs(-2.2, TOLERANCE));
  REQUIRE_THAT(v1.y, WithinAbs(-2.2, TOLERANCE));

  Vector2D v3{3.2, -5.6};
  Vector2D v4{-0.5, 2.4};

  v3 -= v4;
  REQUIRE_THAT(v3.x, WithinAbs(3.7, TOLERANCE));
  REQUIRE_THAT(v3.y, WithinAbs(-8.0, TOLERANCE));
}

TEST_CASE("Vector2D *", "[vector2d]") {
  Vector2D v{1.2, 4.5};

  double r1(1.2);
  Vector2D v1 = v * r1;

  REQUIRE_THAT(v1.x, WithinAbs(1.44, TOLERANCE));
  REQUIRE_THAT(v1.y, WithinAbs(5.4, TOLERANCE));

  double r2(-3.4);
  Vector2D v2 = v * r2;

  REQUIRE_THAT(v2.x, WithinAbs(-4.08, TOLERANCE));
  REQUIRE_THAT(v2.y, WithinAbs(-15.3, TOLERANCE));
}

TEST_CASE("Vector2D *=", "[vector2d]")
{
  Vector2D v1{1.2, 5.6};
  v1 *= 3.4;

  REQUIRE_THAT(v1.x, WithinAbs(4.08, TOLERANCE));
  REQUIRE_THAT(v1.y, WithinAbs(19.04, TOLERANCE));


  Vector2D v2{-1.3, 4.6};
  v2 *= -4.5;

  REQUIRE_THAT(v2.x, WithinAbs(5.85, TOLERANCE));
  REQUIRE_THAT(v2.y, WithinAbs(-20.7, TOLERANCE));
}
