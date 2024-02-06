///
/// \file test_se2d.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief Test 2D transformation functions
/// \version 0.1
/// \date 2024-01-25
///
/// \copyright Copyright (c) 2024
///
///
#include "turtlelib/se2d.hpp"
#include "catch2/catch_all.hpp"

#define TOLERANCE 1e-12

using Catch::Matchers::WithinAbs;

namespace turtlelib
{
TEST_CASE("Twist2D <<", "[twist2d]") // Allen Liu
{
  struct Twist2D test = {1.2, 1.4, 1.5};
  std::stringstream ss;
  ss << test;

  REQUIRE(ss.str() == "[1.2 1.4 1.5]");

  struct Twist2D test2 = {1.0, 2.0, 3.0};
  std::stringstream ss2;
  ss2 << test2;

  REQUIRE(ss2.str() == "[1 2 3]");
}

TEST_CASE("Twist2D >>", "[twist2d]") // Allen Liu
{
  std::string test1 = "1.0 2.2 3.0";
  std::string test2 = "[2.3 4.5 1.5]";

  std::stringstream ss1;
  std::stringstream ss2;

  ss1 << test1;
  ss2 << test2;

  struct Twist2D tw1;
  struct Twist2D tw2;

  ss1 >> tw1;
  ss2 >> tw2;

  REQUIRE_THAT(tw1.omega, WithinAbs(1.0, TOLERANCE));
  REQUIRE_THAT(tw1.x, WithinAbs(2.2, TOLERANCE));
  REQUIRE_THAT(tw1.y, WithinAbs(3.0, TOLERANCE));
  REQUIRE_THAT(tw2.omega, WithinAbs(2.3, TOLERANCE));
  REQUIRE_THAT(tw2.x, WithinAbs(4.5, TOLERANCE));
  REQUIRE_THAT(tw2.y, WithinAbs(1.5, TOLERANCE));
}

TEST_CASE("Transform2D()", "[transform]") // Allen Liu
{
  Transform2D tf;

  REQUIRE_THAT(tf.rotation(), WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf.translation().x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf.translation().y, WithinAbs(0.0, TOLERANCE));
}

TEST_CASE("Transform2D(trans)") // Allen Liu
{
  Vector2D v = {1.2, 2.3};
  Transform2D tf(v);

  REQUIRE_THAT(tf.rotation(), WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf.translation().x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(tf.translation().y, WithinAbs(2.3, TOLERANCE));
}

TEST_CASE("Transform2D(radian)", "[transform]") // Allen Liu
{
  Transform2D tf(PI);

  REQUIRE_THAT(tf.rotation(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(tf.translation().x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf.translation().y, WithinAbs(0.0, TOLERANCE));
}

TEST_CASE("Transform2D(trans, radian)", "[transform]") // Allen Liu
{
  Vector2D v = {1.2, 2.3};
  Transform2D tf(v, PI);

  REQUIRE_THAT(tf.rotation(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(tf.translation().x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(tf.translation().y, WithinAbs(2.3, TOLERANCE));
}

TEST_CASE("Tranform point", "[transform]") // Allen Liu
{
  Transform2D tf1;

  Point2D p = {1.2, 3.5};
  Point2D p1 = tf1(p);

  REQUIRE_THAT(p1.x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(p1.y, WithinAbs(3.5, TOLERANCE));

  Transform2D tf2(PI / 2);
  Point2D p2 = tf2(p);

  REQUIRE_THAT(p2.x, WithinAbs(-3.5, TOLERANCE));
  REQUIRE_THAT(p2.y, WithinAbs(1.2, TOLERANCE));
}

TEST_CASE("Transform vector", "[transform]") // Allen Liu
{
  Transform2D tf1;

  Vector2D v = {1.2, 3.5};
  Vector2D v1 = tf1(v);

  REQUIRE_THAT(v1.x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(v1.y, WithinAbs(3.5, TOLERANCE));

  Transform2D tf2(PI / 2);
  Vector2D v2 = tf2(v);

  REQUIRE_THAT(v2.x, WithinAbs(-3.5, TOLERANCE));
  REQUIRE_THAT(v2.y, WithinAbs(1.2, TOLERANCE));
}

TEST_CASE("Transform twist", "[transform]") // Allen Liu
{
  Twist2D tw = {PI, 1.2, 2.3};

  Transform2D tf1;
  Twist2D tw1 = tf1(tw);

  REQUIRE_THAT(tw1.omega, WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(tw1.x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(tw1.y, WithinAbs(2.3, TOLERANCE));

  Vector2D v2 = {2.3, 3.3};
  Transform2D tf2(v2);
  Twist2D tw2 = tf2(tw);

  REQUIRE_THAT(tw2.omega, WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(tw2.x, WithinAbs(11.5673, 1e-4));
  REQUIRE_THAT(tw2.y, WithinAbs(-4.9257, 1e-4));
}

TEST_CASE("Transform inverse", "[transform]") // Allen Liu
{
  Transform2D tf1;
  Transform2D tf1_inv = tf1.inv();

  REQUIRE_THAT(tf1_inv.rotation(), WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf1_inv.translation().x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf1_inv.translation().y, WithinAbs(0.0, TOLERANCE));

  Transform2D tf2(PI);
  Transform2D tf2_inv = tf2.inv();

  REQUIRE_THAT(tf2_inv.rotation(), WithinAbs(-PI, TOLERANCE));
  REQUIRE_THAT(tf2_inv.translation().x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf2_inv.translation().y, WithinAbs(0.0, TOLERANCE));
}

TEST_CASE("Transform *=", "[transform]") // Allen Liu
{
  Transform2D tf1;
  Transform2D tf2(PI);

  tf1 *= tf2;
  REQUIRE_THAT(tf1.rotation(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(tf1.translation().x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf1.translation().y, WithinAbs(0.0, TOLERANCE));

  Vector2D v3 = {1.2, 4.5};
  Vector2D v4 = {3.4, 2.2};

  Transform2D tf3(v3, 1.23);
  Transform2D tf4(v4, 2.35);

  tf3 *= tf4;

  REQUIRE_THAT(tf3.rotation(), WithinAbs(-2.7032, 1e-4));
  REQUIRE_THAT(tf3.translation().x, WithinAbs(0.2629, 1e-4));
  REQUIRE_THAT(tf3.translation().y, WithinAbs(8.4398, 1e-4));

  Vector2D v5 = {1.2, 2.3};
  Vector2D v6 = {3.5, 4.5};

  Transform2D tf5(v5, PI / 2.0);
  Transform2D tf6(v6, PI / 2.0);

  tf5 *= tf6;

  REQUIRE_THAT(tf5.rotation(), WithinAbs(3.1416, 1e-4));
  REQUIRE_THAT(tf5.translation().x, WithinAbs(-3.3, 1e-4));
  REQUIRE_THAT(tf5.translation().y, WithinAbs(5.8, 1e-4));
}

TEST_CASE("Transform rotation", "[transform]") // Allen Liu
{
  Transform2D tf1;
  Transform2D tf2(PI);
  Transform2D tf3(1.34);

  Vector2D v4 = {1.2, 3.4};
  Transform2D tf4(v4);

  REQUIRE_THAT(tf1.rotation(), WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf2.rotation(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(tf3.rotation(), WithinAbs(1.34, TOLERANCE));
  REQUIRE_THAT(tf4.rotation(), WithinAbs(0.0, TOLERANCE));
}

TEST_CASE("Transform translation", "[transform]") // Allen Liu
{
  Transform2D tf1;

  Vector2D v2 = {1.2, 3.4};
  Vector2D v3 = {3.4, 5.2};

  Transform2D tf2(v2, PI);
  Transform2D tf3(v3);
  Transform2D tf4(PI / 2.0);

  REQUIRE_THAT(tf1.translation().x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf1.translation().y, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf2.translation().x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(tf2.translation().y, WithinAbs(3.4, TOLERANCE));
  REQUIRE_THAT(tf3.translation().x, WithinAbs(3.4, TOLERANCE));
  REQUIRE_THAT(tf3.translation().y, WithinAbs(5.2, TOLERANCE));
  REQUIRE_THAT(tf4.translation().x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf4.translation().y, WithinAbs(0.0, TOLERANCE));
}

TEST_CASE("Transform >>", "[transform]") // Allen Liu
{
  Transform2D tf1;
  Transform2D tf2(1.28);

  Vector2D v3 = {1.2, 3.3};
  Transform2D tf3(v3, 1.45);

  Vector2D v4 = {1.8, 2.2};
  Transform2D tf4(v4);

  std::stringstream ss1;
  std::stringstream ss2;
  std::stringstream ss3;
  std::stringstream ss4;

  ss1 << tf1;
  ss2 << tf2;
  ss3 << tf3;
  ss4 << tf4;

  REQUIRE(ss1.str() == "deg: 0 x: 0 y: 0");
  REQUIRE(ss2.str() == "deg: 73.3386 x: 0 y: 0");
  REQUIRE(ss3.str() == "deg: 83.0789 x: 1.2 y: 3.3");
  REQUIRE(ss4.str() == "deg: 0 x: 1.8 y: 2.2");
}

TEST_CASE("Transform <<", "[transform]") // Allen Liu
{
  std::stringstream ss1("90 1.2 2.3");
  std::stringstream ss2("180\n2.2\n3.2");

  Transform2D tf1;
  Transform2D tf2;

  ss1 >> tf1;
  ss2 >> tf2;

  REQUIRE_THAT(tf1.rotation(), WithinAbs(PI / 2.0, TOLERANCE));
  REQUIRE_THAT(tf1.translation().x, WithinAbs(1.2, TOLERANCE));
  REQUIRE_THAT(tf1.translation().y, WithinAbs(2.3, TOLERANCE));

  REQUIRE_THAT(tf2.rotation(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(tf2.translation().x, WithinAbs(2.2, TOLERANCE));
  REQUIRE_THAT(tf2.translation().y, WithinAbs(3.2, TOLERANCE));
}

TEST_CASE("Transform *", "[transform]") // Allen Liu
{
  Transform2D tf1;
  Transform2D tf2(PI);

  Transform2D tf12 = tf1 * tf2;
  REQUIRE_THAT(tf12.rotation(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(tf12.translation().x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf12.translation().y, WithinAbs(0.0, TOLERANCE));

  Vector2D v3 = {1.2, 4.5};
  Vector2D v4 = {3.4, 2.2};

  Transform2D tf3(v3, 1.23);
  Transform2D tf4(v4, 2.35);

  Transform2D tf34 = tf3 * tf4;

  REQUIRE_THAT(tf34.rotation(), WithinAbs(-2.7032, 1e-4));
  REQUIRE_THAT(tf34.translation().x, WithinAbs(0.2629, 1e-4));
  REQUIRE_THAT(tf34.translation().y, WithinAbs(8.4398, 1e-4));

  Vector2D v5 = {1.2, 2.3};
  Vector2D v6 = {3.5, 4.5};

  Transform2D tf5(v5, PI / 2.0);
  Transform2D tf6(v6, PI / 2.0);

  Transform2D tf56 = tf5 * tf6;

  REQUIRE_THAT(tf56.rotation(), WithinAbs(3.1416, 1e-4));
  REQUIRE_THAT(tf56.translation().x, WithinAbs(-3.3, 1e-4));
  REQUIRE_THAT(tf56.translation().y, WithinAbs(5.8, 1e-4));
}

// TEST_CASE("Pure Translation", "[int_twist]")   // Allen Liu
// {
//   Twist2D tw1{0.0, 1.0, 1.2};
//   Twist2D tw2{0.0, 2.3, 4.4};

//   Point2D p1{1.2, 3.4};
//   Point2D p2{3.4, 5.5};

//   Transform2D tf1 = integrate_twist(tw1);
//   Transform2D tf2 = integrate_twist(tw2);

//   Point2D p1_tf1 = tf1(p1);
//   Point2D p2_tf1 = tf1(p2);
//   Point2D p1_tf2 = tf2(p1);
//   Point2D p2_tf2 = tf2(p2);

//   REQUIRE_THAT(p1_tf1.x, WithinAbs(2.2, TOLERANCE));
//   REQUIRE_THAT(p1_tf1.y, WithinAbs(4.6, TOLERANCE));
//   REQUIRE_THAT(p1_tf2.x, WithinAbs(3.5, TOLERANCE));
//   REQUIRE_THAT(p1_tf2.y, WithinAbs(7.8, TOLERANCE));

//   REQUIRE_THAT(p2_tf1.x, WithinAbs(4.4, TOLERANCE));
//   REQUIRE_THAT(p2_tf1.y, WithinAbs(6.7, TOLERANCE));
//   REQUIRE_THAT(p2_tf2.x, WithinAbs(5.7, TOLERANCE));
//   REQUIRE_THAT(p2_tf2.y, WithinAbs(9.9, TOLERANCE));
// }

// TEST_CASE("Pure Rotation", "[int_twist]")
// {
//   Twist2D tw1{deg2rad(90.0), 0, 0};
//   Twist2D tw2{deg2rad(-180.0), 0, 0};

//   Point2D p1{1.2, 4.5};
//   Point2D p2{4.2, 5.5};

//   Transform2D tf1 = integrate_twist(tw1);
//   Transform2D tf2 = integrate_twist(tw2);

//   Point2D p1_tf1 = tf1(p1);
//   Point2D p2_tf1 = tf1(p2);
//   Point2D p1_tf2 = tf2(p1);
//   Point2D p2_tf2 = tf2(p2);

//   REQUIRE_THAT(p1_tf1.x, WithinAbs(-4.5, TOLERANCE));
//   REQUIRE_THAT(p1_tf1.y, WithinAbs(1.2, TOLERANCE));

//   REQUIRE_THAT(p1_tf2.x, WithinAbs(-1.2, TOLERANCE));
//   REQUIRE_THAT(p1_tf2.y, WithinAbs(-4.5, TOLERANCE));

//   REQUIRE_THAT(p2_tf1.x, WithinAbs(-5.5, TOLERANCE));
//   REQUIRE_THAT(p2_tf1.y, WithinAbs(4.2, TOLERANCE));

//   REQUIRE_THAT(p2_tf2.x, WithinAbs(-4.2, TOLERANCE));
//   REQUIRE_THAT(p2_tf2.y, WithinAbs(-5.5, TOLERANCE));
// }

// TEST_CASE("Translation and rotation", "[int_twist]")
// {
//   Twist2D tw1{deg2rad(90.0), 4.5, 2.1};
//   Twist2D tw2{deg2rad(-180.0), -3.4, 2.3};

//   Point2D p1{1.2, 4.5};
//   Point2D p2{4.2, 5.5};

//   Transform2D tf1 = integrate_twist(tw1);
//   Transform2D tf2 = integrate_twist(tw2);

//   Point2D p1_tf1 = tf1(p1);
//   Point2D p2_tf1 = tf1(p2);
//   Point2D p1_tf2 = tf2(p1);
//   Point2D p2_tf2 = tf2(p2);

//   REQUIRE_THAT(p1_tf1.x, WithinAbs(0.0, TOLERANCE));
//   REQUIRE_THAT(p1_tf1.y, WithinAbs(3.3, TOLERANCE));

//   REQUIRE_THAT(p1_tf2.x, WithinAbs(-4.6, TOLERANCE));
//   REQUIRE_THAT(p1_tf2.y, WithinAbs(-2.2, TOLERANCE));

//   REQUIRE_THAT(p2_tf1.x, WithinAbs(-1.0, TOLERANCE));
//   REQUIRE_THAT(p2_tf1.y, WithinAbs(6.3, TOLERANCE));

//   REQUIRE_THAT(p2_tf2.x, WithinAbs(-7.6, TOLERANCE));
//   REQUIRE_THAT(p2_tf2.y, WithinAbs(-3.2, TOLERANCE));
// }
TEST_CASE("Integrate Twist Pure Translation", "[integrate_twist]")
{
  Transform2D tf1 = integrate_twist(Twist2D{0, 1.0, 2.0});

  REQUIRE_THAT(tf1.rotation(), WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf1.translation().x, WithinAbs(1.0, TOLERANCE));
  REQUIRE_THAT(tf1.translation().y, WithinAbs(2.0, TOLERANCE));

  Transform2D tf2 = integrate_twist(Twist2D{0, 3.4, 4.2});

  REQUIRE_THAT(tf2.rotation(), WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf2.translation().x, WithinAbs(3.4, TOLERANCE));
  REQUIRE_THAT(tf2.translation().y, WithinAbs(4.2, TOLERANCE));
}

TEST_CASE("Integrate Twist Pure Rotation", "[integrate_twist]")
{
  Transform2D tf1 = integrate_twist(Twist2D{PI, 0.0, 0.0});

  REQUIRE_THAT(tf1.rotation(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(tf1.translation().x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf1.translation().y, WithinAbs(0.0, TOLERANCE));

  Transform2D tf2 = integrate_twist(Twist2D{-PI / 2.0, 0.0, 0.0});

  REQUIRE_THAT(tf2.rotation(), WithinAbs(-PI / 2.0, TOLERANCE));
  REQUIRE_THAT(tf2.translation().x, WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(tf2.translation().y, WithinAbs(0.0, TOLERANCE));
}

TEST_CASE("Integrate Twist Rotation and Translation", "[integrate_twist]")
{
  Transform2D tf1 = integrate_twist(Twist2D{PI, 1.2, 4.6});

  REQUIRE_THAT(tf1.rotation(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(tf1.translation().x, WithinAbs(-2.9285, 1e-4));
  REQUIRE_THAT(tf1.translation().y, WithinAbs(0.7639, 1e-4));

  Transform2D tf2 = integrate_twist(Twist2D{0.6480, 0.1555, 0.0});

  REQUIRE_THAT(tf2.rotation(), WithinAbs(0.6480, 1e-4));
  REQUIRE_THAT(tf2.translation().x, WithinAbs(0.1448, 1e-4));
  REQUIRE_THAT(tf2.translation().y, WithinAbs(0.0486, 1e-4));
}
}
