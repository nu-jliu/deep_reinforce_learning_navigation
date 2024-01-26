///
/// \file test_svg.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief Test svg display functionalities
/// \version 0.1
/// \date 2024-01-25
///
/// \copyright Copyright (c) 2024
///
///
#include "turtlelib/svg.hpp"
#include "catch2/catch_all.hpp"

namespace turtlelib
{
TEST_CASE("Testing line svg writen to file", "[svg]") // Allen Liu
{
  Svg svg("svg_line.svg");
  Transform2D frame;
  Vector2D v1 = {2.0, 3.0};
  Vector2D v2 = {5.0, 1.0};
  Vector2D v3 = {-1.0, 3.0};
  Vector2D v4 = {-2.0, -2.5};

  Point2D tail = {1.0, -2.0};

  svg.draw_line(frame, tail, v1, "blue");
  svg.draw_line(frame, tail, v2, "red");
  svg.draw_line(frame, tail, v3, "green");
  svg.draw_line(frame, tail, v4, "purple");
  svg.finish();
}

TEST_CASE("Testing point svg writen to a file", "[svg]") // Allen Liu
{
  Svg svg("svg_point.svg");
  Transform2D frame;
  Point2D p1 = {2.2, 1.3};
  Point2D p2 = {1.52, -1.3};
  Point2D p3 = {-0.52, -1.35};

  svg.draw_point(frame, p1, 3, "red");
  svg.draw_point(frame, p2, 5, "green");
  svg.draw_point(frame, p3, 7, "blue");
  svg.finish();
}

TEST_CASE("Testing frame svg writen to a file", "[svg]") // Allen Liu
{
  Svg svg("svg_frame.svg");

  Transform2D tf1 = {Vector2D{1.0, 1.0}, PI};
  Transform2D tf2 = {Vector2D{-1.0, -1.5}, PI / 2.0};

  svg.draw_frame(tf1, "{a}");
  svg.draw_frame(tf2, "{b}");

  svg.finish();
}
}
