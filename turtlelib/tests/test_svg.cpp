#include "turtlelib/svg.hpp"
#include "catch2/catch_all.hpp"

using namespace turtlelib;

TEST_CASE("Testing line svg writen to file", "[svg]")
{
    Svg svg("svg_line.svg");
    Vector2D v1 = {20.3, 30.5};
    Vector2D v2 = {50.2, 10.3};
    Vector2D v3 = {-10.3, 30.5};
    Vector2D v4 = {-20.9, -80.3};

    Point2D tail = {500.0, 500.0};

    svg.draw_line(tail, v1, "blue");
    svg.draw_line(tail, v2, "red");
    svg.draw_line(tail, v3, "green");
    svg.draw_line(tail, v4, "purple");
    svg.finish();
}

TEST_CASE("Testing point svg writen to a file", "[svg]")
{
    Svg svg("svg_point.svg");
    Point2D p1 = {200.2, 300.3};
    Point2D p2 = {150.2, 100.3};
    Point2D p3 = {500.2, 235.5};

    svg.draw_point(p1, "red");
    svg.draw_point(p2, "green");
    svg.draw_point(p3, "blue");
    svg.finish();
}

TEST_CASE("Testing frame svg writen to a file", "[svg]")
{
    Svg svg("svg_frame.svg");

    Transform2D tf1 = {Vector2D{100.0, 100.0}, PI};
    Transform2D tf2 = {Vector2D{-200.0, -350.0}, PI / 2.0};

    svg.draw_frame(tf1, "{a}");
    svg.draw_frame(tf2, "{b}");

    svg.finish();
}