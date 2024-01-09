#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_all.hpp>

using namespace turtlelib;
using namespace Catch::Matchers;

double tolerance = 1e-12;

TEST_CASE("Almost Equal", "[almost_equal]")
{
    REQUIRE(almost_equal(2.0, 2.0 + 1e-15));
    REQUIRE(almost_equal(-5.3, -5.3 - 1e-13));
    REQUIRE_FALSE(almost_equal(1.1, 1.2));
    REQUIRE_FALSE(almost_equal(1.2, 1.2 + 1e-5));
}

TEST_CASE("Degree to radian", "[deg2rad]")
{
    REQUIRE_THAT(deg2rad(360), WithinRel(2.0 * turtlelib::PI, tolerance));
    REQUIRE_THAT(deg2rad(180), WithinRel(turtlelib::PI, tolerance));
    REQUIRE_THAT(deg2rad(135), WithinRel(turtlelib::PI * 0.75, tolerance));
    REQUIRE_THAT(deg2rad(90), WithinRel(turtlelib::PI / 2.0, tolerance));
    REQUIRE_THAT(deg2rad(45), WithinRel(turtlelib::PI / 4.0, tolerance));
}

TEST_CASE("Radian to degree", "[rad2deg]")
{
    REQUIRE_THAT(rad2deg(PI), WithinRel(180.0, tolerance));
    REQUIRE_THAT(rad2deg(PI / 2.0), WithinRel(90.0, tolerance));
    REQUIRE_THAT(rad2deg(PI * 2.0), WithinRel(360.0, tolerance));
}

TEST_CASE("Normalize angle", "[normalize_angle]")
{
    REQUIRE_THAT(normalize_angle(PI), WithinRel(PI, tolerance));
    REQUIRE_THAT(normalize_angle(-PI), WithinRel(PI, tolerance));
    REQUIRE_THAT(normalize_angle(0), WithinRel(0, tolerance));
    REQUIRE_THAT(normalize_angle(-PI / 4.0), WithinRel(-PI / 4.0, tolerance));
    REQUIRE_THAT(normalize_angle(3.0 / 2.0 * PI), WithinRel(-PI / 2.0, tolerance));
    REQUIRE_THAT(normalize_angle(-5.0 * PI / 2.0), WithinRel(-PI / 2.0, tolerance));
}

TEST_CASE("Point2D stream", "[point2d]")
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

TEST_CASE("Point2D read", "[point2d]")
{
    struct Point2D test;
    struct Point2D sample;
    sample.x = 1.2;
    sample.y = 2.5;
    std::stringstream ss;

    ss << sample;
    ss >> test;
    REQUIRE_THAT(test.x, WithinRel(1.2, tolerance));
    REQUIRE_THAT(test.y, WithinRel(2.5, tolerance));
}

TEST_CASE("Point subs", "[vector]")
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

    REQUIRE_THAT(v12.x, WithinRel(3.3, tolerance));
    REQUIRE_THAT(v12.y, WithinRel(3.2, tolerance));
    REQUIRE_THAT(v23.x, WithinRel(-5.8, tolerance));
    REQUIRE_THAT(v23.y, WithinRel(-8.0, tolerance));
    REQUIRE_THAT(v13.x, WithinRel(-2.5, tolerance));
    REQUIRE_THAT(v13.y, WithinRel(-4.8, tolerance));
}

TEST_CASE("Point adds", "[vector]")
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

    REQUIRE_THAT(p1.x, WithinRel(2.4, tolerance));
    REQUIRE_THAT(p1.y, WithinRel(1.0, tolerance));
    REQUIRE_THAT(p2.x, WithinRel(2.6, tolerance));
    REQUIRE_THAT(p2.y, WithinRel(5.2, tolerance));
    REQUIRE_THAT(p3.x, WithinRel(-1.0, tolerance));
    REQUIRE_THAT(p3.y, WithinRel(-2.2, tolerance));
}

TEST_CASE("Vector2D stream", "[vector2d]")
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

TEST_CASE("Vector2D read", "[vector2d]")
{
    struct Vector2D test;
    struct Vector2D sample;
    sample.x = 1.2;
    sample.y = 2.5;
    std::stringstream ss;

    ss << sample;
    ss >> test;
    REQUIRE_THAT(test.x, WithinRel(1.2, tolerance));
    REQUIRE_THAT(test.y, WithinRel(2.5, tolerance));
}
