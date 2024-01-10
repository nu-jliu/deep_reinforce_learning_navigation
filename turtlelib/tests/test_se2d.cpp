#include "turtlelib/se2d.hpp"
#include "catch2/catch_all.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;

double tolerance = 1e-12;

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

    REQUIRE_THAT(tw1.omega, WithinRel(1.0, tolerance));
    REQUIRE_THAT(tw1.x, WithinRel(2.2, tolerance));
    REQUIRE_THAT(tw1.y, WithinRel(3.0, tolerance));
    REQUIRE_THAT(tw2.omega, WithinRel(2.3, tolerance));
    REQUIRE_THAT(tw2.x, WithinRel(4.5, tolerance));
    REQUIRE_THAT(tw2.y, WithinRel(1.5, tolerance));
}

TEST_CASE("Transform2D()", "[transform]") // Allen Liu
{
    Transform2D tf;

    REQUIRE_THAT(tf.rotation(), WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf.translation().x, WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf.translation().y, WithinRel(0.0, tolerance));
}

TEST_CASE("Transform2D(trans)") // Allen Liu
{
    Vector2D v = {1.2, 2.3};
    Transform2D tf(v);

    REQUIRE_THAT(tf.rotation(), WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf.translation().x, WithinRel(1.2, tolerance));
    REQUIRE_THAT(tf.translation().y, WithinRel(2.3, tolerance));
}

TEST_CASE("Transform2D(radian)", "[transform]") // Allen Liu
{
    Transform2D tf(PI);

    REQUIRE_THAT(tf.rotation(), WithinRel(PI, tolerance));
    REQUIRE_THAT(tf.translation().x, WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf.translation().y, WithinRel(0.0, tolerance));
}

TEST_CASE("Transform2D(trans, radian)", "[transform]") // Allen Liu
{
    Vector2D v = {1.2, 2.3};
    Transform2D tf(v, PI);

    REQUIRE_THAT(tf.rotation(), WithinRel(PI, tolerance));
    REQUIRE_THAT(tf.translation().x, WithinRel(1.2, tolerance));
    REQUIRE_THAT(tf.translation().y, WithinRel(2.3, tolerance));
}

TEST_CASE("Tranform point", "[transform]") // Allen Liu
{
    Transform2D tf1;

    Point2D p = {1.2, 3.5};
    Point2D p1 = tf1(p);

    REQUIRE_THAT(p1.x, WithinRel(1.2, tolerance));
    REQUIRE_THAT(p1.y, WithinRel(3.5, tolerance));

    Transform2D tf2(PI / 2);
    Point2D p2 = tf2(p);

    REQUIRE_THAT(p2.x, WithinRel(-3.5, tolerance));
    REQUIRE_THAT(p2.y, WithinRel(1.2, tolerance));
}

TEST_CASE("Transform vector", "[transform]") // Allen Liu
{
    Transform2D tf1;

    Vector2D v = {1.2, 3.5};
    Vector2D v1 = tf1(v);

    REQUIRE_THAT(v1.x, WithinRel(1.2, tolerance));
    REQUIRE_THAT(v1.y, WithinRel(3.5, tolerance));

    Transform2D tf2(PI / 2);
    Vector2D v2 = tf2(v);

    REQUIRE_THAT(v2.x, WithinRel(-3.5, tolerance));
    REQUIRE_THAT(v2.y, WithinRel(1.2, tolerance));
}

TEST_CASE("Transform twist", "[transform]") // Allen Liu
{
    Twist2D tw = {PI, 1.2, 2.3};

    Transform2D tf1;
    Twist2D tw1 = tf1(tw);

    REQUIRE_THAT(tw1.omega, WithinRel(PI, tolerance));
    REQUIRE_THAT(tw1.x, WithinRel(1.2, tolerance));
    REQUIRE_THAT(tw1.y, WithinRel(2.3, tolerance));

    Vector2D v2 = {2.3, 3.3};
    Transform2D tf2(v2);
    Twist2D tw2 = tf2(tw);

    REQUIRE_THAT(tw2.omega, WithinRel(PI, tolerance));
    REQUIRE_THAT(tw2.x, WithinRel(3.5, tolerance));
    REQUIRE_THAT(tw2.y, WithinRel(5.6, tolerance));
}

TEST_CASE("Transform inverse", "[transform]") // Allen Liu
{
    Transform2D tf1;
    Transform2D tf1_inv = tf1.inv();

    REQUIRE_THAT(tf1_inv.rotation(), WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf1_inv.translation().x, WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf1_inv.translation().y, WithinRel(0.0, tolerance));

    Transform2D tf2(PI);
    Transform2D tf2_inv = tf2.inv();

    REQUIRE_THAT(tf2_inv.rotation(), WithinRel(-PI, tolerance));
    REQUIRE_THAT(tf2_inv.translation().x, WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf2_inv.translation().y, WithinRel(0.0, tolerance));
}

TEST_CASE("Transform *=", "[transform]") // Allen Liu
{
    Transform2D tf1;
    Transform2D tf2(PI);

    tf1 *= tf2;
    REQUIRE_THAT(tf1.rotation(), WithinRel(PI, tolerance));
    REQUIRE_THAT(tf1.translation().x, WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf1.translation().y, WithinRel(0.0, tolerance));

    Vector2D v3 = {1.2, 4.5};
    Vector2D v4 = {3.4, 2.2};

    Transform2D tf3(v3, 1.23);
    Transform2D tf4(v4, 2.35);

    tf3 *= tf4;

    REQUIRE_THAT(tf3.rotation(), WithinRel(-2.7032, 1e-2));
    REQUIRE_THAT(tf3.translation().x, WithinRel(0.2629, 1e-2));
    REQUIRE_THAT(tf3.translation().y, WithinRel(8.4398, 1e-2));
}

TEST_CASE("Transform rotation", "[transform]") // Allen Liu
{
    Transform2D tf1;
    Transform2D tf2(PI);
    Transform2D tf3(1.34);

    Vector2D v4 = {1.2, 3.4};
    Transform2D tf4(v4);

    REQUIRE_THAT(tf1.rotation(), WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf2.rotation(), WithinRel(PI, tolerance));
    REQUIRE_THAT(tf3.rotation(), WithinRel(1.34, tolerance));
    REQUIRE_THAT(tf4.rotation(), WithinRel(0.0, tolerance));
}

TEST_CASE("Transform translation", "[transform]") // Allen Liu
{
    Transform2D tf1;

    Vector2D v2 = {1.2, 3.4};
    Vector2D v3 = {3.4, 5.2};

    Transform2D tf2(v2, PI);
    Transform2D tf3(v3);
    Transform2D tf4(PI / 2.0);

    REQUIRE_THAT(tf1.translation().x, WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf1.translation().y, WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf2.translation().x, WithinRel(1.2, tolerance));
    REQUIRE_THAT(tf2.translation().y, WithinRel(3.4, tolerance));
    REQUIRE_THAT(tf3.translation().x, WithinRel(3.4, tolerance));
    REQUIRE_THAT(tf3.translation().y, WithinRel(5.2, tolerance));
    REQUIRE_THAT(tf4.translation().x, WithinRel(0.0, tolerance));
    REQUIRE_THAT(tf4.translation().y, WithinRel(0.0, tolerance));
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
    std::string s1 = "deg: 90 x: 1.2 y: 2.3";
    std::string s2 = "[deg 180 x: 2.2 y: 3.2]";

    std::stringstream ss1;
    std::stringstream ss2;

    ss1 << s1;
    ss2 << s2;

    Transform2D tf1;
    Transform2D tf2;

    ss1 >> tf1;
    ss2 >> tf2;

    REQUIRE_THAT(tf1.rotation(), WithinRel(PI / 2.0, tolerance));
    REQUIRE_THAT(tf1.translation().x, WithinRel(1.2, tolerance));
    REQUIRE_THAT(tf1.translation().y, WithinRel(2.3, tolerance));

    REQUIRE_THAT(tf2.rotation(), WithinRel(PI, tolerance));
    REQUIRE_THAT(tf2.translation().x, WithinRel(2.2, tolerance));
    REQUIRE_THAT(tf2.translation().y, WithinRel(3.2, tolerance));
}