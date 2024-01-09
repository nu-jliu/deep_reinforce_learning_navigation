#include "turtlelib/se2d.hpp"
#include "catch2/catch_all.hpp"

using namespace turtlelib;

TEST_CASE("Twist2D <<", "[twist2d]")
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
