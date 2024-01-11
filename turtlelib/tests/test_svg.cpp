#include "turtlelib/svg.hpp"
#include "catch2/catch_all.hpp"

using namespace turtlelib;

TEST_CASE("Testing svg writen to file", "[svg]")
{
    Svg svg;
    Vector2D v = {20.3, 30.5};

    svg.draw_line(v, "blue");
    svg.finish();
}