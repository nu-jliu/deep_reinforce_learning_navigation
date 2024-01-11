#include <iostream>
#include <cstdlib>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

using namespace turtlelib;

int main(int argc, char **argv)
{
    double tab_rot;
    Vector2D trans;
    Svg svg_output("svg_main.svg");

    std::cout << "Please input the Tab rotation in radians:" << std::endl;
    std::cin >> tab_rot;
    std::cout << "Please input the Tab translation seperated by space" << std::endl;
    std::cin >> trans;

    Transform2D tf_ab(trans, tab_rot);
    svg_output.draw_frame(tf_ab, "{b}");

    return EXIT_SUCCESS;
}