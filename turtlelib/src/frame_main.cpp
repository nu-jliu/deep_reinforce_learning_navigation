///
/// \file frame_main.cpp
/// \author your name (you@domain.com)
/// \brief
/// \version 0.1
/// \date 2024-01-12
///
/// \copyright Copyright (c) 2024
///
///
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

using namespace turtlelib;

/// @brief
/// @return
int main()
{
    Transform2D tf_aa;
    Transform2D tf_ab;
    Transform2D tf_bc;
    Point2D p_a;
    Vector2D v_b;
    Twist2D V_b;
    Svg svg_output("/tmp/frames.svg");

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> tf_ab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> tf_bc;

    Transform2D tf_ba = tf_ab.inv();
    Transform2D tf_cb = tf_bc.inv();
    Transform2D tf_ac = tf_ab * tf_bc;
    Transform2D tf_ca = tf_cb * tf_ba;

    std::cout << "T_{a,b}: " << tf_ab << std::endl;
    std::cout << "T_{b,a}: " << tf_ba << std::endl;
    std::cout << "T_{b,c}: " << tf_bc << std::endl;
    std::cout << "T_{c,b}: " << tf_cb << std::endl;
    std::cout << "T_{a,c}: " << tf_ac << std::endl;
    std::cout << "T_{c,a}: " << tf_ca << std::endl;

    svg_output.draw_frame(tf_aa, "{a}");
    svg_output.draw_frame(tf_ab, "{b}");
    svg_output.draw_frame(tf_ac, "{c}");

    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> p_a;

    Point2D p_b = tf_ba(p_a);
    Point2D p_c = tf_ca(p_a);

    std::cout << "p_a: " << p_a << std::endl;
    std::cout << "p_b: " << p_b << std::endl;
    std::cout << "p_c: " << p_c << std::endl;

    svg_output.draw_point(tf_aa, p_a, 5, "purple");
    svg_output.draw_point(tf_ab, p_b, 5, "brown");
    svg_output.draw_point(tf_ac, p_c, 5, "orange");

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;

    double mag_vb = sqrt(pow(v_b.x, 2.0) + pow(v_b.y, 2.0));
    Vector2D v_bhat = {v_b.x / mag_vb, v_b.y / mag_vb};

    Vector2D v_a = tf_ab(v_b);
    Vector2D v_c = tf_cb(v_b);

    std::cout << "v_bhat: " << v_bhat << std::endl;
    std::cout << "v_a: " << v_a << std::endl;
    std::cout << "v_b: " << v_b << std::endl;
    std::cout << "v_c: " << v_c << std::endl;

    svg_output.draw_line(tf_ab, Point2D{0.0, 0.0}, v_bhat, "brown");
    svg_output.draw_line(tf_aa, Point2D{0.0, 0.0}, v_a, "purple");
    svg_output.draw_line(tf_ac, Point2D{0.0, 0.0}, v_c, "orange");

    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> V_b;

    Twist2D V_a = tf_ab(V_b);
    Twist2D V_c = tf_cb(V_b);

    std::cout << "V_a: " << V_a << std::endl;
    std::cout << "V_b: " << V_b << std::endl;
    std::cout << "V_c: " << V_c << std::endl;

    svg_output.finish();
    return EXIT_SUCCESS;
}
