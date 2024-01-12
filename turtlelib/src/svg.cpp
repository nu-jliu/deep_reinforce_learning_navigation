#include <iostream>
#include <fstream>

#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    Svg::Svg()
    {
        __ofs.open("default.svg");
        init_svg();
    }

    Svg::Svg(const std::string filename)
    {
        __ofs.open(filename);
        init_svg();
    }

    void Svg::draw_line(Point2D tail, Vector2D v, std::string color)
    {
        Point2D head = tail + v;

        Point2D head_tf = __point_tf(head);
        Point2D tail_tf = __point_tf(tail);

        if (__ofs.is_open())
        {
            __ofs << "<line x1=\"" << head_tf.x << "\" x2=\"" << tail_tf.x << "\" ";
            __ofs << "y1=\"" << head_tf.y << "\" y2=\"" << tail_tf.y << "\" ";
            __ofs << "stroke=\"" << color;
            __ofs << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
            __ofs << std::endl;
        }
    }

    void Svg::draw_point(Point2D p, int radius, std::string color)
    {
        Point2D p_tf = __point_tf(p);

        if (__ofs.is_open())
        {
            __ofs << "<circle cx=\"";
            __ofs << p_tf.x << "\" ";
            __ofs << "cy=\"" << p_tf.y << "\" ";
            __ofs << "r=\"" << radius << "\" ";
            __ofs << "stroke=\"" << color << "\" fill=\"" << color << "\" stroke-width=\"1\" />";
            __ofs << std::endl;
        }
    }

    void Svg::draw_frame(Transform2D tf, std::string name)
    {
        Vector2D v_x = {-70.0, 70.0};
        Vector2D v_y = {100.0, 0.0};
        Vector2D v_z = {0.0, -100.0};

        // double origin_x = tf.translation().x + __origin_x;
        // double origin_y = tf.translation().y - __origin_y;

        // Point2D origin{origin_x, origin_y};
        Point2D origin{tf.translation().x, tf.translation().y};
        Point2D origin_tf = __point_tf(origin);

        v_x = tf(v_x);
        v_y = tf(v_y);
        v_z = tf(v_z);

        Svg::draw_line(origin, v_x, "red");
        Svg::draw_line(origin, v_y, "green");
        Svg::draw_line(origin, v_z, "blue");

        if (__ofs.is_open())
        {
            __ofs << "<text x=\"" << origin_tf.x << "\" ";
            __ofs << "y=\"" << origin_tf.y << "\"";
            __ofs << ">" << name << "</text>";
            __ofs << std::endl;
        }
    }

    void Svg::finish()
    {
        __ofs << "</svg>";
        __ofs.close();
    }

};