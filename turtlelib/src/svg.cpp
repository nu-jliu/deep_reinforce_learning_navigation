#include <iostream>
#include <fstream>

#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    Svg::Svg()
    {
        __ofs.open("default.svg", std::ios::out | std::ios::app | std::ios::trunc);
        init_svg();
    }

    Svg::Svg(const std::string filename)
    {
        __ofs.open(filename, std::ios::out | std::ios::app | std::ios::trunc);
        init_svg();
    }

    void Svg::draw_line(Vector2D v, std::string color)
    {
        Point2D tail = {__origin_x, __origin_y};
        Point2D head = tail + v;

        if (__ofs.is_open())
        {
            __ofs << "<line x1=\"" << head.x << "\" x2=\"" << tail.x << "\" ";
            __ofs << "y1=\"" << head.y << "\" y2=\"" << tail.y << "\" ";
            __ofs << "stroke=\"" << color;
            __ofs << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
            __ofs << std::endl;
        }
    }

    void Svg::draw_point(Point2D p, std::string color)
    {
        Point2D p_tf = __tf_origin(p);

        if (__ofs.is_open())
        {
            __ofs << "<! -- Test -- >" << std::endl;
        }
    }

    void Svg::finish()
    {
        __ofs << "</svg>" << std::ends;
        __ofs.close();
    }

};