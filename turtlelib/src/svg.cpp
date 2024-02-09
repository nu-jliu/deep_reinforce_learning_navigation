///
/// \file svg.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu.com)
/// \brief Integrate the functions to display the points and vectors on a svg file
/// \version 0.1
/// \date 2024-01-23
///
/// \copyright Copyright (c) 2024
///
///
#include <cmath>
#include <fstream>
#include <iostream>

#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

namespace turtlelib
{
Svg::Svg()
{
  ofs_.open("default.svg");
  init_svg();
}

Svg::Svg(const std::string filename)
{
  ofs_.open(filename);
  init_svg();
}

void Svg::init_svg()
{
  scale_ = 96.0;
  origin_x_ = 408.0;
  origin_y_ = 528.0;

  tf_origin_ = Transform2D(Vector2D{origin_x_, origin_y_}, 0.0);

  ofs_ <<
    "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">"
       << std::endl;
  ofs_ << "<defs>" << std::endl;
  ofs_ << "<marker" << std::endl;
  ofs_ << "\tstyle=\"overflow:visible\"" << std::endl;
  ofs_ << "\tid=\"Arrow1Sstart\"" << std::endl;
  ofs_ << "\trefX=\"0.0\"" << std::endl;
  ofs_ << "\trefY=\"0.0\"" << std::endl;
  ofs_ << "\torient=\"auto\">" << std::endl;
  ofs_ << "<path" << std::endl;
  ofs_ << "\ttransform=\"scale(0.2) translate(6,0)\"" << std::endl;
  ofs_ <<
    "\tstyle=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\""
       << std::endl;
  ofs_ << "\td=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"" << std::endl;
  ofs_ << "/>" << std::endl;
  ofs_ << "</marker>" << std::endl;
  ofs_ << "</defs>" << std::endl;

  ofs_ << "<circle cx=\"" << origin_x_;
  ofs_ << "\" cy=\"" << origin_y_;
  ofs_ << "\" r=\"5\" stroke=\"black\" fill=\"black\" stroke-width=\"1\" />";
  ofs_ << std::endl;
}

void Svg::draw_line(Transform2D frame, Point2D tail, Vector2D v, std::string color)
{
  tail = frame(tail);
  v = frame(v);
  Point2D head = tail + v;

  Point2D head_tf = point_tf_(head);
  Point2D tail_tf = point_tf_(tail);

  if (ofs_.is_open()) {
    ofs_ << "<line x1=\"" << head_tf.x << "\" x2=\"" << tail_tf.x << "\" ";
    ofs_ << "y1=\"" << head_tf.y << "\" y2=\"" << tail_tf.y << "\" ";
    ofs_ << "stroke=\"" << color;
    ofs_ << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
    ofs_ << std::endl;
  }
}

void Svg::draw_point(Transform2D frame, Point2D p, int radius, std::string color)
{
  p = frame(p);
  Point2D p_tf = point_tf_(p);

  if (ofs_.is_open()) {
    ofs_ << "<circle cx=\"";
    ofs_ << p_tf.x << "\" ";
    ofs_ << "cy=\"" << p_tf.y << "\" ";
    ofs_ << "r=\"" << radius << "\" ";
    ofs_ << "stroke=\"" << color << "\" fill=\"" << color
         << "\" stroke-width=\"1\" />";
    ofs_ << std::endl;
  }
}

void Svg::draw_frame(Transform2D tf, std::string name)
{
//   Vector2D v_x = {-sqrt(2.0) / 2.0, -sqrt(2.0) / 2.0};
  Vector2D v_x = {1.0, 0.0};
  Vector2D v_y = {0.0, 1.0};

  Point2D origin{tf.translation().x, tf.translation().y};
  Point2D origin_tf = point_tf_(origin);

  Svg::draw_line(tf, Point2D{0.0, 0.0}, v_x, "red");
  Svg::draw_line(tf, Point2D{0.0, 0.0}, v_y, "green");
//   Svg::draw_line(tf, Point2D{0.0, 0.0}, v_z, "blue");

  if (ofs_.is_open()) {
    ofs_ << "<text x=\"" << origin_tf.x << "\" ";
    ofs_ << "y=\"" << origin_tf.y << "\"";
    ofs_ << ">" << name << "</text>";
    ofs_ << std::endl;
  }
}

void Svg::finish()
{
  ofs_ << "</svg>";
  ofs_.close();
}

}  // namespace turtlelib
