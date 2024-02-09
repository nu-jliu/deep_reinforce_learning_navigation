///
/// \file svg.hpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief The svg class used for display points and lines in svg file
/// \version 0.1
/// \date 2024-01-22
///
/// \copyright Copyright (c) 2024
///
///
#ifndef TURTLELIB_SVG_HPP_INCLUDE_GUARD
#define TURTLELIB_SVG_HPP_INCLUDE_GUARD

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include <iostream>
#include <fstream>

namespace turtlelib
{
/// @brief The class for displaying svg on a file
class Svg
{

private:
  std::ofstream ofs_;
  Transform2D tf_origin_;
  double origin_x_;
  double origin_y_;
  double scale_;

  /// \brief Initialize the environment for the frame
  void init_svg();

  /// \brief Calculate the transform from turtlelib frame to svg frame
  /// \param p The point to transform
  Point2D point_tf_(Point2D p)
  {
    return Point2D{origin_x_ + p.x * scale_, origin_y_ - p.y * scale_};
  }

public:
  /// \brief Opens a default svg file.
  Svg();

  /// \brief Opens the svg file with specified filename
  /// \param filename the name of the svg file.
  /// \throws IOException when filename is invalid.
  explicit Svg(std::string filename);

  /// \brief Draw a line for the specified vector
  /// \param frame
  /// \param tail the tail of the vector
  /// \param v the vector to be drawn.
  /// \param color the color for the line
  void draw_line(Transform2D frame, Point2D tail, Vector2D v, std::string color);

  /// \brief Draw a point in svg file
  /// \param frame the frame to draw on
  /// \param p the point to draw
  /// \param radius the radius for the point
  /// \param color the color for the point
  void draw_point(Transform2D frame, Point2D p, int radius, std::string color);

  /// \brief Draw a frame on the svg file
  /// \param tf The frame to draw
  /// \param name The name of the frame
  void draw_frame(Transform2D tf, std::string name);

  /// \brief Finish drawing, close the svg file
  void finish();
};
}

#endif
