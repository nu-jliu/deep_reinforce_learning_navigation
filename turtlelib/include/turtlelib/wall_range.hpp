#ifndef WALL_RANGE_HPP_INCLUDE_GUARD
#define WALL_RANGE_HPP_INCLUDE_GUARD

#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
struct Wall
{
  Point2D start;
  Point2D end;
};

double get_angle(Point2D origin, Point2D pa);

bool can_intersect(Point2D origin, double theta, Wall w);

double find_wall_distance(Point2D origin, double theta, Wall w);
} /// namespace turtlelib

#endif
