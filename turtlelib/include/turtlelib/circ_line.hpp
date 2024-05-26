#ifndef CIRC_LINE_HPP_INCLUDE_GUARD
#define CIRC_LINE_HPP_INCLUDE_GUARD

#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
double compute_mid_angle(double a, double b, double c);

bool can_intersect(Point2D pA, Point2D pB, Point2D pO, double r);
} // namespace turtlelib


#endif
