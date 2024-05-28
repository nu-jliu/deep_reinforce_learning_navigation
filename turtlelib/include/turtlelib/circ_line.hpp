#ifndef CIRC_LINE_HPP_INCLUDE_GUARD
#define CIRC_LINE_HPP_INCLUDE_GUARD

#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
/// @brief Compute the angle between two lines in a triangle
///
/// @param a The first adjacent length
/// @param b The second adjacent length
/// @param c The length of cross edge
/// @return The mid angle
double compute_mid_angle(double a, double b, double c);


/// @brief Determine if a circle and line  can intersect
///
/// @param pA The start point at the line
/// @param pB The end point at the line
/// @param pO The center of the circle
/// @param r The radius of the circle
/// @return true if the line and circle can intersect
/// @return false if the line and the circle cannot intersect
bool line_circ_intersect(Point2D pA, Point2D pB, Point2D pO, double r);
} // namespace turtlelib


#endif
