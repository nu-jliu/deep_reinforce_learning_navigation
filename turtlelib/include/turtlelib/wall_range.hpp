#ifndef WALL_RANGE_HPP_INCLUDE_GUARD
#define WALL_RANGE_HPP_INCLUDE_GUARD

#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
/// @brief Wall structure
struct Wall
{
  /// @brief Start position of the wall
  Point2D start;

  /// @brief End position of the wall
  Point2D end;
};

/// @brief Get the angle between to points
/// @param origin The origin coordinate
/// @param pa The point coordinate
/// @return The result angle
double get_angle(Point2D origin, Point2D pa);

/// @brief Determine if the ray with origin and theta can intersect with a wall
/// @param origin The origin of the ray
/// @param theta The angle of the ray
/// @param w The wall structure
/// @return If the wall can intersect with other
bool can_intersect(Point2D origin, double theta, Wall w);

/// @brief Find the distance between the ray and the wall
/// @param origin The origin of the ray
/// @param theta The angle of the ray
/// @param w The wall structure
/// @return The distance
double find_wall_distance(Point2D origin, double theta, Wall w);
} /// namespace turtlelib

#endif
