#ifndef TRIG2D_HPP_INCLUDE_GUARD
#define TRIG2D_HPP_INCLUDE_GUARD

#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
struct Obstacle
{
  /// \brief x coordinate of the obstacle
  double x;

  /// \brief y coordinate of the obstacle
  double y;

  /// \brief radius of thhe obstacle
  double r;
};

/// \brief
/// \param v1
/// \param v2
/// \return double
double cross2d(Vector2D v1, Vector2D v2);

/// \brief
/// \param v1
/// \param v2
/// \return double
double dot2d(Vector2D v1, Vector2D v2);

/// \brief
/// \param v
/// \param obs
/// \return bool
bool can_intersect(double alpha, Obstacle obs);

/// \brief
/// \param alpha
/// \param obs
/// \return double
double find_distance(double alpha, Obstacle obs);
}

#endif
