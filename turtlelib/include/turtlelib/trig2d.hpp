#ifndef TRIG2D_HPP_INCLUDE_GUARD
#define TRIG2D_HPP_INCLUDE_GUARD

#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
/// @brief The obstacle struct representing a cylindar obstacle
struct Obstacle
{
  /// \brief x coordinate of the obstacle
  double x;

  /// \brief y coordinate of the obstacle
  double y;

  /// \brief radius of thhe obstacle
  double r;
};

/// \brief compute cross product between two vector
/// \param v1 first vector
/// \param v2 second vector
/// \return double the result cross product value
double cross2d(Vector2D v1, Vector2D v2);

/// \brief compute dot product between two vector
/// \param v1 first vector
/// \param v2 second vector
/// \return double the result dot product value
double dot2d(Vector2D v1, Vector2D v2);

/// \brief wether a vector with inclination angle can intersect with the obsatacle
/// \param alpha the inclination angle
/// \param obs the obstacle
/// \return bool if it can intersect
bool can_intersect(double alpha, Obstacle obs);

/// \brief find the distance from the origin to obstacle with inclination angle
/// \param alpha the inclination angle
/// \param obs the obstacle
/// \return double the resulting length
double find_distance(double alpha, Obstacle obs);
}

#endif
