#include <cmath>
#include <iostream>

#include "turtlelib/trig2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
double cross2d(Vector2D v1, Vector2D v2)
{
  const auto x1 = v1.x;
  const auto y1 = v1.y;

  const auto x2 = v2.x;
  const auto y2 = v2.y;

  return x1 * y2 - x2 * y1;
}

double dot2d(Vector2D v1, Vector2D v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

bool can_intersect(double alpha, Obstacle obs)
{
  Vector2D v_AO{obs.x, obs.y};
  if (magnitude(v_AO) < obs.r) {
    return false;
  }

  Vector2D v_AB{cos(alpha), sin(alpha)};
  const auto d = fabs(cross2d(v_AO, v_AB)) / magnitude(v_AB);
  const auto alpha_prime = acos(dot2d(v_AO, v_AB) / (magnitude(v_AO) * magnitude(v_AB)));
  return d <= obs.r && fabs(alpha_prime) < PI;
}

double find_distance(double alpha, Obstacle obs)
{
  Vector2D v_AO = Vector2D{obs.x, obs.y};
  const auto a = magnitude(v_AO);

  // alpha = alpha < 0 ? alpha + 2.0 * turtlelib::PI : alpha;
  const auto beta = turtlelib::PI - asin(a / obs.r * sin(alpha));
  std::cout << "beta = " << beta << std::endl;
  const auto gamma = turtlelib::PI - alpha - beta;

  return obs.r * sin(gamma) / sin(alpha);
}
}
