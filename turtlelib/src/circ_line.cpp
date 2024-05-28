#include <cmath>
#include <iostream>
#include "turtlelib/circ_line.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/trig2d.hpp"


namespace turtlelib
{
double compute_mid_angle(double a, double b, double c)
{
  const auto num = pow(a, 2.0) + pow(b, 2.0) - pow(c, 2.0);
  const auto den = 2 * a * b;
  const auto ctheta = num / den;
  const auto theta = acos(ctheta);

  return theta;
}

bool line_circ_intersect(Point2D pA, Point2D pB, Point2D pO, double r)
{
  const Vector2D v_AB = pB - pA;
  const Vector2D v_AO = pO - pA;
  const Vector2D v_BO = pO - pB;

  const auto a = magnitude(v_BO);
  const auto b = magnitude(v_AO);
  const auto o = magnitude(v_AB);

  const auto dist = fabs(cross2d(v_AO, v_AB) / magnitude(v_AB));
  // std::cout << dist << std::endl;

  if (dist <= r) {
    const auto alpha = compute_mid_angle(b, o, a);
    const auto beta = compute_mid_angle(a, o, b);

    if (alpha > PI / 2.0 || beta > PI / 2.0) {
      return false;
    }

    return true;
  } else {
    return false;
  }
}
} // namespace turtlelib
