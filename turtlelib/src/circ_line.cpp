#include <cmath>
#include "turtlelib/circ_line.hpp"


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
} // namespace turtlelib
