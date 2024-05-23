#include <cmath>
#include <armadillo>

#include "turtlelib/wall_range.hpp"


namespace turtlelib
{
double get_angle(Point2D origin, Point2D pa)
{
  Vector2D oa = pa - origin;
  return atan2(oa.y, oa.x);
}

bool can_intersect(Point2D origin, double theta, Wall w)
{
  const auto theta1 = get_angle(origin, w.start);
  const auto theta2 = get_angle(origin, w.end);

  const auto dtheta = normalize_angle(theta2 - theta1);

  if (dtheta < 0) {
    if (theta > 0 && theta1 + dtheta < -PI) {
      theta -= 2 * PI;
    }
    return theta <= theta1 && theta >= theta1 + dtheta;
  } else if (dtheta == 0) {
    return theta1 == theta;
  } else {
    if (theta < 0 && theta1 + dtheta > PI) {
      theta += 2 * PI;
    }
    return theta >= theta1 && theta <= theta1 + dtheta;
  }
}

double find_wall_distance(Point2D origin, double theta, Wall w)
{
  const Vector2D vAB = w.end - w.start;

  const Vector2D d{cos(theta), sin(theta)};
  const auto abx = vAB.x;
  const auto aby = vAB.y;
  const auto dx = d.x;
  const auto dy = d.y;
  const auto ox = origin.x;
  const auto oy = origin.y;
  const auto ax = w.start.x;
  const auto ay = w.start.y;

  arma::mat A_mat{{abx, -dx}, {aby, -dy}};
  arma::vec b_vec{ox - ax, oy - ay};

  arma::vec ut_vec = arma::solve(A_mat, b_vec);
  // const auto u = ut_vec.at(0);
  // const auto t = ut_vec.at(1);

  return ut_vec.at(1);
}
} /// namespace turtlelib
