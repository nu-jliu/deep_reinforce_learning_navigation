#include <catch2/catch_all.hpp>
#include "turtlelib/circ_line.hpp"
#include "turtlelib/geometry2d.hpp"

#define TOLERANCE 1e-5

namespace turtlelib
{
TEST_CASE("Test law of cosine calculation", "[computer_mid_angle]")
{
  const double a1 = 1.0;
  const double b1 = 1.0;
  const double c1 = sqrt(2.0);

  const double theta1 = compute_mid_angle(a1, b1, c1);
  CHECK_THAT(theta1, Catch::Matchers::WithinAbs(PI / 2.0, TOLERANCE));

  const auto a = 20.0;
  const auto b = 18.0;
  const auto c = 5.0;

  const auto theta2 = compute_mid_angle(a, b, c);
  CHECK_THAT(theta2, Catch::Matchers::WithinAbs(0.242113867, TOLERANCE));
}

TEST_CASE("Test line circle intersection", "[line_circ_intersect]")
{
  const Point2D pA1{-1.0, 0.0};
  const Point2D pB1{1.0, 0.0};
  const Point2D pO1{0.0, 0.8};
  const double r1 = 1.0;

  CHECK(line_circ_intersect(pA1, pB1, pO1, r1));

  const Point2D pA2{0.0, 1.0};
  const Point2D pB2{1.0, 0.0};
  const Point2D pO2{1.0, 1.0};
  const double r2 = 0.2;

  CHECK_FALSE(line_circ_intersect(pA2, pB2, pO2, r2));
}
} // namespace turtlelib
