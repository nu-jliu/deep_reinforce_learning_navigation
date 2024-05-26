#include <catch2/catch_all.hpp>
#include "turtlelib/circ_line.hpp"

#define TOLERANCE 1e-5

namespace turtlelib
{
TEST_CASE("Test law of cosine calculation", "[computer_mid_angle]")
{
  double a = 1.0;
  double b = 1.0;
  double c = sqrt(2.0);

  double theta = compute_mid_angle(a, b, c);
  CHECK_THAT(theta, Catch::Matchers::WithinAbs(PI / 2.0, TOLERANCE));

  a = 20;
  b = 18;
  c = 5;

  theta = compute_mid_angle(a, b, c);
  CHECK_THAT(theta, Catch::Matchers::WithinAbs(0.242113867, TOLERANCE));
}
} // namespace turtlelib
