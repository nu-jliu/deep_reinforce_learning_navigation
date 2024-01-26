#include <vector>

#include "turtlelib/diff_drive.hpp"
#include "catch2/catch_all.hpp"

#define TOLERANCE 1e-15

const double turtleobt_wheel_radius = 33e-3; // m
const double turtlebot_track_width = 160e-3; // m

using Catch::Matchers::WithinAbs;

namespace turtlelib
{
TEST_CASE("Drive forward fk", "[forward]") /// Allen Liu
{
  DiffDrive turtlebot1;
  turtlebot1.compute_fk(PI, PI);

  REQUIRE_THAT(turtlebot1.left_wheel(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(turtlebot1.right_wheel(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(turtlebot1.config_x(), WithinAbs(PI * turtleobt_wheel_radius, TOLERANCE));
  REQUIRE_THAT(turtlebot1.config_y(), WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(turtlebot1.config_theta(), WithinAbs(0.0, TOLERANCE));

  DiffDrive turtlebot2;
  turtlebot2.compute_fk(-PI, -PI);

  REQUIRE_THAT(turtlebot2.left_wheel(), WithinAbs(-PI, TOLERANCE));
  REQUIRE_THAT(turtlebot2.right_wheel(), WithinAbs(-PI, TOLERANCE));
  REQUIRE_THAT(turtlebot2.config_x(), WithinAbs(-PI * turtleobt_wheel_radius, TOLERANCE));
  REQUIRE_THAT(turtlebot2.config_y(), WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(turtlebot2.config_theta(), WithinAbs(0.0, TOLERANCE));
}

TEST_CASE("Drive forward ik", "[inverse]") /// Allen Liu
{
  DiffDrive turtlebot1;
  WheelSpeed phidot1 = turtlebot1.compute_ik(Twist2D{0.0, 1.5, 0.0});

  REQUIRE_THAT(phidot1.left, WithinAbs(1.5 / turtleobt_wheel_radius, TOLERANCE));
  REQUIRE_THAT(phidot1.right, WithinAbs(1.5 / turtleobt_wheel_radius, TOLERANCE));

  DiffDrive turtlebot2;
  WheelSpeed phidot2 = turtlebot2.compute_ik(Twist2D{0.0, -2.4, 0.0});

  REQUIRE_THAT(phidot2.left, WithinAbs(-2.4 / turtleobt_wheel_radius, TOLERANCE));
  REQUIRE_THAT(phidot2.right, WithinAbs(-2.4 / turtleobt_wheel_radius, TOLERANCE));
}

TEST_CASE("Pure Rotations fk", "[forward]") /// Allen Liu
{
  DiffDrive turtlebot1;
  turtlebot1.compute_fk(-PI, PI);

  REQUIRE_THAT(turtlebot1.left_wheel(), WithinAbs(-PI, TOLERANCE));
  REQUIRE_THAT(turtlebot1.right_wheel(), WithinAbs(PI, TOLERANCE));
  REQUIRE_THAT(turtlebot1.config_x(), WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(turtlebot1.config_y(), WithinAbs(0.0, TOLERANCE));
  REQUIRE_THAT(
    turtlebot1.config_theta(),
    WithinAbs(PI * turtleobt_wheel_radius / (turtlebot_track_width / 2.0), TOLERANCE));
}
}
