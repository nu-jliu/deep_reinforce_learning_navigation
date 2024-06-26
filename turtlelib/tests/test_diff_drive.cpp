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

  CHECK_THAT(turtlebot1.left_wheel(), WithinAbs(PI, TOLERANCE));
  CHECK_THAT(turtlebot1.right_wheel(), WithinAbs(PI, TOLERANCE));
  CHECK_THAT(turtlebot1.config_x(), WithinAbs(PI * turtleobt_wheel_radius, TOLERANCE));
  CHECK_THAT(turtlebot1.config_y(), WithinAbs(0.0, TOLERANCE));
  CHECK_THAT(turtlebot1.config_theta(), WithinAbs(0.0, TOLERANCE));

  DiffDrive turtlebot2;
  turtlebot2.compute_fk(-PI, -PI);

  CHECK_THAT(turtlebot2.left_wheel(), WithinAbs(-PI, TOLERANCE));
  CHECK_THAT(turtlebot2.right_wheel(), WithinAbs(-PI, TOLERANCE));
  CHECK_THAT(turtlebot2.config_x(), WithinAbs(-PI * turtleobt_wheel_radius, TOLERANCE));
  CHECK_THAT(turtlebot2.config_y(), WithinAbs(0.0, TOLERANCE));
  CHECK_THAT(turtlebot2.config_theta(), WithinAbs(0.0, TOLERANCE));
}

TEST_CASE("Drive forward ik", "[inverse]") /// Allen Liu
{
  DiffDrive turtlebot1;
  WheelSpeed phidot1 = turtlebot1.compute_ik(Twist2D{0.0, 1.5, 0.0});

  CHECK_THAT(phidot1.left, WithinAbs(1.5 / turtleobt_wheel_radius, TOLERANCE));
  CHECK_THAT(phidot1.right, WithinAbs(1.5 / turtleobt_wheel_radius, TOLERANCE));

  DiffDrive turtlebot2;
  WheelSpeed phidot2 = turtlebot2.compute_ik(Twist2D{0.0, -2.4, 0.0});

  CHECK_THAT(phidot2.left, WithinAbs(-2.4 / turtleobt_wheel_radius, TOLERANCE));
  CHECK_THAT(phidot2.right, WithinAbs(-2.4 / turtleobt_wheel_radius, TOLERANCE));
}

TEST_CASE("Pure Rotations fk", "[forward]") /// Allen Liu
{
  DiffDrive turtlebot1;
  turtlebot1.compute_fk(-PI, PI);

  CHECK_THAT(turtlebot1.left_wheel(), WithinAbs(-PI, TOLERANCE));
  CHECK_THAT(turtlebot1.right_wheel(), WithinAbs(PI, TOLERANCE));
  CHECK_THAT(turtlebot1.config_x(), WithinAbs(0.0, TOLERANCE));
  CHECK_THAT(turtlebot1.config_y(), WithinAbs(0.0, TOLERANCE));
  CHECK_THAT(
    turtlebot1.config_theta(),
    WithinAbs(PI * turtleobt_wheel_radius / (turtlebot_track_width / 2.0), TOLERANCE));

  DiffDrive turtlebot2;
  turtlebot2.compute_fk(PI, -PI);

  CHECK_THAT(turtlebot2.left_wheel(), WithinAbs(PI, TOLERANCE));
  CHECK_THAT(turtlebot2.right_wheel(), WithinAbs(-PI, TOLERANCE));
  CHECK_THAT(turtlebot2.config_x(), WithinAbs(0.0, TOLERANCE));
  CHECK_THAT(turtlebot2.config_y(), WithinAbs(0.0, TOLERANCE));
  CHECK_THAT(
    turtlebot2.config_theta(),
    WithinAbs(-PI * turtleobt_wheel_radius / (turtlebot_track_width / 2.0), TOLERANCE));
}

TEST_CASE("Pure Rotations ik", "[inverse]") // Allen Liu
{
  DiffDrive turtlebot1;
  WheelSpeed phidot1 = turtlebot1.compute_ik(Twist2D{PI, 0.0, 0.0});

  CHECK_THAT(
    phidot1.left,
    WithinAbs(-PI * (turtlebot_track_width / 2.0) / turtleobt_wheel_radius, TOLERANCE));
  CHECK_THAT(
    phidot1.right,
    WithinAbs(PI * (turtlebot_track_width / 2.0) / turtleobt_wheel_radius, TOLERANCE));

  DiffDrive turtlebot2;
  WheelSpeed phidot2 = turtlebot2.compute_ik(Twist2D{-PI, 0.0, 0.0});

  CHECK_THAT(
    phidot2.left,
    WithinAbs(PI * (turtlebot_track_width / 2.0) / turtleobt_wheel_radius, TOLERANCE));
  CHECK_THAT(
    phidot2.right,
    WithinAbs(-PI * (turtlebot_track_width / 2.0) / turtleobt_wheel_radius, TOLERANCE));
}

TEST_CASE("Round Track fk", "[forward]")
{
  DiffDrive turtlebot1;
  turtlebot1.compute_fk(PI, 2.0 * PI);

  CHECK_THAT(turtlebot1.left_wheel(), WithinAbs(PI, TOLERANCE));
  CHECK_THAT(turtlebot1.right_wheel(), WithinAbs(2.0 * PI, TOLERANCE));
  CHECK_THAT(turtlebot1.config_x(), WithinAbs(0.1449, 1e-4));
  CHECK_THAT(turtlebot1.config_y(), WithinAbs(0.0486, 1e-4));
  CHECK_THAT(turtlebot1.config_theta(), WithinAbs(0.6480, 1e-4));

  DiffDrive turtlebot2;
  turtlebot2.compute_fk(0.5 * PI, 1.5 * PI);

  CHECK_THAT(turtlebot2.left_wheel(), WithinAbs(0.5 * PI, TOLERANCE));
  CHECK_THAT(turtlebot2.right_wheel(), WithinAbs(1.5 * PI, TOLERANCE));
  CHECK_THAT(turtlebot2.config_x(), WithinAbs(0.0966, 1e-4));
  CHECK_THAT(turtlebot2.config_y(), WithinAbs(0.0324, 1e-4));
  CHECK_THAT(turtlebot2.config_theta(), WithinAbs(0.6480, 1e-4));
}

TEST_CASE("Round Track ik", "[inverse]") // Allen Liu
{
  DiffDrive turtlebo1;
  WheelSpeed phidot1 = turtlebo1.compute_ik(Twist2D{PI, PI * turtlebot_track_width / 2.0, 0.0});

  CHECK_THAT(phidot1.left, WithinAbs(0.0, TOLERANCE));
  CHECK_THAT(
    phidot1.right,
    WithinAbs(PI * turtlebot_track_width / turtleobt_wheel_radius, TOLERANCE));
}

TEST_CASE("Unreachable twist", "[inverse]")
{
  DiffDrive turtlebot;

  CHECK_THROWS_AS(turtlebot.compute_ik(Twist2D{0.0, 0.0, 1.0}), std::logic_error);
  CHECK_THROWS_AS(turtlebot.compute_ik(Twist2D{PI, 2.0 * PI, 0.5}), std::logic_error);
}
}
