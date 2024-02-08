///
/// \file diff_drive.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief The library for computing kinematics of a differential-drive robot
/// \version 0.1
/// \date 2024-02-08
///
/// \copyright Copyright (c) 2024
///
///
#include <cmath>
#include <vector>
#include <stdexcept>

#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
DiffDrive::DiffDrive()
: left_wheel_(0.0), right_wheel_(0.0), robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0)
{
  /// Start citation --> Obtained specification from https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications
  track_width_ = 160e-3; // m
  wheel_radius_ = 33e-3; // m
  /// <-- End citation
}

DiffDrive::DiffDrive(double track_width, double wheel_radius)
: left_wheel_(0.0), right_wheel_(0.0), robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0)
{
  track_width_ = track_width;
  wheel_radius_ = wheel_radius;
}

Twist2D DiffDrive::compute_fk(double left_wheel, double right_wheel)
{
  const auto delta_left = left_wheel - left_wheel_;
  const auto delta_right = right_wheel - right_wheel_;
  const auto d = track_width_ / 2.0;
  const auto r = wheel_radius_;

  const auto omega_z = r / (2.0 * d) * (-delta_left + delta_right);
  const auto v_x = r / 2.0 * (delta_left + delta_right);
  const auto v_y = 0.0;

  Transform2D Twb(Vector2D{robot_x_, robot_y_}, robot_theta_);
  Transform2D Tbbp = integrate_twist(Twist2D{omega_z, v_x, v_y});
  Transform2D Twbp = Twb * Tbbp;

  robot_theta_ = Twbp.rotation();
  robot_x_ = Twbp.translation().x;
  robot_y_ = Twbp.translation().y;
  left_wheel_ = left_wheel;
  right_wheel_ = right_wheel;

  return {omega_z, v_x, v_y};
}

WheelSpeed DiffDrive::compute_ik(Twist2D body_twist)
{
  const auto theta_dot = body_twist.omega;
  const auto x_dot = body_twist.x;
  const auto y_dot = body_twist.y;

  if (!almost_equal(y_dot, 0.0)) {
    throw std::logic_error("Invalid twist");
  }

  const auto d = track_width_ / 2.0;
  const auto r = wheel_radius_;

  const auto phidot_x = (-d * theta_dot + x_dot) / r;
  const auto phidot_y = (d * theta_dot + x_dot) / r;

  return {phidot_x, phidot_y};
}

void DiffDrive::update_config(double x, double y, double theta)
{
  robot_x_ = x;
  robot_y_ = y;
  robot_theta_ = theta;
}

void DiffDrive::update_wheel(double left, double right)
{
  left_wheel_ = left;
  right_wheel_ = right;
}

double DiffDrive::left_wheel()
{
  return left_wheel_;
}

double DiffDrive::right_wheel()
{
  return right_wheel_;
}

double DiffDrive::config_x()
{
  return robot_x_;
}

double DiffDrive::config_y()
{
  return robot_y_;
}

double DiffDrive::config_theta()
{
  return robot_theta_;
}
}
