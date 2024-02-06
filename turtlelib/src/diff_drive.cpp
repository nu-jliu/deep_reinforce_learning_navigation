#include <cmath>
#include <vector>
#include <stdexcept>

#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
DiffDrive::DiffDrive()
: left_wheel__(0.0), right_wheel__(0.0), robot_x__(0.0), robot_y__(0.0), robot_theta__(0.0)
{
  /// Start citation --> Obtained specification from https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications
  track_width__ = 160e-3; // m
  wheel_radius__ = 33e-3; // m
  /// <-- End citation
}

DiffDrive::DiffDrive(double track_width, double wheel_radius)
: left_wheel__(0.0), right_wheel__(0.0), robot_x__(0.0), robot_y__(0.0), robot_theta__(0.0)
{
  track_width__ = track_width;
  wheel_radius__ = wheel_radius;
}

Twist2D DiffDrive::compute_fk(double left_wheel, double right_wheel)
{
  double delta_left = left_wheel - left_wheel__;
  double delta_right = right_wheel - right_wheel__;
  double d = track_width__ / 2.0;
  double r = wheel_radius__;

  double omega_z = r / (2.0 * d) * (-delta_left + delta_right);
  double v_x = r / 2.0 * (delta_left + delta_right);
  double v_y = 0.0;

  Transform2D Twb(Vector2D{robot_x__, robot_y__}, robot_theta__);
  Transform2D Tbbp = integrate_twist(Twist2D{omega_z, v_x, v_y});
  Transform2D Twbp = Twb * Tbbp;

  robot_theta__ = Twbp.rotation();
  robot_x__ = Twbp.translation().x;
  robot_y__ = Twbp.translation().y;
  left_wheel__ = left_wheel;
  right_wheel__ = right_wheel;

  return {omega_z, v_x, v_y};
}

WheelSpeed DiffDrive::compute_ik(Twist2D body_twist)
{
  double theta_dot = body_twist.omega;
  double x_dot = body_twist.x;
  double y_dot = body_twist.y;

  if (!almost_equal(y_dot, 0.0)) {
    throw std::logic_error("Invalid twist");
  }

  double d = track_width__ / 2.0;
  double r = wheel_radius__;

  double phidot_x = (-d * theta_dot + x_dot) / r;
  double phidot_y = (d * theta_dot + x_dot) / r;

  return {phidot_x, phidot_y};
}

void DiffDrive::update_config(double x, double y, double theta)
{
  robot_x__ = x;
  robot_y__ = y;
  robot_theta__ = theta;
}

void DiffDrive::update_wheel(double left, double right)
{
  left_wheel__ = left;
  right_wheel__ = right;
}

double DiffDrive::left_wheel()
{
  return left_wheel__;
}

double DiffDrive::right_wheel()
{
  return right_wheel__;
}

double DiffDrive::config_x()
{
  return robot_x__;
}

double DiffDrive::config_y()
{
  return robot_y__;
}

double DiffDrive::config_theta()
{
  return robot_theta__;
}
}
