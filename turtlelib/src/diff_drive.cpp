#include <cmath>

#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
DiffDrive::DiffDrive()
{
  /// Start citation --> Obtained specification from https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications
  track_width__ = 160e-3;
  wheel_radius__ = 33e-3;
  /// <-- End citation

  initialize_config__();
  compute_wheel_tf__();
}

DiffDrive::DiffDrive(double track_width, double wheel_radius)
{
  track_width__ = track_width;
  wheel_radius__ = wheel_radius;

  initialize_config__();
  compute_wheel_tf__();
}

void DiffDrive::compute_fk(double left_wheel, double right_wheel)
{
  double delta_left = left_wheel - left_wheel__;
  double delta_right = right_wheel - right_wheel__;
  double d = wheel_radius__ / 2.0;
  double r = wheel_radius__;

  double omega_z = r / (2.0 * d) * (-delta_left + delta_right);
  double v_x = r / 2.0 * (delta_left + delta_right);
  double v_y = 0.0;

  double delta_theta_b;
  double delta_x_b;
  double delta_y_b;

  if (almost_equal(omega_z, 0.0)) {
    delta_theta_b = 0.0;
    delta_x_b = v_x;
    delta_y_b = v_y;
  } else {
    delta_theta_b = omega_z;
    delta_x_b = (v_x * sin(omega_z) + v_y * (cos(omega_z) - 1.0)) / omega_z;
    delta_y_b = (v_y * sin(omega_z) + v_x * (1.0 - cos(omega_z))) / omega_z;
  }

  double sin_theta = sin(robot_theta__);
  double cos_theta = cos(robot_theta__);

  double delta_theta = delta_theta_b;
  double delta_x = cos_theta * delta_x_b - sin_theta * delta_y_b;
  double delta_y = sin_theta * delta_x_b + cos_theta * delta_y_b;

  robot_theta__ += delta_theta;
  robot_x__ += delta_x;
  robot_y__ += delta_y;
}
}
