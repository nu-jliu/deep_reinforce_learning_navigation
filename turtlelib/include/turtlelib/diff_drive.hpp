#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#ifndef DIFF_DRIVE_HPP_INCLUDE_GUARD
#define DIFF_DRIVE_HPP_INCLUDE_GUARD

namespace turtlelib
{
class DiffDrive
{
private:
  double left_wheel__;
  double right_wheel__;
  double robot_x__;
  double robot_y__;
  double robot_theta__;

public:
  /// @brief
  DiffDrive();

  /// @brief
  /// @param left_wheel
  /// @param right_wheel
  /// @param x
  /// @param y
  /// @param theta
  DiffDrive(double left_wheel, double right_wheel, double x, double y, double theta);

  /// @brief
  /// @param left_wheel
  /// @param right_wheel
  void compute_fk(double left_wheel, double right_wheel);

  /// @brief
  /// @param body_twist
  void compute_ik(Twist2D body_twist);
};

}

#endif
