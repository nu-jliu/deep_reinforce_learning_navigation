#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#ifndef DIFF_DRIVE_HPP_INCLUDE_GUARD
#define DIFF_DRIVE_HPP_INCLUDE_GUARD

namespace turtlelib
{
/// @brief Structure to represent the wheel speed
struct WheelSpeed
{
  /// @brief The left wheel speed
  double left;

  /// @brief The right wheel speed
  double right;
};

/// @brief The DiffDrive class to represent a diffdrive mobile robot
class DiffDrive
{
private:
  double track_width__;
  double wheel_radius__;
  double left_wheel__;
  double right_wheel__;
  double robot_x__;
  double robot_y__;
  double robot_theta__;

public:
  /// \brief Creates a default diffdrive robot using turtlebot3 specs
  DiffDrive();

  /// \brief Creates a custom diffdrive robot
  /// \param track_width The distance between to wheel
  /// \param wheel_radius The radius of the wheel
  DiffDrive(double track_width, double wheel_radius);

  /// \brief Computes forward kinematics
  /// \param left_wheel The left wheel target configuration
  /// \param right_wheel The right wheel target configuration
  void compute_fk(double left_wheel, double right_wheel);

  /// \brief
  /// \param body_twist
  /// \return
  WheelSpeed compute_ik(Twist2D body_twist);

  /// \brief
  /// \return
  double left_wheel();

  /// \brief
  /// \return
  double right_wheel();

  /// \brief
  /// \return
  double config_x();

  /// \brief
  /// \return
  double config_y();

  /// \brief
  /// \return
  double config_theta();
};

}

#endif
