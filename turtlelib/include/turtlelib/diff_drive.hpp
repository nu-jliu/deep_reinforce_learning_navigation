///
/// \file diff_drive.hpp
/// \author your name (you@domain.com)
/// \brief
/// \version 0.1
/// \date 2024-02-08
///
/// \copyright Copyright (c) 2024
///
///
#ifndef DIFF_DRIVE_HPP_INCLUDE_GUARD
#define DIFF_DRIVE_HPP_INCLUDE_GUARD

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
/// \brief Structure to represent the wheel speed
struct WheelSpeed
{
  /// \brief The left wheel speed
  double left;

  /// \brief The right wheel speed
  double right;
};

/// \brief The DiffDrive class to represent a diffdrive mobile robot
class DiffDrive
{
private:
  double track_width_;
  double wheel_radius_;
  double left_wheel_;
  double right_wheel_;
  double robot_x_;
  double robot_y_;
  double robot_theta_;

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
  Twist2D compute_fk(double left_wheel, double right_wheel);

  /// \brief Computes inverse kinematics
  /// \param body_twist The twist command
  /// \return The wheel velocity that follows the command
  WheelSpeed compute_ik(Twist2D body_twist);

  /// \brief Update the robot configuration
  /// \param x The new x configuration
  /// \param y The new y configuration
  /// \param theta The new theta configuration
  void update_config(double x, double y, double theta);

  /// \brief Update the new wheel configuration
  /// \param left New left wheel configuration
  /// \param right New right configuration
  void update_wheel(double left, double right);

  /// \brief Get left wheel configuration
  /// \return [double] Left wheel angle
  double left_wheel() const;

  /// \brief Get right wheel configuration
  /// \return [double] right wheel angle
  double right_wheel() const;

  /// \brief Get x configuration
  /// \return [double] x configuration
  double config_x() const;

  /// \brief Get y configuration
  /// \return [double] y configuration
  double config_y() const;

  /// \brief Get theta configuration
  /// \return [double] theta configuration
  double config_theta() const;
};

}

#endif
