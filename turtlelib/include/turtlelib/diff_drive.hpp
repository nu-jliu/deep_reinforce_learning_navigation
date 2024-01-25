#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#ifndef DIFF_DRIVE_HPP_INCLUDE_GUARD
#define DIFF_DRIVE_HPP_INCLUDE_GUARD

namespace turtlelib
{
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
  Transform2D tf_left__;
  Transform2D tf_right__;

  constexpr void initialize_config__()
  {
    left_wheel__ = 0.0;
    right_wheel__ = 0.0;
    robot_x__ = 0.0;
    robot_y__ = 0.0;
    robot_theta__ = 0.0;
  }

  constexpr void compute_wheel_tf__()
  {
    tf_left__ = Transform2D(Vector2D{0.0, track_width__ / 2.0}, 0.0);
    tf_right__ = Transform2D(Vector2D{0.0, -track_width__ / 2.0}, 0.0);
  }

public:
  /// \brief
  DiffDrive();

  /// \brief
  /// \param track_width
  /// \param wheel_radius
  DiffDrive(double track_width, double wheel_radius);

  /// \brief
  /// \param left_wheel
  /// \param right_wheel
  void compute_fk(double left_wheel, double right_wheel);

  /// \brief
  /// \param body_twist
  void compute_ik(Twist2D body_twist);
};

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
}

#endif
