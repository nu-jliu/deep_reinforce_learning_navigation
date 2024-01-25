#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
DiffDrive::DiffDrive()
{
  left_wheel__ = 0.0;
  right_wheel__ = 0.0;
  robot_x__ = 0.0;
  robot_y__ = 0.0;
  robot_theta__ = 0.0;
}

DiffDrive::DiffDrive(double track_width, double wheel_radius)
{

  DiffDrive();
}

void DiffDrive::compute_fk(double left_wheel, double right_wheel)
{

}
}
