#include <armadillo>
#include <limits>

#include "turtlelib/ekf_slam.hpp"

namespace turtlelib
{
arma::vec get_h_vec(Measurement landmark)
{
  return {landmark.x, landmark.y};
}

EKF::EKF()
{
  EKF(20);
}

EKF::EKF(int num_obstacles)
{
  num_obstacles_ = num_obstacles;
  state_ = {0.0, 0.0, 0.0};

  arma::mat up_left(3, 3, arma::fill::zeros);
  arma::mat up_right(3, 2 * num_obstacles, arma::fill::zeros);
  arma::mat bottom_left(2 * num_obstacles, 3, arma::fill::zeros);
  arma::mat bottom_right = std::numeric_limits<double>::max() * arma::mat(
    2 * num_obstacles,
    2 * num_obstacles,
    arma::fill::eye);

  covariance_mat_ = arma::join_horiz(
    arma::join_vert(up_left, bottom_left),
    arma::join_vert(up_right, bottom_right));
}

// void EKF::set_num_obstacle(std::vector<Measurement> obs) {

// }

arma::vec EKF::get_state_vec(std::vector<Measurement> obstacles)
{
  arma::vec state(3 + 2 * num_obstacles_);

  for (size_t i = 0; i < obstacles.size(); ++i) {
    state(3 + 2 * i) = obstacles.at(i).x;
    state(3 + 2 * i + 1) = obstacles.at(i).y;
  }

  return state;
}


arma::mat EKF::get_A_mat(Twist2D body_twist)
{
  const auto dx = body_twist.x;
  const auto dtheta = body_twist.omega;
  // const auto num_obs = obstacles_.size();

  // arma::mat A_mat(2 * num_obs + 3, 2 * num_obs + 3, arma::fill::zeros);
  arma::mat I_mat(2 * num_obstacles_ + 3, 2 * num_obstacles_ + 3, arma::fill::eye);

  const arma::mat up_right(3, 2 * num_obstacles_, arma::fill::zeros);
  const arma::mat down_left(2 * num_obstacles_, 3, arma::fill::zeros);
  const arma::mat down_right(2 * num_obstacles_, 2 * num_obstacles_, arma::fill::zeros);

  arma::mat up_left;

  if (almost_equal(dtheta, 0.0)) {
    up_left = {{0, 0, 0},
      {-dx * sin(state_.theta), 0, 0},
      {dx * cos(state_.theta), 0, 0}};

  } else {
    up_left = {{0, 0, 0},
      {-dx / dtheta * cos(state_.theta) + dx / dtheta * cos(state_.theta + dtheta), 0, 0},
      {-dx / dtheta * sin(state_.theta) + dx / dtheta * sin(state_.theta + dtheta), 0, 0}};
  }

  const arma::mat A_mat = I_mat +
    arma::join_horiz(
    arma::join_vert(up_left, down_left),
    arma::join_vert(up_right, down_right));

  return A_mat;
}

// bool EKF::is_measure_initialized() const
// {
//   return obstacles_.empty();
// }

RobotState EKF::get_robot_state() const
{
  return state_;
}

arma::mat EKF::get_covariance_mat() const
{
  return covariance_mat_;
}
} // namespace turtlelib
