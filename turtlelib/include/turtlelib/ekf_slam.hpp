#ifndef EKF_SLAM_HPP_INCLUDE_GUARD
#define EKF_SLAM_HPP_INCLUDE_GUARD

#include <armadillo>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
struct RobotState
{
  double theta;
  double x;
  double y;
};

struct Measurement
{
  double x;
  double y;
  int uid;
};

arma::vec get_h_vec(Measurement landmark);
arma::mat get_H_mat(Measurement landmark);
class EKF
{
private:
  RobotState state_;
  // std::vector<Measurement> obstacles_;
  arma::mat covariance_mat_;
  double num_obstacles_;

  arma::vec get_state_vector_();

public:
  EKF();
  explicit EKF(int num_obstacles);

  // void set_num_obstacle(std::vector<Measurement> obs);

  arma::vec get_state_vec(std::vector<Measurement> obstacles);
  arma::mat get_A_mat(Twist2D body_twist);

  // bool is_measure_initialized() const;
  RobotState get_robot_state() const;
  arma::mat get_covariance_mat() const;
};


} // namespace turtlelib


#endif
