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

class EKF
{
private:
  RobotState state_;
  std::vector<Measurement> obstacles_;
  arma::mat covariance_mat_;
  double num_obstacles_;
  bool landmark_pos_ready_;

  arma::vec get_state_vector_();

public:
  EKF();
  explicit EKF(int num_obstacles);

  // void set_num_obstacle(std::vector<Measurement> obs);

  arma::vec get_state_vec(std::vector<Measurement> obstacles);
  Measurement get_landmark_pos(int uid);
  arma::vec get_h_vec(Measurement landmark);
  arma::mat get_H_mat(Measurement landmark, int index);
  arma::mat get_A_mat(Twist2D body_twist);

  void update_covariance(arma::mat sigma_new);
  void update_state(double x, double y, double theta);
  void update_landmark_pos(std::vector<Measurement> landmarks);

  bool is_landmark_pos_ready() const;
  RobotState get_robot_state() const;
  arma::mat get_covariance_mat() const;
};


} // namespace turtlelib


#endif
