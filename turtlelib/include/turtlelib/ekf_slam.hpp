///
/// @file ekf_slam.hpp
/// @author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// @brief
/// @version 0.1
/// @date 2024-02-25
///
/// @copyright Copyright (c) 2024
///
#ifndef EKF_SLAM_HPP_INCLUDE_GUARD
#define EKF_SLAM_HPP_INCLUDE_GUARD

#include <armadillo>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
/// \brief
struct RobotState
{
  /// \brief
  double theta;

  /// \brief
  double x;

  /// \brief
  double y;
};

/// \brief
struct Measurement
{
  /// \brief
  double x;

  /// \brief
  double y;

  /// \brief
  int uid;
};

/// @brief
class EKF
{
private:
  RobotState state_;
  std::vector<Measurement> obstacles_;
  arma::mat covariance_mat_;
  double num_obstacles_;
  bool landmark_pos_ready_;

public:
  /// @brief
  EKF();

  /// \brief Construct a new EKF object
  /// \param num_obstacles
  explicit EKF(int num_obstacles);

  /// \brief Get the state vec object
  /// \param obstacles
  /// \return arma::vec
  arma::vec get_state_vec(std::vector<Measurement> obstacles);

  /// \brief
  /// \param uid
  /// \return Measurement
  Measurement get_landmark_pos(int uid);

  /// \brief Get the h vec object
  /// \param landmark
  /// \return arma::vec
  arma::vec get_h_vec(Measurement landmark);

  /// \brief Get the H mat object
  /// \param landmark
  /// \param index
  /// \return arma::mat
  arma::mat get_H_mat(Measurement landmark, int index);

  /// @brief Get the A mat object
  /// @param body_twist
  /// @return arma::mat
  arma::mat get_A_mat(Twist2D body_twist);

  /// @brief
  /// @param sigma_new
  void update_covariance(arma::mat sigma_new);

  /// @brief
  /// @param x
  /// @param y
  /// @param theta
  void update_state(double x, double y, double theta);

  /// @brief
  /// @param landmarks
  void update_landmark_pos(std::vector<Measurement> landmarks);

  /// @brief
  /// @return true
  /// @return false
  ////
  bool is_landmark_pos_ready() const;

  /// @brief Get the robot state object
  /// @return RobotState
  ////
  RobotState get_robot_state() const;

  /// @brief Get the covariance mat object
  /// @return arma::mat
  arma::mat get_covariance_mat() const;
};


} // namespace turtlelib


#endif
