/// \file ekf_slam.hpp
/// \author Allen Liu (jingkunliu2025\u.northwestern.edu)
/// \brief The library functions for all slam calculations.
/// \version 0.1
/// \date 2024-02-25
///
/// \copyright Copyright (c) 2024
#ifndef EKF_SLAM_HPP_INCLUDE_GUARD
#define EKF_SLAM_HPP_INCLUDE_GUARD

#include <iostream>
#include <armadillo>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
/// \brief The state of the robot
struct RobotState
{
  /// \brief The orientation
  double theta;

  /// \brief The x position
  double x;

  /// \brief The y position
  double y;
};

std::ostream & operator<<(std::ostream & os, const RobotState & rs);

/// \brief The measurement of a obstacle
struct Measurement
{
  /// \brief The x difference
  double x;

  /// \brief The y difference
  double y;

  /// \brief The id of the obstacle
  int uid;
};

std::ostream & operator<<(std::ostream & os, const Measurement & ms);

/// \brief The EKF class for Extented Kalman Filter calculations
class EKF
{
private:
  RobotState state_;
  std::vector<Measurement> obstacles_;
  arma::mat covariance_mat_;
  double num_obstacles_;

public:
  /// \brief Construct a new EFK object
  EKF();

  /// \brief Construct a new EKF object
  /// \param num_obstacles
  explicit EKF(int num_obstacles);

  /// \brief Get the current state vector
  /// \return arma::vec The state vector
  arma::vec get_state_vec();

  /// \brief Get all mapped landmarks
  /// \return All landmark objects
  std::vector<Measurement> get_all_landmarks();

  /// \brief Get the position of landmark for a specific id.
  /// \param uid The id of the landmark
  /// \return Measurement
  Measurement get_landmark_pos(int uid);

  /// \brief Get the h vector for measurment
  /// \param landmark The landmark object
  /// \return arma::vec
  arma::vec get_h_vec(Measurement landmark);

  /// \brief Get the H mattrix
  /// \param landmark The landmark object
  /// \param index The correponding index
  /// \return arma::mat The H matrix
  arma::mat get_H_mat(Measurement landmark, int index);

  /// \brief Get the A matrix
  /// \param dx difference in x coordinate
  /// \param dy difference in y coordinate
  /// \return arma::mat The resulting A matrix
  arma::mat get_A_mat(double dx, double dy);

  /// \brief Update the covariance matrix
  /// \param sigma_new The new covariance matrix
  void update_covariance(arma::mat sigma_new);

  /// \brief Update the state of the robot
  /// \param x The new x postion
  /// \param y The new y position
  /// \param theta The new orienrtation
  void update_state(double x, double y, double theta);

  /// \brief Update landmark measurements
  /// \param landmarks The landmark positions
  void update_landmark_pos(arma::vec state);

  /// \brief Get the robot state for previous update
  /// \return RobotState The result robot state
  RobotState get_robot_state() const;

  /// \brief Get the covariance mattrix
  /// \return arma::mat The covariance matrix
  arma::mat get_covariance_mat() const;
};
} // namespace turtlelib


#endif
