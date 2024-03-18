/// @file detect.hpp
/// @author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// @brief Library for detecting the circle
/// @version 0.1
/// @date 2024-03-17
///
/// @copyright Copyright (c) 2024
#ifndef DETECT_HPP_INCLUDE_GUARD
#define DETECT_HPP_INCLUDE_GUARD

#include <vector>
#include <armadillo>

#include "turtlelib/se2d.hpp"

namespace turtlelib
{
/// \brief The object for the detect landmark
struct Landmark
{
  /// \brief The x position of landmark
  double x;

  /// \brief The y position of landmark
  double y;

  /// \brief The radius of landmark
  double r;
};

/// \brief The class for landmark detection
class CircleDetect
{
private:
  std::vector<Point2D> data_points_;
  size_t num_data_;

public:
  /// \brief Construct a landmark detection object
  CircleDetect();

  /// \brief Construct a landmark detection object
  /// \param num The number of datapoint
  CircleDetect(const int num);

  /// \brief Update the data point
  /// \param data The data point
  void update_data(std::vector<Point2D> & data);

  /// \brief Compute the mean x coordinate
  /// \return The mean of x coordinates
  double compute_x_mean();

  /// \brief Compute the mean y coordinate
  /// \return The mean of y coordinate
  double compute_y_mean();

  /// \brief Shift x and y coordinate to that mean is 0
  /// \param x_mean The mean of x coordinate
  /// \param y_mean The mean of y coordinate
  void shift_coordinate(const double x_mean, const double y_mean);

  /// \brief Compute the mean of z, where z=x^2+y^2
  /// \return The mean of z
  double compute_z_mean();

  /// \brief Construct a Z matrix
  /// \return The Z matrix
  arma::mat construct_Z_mat();

  /// \brief Compute the value of M matrix
  /// \param Z_mat The Z matrix
  /// \return The M matrix
  arma::mat compute_M_mat(const arma::mat & Z_mat);

  /// \brief Construct the H matrix
  /// \param z_mean The mean of z
  /// \return The H matrix
  arma::mat construct_H_mat(const double z_mean);

  /// \brief Detect the circle
  /// \return The resulting detected landmark
  Landmark detect_circle();

  /// \brief Get all data points
  /// \return All data points
  std::vector<Point2D> & get_data_points();
};
} /// namespace turtlelib

#endif /// DETECT_HPP_INCLUDE_GUARD
