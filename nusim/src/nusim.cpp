/// \file nusim.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief This node simulates the turtlebot in a world
///
/// PUBLISHERS:
///     ~/timestep: [std_msgs/msg/UInt64]                 Current timestep
///     ~/walls:    [visualization_msgs/msg/MarkerArray]  Wall markers
///     ~/obstacles: [visualization_msgs/msg/MarkerArray] Obstacle markers
///
/// SERVICES:
///     ~/reset:    [std_srvs/srv/Empty] Reset the turtlebot
///     ~/teleport: [nusim/srv/Teleport] Teleport the turtlebot
///
/// PARAMETERS:
///     \param rate                   (double)    The rate of the simulator
///     \param x0                     (double)    Tnitial x value
///     \param y0                     (double)    Initial y value
///     \param theta0                 (double)    Initial theta value
///     \param arena_x_length         (double)    Length of arena in x direction
///     \param arena_y_length         (double)    Length of arena in y direction
///     \param obstacles/x            (double[])  X coordinates of obstacles
///     \param obstacles/y            (double[])  Y coordinates of obstacles
///     \param obstacles/r            (double)    Radius of obstacles
///     \param wheel_radius           (double)    Radius of the wheel
///     \param track_width            (double)    Distance between two wheels
///     \param motor_cmd_max          (int)       Maximum motor command velocity
///     \param motor_cmd_per_rad_sec  (double)    motor command per rad/s
///     \param encoder_ticks_per_rad  (double)    Encoder ticks per radian of rotation
///
/// \version 0.1
/// \date 2024-01-22
///
/// \copyright Copyright (c) 2024
#include <chrono>
#include <random>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

#include <std_srvs/srv/empty.hpp>
#include "nuturtle_interfaces/srv/teleport.hpp"
#include "nuturtle_interfaces/msg/obstacle_measurements.hpp"
#include "nuturtle_interfaces/msg/measurement.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/trig2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/wall_range.hpp"
#include "turtlelib/circ_line.hpp"


using rclcpp::QoS;
using rclcpp::Node;
using tf2_ros::TransformBroadcaster;

/// messages
using rcl_interfaces::msg::ParameterDescriptor;
using std_msgs::msg::UInt64;
using std_msgs::msg::Bool;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PolygonStamped;
using geometry_msgs::msg::Point32;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using nav_msgs::msg::Path;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::LaserScan;
using nuturtlebot_msgs::msg::WheelCommands;
using nuturtlebot_msgs::msg::SensorData;
using nuturtle_interfaces::msg::Measurement;
using nuturtle_interfaces::msg::ObstacleMeasurements;

/// services
using std_srvs::srv::Empty;
using nuturtle_interfaces::srv::Teleport;

/// \brief The state of the wall
enum WallState
{
  /// \brief The up side of the wall
  NORTH,

  /// \brief The down side of the wall
  SOUTH,

  /// \brief The left side of the wall
  WEST,

  /// \brief The right side of the wall
  EAST
};

/// \brief Simulate the turtlebot in a rviz world.
class NuSim : public Node
{
private:
  /// \brief Timer callback funcrion of the nusim node, calls at every cycle
  void timer_callback_()
  {
    current_time_ = get_clock()->now();

    if (!draw_only_) {
      UInt64 msg_timestep;
      msg_timestep.data = timestep_++;
      pub_timestep_->publish(msg_timestep);

      Bool msg_collision;
      msg_collision.data = collide_with_wall_;
      pub_collide_->publish(msg_collision);

      if (++count_ >= static_cast<int>(0.2 / period_)) {
        generate_sensor_obs_pos_();
        // publish_laser_scan_();
        publish_cell_scan_();
        // publish_obstacle_markers_();
        publish_fake_sensor_();
        count_ = 0;
      }

      // if (wheel_cmd_available_) {
      //   update_turtlebot_pos_();
      //   wheel_cmd_available_ = false;
      // }

      publish_sensor_data_();
      publish_path_();
      broadcast_tf_();
      publish_odom_();
      publish_polygon_();
    }
  }

  /// @brief Get the fake sensor scan data about obstacle positions
  void generate_sensor_obs_pos_()
  {
    obstacle_pos_sensor_.clear();

    for (size_t i = 0; i < obstacles_x_.size(); ++i) {
      const auto obs_x = obstacles_x_.at(i) + distribution_sensor_(generator_);
      const auto obs_y = obstacles_y_.at(i) + distribution_sensor_(generator_);

      turtlelib::Point2D obs_pos{obs_x, obs_y};
      obstacle_pos_sensor_.push_back(obs_pos);
    }
  }

  /// @brief Update the position of the turtlebot
  void update_turtlebot_pos_()
  {
    double left_wheel_speed = wheel_cmd_.left_velocity * motor_cmd_per_rad_sec_;
    double right_wheel_speed = wheel_cmd_.right_velocity * motor_cmd_per_rad_sec_;
    /// ##### Generating gaussian noice
    /// CITE: https://stackoverflow.com/questions/32889309/adding-gaussian-noise
    const auto left_wheel_noice = distribution_input_(generator_);
    const auto right_wheel_noice = distribution_input_(generator_);

    RCLCPP_DEBUG_STREAM(get_logger(), "noice: " << left_wheel_noice);
    if (
      !turtlelib::almost_equal(left_wheel_speed, 0.0, 1e-2) ||
      !turtlelib::almost_equal(right_wheel_speed, 0.0, 1e-2))
    {
      left_wheel_speed += left_wheel_noice;
      right_wheel_speed += right_wheel_noice;
    }
    /// ##### END CITATION
    const auto phi_left_new = turtlebot_.left_wheel() + left_wheel_speed * period_;
    const auto phi_right_new = turtlebot_.right_wheel() + right_wheel_speed * period_;

    const auto ita_left = distribution_slip_(generator_);
    const auto ita_right = distribution_slip_(generator_);

    const auto phi_left_slip = turtlebot_.left_wheel() + left_wheel_speed * period_ *
      (1.0 + ita_left);
    const auto phi_right_slip = turtlebot_.right_wheel() + right_wheel_speed * period_ *
      (1.0 + ita_right);

    turtlebot_.compute_fk(phi_left_slip, phi_right_slip);
    turtlebot_.update_wheel(phi_left_new, phi_right_new);

    check_collision_(turtle_x_, turtle_y_, turtle_theta_);

    turtle_x_ = turtlebot_.config_x();
    turtle_y_ = turtlebot_.config_y();
    turtle_theta_ = turtlebot_.config_theta();
  }

  /// @brief Check if turtlebot has collided with one of the obstacles
  /// @param pre_x previous x position
  /// @param pre_y previous y position
  void check_collision_(double pre_x, double pre_y, double pre_theta)
  {
    const auto robot_x = turtlebot_.config_x();
    const auto robot_y = turtlebot_.config_y();

    // for (std::size_t i = 0; i < obstacles_x_.size(); ++i) {
    //   const auto obs_x = obstacles_x_.at(i);
    //   const auto obs_y = obstacles_y_.at(i);
    //   const auto dist = sqrt(pow(obs_x - robot_x, 2.0) + pow(obs_y - robot_y, 2.0));

    //   if (dist < collision_radius_ + obstacle_radius_) {
    //     const auto dx = obs_x - robot_x;
    //     const auto dy = obs_y - robot_y;

    //     const auto theta_new = atan2(dy, dx);
    //     turtlebot_.update_config(pre_x, pre_y, theta_new);
    //     break;
    //   }
    // }

    for (size_t i = 0; i < cell_walls_.size(); ++i) {
      const turtlelib::Wall wall_curr = cell_walls_.at(i);
      const turtlelib::Point2D pA = wall_curr.start;
      const turtlelib::Point2D pB = wall_curr.end;
      const turtlelib::Point2D pO{robot_x, robot_y};

      if (turtlelib::line_circ_intersect(pA, pB, pO, collision_radius_)) {
        turtlebot_.update_config(pre_x, pre_y, pre_theta);
        collide_with_wall_ = true;
      }
    }
  }

  /// @brief broadcast the transform
  void broadcast_tf_()
  {
    TransformStamped tf;
    tf.header.stamp = current_time_;
    tf.header.frame_id = odom_frame_id_;
    tf.child_frame_id = body_frame_id_;

    tf.transform.translation.x = turtle_x_;
    tf.transform.translation.y = turtle_y_;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = sin(turtle_theta_ / 2.0);
    tf.transform.rotation.w = cos(turtle_theta_ / 2.0);

    tf_broadcaster_->sendTransform(tf);
  }

  void publish_odom_()
  {
    Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = body_frame_id_;

    odom.pose.pose.position.x = turtle_x_;
    odom.pose.pose.position.y = turtle_y_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = sin(turtle_theta_ / 2.0);
    odom.pose.pose.orientation.w = cos(turtle_theta_ / 2.0);

    pub_odom_->publish(odom);
  }

  /// \brief publish lidar data on laser scan message
  void publish_laser_scan_()
  {
    LaserScan msg;
    msg.header.stamp = current_time_;
    msg.header.frame_id = scan_frame_id_;

    msg.angle_min = -turtlelib::PI;
    msg.angle_max = turtlelib::PI;
    msg.angle_increment = lidar_resolution_;

    msg.range_max = lidar_range_max_;
    msg.range_min = lidar_range_min_;

    WallState wall = WallState::EAST;

    if (turtle_theta_ >= -turtlelib::PI / 4.0 && turtle_theta_ < turtlelib::PI / 4.0) {
      wall = WallState::WEST;
    } else if (turtle_theta_ >= turtlelib::PI / 4.0 && turtle_theta_ < turtlelib::PI * 0.75) {
      wall = WallState::SOUTH;
    } else if (turtle_theta_ >= turtlelib::PI * 0.75 || turtle_theta_ < -turtlelib::PI * 0.75) {
      wall = WallState::EAST;
    } else {
      wall = WallState::NORTH;
    }

    std::vector<turtlelib::Obstacle> obstacles;
    const turtlelib::Transform2D T_sb({0.032, 0.0}, 0.0);
    const turtlelib::Transform2D T_wb({turtle_x_, turtle_y_}, turtle_theta_);
    const turtlelib::Transform2D T_bw = T_wb.inv();
    const turtlelib::Transform2D T_sw = T_sb * T_bw;
    const turtlelib::Transform2D T_ws = T_sw.inv();
    RCLCPP_DEBUG_STREAM(get_logger(), "T_bs: " << T_bw);

    const auto x_scan = T_ws.translation().x;
    const auto y_scan = T_ws.translation().y;
    const auto theta_scan = T_ws.rotation();

    for (size_t i = 0; i < obstacles_x_.size(); ++i) {
      const turtlelib::Point2D pw{obstacles_x_.at(i), obstacles_y_.at(i)};
      const turtlelib::Point2D ps = T_sw(pw);
      obstacles.push_back({ps.x, ps.y, obstacle_radius_});
      RCLCPP_DEBUG_STREAM(get_logger(), "Ps: " << ps);
    }

    RCLCPP_DEBUG_STREAM(get_logger(), "theta: " << turtle_theta_);
    for (int i = 0; i < static_cast<int>(turtlelib::PI * 2.0 / lidar_resolution_); ++i) {
      const auto alpha = lidar_resolution_ * i - turtlelib::PI;
      std::vector<double> obs_dists;

      for (size_t j = 0; j < obstacles.size(); ++j) {
        const auto obs = obstacles.at(j);

        if (turtlelib::can_intersect(alpha, obs)) {
          const auto obs_dist = turtlelib::find_distance(alpha, obs);
          RCLCPP_DEBUG_STREAM(get_logger(), "distance: " << obs_dist);
          obs_dists.push_back(obs_dist + distribution_laser_(generator_));
        }
      }

      if (!obs_dists.empty()) {
        double min_dist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < obs_dists.size(); ++j) {
          const auto dist = obs_dists.at(j);

          if (dist < min_dist) {
            min_dist = dist;
          }
        }

        msg.ranges.push_back(min_dist);
      } else {
        switch (wall) {
          case WallState::EAST:
            {
              const auto d = arena_x_length_ / 2.0 - x_scan;
              const auto beta = turtlelib::normalize_angle(alpha + theta_scan);
              const auto range = d / cos(beta) + distribution_laser_(generator_);
              msg.ranges.push_back(range);

              const auto py = y_scan + d * tan(beta);
              if (py > arena_y_length_ / 2.0) {
                wall = WallState::NORTH;
              }
              break;
            }

          case WallState::NORTH:
            {
              const auto d = arena_y_length_ / 2.0 - y_scan;
              const auto beta = turtlelib::normalize_angle(alpha + theta_scan) -
                turtlelib::PI / 2.0;
              const auto range = d / cos(beta) + distribution_laser_(generator_);
              msg.ranges.push_back(range);

              const auto px = x_scan - d * tan(beta);
              if (px < -arena_x_length_ / 2.0) {
                wall = WallState::WEST;
              }
              break;
            }

          case WallState::WEST:
            {
              const auto d = arena_x_length_ / 2.0 + x_scan;
              const auto beta = turtlelib::normalize_angle(alpha + theta_scan) - turtlelib::PI;
              const auto range = d / cos(beta) + distribution_laser_(generator_);
              msg.ranges.push_back(range);

              const auto py = y_scan - d * tan(beta);
              if (py < -arena_y_length_ / 2.0) {
                wall = WallState::SOUTH;
              }
              break;
            }

          case WallState::SOUTH:
            {
              const auto d = arena_y_length_ / 2.0 + y_scan;
              const auto beta = turtlelib::normalize_angle(alpha + theta_scan) +
                turtlelib::PI / 2.0;
              const auto range = d / cos(beta) + distribution_laser_(generator_);
              msg.ranges.push_back(range);

              const auto px = x_scan + d * tan(beta);
              if (px > arena_x_length_ / 2.0) {
                wall = WallState::EAST;
              }
              break;
            }

          default:
            {
              RCLCPP_ERROR_STREAM(get_logger(), "Invalid state");
              throw std::logic_error("Invalid wall state");
              break;
            }
        }
      }
    }

    pub_laser_scan_->publish(msg);
    RCLCPP_DEBUG_STREAM(get_logger(), "laser published");
  }

  void publish_cell_scan_()
  {
    LaserScan msg_scan;
    msg_scan.header.stamp = current_time_;
    msg_scan.header.frame_id = scan_frame_id_;

    msg_scan.angle_min = -turtlelib::PI;
    msg_scan.angle_max = turtlelib::PI;
    msg_scan.angle_increment = lidar_resolution_;

    msg_scan.range_min = lidar_range_min_;
    msg_scan.range_max = lidar_range_max_;

    const turtlelib::Transform2D T_sb({0.032, 0.0}, 0.0);
    const turtlelib::Transform2D T_wb({turtle_x_, turtle_y_}, turtle_theta_);
    const turtlelib::Transform2D T_bw = T_wb.inv();
    const turtlelib::Transform2D T_sw = T_sb * T_bw;
    const turtlelib::Transform2D T_ws = T_sw.inv();

    const auto x_scan = T_ws.translation().x;
    const auto y_scan = T_ws.translation().y;
    const auto theta_scan = T_ws.rotation();

    const turtlelib::Point2D origin_scan{x_scan, y_scan};

    for (int i = 0; i < static_cast<int>(turtlelib::PI * 2.0 / lidar_resolution_); ++i) {
      const double alpha = turtlelib::normalize_angle(
        lidar_resolution_ * i - turtlelib::PI + theta_scan
      );

      std::vector<double> distances;

      for (size_t j = 0; j < cell_walls_.size(); ++j) {
        if (turtlelib::can_intersect(origin_scan, alpha, cell_walls_.at(j))) {
          const auto dist = turtlelib::find_wall_distance(origin_scan, alpha, cell_walls_.at(j));
          distances.push_back(dist);
        }
      }

      if (distances.empty()) {
        msg_scan.ranges.push_back(100.0);
      } else {
        const auto min_iter = std::min_element(distances.begin(), distances.end());
        const auto min_dist = *min_iter + distribution_laser_(generator_);
        msg_scan.ranges.push_back(min_dist);
        // RCLCPP_INFO_STREAM(get_logger(), min_dist);
      }

    }

    pub_laser_scan_->publish(msg_scan);
  }

  void publish_polygon_()
  {
    PolygonStamped msg;

    msg.header.stamp = current_time_;
    msg.header.frame_id = body_frame_id_;

    for (int i = 0; i < 100; ++i) {
      Point32 point;

      const auto theta = -turtlelib::PI + 2.0 * turtlelib::PI / 100 * i;

      point.x = collision_radius_ * cos(theta);
      point.y = collision_radius_ * sin(theta);
      point.z = 0.0;

      msg.polygon.points.push_back(point);
    }

    pub_footprint_->publish(msg);
  }

  /// @brief publish a path message that displays the of the robot on rviz
  void publish_path_()
  {
    PoseStamped pose_curr;

    pose_curr.header.stamp = current_time_;
    pose_curr.header.frame_id = odom_frame_id_;

    pose_curr.pose.position.x = turtle_x_;
    pose_curr.pose.position.y = turtle_y_;
    pose_curr.pose.position.z = 0.0;

    pose_curr.pose.orientation.x = 0.0;
    pose_curr.pose.orientation.y = 0.0;
    pose_curr.pose.orientation.z = sin(turtle_theta_ / 2.0);
    pose_curr.pose.orientation.w = cos(turtle_theta_ / 2.0);

    poses_.push_back(pose_curr);

    Path msg_path;

    msg_path.header.stamp = current_time_;
    msg_path.header.frame_id = odom_frame_id_;
    msg_path.poses = poses_;

    pub_path_->publish(msg_path);
  }

  /// @brief publish the sensor data of the turtlebot
  void publish_sensor_data_()
  {
    SensorData msg_sensor;

    msg_sensor.stamp = current_time_;
    msg_sensor.left_encoder =
      static_cast<int32_t>(turtlebot_.left_wheel() * encoder_ticks_per_rad_);
    msg_sensor.right_encoder =
      static_cast<int32_t>(turtlebot_.right_wheel() * encoder_ticks_per_rad_);

    pub_sensor_data_->publish(msg_sensor);
  }

  void parse_walls_()
  {
    for (size_t i = 0; i < wall_starts_x_.size(); ++i) {
      turtlelib::Wall wall_curr;
      const auto start_x = wall_starts_x_.at(i);
      const auto start_y = wall_starts_y_.at(i);
      const auto end_x = wall_ends_x_.at(i);
      const auto end_y = wall_ends_y_.at(i);

      wall_curr.start = {start_x, start_y};
      wall_curr.end = {end_x, end_y};
      cell_walls_.push_back(wall_curr);
    }
  }

  /// \brief publish the markers to display wall on rviz
  void publish_wall_markers_()
  {
    Marker m1;
    Marker m2;
    Marker m3;
    Marker m4;
    MarkerArray m_array;

    /// first wall
    m1.header.stamp = get_clock()->now();
    m1.header.frame_id = world_frame_id_;
    m1.id = 1;
    m1.type = Marker::CUBE;
    m1.action = Marker::ADD;
    m1.pose.position.x = arena_x_length_ / 2.0 + wall_thickness_ / 2.0;
    m1.pose.position.y = wall_thickness_ / 2.0;
    m1.pose.position.z = wall_height_ / 2.0;
    m1.scale.x = wall_thickness_;
    m1.scale.y = arena_y_length_ + wall_thickness_;
    m1.scale.z = wall_height_;
    m1.color.r = wall_r_;
    m1.color.g = wall_g_;
    m1.color.b = wall_b_;
    m1.color.a = 1.0;

    /// second wall
    m2.header.stamp = get_clock()->now();
    m2.header.frame_id = world_frame_id_;
    m2.id = 2;
    m2.type = Marker::CUBE;
    m2.action = Marker::ADD;
    m2.pose.position.x = -wall_thickness_ / 2.0;
    m2.pose.position.y = arena_y_length_ / 2.0 + wall_thickness_ / 2.0;
    m2.pose.position.z = wall_height_ / 2.0;
    m2.scale.x = arena_x_length_ + wall_thickness_;
    m2.scale.y = wall_thickness_;
    m2.scale.z = wall_height_;
    m2.color.r = wall_r_;
    m2.color.g = wall_g_;
    m2.color.b = wall_b_;
    m2.color.a = 1.0;

    /// third wall
    m3.header.stamp = get_clock()->now();
    m3.header.frame_id = world_frame_id_;
    m3.id = 3;
    m3.type = Marker::CUBE;
    m3.action = Marker::ADD;
    m3.pose.position.x = -arena_x_length_ / 2.0 - wall_thickness_ / 2.0;
    m3.pose.position.y = -wall_thickness_ / 2.0;
    m3.pose.position.z = wall_height_ / 2.0;
    m3.scale.x = wall_thickness_;
    m3.scale.y = arena_y_length_ + wall_thickness_;
    m3.scale.z = wall_height_;
    m3.color.r = wall_r_;
    m3.color.g = wall_g_;
    m3.color.b = wall_b_;
    m3.color.a = 1.0;

    /// fourth wall
    m4.header.stamp = get_clock()->now();
    m4.header.frame_id = world_frame_id_;
    m4.id = 4;
    m4.type = Marker::CUBE;
    m4.action = Marker::ADD;
    m4.pose.position.x = wall_thickness_ / 2.0;
    m4.pose.position.y = -arena_y_length_ / 2.0 - wall_thickness_ / 2.0;
    m4.pose.position.z = wall_height_ / 2.0;
    m4.scale.x = arena_x_length_ + wall_thickness_;
    m4.scale.y = wall_thickness_;
    m4.scale.z = wall_height_;
    m4.color.r = wall_r_;
    m4.color.g = wall_g_;
    m4.color.b = wall_b_;
    m4.color.a = 1.0;

    m_array.markers.push_back(m1);
    m_array.markers.push_back(m2);
    m_array.markers.push_back(m3);
    m_array.markers.push_back(m4);

    pub_wall_markers_->publish(m_array);
  }

  void publish_cell_markers_()
  {
    MarkerArray m_cell_array;

    for (size_t i = 0; i < cell_walls_.size(); ++i) {
      Marker m;

      const auto wall_curr = cell_walls_.at(i);
      const auto start_x = wall_curr.start.x;
      const auto start_y = wall_curr.start.y;
      const auto end_x = wall_curr.end.x;
      const auto end_y = wall_curr.end.y;

      m.header.stamp = get_clock()->now();
      m.header.frame_id = world_frame_id_;
      m.id = 100 + i;
      m.type = Marker::CUBE;
      m.action = Marker::ADD;
      m.pose.position.x = (start_x + end_x) / 2.0;
      m.pose.position.y = (start_y + end_y) / 2.0;
      m.pose.position.z = wall_height_ / 2.0;
      m.scale.x = abs(end_x - start_x) + 0.1;
      m.scale.y = abs(end_y - start_y) + 0.1;
      m.scale.z = wall_height_;
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.color.a = 0.5;

      m_cell_array.markers.push_back(m);
    }

    pub_cell_markers_->publish(m_cell_array);
  }

  /// \brief publish marker to display obstacle on rviz
  void publish_obstacle_markers_()
  {
    MarkerArray m_array_obs;

    for (std::size_t i = 0; i < obstacles_x_.size(); ++i) {
      const auto x_pos = obstacles_x_.at(i);
      const auto y_pos = obstacles_y_.at(i);

      Marker m_obs;

      m_obs.header.stamp = get_clock()->now();
      m_obs.header.frame_id = world_frame_id_;
      m_obs.id = i + 10;
      m_obs.type = Marker::CYLINDER;
      m_obs.action = Marker::ADD;
      m_obs.pose.position.x = x_pos;
      m_obs.pose.position.y = y_pos;
      m_obs.pose.position.z = obstacle_height_ / 2.0;
      m_obs.scale.x = 2.0 * obstacle_radius_;
      m_obs.scale.y = 2.0 * obstacle_radius_;
      m_obs.scale.z = obstacle_height_;
      m_obs.color.r = 1.0;
      m_obs.color.g = 0.0;
      m_obs.color.b = 0.0;
      m_obs.color.a = 1.0;

      m_array_obs.markers.push_back(m_obs);
    }

    pub_obstacle_markers_->publish(m_array_obs);
  }

  void publish_fake_sensor_()
  {

    // MarkerArray m_array_obs;
    MarkerArray m_array_sensor;
    ObstacleMeasurements m_measurements;

    turtlelib::Transform2D Tsb(turtlelib::Vector2D{turtle_x_, turtle_y_}, turtle_theta_);
    turtlelib::Transform2D Tbs = Tsb.inv();

    for (std::size_t i = 0; i < obstacles_x_.size(); ++i) {
      const auto x_pos = obstacles_x_.at(i);
      const auto y_pos = obstacles_y_.at(i);
      const auto dist = sqrt(pow(x_pos - turtle_x_, 2.0) + pow(y_pos - turtle_y_, 2.0));

      const auto ps = obstacle_pos_sensor_.at(i);
      const auto pb = Tbs(ps);

      Marker m_sensor;
      Measurement m_measure;

      /// Sensor marker
      m_sensor.header.stamp = current_time_;
      m_sensor.header.frame_id = body_frame_id_;
      m_sensor.id = i + 20;
      m_sensor.type = Marker::CYLINDER;
      m_sensor.pose.position.x = pb.x;
      m_sensor.pose.position.y = pb.y;
      m_sensor.pose.position.z = obstacle_height_ / 2.0;
      m_sensor.scale.x = 2.0 * obstacle_radius_;
      m_sensor.scale.y = 2.0 * obstacle_radius_;
      m_sensor.scale.z = obstacle_height_;
      m_sensor.color.r = 1.0;
      m_sensor.color.g = 1.0;
      m_sensor.color.b = 0.0;
      m_sensor.color.a = 1.0;

      if (dist < max_range_) {
        m_sensor.action = Marker::ADD;

        m_measure.x = pb.x;
        m_measure.y = pb.y;
        m_measure.uid = i;
      } else {
        m_sensor.action = Marker::DELETE;

        m_measure.x = 1e4;
        m_measure.y = 1e4;
        m_measure.uid = i;
      }

      m_array_sensor.markers.push_back(m_sensor);
      m_measurements.measurements.push_back(m_measure);
    }

    pub_fake_sensor_markers_->publish(m_array_sensor);
    pub_obstacles_->publish(m_measurements);
  }

  /// \brief reset the position of the turtlebot.
  void reset_turtle_pose_()
  {
    turtlebot_.update_config(x0_, y0_, theta0_);
    turtlebot_ = turtlelib::DiffDrive(track_width_, wheel_radius_);
  }

  /// \brief Callback function for reset service, reset the position of turtlebot and timestep
  /// \param request The request object
  /// \param response The response object
  void srv_reset_callback_(
    std::shared_ptr<Empty::Request> request,
    std::shared_ptr<Empty::Response> response)
  {
    (void) request;
    (void) response;

    RCLCPP_WARN_STREAM(get_logger(), "Resetting simulator");

    reset_turtle_pose_();
    poses_.clear();
    timestep_ = 0;
    collide_with_wall_ = false;
  }

  /// \brief Callback function of the teleport service, teleport the turtlebot to a place
  /// \param request The request object
  /// \param response The response object
  void srv_teleport_callback_(
    std::shared_ptr<Teleport::Request> request,
    std::shared_ptr<Teleport::Response> response)
  {
    const auto x = request->x;
    const auto y = request->y;
    const auto theta = request->theta;

    turtlebot_.update_config(x, y, theta);

    response->result = true;
  }

  /// @brief Callback function of the wheel_cmd message
  /// @param msg the wheel_cmd message
  void sub_wheel_cmd_callback_(WheelCommands::SharedPtr msg)
  {
    if (!wheel_cmd_available_) {
      wheel_cmd_available_ = true;
    }

    wheel_cmd_ = *msg;
    update_turtlebot_pos_();
  }

  /// timer
  rclcpp::TimerBase::SharedPtr timer_;

  /// services
  rclcpp::Service<Empty>::SharedPtr srv_reset_;
  rclcpp::Service<Teleport>::SharedPtr srv_teleport_;

  /// subscribers
  rclcpp::Subscription<WheelCommands>::SharedPtr sub_wheel_cmd_;

  /// publishers
  rclcpp::Publisher<UInt64>::SharedPtr pub_timestep_;
  rclcpp::Publisher<Bool>::SharedPtr pub_collide_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_wall_markers_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_cell_markers_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_obstacle_markers_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_fake_sensor_markers_;
  rclcpp::Publisher<SensorData>::SharedPtr pub_sensor_data_;
  rclcpp::Publisher<Path>::SharedPtr pub_path_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr pub_footprint_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<LaserScan>::SharedPtr pub_laser_scan_;
  rclcpp::Publisher<ObstacleMeasurements>::SharedPtr pub_obstacles_;

  /// transform broadcasters
  std::unique_ptr<TransformBroadcaster> tf_broadcaster_;

  /// qos profile
  rclcpp::QoS marker_qos_;
  rclcpp::QoS laser_qos_;

  /// Time stamp
  rclcpp::Time current_time_;

  /// subscribed messages
  WheelCommands wheel_cmd_;

  /// parameters
  double rate_;
  double x0_;
  double y0_;
  double theta0_;
  double arena_x_length_;
  double arena_y_length_;
  std::vector<double> obstacles_x_;
  std::vector<double> obstacles_y_;
  double obstacle_radius_;
  double wheel_radius_;
  double track_width_;
  int64_t motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
  double input_noice_;
  double slip_fraction_;
  double basic_sensor_variance_;
  double max_range_;
  double lidar_range_min_;
  double lidar_range_max_;
  double lidar_accuracy_;
  double lidar_resolution_;
  bool draw_only_;
  std::string odom_frame_id_;
  std::string world_frame_id_;
  std::vector<double> wall_starts_x_;
  std::vector<double> wall_starts_y_;
  std::vector<double> wall_ends_x_;
  std::vector<double> wall_ends_y_;

  /// other attributes
  int count_;
  double period_;
  uint64_t timestep_;
  double turtle_x_;
  double turtle_y_;
  double turtle_theta_;
  double wall_r_;
  double wall_g_;
  double wall_b_;
  double wall_height_;
  double wall_thickness_;
  double obstacle_height_;
  bool wheel_cmd_available_;
  bool collide_with_wall_;
  std::string body_frame_id_;
  std::string scan_frame_id_;
  turtlelib::DiffDrive turtlebot_;
  std::vector<PoseStamped> poses_;
  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_input_;
  std::uniform_real_distribution<double> distribution_slip_;
  std::normal_distribution<double> distribution_sensor_;
  std::normal_distribution<double> distribution_laser_;
  std::vector<turtlelib::Point2D> obstacle_pos_sensor_;
  std::vector<turtlelib::Wall> cell_walls_;

public:
  /// \brief Initialize the nusim node
  NuSim()
  : Node("nusim"), marker_qos_(10), laser_qos_(10), count_(0), timestep_(0), wall_r_(1.0),
    wall_g_(0.0), wall_b_(0.0), wall_height_(0.25), wall_thickness_(0.1), obstacle_height_(0.25),
    wheel_cmd_available_(false), collide_with_wall_(false), body_frame_id_("red/base_footprint"),
    scan_frame_id_("red/base_scan")
  {
    /// parameter descriptions
    ParameterDescriptor rate_des;
    ParameterDescriptor x0_des;
    ParameterDescriptor y0_des;
    ParameterDescriptor theta0_des;
    ParameterDescriptor arena_x_des;
    ParameterDescriptor arena_y_des;
    ParameterDescriptor obs_x_des;
    ParameterDescriptor obs_y_des;
    ParameterDescriptor obs_r_des;
    ParameterDescriptor wheel_radius_des;
    ParameterDescriptor track_width_des;
    ParameterDescriptor motor_cmd_max_des;
    ParameterDescriptor motor_cmd_per_rad_sec_des;
    ParameterDescriptor encoder_ticks_per_rad_des;
    ParameterDescriptor collision_radius_des;
    ParameterDescriptor input_noice_des;
    ParameterDescriptor slip_fraction_des;
    ParameterDescriptor basic_sensor_variance_des;
    ParameterDescriptor max_range_des;
    ParameterDescriptor lidar_range_min_des;
    ParameterDescriptor lidar_range_max_des;
    ParameterDescriptor lidar_accuracy_des;
    ParameterDescriptor lidar_resolution_des;
    ParameterDescriptor draw_only_des;
    ParameterDescriptor odom_frame_id_des;
    ParameterDescriptor world_frame_id_des;
    ParameterDescriptor wall_starts_x_des;
    ParameterDescriptor wall_starts_y_des;
    ParameterDescriptor wall_ends_x_des;
    ParameterDescriptor wall_ends_y_des;
    rate_des.description = "The rate of the simulator";
    x0_des.description = "The initial x location";
    y0_des.description = "The initial y location";
    theta0_des.description = "The initial theta pose";
    arena_x_des.description = "The wall length in x direction";
    arena_y_des.description = "The wall length in y direction";
    obs_x_des.description = "The list of x coordinates of the obstacles";
    obs_y_des.description = "The list of y coordinates of the obstacles";
    obs_r_des.description = "The radius of the obstacles";
    wheel_radius_des.description = "The radius of the wheel";
    track_width_des.description = "The width of the track";
    motor_cmd_max_des.description = "The maximum motor command";
    motor_cmd_per_rad_sec_des.description = "The motor command per rad/s speed";
    encoder_ticks_per_rad_des.description = "Number of encoder ticks per radian";
    collision_radius_des.description = "The collision radois of the robot";
    input_noice_des.description = "Gaussian noice variance";
    slip_fraction_des.description = "Fraction of the slip";
    basic_sensor_variance_des.description = "The variance of the sensor";
    max_range_des.description = "The maximum range of the sensor";
    lidar_range_min_des.description = "The minimum range of the lidar";
    lidar_range_max_des.description = "The maximum range of the lidar";
    lidar_accuracy_des.description = "The accuracy of the lidar";
    lidar_resolution_des.description = "The angular resolution of the lidar";
    draw_only_des.description = "Whether this is behave as draw only";
    odom_frame_id_des.description = "The frame id of the odom frame";
    world_frame_id_des.description = "The frame id of the world frame";
    wall_starts_x_des.description = "The start x coordinates of the wall";
    wall_starts_y_des.description = "The start y coordinates of the wall";
    wall_ends_x_des.description = "The end x coordinates of the wall";
    wall_ends_y_des.description = "The end y coordinates of the wall";

    /// declare parameters
    declare_parameter<double>("rate", 100.0, rate_des);
    declare_parameter<double>("x0", 0.0, x0_des);
    declare_parameter<double>("y0", 0.0, y0_des);
    declare_parameter<double>("theta0", 0.0, theta0_des);
    declare_parameter<double>("arena_x_length", 10.0, arena_x_des);
    declare_parameter<double>("arena_y_length", 10.0, arena_y_des);
    declare_parameter<std::vector<double>>(
      "obstacles.x",
      std::vector<double>{1.2, 2.3},
      obs_x_des
    );
    declare_parameter<std::vector<double>>(
      "obstacles.y",
      std::vector<double>{2.3, 4.5},
      obs_y_des
    );
    declare_parameter<double>("obstacles.r", 0.05, obs_r_des);
    declare_parameter<double>("wheel_radius", 0.033, wheel_radius_des);
    declare_parameter<double>("track_width", 0.16, track_width_des);
    declare_parameter<int64_t>("motor_cmd_max", 265, motor_cmd_max_des);
    declare_parameter<double>("motor_cmd_per_rad_sec", 0.024, motor_cmd_max_des);
    declare_parameter<double>("encoder_ticks_per_rad", 651.9, encoder_ticks_per_rad_des);
    declare_parameter<double>("collision_radius", 0.2, collision_radius_des);
    declare_parameter<double>("input_noice", 0.0, input_noice_des);
    declare_parameter<double>("slip_fraction", 0.0, slip_fraction_des);
    declare_parameter<double>("basic_sensor_variance", 0.02, basic_sensor_variance_des);
    declare_parameter<double>("max_range", 2.0, max_range_des);
    declare_parameter<double>("lidar_range_min", 0.12, lidar_range_min_des);
    declare_parameter<double>("lidar_range_max", 3.5, lidar_range_max_des);
    declare_parameter<double>("lidar_accuracy", 0.015, lidar_accuracy_des);
    declare_parameter<double>("lidar_resolution", 0.02, lidar_resolution_des);
    declare_parameter<bool>("draw_only", false, draw_only_des);
    declare_parameter<std::string>("odom_frame_id", "nusim/world", odom_frame_id_des);
    declare_parameter<std::string>("world_frame_id", "nusim/world", world_frame_id_des);
    declare_parameter<std::vector<double>>(
      "wall_starts.x",
      std::vector<double>{0.0},
      wall_starts_x_des
    );
    declare_parameter<std::vector<double>>(
      "wall_starts.y",
      std::vector<double>{0.0},
      wall_starts_y_des
    );
    declare_parameter<std::vector<double>>(
      "wall_ends.x",
      std::vector<double>{0.0},
      wall_ends_x_des
    );
    declare_parameter<std::vector<double>>(
      "wall_ends.y",
      std::vector<double>{0.0},
      wall_ends_y_des
    );


    /// get parameter values
    rate_ = get_parameter("rate").as_double();
    x0_ = get_parameter("x0").as_double();
    y0_ = get_parameter("y0").as_double();
    theta0_ = get_parameter("theta0").as_double();
    arena_x_length_ = get_parameter("arena_x_length").as_double();
    arena_y_length_ = get_parameter("arena_y_length").as_double();
    obstacles_x_ = get_parameter("obstacles.x").as_double_array();
    obstacles_y_ = get_parameter("obstacles.y").as_double_array();
    obstacle_radius_ = get_parameter("obstacles.r").as_double();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();
    input_noice_ = get_parameter("input_noice").as_double();
    slip_fraction_ = get_parameter("slip_fraction").as_double();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").as_double();
    max_range_ = get_parameter("max_range").as_double();
    lidar_range_min_ = get_parameter("lidar_range_min").as_double();
    lidar_range_max_ = get_parameter("lidar_range_max").as_double();
    lidar_accuracy_ = get_parameter("lidar_accuracy").as_double();
    lidar_resolution_ = get_parameter("lidar_resolution").as_double();
    draw_only_ = get_parameter("draw_only").as_bool();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    world_frame_id_ = get_parameter("world_frame_id").as_string();
    wall_starts_x_ = get_parameter("wall_starts.x").as_double_array();
    wall_starts_y_ = get_parameter("wall_starts.y").as_double_array();
    wall_ends_x_ = get_parameter("wall_ends.x").as_double_array();
    wall_ends_y_ = get_parameter("wall_ends.y").as_double_array();


    /// check for x y length
    if (obstacles_x_.size() != obstacles_y_.size()) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "The x and y coordinates should have the same length"
      );
      throw std::invalid_argument("The x and y coordinate should have same length");
      exit(EXIT_FAILURE);
    }

    if (!draw_only_) {
      /// initialize attributes
      period_ = 1.0 / rate_;
      distribution_input_ = std::normal_distribution<double>(0.0, sqrt(input_noice_));
      distribution_slip_ = std::uniform_real_distribution<double>(-slip_fraction_, slip_fraction_);
      distribution_sensor_ = std::normal_distribution<double>(0.0, sqrt(basic_sensor_variance_));
      distribution_laser_ = std::normal_distribution<double>(0.0, sqrt(basic_sensor_variance_));
      reset_turtle_pose_();

      // set marker qos policy
      marker_qos_.transient_local();
      // laser_qos_.transient_local();
      laser_qos_.best_effort();
      laser_qos_.durability_volatile();

      /// timer
      timer_ = create_wall_timer(
        std::chrono::duration<long double>{period_},
        std::bind(&NuSim::timer_callback_, this));

      /// services
      srv_reset_ = create_service<Empty>(
        "~/reset",
        std::bind(
          &NuSim::srv_reset_callback_, this, std::placeholders::_1,
          std::placeholders::_2));
      srv_teleport_ =
        create_service<Teleport>(
        "~/teleport",
        std::bind(
          &NuSim::srv_teleport_callback_, this, std::placeholders::_1,
          std::placeholders::_2));

      // subscribers
      sub_wheel_cmd_ =
        create_subscription<WheelCommands>(
        "red/wheel_cmd", 10,
        std::bind(&NuSim::sub_wheel_cmd_callback_, this, std::placeholders::_1));

      // publishers
      pub_timestep_ = create_publisher<UInt64>("~/timestep", 10);
      pub_collide_ = create_publisher<Bool>("~/collide", 10);
      pub_sensor_data_ = create_publisher<SensorData>("red/sensor_data", 10);
      pub_fake_sensor_markers_ = create_publisher<MarkerArray>("fake_sensor", marker_qos_);
      pub_path_ = create_publisher<Path>("~/path", 10);
      pub_footprint_ = create_publisher<PolygonStamped>("~/footprint", 10);
      pub_odom_ = create_publisher<Odometry>("~/odom", 10);
      pub_laser_scan_ = create_publisher<LaserScan>("scan", laser_qos_);
      pub_obstacles_ = create_publisher<ObstacleMeasurements>("obs_pos", 10);
    }
    pub_wall_markers_ = create_publisher<MarkerArray>("~/walls", marker_qos_);
    pub_cell_markers_ = create_publisher<MarkerArray>("~/cells", marker_qos_);
    pub_obstacle_markers_ = create_publisher<MarkerArray>("~/obstacles", marker_qos_);

    /// transform broadcasters
    tf_broadcaster_ = std::make_unique<TransformBroadcaster>(*this);

    parse_walls_();
    publish_wall_markers_();
    publish_cell_markers_();
    publish_obstacle_markers_();
  }
};

/// \brief The main function for the nusim node
/// \param argc number of arguments
/// \param argv value of the arguments
/// \return result code
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_nusim = std::make_shared<NuSim>();
  rclcpp::spin(node_nusim);

  rclcpp::shutdown();
  return 0;
}
