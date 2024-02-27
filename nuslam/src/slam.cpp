///
/// \file slam.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief Update the odometry of a robot using SLAM algorithm.
///
/// PARAMETERS:
///   \param body_id      [string]  The id of the body frame
///   \param odom_id      [string]  The id of the odom frame
///   \param wheel_left   [string]  The left wheel joint name
///   \param wheel_right  [string]  The right wheel joint name
///   \param wheel_radius [double]  The radius of ythe wheel
///   \param track_width  [double]  The distance between two wheels
///
/// SUBSCRIPTIONS:
///   joint_states  [sensor_msgs/msg/JointState]                    The joint state of the robot.
///   obs_pos       [nuturtle_interfaces/msg/ObstacleMeasurements]  The measurements of the obstacles
///
/// PUBLISHERS:
///   odom          [nav_msgs/msg/Odomoetry]                        The odometry of the node.
///
/// SERVICES:
///   initial_pose [nuturtle_interfaces/srv/InitialPose]            Reset the initial pose.
///
/// \version 0.1
/// \date 2024-02-15
///
/// \copyright Copyright (c) 2024
///
///
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/qos.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "nuturtle_interfaces/msg/obstacle_measurements.hpp"

#include "nuturtle_interfaces/srv/initial_pose.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf_slam.hpp"

using namespace std::chrono_literals;

using rclcpp::Node;
using tf2_ros::TransformBroadcaster;

using rcl_interfaces::msg::ParameterDescriptor;
using sensor_msgs::msg::JointState;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::PoseStamped;
using nuturtle_interfaces::msg::ObstacleMeasurements;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using nuturtle_interfaces::srv::InitialPose;

/// @brief
class Slam : public Node
{
private:
  /// @brief The timer callback of the odometry node
  void timer_callback_()
  {
    if (joint_states_available_) {
      broadcast_tf_();
      publish_odom_();
      publish_path_();
    }
  }

  /// @brief Update the turtlebot configuration based on new joint states,
  ///        then publish new odom
  /// @param msg The subscibed joint_states message.
  void sub_joint_states_callback_(JointState::SharedPtr msg)
  {
    if (!joint_states_available_) {
      joint_states_available_ = true;
      joint_states_curr_.header.stamp = msg->header.stamp;
      joint_states_curr_.name = msg->name;
      joint_states_curr_.position = msg->position;

      for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name.at(i) == wheel_left_) {
          index_left_ = i;
          left_init_ = msg->position.at(i);
        } else if (msg->name.at(i) == wheel_right_) {
          index_right_ = i;
          right_init_ = msg->position.at(i);
        }
      }

      if (index_left_ == INT64_MAX || index_right_ == INT64_MAX) {
        RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel joint name");
        throw std::logic_error("Invalid wheel joint name");
      }

      turtlebot_.update_wheel(left_init_, right_init_);
    }

    joint_states_prev_ = joint_states_curr_;
    // joint_states_curr_ = *msg;
    joint_states_curr_.header.stamp = msg->header.stamp;
    joint_states_curr_.name = msg->name;
    joint_states_curr_.position = msg->position;

    // publish_odom_();
  }

  /// @brief The callback function of the obstacle measurement
  /// @param msg The subcribed message object
  void sub_obs_measure_callback_(ObstacleMeasurements::SharedPtr msg)
  {
    obs_measure_ = *msg;

    std::vector<turtlelib::Measurement> obstacles;
    obstacles.clear();

    for (size_t i = 0; i < msg->measurements.size(); ++i) {
      const auto x = msg->measurements.at(i).x;
      const auto y = msg->measurements.at(i).y;
      const auto uid = msg->measurements.at(i).uid;

      obstacles.push_back({x, y, uid});
    }

    if (turtle_slam_.is_landmark_pos_ready()) {

      const auto x_est = turtlebot_.config_x();
      const auto y_est = turtlebot_.config_y();
      const auto theta_est = turtlebot_.config_theta();

      const auto state_prev = turtle_slam_.get_robot_state();

      const turtlelib::Twist2D odom_twist{
        turtlelib::normalize_angle(theta_est - state_prev.theta),
        x_est - state_prev.x,
        y_est - state_prev.y
      };

      const auto A_mat = turtle_slam_.get_A_mat(odom_twist);
      RCLCPP_DEBUG_STREAM(get_logger(), "A_mat: " << std::endl << A_mat);

      turtle_slam_.update_state(x_est, y_est, theta_est);

      const auto Sigma_old = turtle_slam_.get_covariance_mat();
      const auto Sigma_est = A_mat * Sigma_old * A_mat.t() + Q_mat_;
      RCLCPP_DEBUG_STREAM(get_logger(), "Sigma_mat: " << std::endl << Sigma_est);

      arma::vec state_curr = turtle_slam_.get_state_vec(obstacles);
      arma::mat Sigma_curr(Sigma_est);
      RCLCPP_DEBUG_STREAM(get_logger(), "State: " << std::endl << state_curr);

      for (size_t i = 0; i < obstacles.size(); ++i) {
        const auto uid = obstacles.at(i).uid;

        const arma::vec z_vec = turtle_slam_.get_h_vec(obstacles.at(i));
        RCLCPP_INFO_STREAM(get_logger(), "z_vec: " << std::endl << z_vec);

        const arma::mat H_mat = turtle_slam_.get_H_mat(obstacles.at(i), i);
        RCLCPP_DEBUG_STREAM(get_logger(), "H_mat: " << std::endl << H_mat);

        const arma::mat K_mat = Sigma_curr * H_mat.t() *
          arma::inv(H_mat * Sigma_curr * H_mat.t() + sensor_noice_);
        RCLCPP_DEBUG_STREAM(get_logger(), "K_mat: " << std::endl << K_mat);

        const auto landmark_pos = turtle_slam_.get_landmark_pos(uid);
        if (turtlelib::almost_equal(landmark_pos.x, 100.0) &&
          turtlelib::almost_equal(landmark_pos.y, 100.0))
        {
          continue;
        }

        turtlelib::Point2D ps{landmark_pos.x, landmark_pos.y};
        turtlelib::Transform2D Tsb(
          turtlelib::Vector2D{state_curr.at(1), state_curr.at(2)},
          state_curr.at(0)
        );
        turtlelib::Transform2D Tbs = Tsb.inv();
        turtlelib::Point2D pb = Tbs(ps);
        const auto dx = pb.x;
        const auto dy = pb.y;
        RCLCPP_INFO_STREAM(get_logger(), "x: " << dx << ", y: " << dy);

        const arma::vec z_hat = turtle_slam_.get_h_vec({dx, dy, uid});// + dist_sensor_(generator_);
        RCLCPP_INFO_STREAM(get_logger(), "z_hat: " << std::endl << z_hat);

        arma::vec update = K_mat * (z_vec - z_hat);
        RCLCPP_INFO_STREAM(get_logger(), "state update: " << std::endl << z_vec - z_hat);

        const arma::mat I_mat(2 * num_obstacles_ + 3, 2 * num_obstacles_ + 3, arma::fill::eye);

        state_curr += update;
        Sigma_curr = (I_mat - K_mat * H_mat) * Sigma_curr;
      }

      const auto theta_new = state_curr.at(0);
      const auto x_new = state_curr.at(1);
      const auto y_new = state_curr.at(2);

      turtlebot_.update_config(x_new, y_new, theta_new);
      turtle_slam_.update_state(x_new, y_new, theta_new);
      turtle_slam_.update_covariance(Sigma_curr);
    }

    turtle_slam_.update_landmark_pos(obstacles);
    publish_map_markers();
  }

  /// @brief The initial pose service callback function
  /// @param request The initial pose service request
  /// @param respose The initial pose service response
  void srv_initial_pose_callback_(
    std::shared_ptr<InitialPose::Request> request,
    std::shared_ptr<InitialPose::Response> respose
  )
  {
    double x = request->x;
    double y = request->y;
    double theta = request->theta;

    turtlebot_.update_config(x, y, theta);

    respose->success = true;
  }

  void publish_map_markers()
  {
    MarkerArray map_array_msg;
    std::vector<turtlelib::Measurement> landmarks = turtle_slam_.get_all_landmarks();

    for (size_t i = 0; i < landmarks.size(); ++i) {
      const auto x = landmarks.at(i).x;
      const auto y = landmarks.at(i).y;

      if (!turtlelib::almost_equal(x, 100.0) && !turtlelib::almost_equal(y, 100.0)) {
        Marker m;

        m.header.stamp = get_clock()->now();
        m.header.frame_id = odom_id_;
        m.id = 30 + i;
        m.type = Marker::CYLINDER;
        m.action = Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = marker_height_ / 2.0;
        m.scale.x = 2.0 * marker_radius_;
        m.scale.y = 2.0 * marker_radius_;
        m.scale.z = marker_height_;
        m.color.r = marker_r_ / 255.0;
        m.color.g = marker_g_ / 255.0;
        m.color.b = marker_b_ / 255.0;
        m.color.a = 0.8;

        map_array_msg.markers.push_back(m);
      }
    }

    pub_map_array_->publish(map_array_msg);
  }

  void publish_path_()
  {
    Path msg_path;

    msg_path.header.stamp = get_clock()->now();
    msg_path.header.frame_id = "nusim/world";

    PoseStamped pose;

    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "nusim/world";

    pose.pose.position.x = turtlebot_.config_x();
    pose.pose.position.y = turtlebot_.config_y();
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = sin(turtlebot_.config_theta() / 2.0);
    pose.pose.orientation.w = cos(turtlebot_.config_theta() / 2.0);

    poses_.push_back(pose);
    msg_path.poses = poses_;

    pub_path_->publish(msg_path);
  }

  /// @brief publish the odometry
  void publish_odom_()
  {
    Odometry msg_odom;

    const auto phi_left = joint_states_curr_.position.at(index_left_);
    const auto phi_right = joint_states_curr_.position.at(index_right_);

    turtlelib::Twist2D twist_turtle = turtlebot_.compute_fk(phi_left, phi_right);

    msg_odom.header.stamp = get_clock()->now();
    msg_odom.header.frame_id = odom_id_;
    msg_odom.child_frame_id = body_id_;

    msg_odom.pose.pose.position.x = turtlebot_.config_x();
    msg_odom.pose.pose.position.y = turtlebot_.config_y();
    msg_odom.pose.pose.position.z = 0.0;

    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = sin(turtlebot_.config_theta() / 2.0);
    msg_odom.pose.pose.orientation.w = cos(turtlebot_.config_theta() / 2.0);

    const auto t_prev = static_cast<double>(joint_states_prev_.header.stamp.sec) +
      static_cast<double>(joint_states_prev_.header.stamp.nanosec) * 1e-9;
    const auto t_curr = static_cast<double>(joint_states_curr_.header.stamp.sec) +
      static_cast<double>(joint_states_curr_.header.stamp.nanosec) * 1e-9;
    const auto dt = t_curr - t_prev;

    msg_odom.twist.twist.linear.x = twist_turtle.x / dt;
    msg_odom.twist.twist.linear.y = twist_turtle.y / dt;
    msg_odom.twist.twist.linear.z = 0.0;

    msg_odom.twist.twist.angular.x = 0.0;
    msg_odom.twist.twist.angular.y = 0.0;
    msg_odom.twist.twist.angular.z = twist_turtle.omega / dt;

    pub_odometry_->publish(msg_odom);
  }

  /// @brief Broadcast the transform
  void broadcast_tf_()
  {
    TransformStamped tf_msg;

    tf_msg.header.stamp = get_clock()->now();
    tf_msg.header.frame_id = odom_id_;
    tf_msg.child_frame_id = body_id_;

    tf_msg.transform.translation.x = turtlebot_.config_x();
    tf_msg.transform.translation.y = turtlebot_.config_y();
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = sin(turtlebot_.config_theta() / 2.0);
    tf_msg.transform.rotation.w = cos(turtlebot_.config_theta() / 2.0);

    tf_broadcater_->sendTransform(tf_msg);
  }

  /// Timer
  rclcpp::TimerBase::SharedPtr timer_;

  /// Subscriber
  rclcpp::Subscription<JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<ObstacleMeasurements>::SharedPtr sub_obs_measure_;

  /// Publisher
  rclcpp::Publisher<Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<Path>::SharedPtr pub_path_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_map_array_;

  /// TF Broadcaster
  std::unique_ptr<TransformBroadcaster> tf_broadcater_;

  /// Service
  rclcpp::Service<InitialPose>::SharedPtr srv_initial_pose_;

  /// QoS
  rclcpp::QoS marker_qos_;

  /// Subscribed messages
  JointState joint_states_curr_;
  JointState joint_states_prev_;
  Point odom_position_;
  Quaternion odom_orientation_;
  ObstacleMeasurements obs_measure_;

  /// parameters
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  double track_width_;
  double wheel_radius_;
  double input_noice_;
  double sensor_noice_;

  /// other attributes
  bool joint_states_available_;
  size_t index_left_;
  size_t index_right_;
  double left_init_;
  double right_init_;
  std::vector<PoseStamped> poses_;
  arma::mat Q_mat_;
  int num_obstacles_;
  turtlelib::DiffDrive turtlebot_;
  turtlelib::EKF turtle_slam_;
  std::default_random_engine generator_;
  std::normal_distribution<double> dist_sensor_;
  double marker_radius_;
  double marker_height_;
  double marker_r_;
  double marker_g_;
  double marker_b_;

public:
  /// @brief
  Slam()
  : Node("odometry"), marker_qos_(10), joint_states_available_(false), index_left_(SIZE_MAX),
    index_right_(SIZE_MAX), num_obstacles_(20), turtle_slam_(num_obstacles_),
    marker_radius_(0.05), marker_height_(0.3), marker_r_(78.0), marker_g_(42.0),
    marker_b_(132.0)
  {
    ParameterDescriptor body_id_des;
    ParameterDescriptor odom_id_des;
    ParameterDescriptor wheel_left_des;
    ParameterDescriptor wheel_right_des;
    ParameterDescriptor wheel_radius_des;
    ParameterDescriptor track_width_des;
    ParameterDescriptor input_noice_des;
    ParameterDescriptor sensor_noice_des;

    body_id_des.description = "The name of the body frame of the robot.";
    odom_id_des.description = "The name of the odometry frame.";
    wheel_left_des.description = "The name of the left wheel joint.";
    wheel_right_des.description = "The name of the right wheel joint.";
    wheel_radius_des.description = "Wheel radius of the turtlebot.";
    track_width_des.description = "Track width of the turtlebot.";
    input_noice_des.description = "Input noice of the robot";
    sensor_noice_des.description = "Sensor noice of the robot";

    declare_parameter<std::string>("body_id", "", body_id_des);
    declare_parameter<std::string>("odom_id", "odom", odom_id_des);
    declare_parameter<std::string>("wheel_left", "", wheel_left_des);
    declare_parameter<std::string>("wheel_right", "", wheel_right_des);
    declare_parameter<double>("wheel_radius", 0.033, wheel_radius_des);
    declare_parameter<double>("track_width", 0.16, track_width_des);
    declare_parameter<double>("input_noice", 0.1, input_noice_des);
    declare_parameter<double>("basic_sensor_variance", 0.1, sensor_noice_des);

    body_id_ = get_parameter("body_id").as_string();
    odom_id_ = get_parameter("odom_id").as_string();
    wheel_left_ = get_parameter("wheel_left").as_string();
    wheel_right_ = get_parameter("wheel_right").as_string();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    input_noice_ = get_parameter("input_noice").as_double();
    sensor_noice_ = get_parameter("basic_sensor_variance").as_double();

    dist_sensor_ = std::normal_distribution<double>(0.0, sqrt(sensor_noice_));
    turtlebot_ = turtlelib::DiffDrive(track_width_, wheel_radius_);

    const arma::mat upper_left = arma::mat(3, 3, arma::fill::eye) * input_noice_;
    const arma::mat upper_right(3, 2 * num_obstacles_, arma::fill::zeros);
    const arma::mat bottom_left(2 * num_obstacles_, 3, arma::fill::zeros);
    const arma::mat bottom_right(2 * num_obstacles_, 2 * num_obstacles_, arma::fill::zeros);

    Q_mat_ = arma::join_horiz(
      arma::join_vert(upper_left, bottom_left),
      arma::join_vert(upper_right, bottom_right));

    if (body_id_.size() == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid body id: " << body_id_);
      exit(EXIT_FAILURE);
    }

    if (wheel_left_.size() == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid left wheel: " << wheel_left_);
      exit(EXIT_FAILURE);
    }

    if (wheel_right_.size() == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid right wheel: " << wheel_right_);
      exit(EXIT_FAILURE);
    }

    /// QoS
    marker_qos_.transient_local();

    /// Transform broadcaster
    tf_broadcater_ = std::make_unique<TransformBroadcaster>(*this);

    /// Timer
    timer_ = create_wall_timer(5ms, std::bind(&Slam::timer_callback_, this));

    /// Subscriptions
    sub_joint_states_ =
      create_subscription<JointState>(
      "joint_states",
      10,
      std::bind(
        &Slam::sub_joint_states_callback_,
        this,
        std::placeholders::_1));
    sub_obs_measure_ =
      create_subscription<ObstacleMeasurements>(
      "obs_pos",
      10,
      std::bind(
        &Slam::sub_obs_measure_callback_,
        this,
        std::placeholders::_1));

    /// Publishers
    pub_odometry_ = create_publisher<Odometry>("odom", 10);
    pub_path_ = create_publisher<Path>("~/path", 10);
    pub_map_array_ = create_publisher<MarkerArray>("~/map", marker_qos_);

    /// Services
    srv_initial_pose_ =
      create_service<InitialPose>(
      "initial_pose",
      std::bind(
        &Slam::srv_initial_pose_callback_,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }
};

/// @brief The main entry of the node
/// @param argc The number of arguments
/// @param argv The value of arguments
/// @return The result code.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_slam = std::make_shared<Slam>();
  rclcpp::spin(node_slam);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
