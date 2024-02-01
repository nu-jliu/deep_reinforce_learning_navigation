#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

using rclcpp::Node;
using tf2_ros::TransformBroadcaster;

using rcl_interfaces::msg::ParameterDescriptor;
using sensor_msgs::msg::JointState;
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::TransformStamped;

using nuturtle_control::srv::InitialPose;

/// @brief
class Odom : public Node
{
private:
  /// @brief
  void timer_callback__()
  {
    broadcast_tf__();
  }

  /// @brief
  /// @param msg
  void sub_joint_states_callback__(JointState::SharedPtr msg)
  {

    if (!joint_states_available__) {
      joint_states_available__ = true;
      joint_states_curr__ = *msg;

      for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name.at(i) == wheel_left__) {
          index_left__ = i;
        } else if (msg->name.at(i) == wheel_right__) {
          index_right__ = i;
        }
      }

      if (index_left__ == INT64_MAX || index_right__ == INT64_MAX) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid wheel joint name");
        exit(EXIT_FAILURE);
      }
    }

    joint_states_prev__ = joint_states_curr__;
    joint_states_curr__ = *msg;

    publish_odom__();
  }

  /// @brief
  /// @param request
  /// @param respose
  void srv_initial_pose_callback__(
    std::shared_ptr<InitialPose::Request> request,
    std::shared_ptr<InitialPose::Response> respose)
  {
    double x = request->x;
    double y = request->y;
    double theta = request->theta;
    double left_wheel = request->left_wheel;
    double right_wheel = request->right_wheel;

    turtlebot__.update_config(x, y, theta, left_wheel, right_wheel);

    respose->success;
  }

  /// @brief
  void publish_odom__()
  {
    Odometry msg_odom;

    double phi_left = joint_states_curr__.position.at(index_left__);
    double phi_right = joint_states_curr__.position.at(index_right__);

    turtlelib::Twist2D twist_turtle = turtlebot__.compute_fk(phi_left, phi_right);

    msg_odom.header.stamp = this->get_clock()->now();
    msg_odom.header.frame_id = odom_id__;
    msg_odom.child_frame_id = body_id__;

    msg_odom.pose.pose.position.x = turtlebot__.config_x();
    msg_odom.pose.pose.position.y = turtlebot__.config_y();
    msg_odom.pose.pose.position.z = 0.0;

    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = sin(turtlebot__.config_theta() / 2.0);
    msg_odom.pose.pose.orientation.w = cos(turtlebot__.config_theta() / 2.0);

    double t_prev = (double) joint_states_prev__.header.stamp.sec +
      (double) joint_states_prev__.header.stamp.nanosec * 1e-9;
    double t_curr = (double) joint_states_curr__.header.stamp.sec +
      (double) joint_states_curr__.header.stamp.nanosec * 1e-9;
    double dt = t_curr - t_prev;

    msg_odom.twist.twist.linear.x = twist_turtle.x / dt;
    msg_odom.twist.twist.linear.y = twist_turtle.y / dt;
    msg_odom.twist.twist.linear.z = 0.0;

    msg_odom.twist.twist.angular.x = 0.0;
    msg_odom.twist.twist.angular.y = 0.0;
    msg_odom.twist.twist.angular.z = twist_turtle.omega / dt;

    pub_odometry__->publish(msg_odom);
  }

  /// @brief
  void broadcast_tf__()
  {
    TransformStamped tf_msg;

    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = odom_id__;
    tf_msg.child_frame_id = body_id__;

    tf_msg.transform.translation.x = turtlebot__.config_x();
    tf_msg.transform.translation.y = turtlebot__.config_y();
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = sin(turtlebot__.config_theta() / 2.0);
    tf_msg.transform.rotation.w = cos(turtlebot__.config_theta() / 2.0);

    tf_broadcater__->sendTransform(tf_msg);
  }

  rclcpp::TimerBase::SharedPtr timer__;

  rclcpp::Subscription<JointState>::SharedPtr sub_joint_states__;

  rclcpp::Publisher<Odometry>::SharedPtr pub_odometry__;

  std::unique_ptr<TransformBroadcaster> tf_broadcater__;

  rclcpp::Service<InitialPose>::SharedPtr srv_initial_pose__;

  JointState joint_states_curr__;
  JointState joint_states_prev__;
  Point odom_position__;
  Quaternion odom_orientation__;

  std::string body_id__;
  std::string odom_id__;
  std::string wheel_left__;
  std::string wheel_right__;
  double track_width__;
  double wheel_radius__;

  turtlelib::DiffDrive turtlebot__;
  bool joint_states_available__;
  size_t index_left__;
  size_t index_right__;

public:
  /// @brief
  Odom()
  : Node("odometry"), joint_states_available__(false), index_left__(SIZE_MAX), index_right__(
      SIZE_MAX)
  {
    ParameterDescriptor body_id_des;
    ParameterDescriptor odom_id_des;
    ParameterDescriptor wheel_left_des;
    ParameterDescriptor wheel_right_des;
    ParameterDescriptor wheel_radius_des;
    ParameterDescriptor track_width_des;

    body_id_des.description = "The name of the body frame of the robot.";
    odom_id_des.description = "The name of the odometry frame.";
    wheel_left_des.description = "The name of the left wheel joint.";
    wheel_right_des.description = "The name of the right wheel joint.";
    wheel_radius_des.description = "Wheel radius of the turtlebot.";
    track_width_des.description = "Track width of the turtlebot.";

    this->declare_parameter<std::string>("body_id", "", body_id_des);
    this->declare_parameter<std::string>("odom_id", "odom", odom_id_des);
    this->declare_parameter<std::string>("wheel_left", "", wheel_left_des);
    this->declare_parameter<std::string>("wheel_right", "", wheel_right_des);
    this->declare_parameter<double>("wheel_radius", 0.033, wheel_radius_des);
    this->declare_parameter<double>("track_width", 0.16, track_width_des);

    body_id__ = this->get_parameter("body_id").as_string();
    odom_id__ = this->get_parameter("odom_id").as_string();
    wheel_left__ = this->get_parameter("wheel_left").as_string();
    wheel_right__ = this->get_parameter("wheel_right").as_string();
    wheel_radius__ = this->get_parameter("wheel_radius").as_double();
    track_width__ = this->get_parameter("track_width").as_double();

    turtlebot__ = turtlelib::DiffDrive(track_width__, wheel_radius__);

    if (body_id__.size() == 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid body id: " << body_id__);
      exit(EXIT_FAILURE);
    }

    if (wheel_left__.size() == 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid left wheel: " << wheel_left__);
      exit(EXIT_FAILURE);
    }

    if (wheel_right__.size() == 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid right wheel: " << wheel_right__);
      exit(EXIT_FAILURE);
    }

    tf_broadcater__ = std::make_unique<TransformBroadcaster>(*this);

    timer__ = this->create_wall_timer(10ms, std::bind(&Odom::timer_callback__, this));

    sub_joint_states__ =
      this->create_subscription<JointState>(
      "joint_states", 10,
      std::bind(&Odom::sub_joint_states_callback__, this, std::placeholders::_1));

    pub_odometry__ = this->create_publisher<Odometry>("odom", 10);

    srv_initial_pose__ =
      this->create_service<InitialPose>(
      "initial_pose",
      std::bind(
        &Odom::srv_initial_pose_callback__, this, std::placeholders::_1,
        std::placeholders::_2));
  }
};

/// @brief
/// @param argc
/// @param argv
/// @return
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_odom = std::make_shared<Odom>();
  rclcpp::spin(node_odom);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
