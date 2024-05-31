///
/// \file turtle_control.cpp
/// \author Allen (jingkunliu2025@u.northwestern.edu)
/// \brief Controls the motion of the robot.
///
/// PARAMETERS:
///   \param track_width            [double] The distance between the wheels.
///   \param wheel_radius           [double] The radius of the wheel.
///   \param encoder_ticks_per_rad  [double] Encoder ticks per radian.
///   \param mcu_per_rad_sec        [double] MCU per angular velocity.
///
/// SUBSCRIPTIONS:
///   cmd_vel:      [geometry_msg/msg/Twist]          The command velocity.
///   sensor_data:  [nuturtlebot_msg/msg/SensorData]  The data from all sensors of the turtlebot.
///
/// PUBLISHERS:
///   wheel_cmd:    [nuturtlebot/msg/WheelCommands] The wheel velocity command.
///   joint_states: [sensor_msgs/msg/JointState]    The joint states of the robot.
///
///
/// \version 0.1
/// \date 2024-01-27
///
/// \copyright Copyright (c) 2024
///
///
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

using rcl_interfaces::msg::ParameterDescriptor;
using nuturtlebot_msgs::msg::WheelCommands;
using nuturtlebot_msgs::msg::SensorData;
using sensor_msgs::msg::JointState;
using geometry_msgs::msg::Twist;

/// @brief The class of the turtle_control node
class TurtleControl : public rclcpp::Node
{
private:
  /// @brief The timer callback of the turtle_control node.
  void timer_callback_()
  {
    if (cmd_twist_available_) {
      publish_wheel_cmd_();
    }

    if (sensor_data_available_) {
      publish_joint_states_();
    }
  }

  /// @brief The subcriber callback of the cmd_vel message
  /// @param msg The subscribed message object
  void sub_cmd_vel_callback_(const Twist::SharedPtr msg)
  {
    if (!cmd_twist_available_) {
      cmd_twist_available_ = true;
    }

    cmd_twist_.linear.x = msg->linear.x;
    cmd_twist_.linear.y = msg->linear.y;
    cmd_twist_.linear.z = msg->linear.z;

    cmd_twist_.angular.x = msg->angular.x;
    cmd_twist_.angular.y = msg->angular.y;
    cmd_twist_.angular.z = msg->angular.z;
  }

  /// @brief The subcriber callback of the sensor_data message
  /// @param msg The subscribed sensor data object
  void sub_sensor_data_callback_(const SensorData::SharedPtr msg)
  {
    if (!sensor_data_available_) {
      sensor_data_available_ = true;
      // sensor_data_curr_ = *msg;
      sensor_data_curr_.stamp = msg->stamp;
      sensor_data_curr_.left_encoder = msg->left_encoder;
      sensor_data_curr_.right_encoder = msg->right_encoder;
    }

    sensor_data_prev_ = sensor_data_curr_;
    // sensor_data_curr_ = *msg;
    sensor_data_curr_.stamp = msg->stamp;
    sensor_data_curr_.left_encoder = msg->left_encoder;
    sensor_data_curr_.right_encoder = msg->right_encoder;
  }

  /// @brief Publish wheel command message.
  void publish_wheel_cmd_()
  {
    WheelCommands msg;

    const auto omega_z = cmd_twist_.angular.z;
    const auto v_x = cmd_twist_.linear.x;
    const auto v_y = cmd_twist_.linear.y;

    turtlelib::WheelSpeed phidot = turtlebot_.compute_ik(turtlelib::Twist2D{omega_z, v_x, v_y});

    auto left_vel = static_cast<int32_t>(phidot.left / motor_cmd_per_rad_sec_);
    auto right_vel = static_cast<int32_t>(phidot.right / motor_cmd_per_rad_sec_);

    if (left_vel > motor_cmd_max_) {
      left_vel = motor_cmd_max_;
    } else if (left_vel < -motor_cmd_max_) {
      left_vel = -motor_cmd_max_;
    }

    if (right_vel > motor_cmd_max_) {
      right_vel = motor_cmd_max_;
    } else if (right_vel < -motor_cmd_max_) {
      right_vel = -motor_cmd_max_;
    }

    msg.left_velocity = left_vel;
    msg.right_velocity = right_vel;

    pub_wheel_cmd_->publish(msg);
    cmd_twist_available_ = false;
  }

  /// @brief Publish joint state message.
  void publish_joint_states_()
  {
    const auto phi_left = static_cast<double>(sensor_data_curr_.left_encoder) /
      encoder_tick_per_rad_;
    const auto phi_right = static_cast<double>(sensor_data_curr_.right_encoder) /
      encoder_tick_per_rad_;
    const auto phi_left_prev = static_cast<double>(sensor_data_prev_.left_encoder) /
      encoder_tick_per_rad_;
    const auto phi_right_prev = static_cast<double>(sensor_data_prev_.right_encoder) /
      encoder_tick_per_rad_;

    const auto t_curr = static_cast<double>(sensor_data_curr_.stamp.sec) +
      static_cast<double>(sensor_data_curr_.stamp.nanosec) * 1e-9;
    const auto t_prev = static_cast<double>(sensor_data_prev_.stamp.sec) +
      static_cast<double>(sensor_data_prev_.stamp.nanosec) * 1e-9;
    const auto dt = t_curr - t_prev;

    const auto phidot_left = (phi_left - phi_left_prev) / dt;
    const auto phidot_right = (phi_right - phi_right_prev) / dt;


    turtlebot_.compute_fk(phi_left, phi_right);

    JointState msg;

    msg.header.frame_id = "nusim/world";
    msg.header.stamp = get_clock()->now();
    msg.name = std::vector<std::string>{"wheel_left_joint", "wheel_right_joint"};
    msg.position = std::vector<double>{phi_left, phi_right};
    msg.velocity = std::vector<double>{phidot_left, phidot_right};

    pub_joint_states_->publish(msg);
  }

  /// Timer
  rclcpp::TimerBase::SharedPtr timer_;

  /// Subscriptions
  rclcpp::Subscription<Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<SensorData>::SharedPtr sub_sensor_data_;

  /// Publishers
  rclcpp::Publisher<WheelCommands>::SharedPtr pub_wheel_cmd_;
  rclcpp::Publisher<JointState>::SharedPtr pub_joint_states_;

  /// Subcribed messages
  Twist cmd_twist_;
  SensorData sensor_data_curr_;
  SensorData sensor_data_prev_;

  /// Parameters
  double turtlebot_track_width_;
  double turtlebot_wheel_radius_;
  double encoder_tick_per_rad_;
  double motor_cmd_per_rad_sec_;
  int32_t motor_cmd_max_;

  /// Other attributes
  turtlelib::DiffDrive turtlebot_;
  bool cmd_twist_available_;
  bool sensor_data_available_;

public:
  /// @brief The turtle_control constructor, initialize the environment.
  TurtleControl()
  : Node("turtle_control"), cmd_twist_available_(false), sensor_data_available_(false)
  {
    ParameterDescriptor track_witdth_des;
    ParameterDescriptor wheel_radius_des;
    ParameterDescriptor encoder_ticks_des;
    ParameterDescriptor mcu_per_vel_des;
    ParameterDescriptor motor_cmd_max_des;

    track_witdth_des.description = "Track width of the turtlebot";
    wheel_radius_des.description = "Wheel radius of the turtlebot";
    encoder_ticks_des.description = "Encoder ticks per radian";
    mcu_per_vel_des.description = "Speed per motor mcu";
    motor_cmd_max_des.description = "Maximum velocity";

    declare_parameter<double>("track_width", 160e-3, track_witdth_des);
    declare_parameter<double>("wheel_radius", 33e-3, wheel_radius_des);
    declare_parameter<double>("encoder_ticks_per_rad", 651.9, encoder_ticks_des);
    declare_parameter<double>("motor_cmd_per_rad_sec", 0.024, mcu_per_vel_des);
    declare_parameter<int32_t>("motor_cmd_max", 265, motor_cmd_max_des);

    turtlebot_track_width_ = get_parameter("track_width").as_double();
    turtlebot_wheel_radius_ = get_parameter("wheel_radius").as_double();
    encoder_tick_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_int();

    turtlebot_ = turtlelib::DiffDrive(turtlebot_track_width_, turtlebot_wheel_radius_);

    timer_ = create_wall_timer(10ms, std::bind(&TurtleControl::timer_callback_, this));

    sub_cmd_vel_ = create_subscription<Twist>(
      "cmd_vel", 10,
      std::bind(&TurtleControl::sub_cmd_vel_callback_, this, std::placeholders::_1));
    sub_sensor_data_ =
      create_subscription<SensorData>(
      "sensor_data", 10,
      std::bind(&TurtleControl::sub_sensor_data_callback_, this, std::placeholders::_1));

    pub_wheel_cmd_ = create_publisher<WheelCommands>("wheel_cmd", 10);
    pub_joint_states_ = create_publisher<JointState>("joint_states", 10);
  }
};

/// @brief The main function of the turtle_control node.
/// @param argc number of arguments.
/// @param argv value of the arguments.
/// @return result code.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_control = std::make_shared<TurtleControl>();
  rclcpp::spin(node_control);

  rclcpp::shutdown();
  return 0;
}
