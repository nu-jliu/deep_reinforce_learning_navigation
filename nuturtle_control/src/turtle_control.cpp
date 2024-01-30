/**
 * @file turtle_control.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-27
 *
 * @copyright Copyright (c) 2024
 *
 */
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

/// @brief
class TurtleControl : public rclcpp::Node
{
private:
  /// @brief
  void timer_callback__()
  {
    if (cmd_twist_available__) {
      publish_wheel_cmd__();
    }

    if (sensor_data_available__) {
      publish_joint_states__();
    }
  }

  /// @brief
  /// @param msg
  void sub_cmd_vel_callback__(const Twist::SharedPtr msg)
  {
    if (!cmd_twist_available__) {
      cmd_twist_available__ = true;
    }

    cmd_twist__ = *msg;
  }

  void sub_sensor_data_callback__(const SensorData::SharedPtr msg)
  {
    if (!sensor_data_available__) {
      sensor_data_available__ = true;
    }

    sensor_data__ = *msg;
  }

  void publish_wheel_cmd__()
  {
    WheelCommands msg;

    double omega_z = cmd_twist__.angular.z;
    double v_x = cmd_twist__.linear.x;
    double v_y = cmd_twist__.linear.y;

    turtlelib::WheelSpeed phidot = turtlebot__.compute_ik(turtlelib::Twist2D{omega_z, v_x, v_y});

    msg.left_velocity = (int32_t) (phidot.left / motor_cmd_per_rad_sec__);
    msg.right_velocity = (int32_t) (phidot.right / motor_cmd_per_rad_sec__);

    pub_wheel_cmd__->publish(msg);
  }

  void publish_joint_states__()
  {
    double phi_left = (double) sensor_data__.left_encoder / (double) encoder_tick_per_rad__;
    double phi_right = (double) sensor_data__.right_encoder / (double) encoder_tick_per_rad__;

    turtlebot__.compute_fk(phi_left, phi_right);

    JointState msg;

    msg.header.frame_id = "nusim/world";
    msg.header.stamp = this->get_clock()->now();
    msg.name = std::vector<std::string>{"wheel_left_joint", "wheel_right_joint"};
    msg.position = std::vector<double>{phi_left, phi_right};

    pub_joint_states__->publish(msg);
  }

  /// Timer
  rclcpp::TimerBase::SharedPtr timer__;

  /// Subscriptions
  rclcpp::Subscription<Twist>::SharedPtr sub_cmd_vel__;
  rclcpp::Subscription<SensorData>::SharedPtr sub_sensor_data__;

  /// Publishers
  rclcpp::Publisher<WheelCommands>::SharedPtr pub_wheel_cmd__;
  rclcpp::Publisher<JointState>::SharedPtr pub_joint_states__;

  /// Subcribed messages
  Twist cmd_twist__;
  SensorData sensor_data__;

  /// Parameters
  double turtlebot_track_width__;
  double turtlebot_wheel_radius__;
  int encoder_tick_per_rad__;
  double motor_cmd_per_rad_sec__;

  /// Other attributes
  turtlelib::DiffDrive turtlebot__;
  bool cmd_twist_available__;
  bool sensor_data_available__;

public:
  TurtleControl()
  : Node("turtle_control"), cmd_twist_available__(false), sensor_data_available__(false)
  {
    ParameterDescriptor track_witdth_des;
    ParameterDescriptor wheel_radius_des;
    ParameterDescriptor encoder_ticks_des;
    ParameterDescriptor mcu_per_vel_des;

    track_witdth_des.description = "Track width of the turtlebot";
    wheel_radius_des.description = "Wheel radius of the turtlebot";
    encoder_ticks_des.description = "Encoder ticks per radian";
    mcu_per_vel_des.description = "Speed per motor mcu";

    this->declare_parameter<double>("track_width", 160e-3, track_witdth_des);
    this->declare_parameter<double>("wheel_radius", 33e-3, wheel_radius_des);
    this->declare_parameter<int>("encoder_ticks_per_rad", 4096, encoder_ticks_des);
    this->declare_parameter<double>("motor_cmd_per_rad_sec", 0.024, mcu_per_vel_des);

    turtlebot_track_width__ = this->get_parameter("track_width").as_double();
    turtlebot_wheel_radius__ = this->get_parameter("wheel_radius").as_double();
    encoder_tick_per_rad__ = this->get_parameter("encoder_ticks_per_rad").as_int();
    motor_cmd_per_rad_sec__ = this->get_parameter("motor_cmd_per_rad_sec").as_double();

    turtlebot__ = turtlelib::DiffDrive(turtlebot_track_width__, turtlebot_wheel_radius__);

    timer__ = this->create_wall_timer(100ms, std::bind(&TurtleControl::timer_callback__, this));

    sub_cmd_vel__ = this->create_subscription<Twist>(
      "cmd_vel", 10,
      std::bind(&TurtleControl::sub_cmd_vel_callback__, this, std::placeholders::_1));
    sub_sensor_data__ =
      this->create_subscription<SensorData>(
      "sensor_data", 10,
      std::bind(&TurtleControl::sub_sensor_data_callback__, this, std::placeholders::_1));

    pub_wheel_cmd__ = this->create_publisher<WheelCommands>("wheel_cmd", 10);
    pub_joint_states__ = this->create_publisher<JointState>("joint_states", 10);
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_control = std::make_shared<TurtleControl>();
  rclcpp::spin(node_control);

  rclcpp::shutdown();
  return 0;
}
