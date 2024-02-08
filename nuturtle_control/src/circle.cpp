///
/// @file circle.cpp
/// @author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// @brief The node that publish command to have the robot to run in a circle
///
/// PARAMETERS:
///    \param frequency The frequency that node runs
///
/// SERVICES:
///    revserse  [std_srvs/srv/Empty]            Reserve the direction of the robot.
///    stop      [std:srvs/srv/Empty]            Stop the robot.
///    control   [nuturtle_control/srv/Control]  Start robot in a circle.
///
/// PUBLISHERS:
///    cmd_vel   [geometry_msgs/msg/Twist] The speed command sent to robot.
///
/// @version 0.1
/// @date 2024-02-08
///
/// @copyright Copyright (c) 2024
///
///

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_srvs/srv/empty.hpp>
#include "nuturtle_control/srv/control.hpp"

using rclcpp::Node;

using rcl_interfaces::msg::ParameterDescriptor;
using geometry_msgs::msg::Twist;

using std_srvs::srv::Empty;
using nuturtle_control::srv::Control;

/// @brief
class Circle : public Node
{
private:
  /// \brief Timer callback
  void timer_callback__()
  {
    publish_twist__();
  }

  /// \brief publish a twist
  void publish_twist__()
  {
    Twist msg_twist;

    if (!is_stopped__) {
      msg_twist.linear.x = velocity__;
      msg_twist.linear.y = 0.0;
      msg_twist.linear.z = 0.0;

      msg_twist.angular.x = 0.0;
      msg_twist.angular.y = 0.0;
      msg_twist.angular.z = velocity__ / radius__;
    } else {
      msg_twist.linear.x = 0.0;
      msg_twist.linear.y = 0.0;
      msg_twist.linear.z = 0.0;

      msg_twist.angular.x = 0.0;
      msg_twist.angular.y = 0.0;
      msg_twist.angular.z = 0.0;
    }

    pub_twist__->publish(msg_twist);
  }

  /// \brief Control service callback function
  /// \param request Control service request
  /// \param response Control service response
  void srv_control_callback__(
    std::shared_ptr<Control::Request> request,
    std::shared_ptr<Control::Response> response)
  {
    velocity__ = request->velocity;
    radius__ = request->radius;
    is_stopped__ = false;

    response->success = true;
  }

  /// \brief Reverse service callback function
  /// \param request Reverse service request
  /// \param response Reverse service response
  void srv_reserse_callback__(
    std::shared_ptr<Empty::Request> request,
    std::shared_ptr<Empty::Response> response)
  {
    (void) request;
    (void) response;

    velocity__ *= -1.0;
  }

  /// \brief Stop service callback function
  /// \param request Stop service request
  /// \param response Stop sercuce response
  void srv_stop_callback__(
    std::shared_ptr<Empty::Request> request,
    std::shared_ptr<Empty::Response> response)
  {
    (void) request;
    (void) response;

    is_stopped__ = true;
  }

  rclcpp::TimerBase::SharedPtr timer__;

  rclcpp::Service<Empty>::SharedPtr srv_reserve__;
  rclcpp::Service<Empty>::SharedPtr srv_stop__;
  rclcpp::Service<Control>::SharedPtr srv_control__;

  rclcpp::Publisher<Twist>::SharedPtr pub_twist__;

  double frequency__;

  double velocity__;
  double radius__;
  bool is_stopped__;

public:
  /// @brief
  Circle()
  : Node("circle"), velocity__(0.0), radius__(0.0), is_stopped__(true)
  {
    ParameterDescriptor frequency_des;

    frequency_des.description = "Frequency of the circle node.";

    this->declare_parameter<double>("frequency", 100.0, frequency_des);

    frequency__ = this->get_parameter("frequency").as_double();

    timer__ =
      this->create_wall_timer(
      std::chrono::duration<long double>{1.0 / frequency__},
      std::bind(&Circle::timer_callback__, this));

    srv_reserve__ =
      this->create_service<Empty>(
      "reverse",
      std::bind(
        &Circle::srv_reserse_callback__, this, std::placeholders::_1,
        std::placeholders::_2));
    srv_stop__ =
      this->create_service<Empty>(
      "stop",
      std::bind(&Circle::srv_stop_callback__, this, std::placeholders::_1, std::placeholders::_2));
    srv_control__ =
      this->create_service<Control>(
      "control",
      std::bind(
        &Circle::srv_control_callback__, this, std::placeholders::_1,
        std::placeholders::_2));

    pub_twist__ = this->create_publisher<Twist>("cmd_vel", 10);

  }
};

///
/// @brief The main entry of the circle node
///
/// @param argc The number of arguments
/// @param argv The value of arguments
/// @return int The result code.
///
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_circle = std::make_shared<Circle>();
  rclcpp::spin(node_circle);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
