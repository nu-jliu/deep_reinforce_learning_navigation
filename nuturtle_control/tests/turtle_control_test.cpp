#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <catch_ros2/catch_ros2.hpp>

int32_t left_vel;
int32_t right_vel;

std::string wheel_left;
std::string wheel_right;
double left_wheel_pos;
double right_wheel_pos;

void sub_wheel_cmd_callback(nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
{
  left_vel = msg->left_velocity;
  right_vel = msg->right_velocity;
}

void sub_joint_states_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
  wheel_left = msg->name.at(0);
  wheel_right = msg->name.at(1);
  left_wheel_pos = msg->position.at(0);
  right_wheel_pos = msg->position.at(1);
}

TEST_CASE("Test cmd_vel pure translation", "[cmd_vel]") // Allen Liu
{
  left_vel = 0;
  right_vel = 0;

  auto node = rclcpp::Node::make_shared("turtle_control_test_node");
  node->declare_parameter<double>("test_duration", 2.0);

  const auto TEST_DURATION = node->get_parameter("test_duration").as_double();

  auto pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto sub_wheel_cmd = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd",
    10,
    &sub_wheel_cmd_callback);

  rclcpp::Time start = rclcpp::Clock().now();

  while (rclcpp::ok() &&
    (rclcpp::Clock().now() - start) < rclcpp::Duration::from_seconds(TEST_DURATION))
  {
    geometry_msgs::msg::Twist msg_twist;
    msg_twist.linear.x = 1.0;
    msg_twist.linear.y = 0.0;
    msg_twist.linear.z = 0.0;

    msg_twist.angular.x = 0.0;
    msg_twist.angular.y = 0.0;
    msg_twist.angular.z = 0.0;

    pub_twist->publish(msg_twist);

    rclcpp::spin_some(node);
  }

  CHECK(left_vel == 1262);
  CHECK(right_vel == 1262);
}

TEST_CASE("Test cmd_vel pure rotation", "[cmd_vel]") // Allen Liu
{
  left_vel = 0;
  right_vel = 0;

  auto node = rclcpp::Node::make_shared("turtle_control_test_node");
  node->declare_parameter<double>("test_duration", 2.0);

  const auto TEST_DURATION = node->get_parameter("test_duration").as_double();

  auto pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto sub_wheel_cmd = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd",
    10,
    &sub_wheel_cmd_callback);

  rclcpp::Time start = rclcpp::Clock().now();

  while (rclcpp::ok() &&
    (rclcpp::Clock().now() - start) < rclcpp::Duration::from_seconds(TEST_DURATION))
  {
    geometry_msgs::msg::Twist msg_twist;
    msg_twist.linear.x = 0.0;
    msg_twist.linear.y = 0.0;
    msg_twist.linear.z = 0.0;

    msg_twist.angular.x = 0.0;
    msg_twist.angular.y = 0.0;
    msg_twist.angular.z = 1.0;

    pub_twist->publish(msg_twist);

    rclcpp::spin_some(node);
  }

  CHECK(left_vel == -101);
  CHECK(right_vel == 101);
}

TEST_CASE("Test Sensors", "[joint_states]") // Allen Liu
{
  auto node = rclcpp::Node::make_shared("turtle_control_test_node");
  node->declare_parameter<double>("test_duration", 2.0);

  const auto TEST_DURATION = node->get_parameter("test_duration").as_double();

  auto pub_sensor = node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
  auto sub_joint_states = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    10,
    &sub_joint_states_callback);

  rclcpp::Time start = rclcpp::Clock().now();

  while (rclcpp::ok() &&
    (rclcpp::Clock().now() - start) < rclcpp::Duration::from_seconds(TEST_DURATION))
  {
    nuturtlebot_msgs::msg::SensorData msg_sensor;

    msg_sensor.left_encoder = 4096;
    msg_sensor.right_encoder = 1024;

    pub_sensor->publish(msg_sensor);

    rclcpp::spin_some(node);
  }

  CHECK(wheel_left == "wheel_left_joint");
  CHECK(wheel_right == "wheel_right_joint");
  CHECK_THAT(left_wheel_pos, Catch::Matchers::WithinAbs(1.0, 1e-15));
  CHECK_THAT(right_wheel_pos, Catch::Matchers::WithinAbs(0.25, 1e-15));
}
