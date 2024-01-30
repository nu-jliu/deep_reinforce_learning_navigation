#include <rclcpp/rclcpp.hpp>
#include <catch_ros2/catch_ros2.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nuturtle_control/srv/initial_pose.hpp>

double odom_x;
double odom_y;
double odom_theta;

void sub_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_x = msg->pose.pose.position.x;
  odom_y = msg->pose.pose.position.y;
  odom_theta = msg->pose.pose.orientation.w;
}

TEST_CASE("Initial Pose service", "[odometry]") // Allen Liu
{
  auto node = rclcpp::Node::make_shared("turtle_robot_test");

  node->declare_parameter("test_duration", 2.0);
  const double TEST_DURATION = node->get_parameter("test_duration").as_double();
  auto sub_odom =
    node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, &sub_odom_callback);
  auto cli_init_pose = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");

  while (cli_init_pose->wait_for_service(std::chrono::duration<int>{1})) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Initial pose not available, waiting again ...");
  }

  auto request = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
  request->x = 0.0;
  request->y = 0.0;
  request->theta = 0.0;
  request->left_wheel = 0.0;
  request->right_wheel = 0.0;

  cli_init_pose->async_send_request(request);

  rclcpp::Time start = rclcpp::Clock().now();

  while (rclcpp::ok() &&
    (rclcpp::Clock().now() - start) < rclcpp::Duration::from_seconds(TEST_DURATION))
  {
  }

  CHECK_THAT(odom_x, Catch::Matchers::WithinAbs(0.0, 1e-15));
  CHECK_THAT(odom_y, Catch::Matchers::WithinAbs(0.0, 1e-15));
  CHECK_THAT(odom_theta, Catch::Matchers::WithinAbs(0.0, 1e-15));
}
