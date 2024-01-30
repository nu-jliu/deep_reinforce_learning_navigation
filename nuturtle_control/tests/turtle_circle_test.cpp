#include <rclcpp/rclcpp.hpp>
#include <catch_ros2/catch_ros2.hpp>

#include <geometry_msgs/msg/twist.hpp>

rclcpp::Time time_prev;
rclcpp::Duration time_diff;

TEST_CASE("Test cmd_vel frequency", "[circle]")
{
  auto node = rclcpp::Node::make_shared("turtle_circle_test");
  node->declare_parameter<double>("test_duration", 2.0);
  const double TEST_DURATION = node->get_parameter("test_duration").as_double();

  time_prev = rclcpp::Clock().now();

  node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    10,
    [node](geometry_msgs::msg::Twist::SharedPtr msg) {
      time_diff = rclcpp::Clock().now() - time_prev;
      time_prev = rclcpp::Clock().now();
    }
  );

  rclcpp::Time start = rclcpp::Clock().now();

  while (rclcpp::ok() &&
    (rclcpp::Clock().now() - start) < rclcpp::Duration::from_seconds(TEST_DURATION))
  {
    rclcpp::spin_some(node);
  }

  double frequency = 1.0 / time_diff.seconds();

  CHECK_THAT(frequency, Catch::Matchers::WithinAbs(100.0, 1.0));
}
