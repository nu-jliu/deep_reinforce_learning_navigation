#include <rclcpp/rclcpp.hpp>
#include <catch_ros2/catch_ros2.hpp>

#include <geometry_msgs/msg/twist.hpp>

volatile int num_pubs = 0;
volatile bool get_message = false;

void sub_cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  (void) msg;

  get_message = true;
  ++num_pubs;
}

TEST_CASE("Test cmd_vel frequency", "[circle]") // Allen Liu
{
  auto node = rclcpp::Node::make_shared("turtle_circle_test");

  auto sub_cmd_vel = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10,
    &sub_cmd_vel_callback
  );

  while (node->count_publishers("cmd_vel") < 1) {
    RCLCPP_INFO_STREAM(node->get_logger(), "no cmd_vel publisher, waiting again");
  }

  rclcpp::Time start = rclcpp::Clock().now();

  while (rclcpp::ok() &&
    (rclcpp::Clock().now() - start) < rclcpp::Duration::from_seconds(5.0))
  {
    rclcpp::spin_some(node);
  }

  CHECK(get_message);
  CHECK_THAT(num_pubs, Catch::Matchers::WithinAbs(500, 20));
}
