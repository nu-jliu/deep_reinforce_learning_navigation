#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <catch_ros2/catch_ros2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nuturtle_control/srv/initial_pose.hpp>

using namespace std::chrono_literals;

double odom_x;
double odom_y;
double odom_theta;

TEST_CASE("Initial Pose service", "[odometry]") // Allen Liu
{
  odom_x = odom_y = odom_theta = 0.0;

  auto node = rclcpp::Node::make_shared("turtle_odom_test");

  node->declare_parameter<double>("test_duration", 2.0);
  const double TEST_DURATION = node->get_parameter("test_duration").as_double();

  auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    10,
    [node](nav_msgs::msg::Odometry::SharedPtr msg) {
      odom_x = msg->pose.pose.position.x;
      odom_y = msg->pose.pose.position.y;
      odom_theta = msg->pose.pose.orientation.w;
    }
  );
  auto cli_init_pose = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");

  bool service_found = false;

  rclcpp::Time start = rclcpp::Clock().now();


  while (rclcpp::ok() &&
    (rclcpp::Clock().now() - start) < rclcpp::Duration::from_seconds(TEST_DURATION))
  {
    if (cli_init_pose->wait_for_service(0s)) {
      service_found = true;
      break;
    }

    rclcpp::spin_some(node);
  }

  CHECK(service_found);


  auto request = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
  request->x = 0.0;
  request->y = 0.0;
  request->theta = 0.0;
  request->left_wheel = 0.0;
  request->right_wheel = 0.0;

  cli_init_pose->async_send_request(request);

  CHECK_THAT(odom_x, Catch::Matchers::WithinAbs(0.0, 1e-15));
  CHECK_THAT(odom_y, Catch::Matchers::WithinAbs(0.0, 1e-15));
  CHECK_THAT(odom_theta, Catch::Matchers::WithinAbs(0.0, 1e-15));
}

TEST_CASE("Test Transform", "[odometry]")
{
  auto node = rclcpp::Node::make_shared("turtle_odom_test");

  node->declare_parameter<double>("test_duration", 2.0);
  const double TEST_DURATION = node->get_parameter("test_duration").as_double();

  auto buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_unique<tf2_ros::TransformListener>(*buffer);

  rclcpp::Time time = rclcpp::Clock().now();
  geometry_msgs::msg::TransformStamped tf;

  bool tf_published = false;

  while (rclcpp::ok() &&
    (rclcpp::Clock().now() - time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  {
    try {
      tf = buffer->lookupTransform(
        "base_footprint", "odom",
        tf2::TimePointZero);
      tf_published = true;
      break;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "Unable to look for transform from base_footprint to odom: ");
    }
  }

  CHECK(tf_published);
  CHECK_THAT(tf.transform.translation.x, Catch::Matchers::WithinAbs(0.0, 1e-15));
  CHECK_THAT(tf.transform.translation.y, Catch::Matchers::WithinAbs(0.0, 1e-15));
  CHECK_THAT(tf.transform.rotation.w, Catch::Matchers::WithinAbs(0.0, 1e-15));

}
