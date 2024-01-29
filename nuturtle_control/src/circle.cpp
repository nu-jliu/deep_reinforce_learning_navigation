#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_srvs/srv/empty.hpp>

using rclcpp::Node;

using rcl_interfaces::msg::ParameterDescriptor;
using geometry_msgs::msg::Twist;

using std_srvs::srv::Empty;

class Circle : public Node
{
private:
  void timer_callback__()
  {
    publish_twist__();
  }

  void publish_twist__()
  {
    Twist msg_twist;

    if (is_stopped__) {
      msg_twist.linear.x = 0.0;
      msg_twist.linear.y = 0.0;
      msg_twist.linear.z = 0.0;

      msg_twist.angular.x = 0.0;
      msg_twist.angular.y = 0.0;
      msg_twist.angular.z = 0.0;
    } else {
      msg_twist.linear.x = velocity__;
      msg_twist.linear.y = 0.0;
      msg_twist.linear.z = 0.0;

      msg_twist.angular.x = 0.0;
      msg_twist.angular.y = 0.0;
      msg_twist.angular.z = velocity__ / radius__;
    }

    pub_twist__->publish(msg_twist);
  }

  void srv_reserse_callback__(
    std::shared_ptr<Empty::Request> request,
    std::shared_ptr<Empty::Response> response)
  {
    (void) request;
    (void) response;

    velocity__ *= -1.0;
  }

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

  rclcpp::Publisher<Twist>::SharedPtr pub_twist__;

  double frequency__;
  double velocity__;
  double radius__;

  bool is_stopped__;

public:
  Circle()
  : Node("circle"), is_stopped__(false)
  {
    ParameterDescriptor frequency_des;
    ParameterDescriptor velocity_des;
    ParameterDescriptor radius_des;

    frequency_des.description = "Frequency of the circle node.";
    velocity_des.description =
      "The angular velocity, positive for counter-clockwise, negative is clockwise";
    radius_des.description = "The radius of the arc";

    this->declare_parameter("frequency", 100.0, frequency_des);
    this->declare_parameter("velocity", 0.0, velocity_des);
    this->declare_parameter("radius", 0.0, radius_des);

    frequency__ = this->get_parameter("frequency").as_double();
    velocity__ = this->get_parameter("velocity").as_double();
    radius__ = this->get_parameter("radius").as_double();

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

    pub_twist__ = this->create_publisher<Twist>("cmd_vel", 10);

    // this->create_service()
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_circle = std::make_shared<Circle>();
  rclcpp::spin(node_circle);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
