#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "nusim/srv/teleport.hpp"
#include "std_srvs/srv/empty.hpp"

using std::chrono::duration;
using std::vector;

using rclcpp::QoS;
using rclcpp::Node;
using tf2_ros::TransformBroadcaster;

/// messages
using rcl_interfaces::msg::ParameterDescriptor;
using std_msgs::msg::UInt64;
using geometry_msgs::msg::TransformStamped;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

/// services
using std_srvs::srv::Empty;
using nusim::srv::Teleport;

class NuSim : public Node
{
private:
  /// @brief
  void __timer_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "Timer");
    auto msg_timestep = UInt64();
    msg_timestep.data = __timestep++;
    this->__pub_timestep->publish(msg_timestep);

    TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "nusim/world";
    tf.child_frame_id = "red/base_footprint";

    tf.transform.translation.x = __turtle_x;
    tf.transform.translation.y = __turtle_y;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 1.0;
    tf.transform.rotation.w = __turtle_theta;

    __tf_broadcaster->sendTransform(tf);
  }

  /// @brief
  void __publish_wall_markers()
  {
    Marker m1;
    Marker m2;
    Marker m3;
    Marker m4;
    MarkerArray m_array;

    /// first wall
    m1.header.stamp = this->get_clock()->now();
    m1.header.frame_id = "nusim/world";
    m1.id = 1;
    m1.type = Marker::CUBE;
    m1.action = Marker::ADD;
    m1.pose.position.x = __arena_x_length / 2.0 + __wall_thickness / 2.0;
    m1.pose.position.y = __wall_thickness / 2.0;
    m1.pose.position.z = __wall_height / 2.0;
    m1.scale.x = __wall_thickness;
    m1.scale.y = __arena_y_length + __wall_thickness;
    m1.scale.z = __wall_height;
    m1.color.r = __nu_r;
    m1.color.g = __nu_g;
    m1.color.b = __nu_b;
    m1.color.a = 1.0;

    /// second wall
    m2.header.stamp = this->get_clock()->now();
    m2.header.frame_id = "nusim/world";
    m2.id = 2;
    m2.type = Marker::CUBE;
    m2.action = Marker::ADD;
    m2.pose.position.x = -__wall_thickness / 2.0;
    m2.pose.position.y = __arena_y_length / 2.0 + __wall_thickness / 2.0;
    m2.pose.position.z = __wall_height / 2.0;
    m2.scale.x = __arena_x_length + __wall_thickness;
    m2.scale.y = __wall_thickness;
    m2.scale.z = __wall_height;
    m2.color.r = __nu_r;
    m2.color.g = __nu_g;
    m2.color.b = __nu_b;
    m2.color.a = 1.0;

    /// third wall
    m3.header.stamp = this->get_clock()->now();
    m3.header.frame_id = "nusim/world";
    m3.id = 3;
    m3.type = Marker::CUBE;
    m3.action = Marker::ADD;
    m3.pose.position.x = -__arena_x_length / 2.0 - __wall_thickness / 2.0;
    m3.pose.position.y = -__wall_thickness / 2.0;
    m3.pose.position.z = __wall_height / 2.0;
    m3.scale.x = __wall_thickness;
    m3.scale.y = __arena_y_length + __wall_thickness;
    m3.scale.z = __wall_height;
    m3.color.r = __nu_r;
    m3.color.g = __nu_g;
    m3.color.b = __nu_b;
    m3.color.a = 1.0;

    /// fourth wall
    m4.header.stamp = this->get_clock()->now();
    m4.header.frame_id = "nusim/world";
    m4.id = 4;
    m4.type = Marker::CUBE;
    m4.action = Marker::ADD;
    m4.pose.position.x = __wall_thickness / 2.0;
    m4.pose.position.y = -__arena_y_length / 2.0 - __wall_thickness / 2.0;
    m4.pose.position.z = __wall_height / 2.0;
    m4.scale.x = __arena_x_length + __wall_thickness;
    m4.scale.y = __wall_thickness;
    m4.scale.z = __wall_height;
    m4.color.r = __nu_r;
    m4.color.g = __nu_g;
    m4.color.b = __nu_b;
    m4.color.a = 1.0;

    m_array.markers.push_back(m1);
    m_array.markers.push_back(m2);
    m_array.markers.push_back(m3);
    m_array.markers.push_back(m4);

    __pub_wall_markers->publish(m_array);
  }

  /// @brief
  void __publish_obstacle_markers()
  {
    MarkerArray m_array_obs;

    for (std::size_t i = 0; i < __obstacles_x.size(); i++) {
      Marker m_obs;

      m_obs.header.stamp = this->get_clock()->now();
      m_obs.header.frame_id = "nusim/world";
      m_obs.id = i + 10;
      m_obs.type = Marker::CYLINDER;
      m_obs.action = Marker::ADD;
      m_obs.pose.position.x = __obstacles_x.at(i);
      m_obs.pose.position.y = __obstacles_y.at(i);
      m_obs.pose.position.z = __obstacle_height / 2.0;
      m_obs.scale.x = 2.0 * __obstacle_radius;
      m_obs.scale.y = 2.0 * __obstacle_radius;
      m_obs.scale.z = __obstacle_height;
      m_obs.color.r = 1.0;
      m_obs.color.g = 0.0;
      m_obs.color.b = 0.0;
      m_obs.color.a = 1.0;

      m_array_obs.markers.push_back(m_obs);
    }

    __pub_obstacle_markers->publish(m_array_obs);
  }

  /// @brief
  /// @param request
  /// @param response
  void __srv_reset_callback(
    std::shared_ptr<Empty::Request> request,
    std::shared_ptr<Empty::Response> response)
  {
    (void) request;
    (void) response;

    __reset_turtle_pose();
    __timestep = 0;
  }

  /// @brief
  /// @param request
  /// @param response
  void __srv_teleport_callback(
    std::shared_ptr<Teleport::Request> request,
    std::shared_ptr<Teleport::Response> response)
  {
    __turtle_x = request->x;
    __turtle_y = request->y;
    __turtle_theta = request->theta;

    response->result = true;
  }

  /// @brief
  void __reset_turtle_pose()
  {
    __turtle_x = __x_0;
    __turtle_y = __y_0;
    __turtle_theta = __theta_0;
  }

  /// timer
  rclcpp::TimerBase::SharedPtr __timer;

  /// publishers
  rclcpp::Publisher<UInt64>::SharedPtr __pub_timestep;
  rclcpp::Publisher<MarkerArray>::SharedPtr __pub_wall_markers;
  rclcpp::Publisher<MarkerArray>::SharedPtr __pub_obstacle_markers;

  /// services
  rclcpp::Service<Empty>::SharedPtr __srv_reset;
  rclcpp::Service<Teleport>::SharedPtr __srv_teleport;

  /// transform broadcasters
  std::unique_ptr<TransformBroadcaster> __tf_broadcaster;

  /// qos profile
  rclcpp::QoS __marker_qos;

  /// other attributes
  double __rate;
  uint64_t __timestep;
  double __turtle_x;
  double __turtle_y;
  double __turtle_theta;
  double __x_0;
  double __y_0;
  double __theta_0;
  double __arena_x_length;
  double __arena_y_length;
  double __nu_r;
  double __nu_g;
  double __nu_b;
  double __wall_height;
  double __wall_thickness;
  vector<double> __obstacles_x;
  vector<double> __obstacles_y;
  double __obstacle_radius;
  double __obstacle_height;

public:
  NuSim()
  : Node("nusim"), __marker_qos(10), __timestep(0), __nu_r(78.0 / 255.0), __nu_g(42.0 / 255.0),
    __nu_b(132.0 / 255.0), __wall_height(0.25), __wall_thickness(0.1), __obstacle_height(0.25)
  {
    /// parameter descriptions
    ParameterDescriptor rate_des;
    ParameterDescriptor x0_des;
    ParameterDescriptor y0_des;
    ParameterDescriptor theta0_des;
    ParameterDescriptor arena_x_des;
    ParameterDescriptor arena_y_des;
    ParameterDescriptor obs_x_des;
    ParameterDescriptor obs_y_des;
    ParameterDescriptor obs_r_des;
    rate_des.description = "The rate of the simulator";
    x0_des.description = "The initial x location";
    y0_des.description = "The initial y location";
    theta0_des.description = "The initial theta pose";
    arena_x_des.description = "The wall length in x direction";
    arena_y_des.description = "The wall length in y direction";
    obs_x_des.description = "The list of x coordinates of the obstacles";
    obs_y_des.description = "The list of y coordinates of the obstacles";
    obs_r_des.description = "The radius of the obstacles";

    /// declare parameters
    this->declare_parameter("rate", 100.0, rate_des);
    this->declare_parameter("x0", 0.0, x0_des);
    this->declare_parameter("y0", 0.0, y0_des);
    this->declare_parameter("theta0", 0.0, theta0_des);
    this->declare_parameter("arena_x_length", 10.0, arena_x_des);
    this->declare_parameter("arena_y_length", 10.0, arena_y_des);
    this->declare_parameter("obstacles/x", vector{1.2, 2.3}, obs_x_des);
    this->declare_parameter("obstacles/y", vector{2.3, 4.5}, obs_y_des);
    this->declare_parameter("obstacles/r", 0.05, obs_r_des);

    /// get parameter values
    __rate = this->get_parameter("rate").as_double();
    __x_0 = this->get_parameter("x0").as_double();
    __y_0 = this->get_parameter("y0").as_double();
    __theta_0 = this->get_parameter("theta0").as_double();
    __arena_x_length = this->get_parameter("arena_x_length").as_double();
    __arena_y_length = this->get_parameter("arena_y_length").as_double();
    __obstacles_x = this->get_parameter("obstacles/x").as_double_array();
    __obstacles_y = this->get_parameter("obstacles/y").as_double_array();
    __obstacle_radius = this->get_parameter("obstacles/r").as_double();

    /// check for x y length
    if (__obstacles_x.size() != __obstacles_y.size()) {
      throw std::invalid_argument("The x and y coordinate should have same length");
    }

    /// initialize attributes
    __reset_turtle_pose();

    // set marker qos policy
    __marker_qos.transient_local();

    /// timer
    __timer = this->create_wall_timer(
      duration<long double>{1.0 / __rate},
      std::bind(&NuSim::__timer_callback, this));

    // publishers
    __pub_timestep = this->create_publisher<UInt64>("~/timestep", 10);
    __pub_wall_markers = this->create_publisher<MarkerArray>("~/walls", __marker_qos);
    __pub_obstacle_markers = this->create_publisher<MarkerArray>("~/obstacles", __marker_qos);

    /// services
    __srv_reset = this->create_service<Empty>(
      "~/reset",
      std::bind(
        &NuSim::__srv_reset_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    __srv_teleport =
      this->create_service<Teleport>(
      "~/teleport",
      std::bind(
        &NuSim::__srv_teleport_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    /// transform broadcasters
    __tf_broadcaster = std::make_unique<TransformBroadcaster>(*this);

    __publish_wall_markers();
    __publish_obstacle_markers();
  }
};

/// @brief
/// @param argc
/// @param argv
/// @return
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_nusim = std::make_shared<NuSim>();
  rclcpp::spin(node_nusim);

  rclcpp::shutdown();
  return 0;
}
