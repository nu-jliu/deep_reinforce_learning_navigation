#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "nusim/srv/teleport.hpp"
#include "std_srvs/srv/empty.hpp"

using std::chrono::duration;
using tf2_ros::TransformBroadcaster;

/// messages
using geometry_msgs::msg::TransformStamped;
using rcl_interfaces::msg::ParameterDescriptor;
using std_msgs::msg::UInt64;

/// services
using std_srvs::srv::Empty;
using nusim::srv::Teleport;

class NuSim : public rclcpp::Node
{
public:
    NuSim()
      : Node("nusim"), __timestep(0), __turtle_x(1.0), __turtle_y(2.0), __turtle_theta(0.0)
    {
        /// parameters
        ParameterDescriptor rate_des;
        rate_des.description = "The rate of the simulator";
        this->declare_parameter("rate", 100.0, rate_des);
        __rate = this->get_parameter("rate").as_double();

        /// timer
        __timer = this->create_wall_timer(
            duration<long double>{1.0 / __rate},
            std::bind(&NuSim::__timer_callback, this));

        // publishers
        __pub_timestep = this->create_publisher<UInt64>("~/timestep", 10);

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
    }

private:
    /// @brief
    void __timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "Timer");
        auto             msg_timestep = UInt64();
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
    /// @param request
    /// @param response
    void __srv_reset_callback(
        std::shared_ptr<Empty::Request> request,
        std::shared_ptr<Empty::Response> response)
    {
        (void) request;
        (void) response;
        __timestep = 0;
    }

    void __srv_teleport_callback(
        std::shared_ptr<Teleport::Request> request,
        std::shared_ptr<Teleport::Response> response)
    {
        __turtle_x = request->x;
        __turtle_y = request->y;
        __turtle_theta = request->theta;

        response->result = true;
    }

    /// timer
    rclcpp::TimerBase::SharedPtr __timer;
    /// publishers
    rclcpp::Publisher<UInt64>::SharedPtr __pub_timestep;
    /// services
    rclcpp::Service<Empty>::SharedPtr __srv_reset;
    rclcpp::Service<Teleport>::SharedPtr __srv_teleport;
    /// transform broadcasters
    std::unique_ptr<TransformBroadcaster> __tf_broadcaster;


    /// other attributes
    double __rate;
    uint64_t __timestep;
    double __turtle_x;
    double __turtle_y;
    double __turtle_theta;

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
