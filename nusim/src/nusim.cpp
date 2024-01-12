#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using namespace std_srvs::srv;
using namespace std_msgs::msg;

class NuSim : public rclcpp::Node
{
public:
    NuSim() : Node("nusim"), __timestep(0)
    {
        this->declare_parameter("rate", "The timer frequency");
        __rate = this->get_parameter("rate").as_double();

        __timer = this->create_wall_timer(operator""s(1.0 / __rate), std::bind(&NuSim::__timer_callback, this));

        __pub_timestep = this->create_publisher<UInt64>("~/timestep", 10);
        // this->__srv_reset = this->create_service<Empty>("~/reset", &NuSim::srv_reset_callback);
    }

    /// @brief
    /// @param request
    /// @param response
    void srv_reset_callback(const std::shared_ptr<Empty::Request> request, std::shared_ptr<Empty::Response> response)
    {
        __timestep = 0;
    }

private:
    /// @brief
    void __timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "Timer");
        auto msg_timestep = std_msgs::msg::UInt64();
        msg_timestep.data = 1.0 / this->__rate;
        this->__pub_timestep->publish(msg_timestep);
    }

    rclcpp::TimerBase::SharedPtr __timer;
    rclcpp::Publisher<UInt64>::SharedPtr __pub_timestep;
    rclcpp::Service<Empty>::SharedPtr __srv_reset;
    double __rate;
    int64_t __timestep;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_nusim = std::make_shared<NuSim>();
    rclcpp::spin(node_nusim);

    rclcpp::shutdown();
    return 0;
}