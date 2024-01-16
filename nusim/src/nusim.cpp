#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::UInt64;
using std_srvs::srv::Empty;

class NuSim : public rclcpp::Node
{
  public:
    NuSim() : Node("nusim"), __timestep(0)
    {
        this->declare_parameter("rate", "The timer frequency");
        __rate = this->get_parameter("rate").as_double();

        __timer = this->create_wall_timer(operator""s(1.0 / __rate),
                                          std::bind(&NuSim::__timer_callback, this));

        __pub_timestep = this->create_publisher<UInt64>("~/timestep", 10);
        __srv_reset = this->create_service<Empty>(
            "~/reset", std::bind(&NuSim::__srv_reset_callback, this));
    }

  private:
    /// @brief
    void __timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "Timer");
        auto msg_timestep = UInt64();
        msg_timestep.data = 1.0 / this->__rate;
        this->__pub_timestep->publish(msg_timestep);
    }

    /// @brief
    /// @param request
    /// @param response
    void __srv_reset_callback(std::shared_ptr<Empty::Request> request,
                              std::shared_ptr<Empty::Response> response)
    {
        __timestep = 0;
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