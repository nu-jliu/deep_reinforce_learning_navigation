#include <rclcpp/rclcpp.hpp>

class NuTurtleControl : public rclcpp::Node
{
private:
  /* data */

public:
  NuTurtleControl(/* args */);
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
}
