#include "create2_hardware/create2_hardware_interface.hpp"

namespace create2_hardware
{
Create2HardwareInterface::Create2HardwareInterface(const rclcpp::NodeOptions & options)
: Node("create2_hardware_interface", options)
, joint1_publisher_(this->create_publisher<std_msgs::msg::Float32>("joint1", 10))
, joint2_publisher_(this->create_publisher<std_msgs::msg::Float32>("joint2", 10))
, joint3_publisher_(this->create_publisher<std_msgs::msg::Float32>("joint3", 10))
{
}

void Create2HardwareInterface::control_joints()
{
  // Implement the control logic for the Create2 robot here.
}
} // namespace create2_hardware
