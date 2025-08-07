#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace create2_hardware
{
class Create2HardwareInterface : public rclcpp::Node
{
public:
  explicit Create2HardwareInterface(const rclcpp::NodeOptions & options);

  void control_joints();

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint1_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint2_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint3_publisher_;
};
} // namespace create2_hardware
