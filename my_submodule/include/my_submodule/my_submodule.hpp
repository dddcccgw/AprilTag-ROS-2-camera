#ifndef MY_SUBMODULE__MY_SUBMODULE_HPP_
#define MY_SUBMODULE__MY_SUBMODULE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

class MySubmodule : public rclcpp::Node
{
public:
  explicit MySubmodule(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Timer callback
  void timer_callback();

  // Subscription callback
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Members
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

#endif  // MY_SUBMODULE__MY_SUBMODULE_HPP_
