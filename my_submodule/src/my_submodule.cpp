#include "my_submodule/my_submodule.hpp"

using namespace std::chrono_literals;

MySubmodule::MySubmodule(const rclcpp::NodeOptions & options)
: rclcpp::Node("my_submodule_node", options)
{
  RCLCPP_INFO(this->get_logger(), "MySubmodule initializing...");

  // Create publisher
  publisher_ = this->create_publisher<std_msgs::msg::String>("output", 10);

  // Create subscription
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw",
    10,
    std::bind(&MySubmodule::image_callback, this, std::placeholders::_1)
  );

  // Create timer (100ms interval)
  timer_ = this->create_wall_timer(
    100ms,
    std::bind(&MySubmodule::timer_callback, this)
  );

  RCLCPP_INFO(this->get_logger(), "MySubmodule initialized successfully");
}

void MySubmodule::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello from MySubmodule";
  publisher_->publish(message);
}

void MySubmodule::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received image: %dx%d",
    msg->width,
    msg->height
  );
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with ROS
RCLCPP_COMPONENTS_REGISTER_NODE(MySubmodule)
