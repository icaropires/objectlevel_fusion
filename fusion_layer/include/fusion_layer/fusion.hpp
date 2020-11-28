#pragma once

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Fusion : public rclcpp::Node
{
public:
  Fusion();

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
