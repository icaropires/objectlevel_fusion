#pragma once

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "object_model_msgs/msg/object_model.hpp"

using std::placeholders::_1;

class Fusion : public rclcpp::Node
{
public:
  Fusion();

private:
  void topic_callback(const object_model_msgs::msg::ObjectModel::SharedPtr msg) const;
  rclcpp::Subscription<object_model_msgs::msg::ObjectModel>::SharedPtr subscription_;
};
