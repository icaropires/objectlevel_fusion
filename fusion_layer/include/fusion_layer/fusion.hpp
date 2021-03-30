#pragma once

#include <functional>
#include <memory>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "object_model_msgs/msg/object_model.hpp"

#include "fusion_layer/spatial_alignment.hpp"
#include "types.hpp"

using std::placeholders::_1;

class Fusion : public rclcpp::Node
{
public:
  Fusion();

private:
  void topic_callback(const object_model_msgs::msg::ObjectModel::SharedPtr msg) const;
  static std::string state_to_str(const state_t& state);

  rclcpp::Subscription<object_model_msgs::msg::ObjectModel>::SharedPtr subscription_;
};
