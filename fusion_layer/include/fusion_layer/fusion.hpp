#pragma once

#include <functional>
#include <memory>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "object_model_msgs/msg/object_model.hpp"

#include "fusion_layer/spatial_alignment.hpp"
#include "fusion_layer/temporal_aligner_ekf.hpp"
#include "types.hpp"

using std::placeholders::_1;

class Fusion : public rclcpp::Node
{
private:
  TemporalAlignerEKF temporal_aligner;

  const std::string input_topic;

  bool is_first_message;

  uint64_t time_last_msg;

public:
  Fusion();

private:
  void topic_callback(const object_model_msgs::msg::ObjectModel::SharedPtr msg);

  void log_state(char *label, const state_t& state);

  static uint64_t get_timestamp(const object_model_msgs::msg::ObjectModel::SharedPtr msg);

  static void state_to_str(const state_t& state, char *c_str);

  rclcpp::Subscription<object_model_msgs::msg::ObjectModel>::SharedPtr subscription_;
};
