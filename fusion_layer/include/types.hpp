#pragma once

#include <array>

#include <object_model_msgs/msg/object_model.hpp>

typedef std::array<float, object_model_msgs::msg::Track::STATE_SIZE> state_t;
