#pragma once

#include <cmath>

#include "object_model_msgs/msg/object_model.hpp"
#include "types.hpp"

state_t spatially_align(float delta_x, float delta_y, float theta, const state_t& object_state);
