#pragma once

#include <cmath>

#include "object_model_msgs/msg/object_model.hpp"
#include "types.hpp"

#define PI 3.1415926

void spatially_align(float delta_x, float delta_y, float theta, const state_t& object_state, state_t& result);
