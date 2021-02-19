#pragma once

#include <cmath>

#define PI 3.1415926
#define TRANSFORM_SIZE 9
#define STATE_SIZE 8

// TODO: Unify with ROS message enums
enum {
    STATE_X_IDX,
    STATE_Y_IDX,
    STATE_VELOCITY_X_IDX,
    STATE_VELOCITY_Y_IDX,
    STATE_ACCELERATION_X_IDX,
    STATE_ACCELERATION_Y_IDX,
    STATE_YAW_IDX,
    STATE_YAW_RATE_IDX,
};

void spatially_align(double delta_x, double delta_y, double theta, double object_state[STATE_SIZE], double result[STATE_SIZE]);
