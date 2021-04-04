#pragma once

#include <array>
#include <string>
#include <Eigen/Dense>

#include "object_model_msgs/msg/object_model.hpp"
#include "types.hpp"

// TODO: create Alignment interface with virtual method align
class TemporalAlignmentEKF {
    state_t state;
    std::array<float, 6> state_local_format;

    Eigen::Matrix<float, 6, 1> x_vector;
    Eigen::Matrix<float, 6, 6> P;
    Eigen::Matrix<float, 6, 6> Q;

 public:
    TemporalAlignmentEKF(state_t initial_state);

    enum StateIndexes {X_IDX, Y_IDX, YAW_IDX, VELOCITY_IDX, YAW_RATE_IDX, ACCELERATION_IDX};
    
    state_t align(float delta_t);
    void update(object_model_msgs::msg::Track measurement, state_squared_t measurement_noise_matrix);

    static state_t format_to_object_model(std::array<float, 6> state);

    static std::array<float, 6> format_from_object_model(state_t state);

    state_t get_state() const;

 private:
    void predict(float delta_t);
};
