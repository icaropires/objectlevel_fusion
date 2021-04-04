#pragma once

#include <array>
#include <string>
#include <Eigen/Dense>

#include "object_model_msgs/msg/object_model.hpp"
#include "types.hpp"

// TODO: create Alignment interface with virtual method align
class TemporalAlignmentEKF {
    state_t state;

    state_vector_t x_vector;
    state_covariance_matrix_t P;
    process_noise_matrix_t Q;

 public:
    TemporalAlignmentEKF(object_model_msgs::msg::Track initial_track);
    
    state_t align(float delta_t);
    void update(object_model_msgs::msg::Track measurement, state_squared_t measurement_noise_matrix);

    static state_t format_to_object_model(std::array<float, 6> state);

    static std::array<float, 6> format_from_object_model(state_t state);

    state_t get_state() const;

 private:
    void predict();
};
