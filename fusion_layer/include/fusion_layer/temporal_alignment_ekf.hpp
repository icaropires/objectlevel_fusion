#pragma once

#include <array>
#include <string>
#include <Eigen/Dense>

#include "object_model_msgs/msg/object_model.hpp"
#include "types.hpp"

// TODO: create Alignment interface with virtual method align
// TODO: create constant to substitute all the 6s
class TemporalAlignerEKF {
    ctra_array_t state_local_format;
    ctra_vector_t state_vector;

    ctra_matrix_t P;
    ctra_matrix_t Q;

 public:
    TemporalAlignerEKF(const state_t& initial_state);

    enum CTRAIndexes {X_IDX, Y_IDX, YAW_IDX, VELOCITY_IDX, YAW_RATE_IDX, ACCELERATION_IDX};
    
    state_t align(float delta_t);

    state_t get_state() const;
    void update(const state_t& measurement, const state_squared_t& measurement_noise_matrix);

    static state_t format_to_object_model(const ctra_array_t& state);

    static ctra_array_t format_from_object_model(const state_t& state);

 private:
    void predict(float delta_t);
};
