#pragma once

#include <array>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>

#include "fusion_layer/temporal_aligner.hpp"
#include "object_model_msgs/msg/object_model.hpp"
#include "types.hpp"

class TemporalAlignerEKF : public TemporalAligner {
    bool is_initialized;

    ctra_array_t state_array;

    ctra_matrix_t P;
    ctra_matrix_t Q;

 public:
    TemporalAlignerEKF();
    TemporalAlignerEKF(const state_t& initial_state);

    enum CTRAIndexes {X_IDX, Y_IDX, YAW_IDX, VELOCITY_IDX, YAW_RATE_IDX, ACCELERATION_IDX};
    
    state_t align(float delta_t);

    state_t get_state() const;
    void update(const state_t& measurement, const ctra_squared_t& measurement_noise_matrix, const capable_vector_t& capable);

    static state_t format_to_object_model(const ctra_array_t& state);

    static ctra_array_t format_from_object_model(const state_t& state);

 private:
    void predict(float delta_t);

    void disable_not_capable_attributes(ctra_matrix_t& JH, const capable_vector_t& capable);
};
