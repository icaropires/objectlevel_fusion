#include <vector>
#include <cstring>
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <Eigen/Dense>

#include "fusion_layer/spatial_alignment.hpp"
#include "fusion_layer/temporal_alignment_ekf.hpp"
#include "object_model_msgs/msg/object_model.hpp"


void spatial_alignment_check(float delta_x, float delta_y, float theta, const state_t& object_state, const state_t& expected) {
    state_t result;
    spatially_align(delta_x, delta_y, theta, object_state, result);

    const float precision = 1e-4;
    ASSERT_THAT(
        result,
        testing::Pointwise(testing::FloatNear(precision), expected)
    );
}

TEST(TestSpatialAlignment, spatiallyAlign) {
    // delta x, delta y, theta, object_state, expected
    spatial_alignment_check(1.0, -0.5, 0, {1, 1, 2, 2, 3, 3, 4, 4}, {2, 0.5, 2, 2, 3, 3, 4, 4});
    spatial_alignment_check(-10, 2, 0, {1, 1, 2, 2, 3, 3, 4, 4}, {-9, 3, 2, 2, 3, 3, 4, 4});
    spatial_alignment_check(-10, 2, 90, {1, 1, 2, 2, 3, 3, 4, 4}, {-11.3421, 2.4459, -2.6841, 0.8918, -4.0262, 1.3377, 94, 4});
}

TEST(TestTemporalAlignment, convertToObjectModel) {
    std::array<float, 6> state = {4.0, -2.0, M_PI/2, 5.0, M_PI/7, 4.0};
    state_t result = TemporalAlignmentEKF::format_to_object_model(state);

    const float precision = 1e-4;
    state_t expected = {4, -2, 0, 5, 0, 4, M_PI/2, M_PI/7};
    ASSERT_THAT(
        result,
        testing::Pointwise(testing::FloatNear(precision), expected)
    );
}

TEST(TestTemporalAlignment, convertFromObjectModel) {
    state_t state = {4, -2, 0, 5, 0, 4, M_PI/2, M_PI/7};
    std::array<float, 6> result = TemporalAlignmentEKF::format_from_object_model(state);

    const float precision = 1e-4;
    std::array<float, 6> expected = {4.0, -2.0, M_PI/2, 5.0, M_PI/7, 4.0};
    ASSERT_THAT(
        result,
        testing::Pointwise(testing::FloatNear(precision), expected)
    );
}

/*
 * Static values used here were generated at utils/draft_EKF_temporal_alignment.ipynb
 * */
TEST(TestTemporalAlignment, temporalAlignEKF) {
    constexpr int state_size = object_model_msgs::msg::Track::STATE_SIZE;
    object_model_msgs::msg::Track initial_track;

    // Fill track state
    state_t inital_state = TemporalAlignmentEKF::format_to_object_model({4, -2, M_PI/2, 5, M_PI/7, 4});

    state_t noise = TemporalAlignmentEKF::format_to_object_model({-1.044, 1.0289, -0.0145, 0.9013, -0.0183, 0.0579});

    // Add noise to track state
    for (int i = 0; i < state_size; ++i) {
        initial_track.state[i] += noise[i];
    }

    TemporalAlignmentEKF temporal_alignment(inital_state);

    const float delta_t = 0.4571;
    state_t aligned_state = temporal_alignment.align(delta_t);

    const float checking_precision = 1e-4;
    state_t expected = TemporalAlignmentEKF::format_to_object_model({3.70947952675031, 0.682997338756643, 1.77594232707431, 6.8284, 0.448798950512828, 4.0});
    ASSERT_THAT(
        aligned_state,
        testing::Pointwise(testing::FloatNear(checking_precision), expected)
    );

    object_model_msgs::msg::Track measured_track;

    // Fill measured track
    measured_track.state = TemporalAlignmentEKF::format_to_object_model({3.7538, 0.4616, 1.761, 6.6948, 0.4488, 4.0});
    noise = TemporalAlignmentEKF::format_to_object_model({0.7317, 0.8656, 0.0095, -0.5005, 0.0835, -0.4629});

    // Add noise to measurement
    for (int i = 0; i < state_size; ++i) {
        measured_track.state[i] += noise[i];
    }

    // |v| = 1, |a| = 0.5
    constexpr float x_variance = 1.5*1.5, y_variance = 1.5*1.5, vx_variance = 0*0, vy_variance = 1*1;
    constexpr float ax_variance = 0*0, ay_variance = 0.5*0.5, yaw_variance = 0.02*0.02, yaw_rate_variance = 0.1*0.1;

    state_squared_t measurement_noise_array;

    // Fill measurement noises
    measurement_noise_matrix_t measurement_noise_matrix = measurement_noise_matrix_t::Zero();
    measurement_noise_matrix.diagonal() << x_variance, y_variance, vx_variance, vy_variance, ax_variance, ay_variance, yaw_variance, yaw_rate_variance;
    float *measurement_carray_ptr = measurement_noise_matrix.data();
    std::copy(measurement_carray_ptr, measurement_carray_ptr+(state_size*state_size), std::begin(measurement_noise_array));

    temporal_alignment.update(measured_track, measurement_noise_array);

    state_t updated_state = temporal_alignment.get_state();

    expected = {3.2195997816687, -0.691060294937198, 1.55651945481671, 5.92223886096595, 0.42931542132464, 4.05230530018931};
    ASSERT_THAT(
        updated_state,
        testing::Pointwise(testing::FloatNear(checking_precision), expected)
    );
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
