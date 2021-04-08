#include <vector>
#include <cstring>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

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
    ctra_array_t state = {4.0, -2.0, M_PI/2, 5.0, M_PI/7, 4.0};
    state_t result = TemporalAlignerEKF::format_to_object_model(state);

    const float precision = 1e-4;
    state_t expected = {4, -2, 0, 5, 0, 4, M_PI/2, M_PI/7};
    ASSERT_THAT(
        result,
        testing::Pointwise(testing::FloatNear(precision), expected)
    );
}

TEST(TestTemporalAlignment, convertFromObjectModel) {
    state_t state = {4, -2, 0, 5, 0, 4, M_PI/2, M_PI/7};
    ctra_array_t result = TemporalAlignerEKF::format_from_object_model(state);

    const float precision = 1e-4;
    ctra_array_t expected = {4.0, -2.0, M_PI/2, 5.0, M_PI/7, 4.0};
    ASSERT_THAT(
        result,
        testing::Pointwise(testing::FloatNear(precision), expected)
    );
}

/*
 * Static values used here were generated at utils/draft_EKF_temporal_alignment.ipynb
 * Big test but too much work to split it, not worth it
 * */
TEST(TestTemporalAlignment, temporalAlignEKF) {
    constexpr int state_size = object_model_msgs::msg::Track::STATE_SIZE;

    // ================================ Check first predict =============================

    // Fill track state
    // CTRA = Constant Turn Rate and Acceleration model currently used by the temporal alignment EKF
    ctra_array_t state_ctra_format = {4, -2, M_PI/2, 5, M_PI/7, 4};
    ctra_array_t noise_ctra_format = {-1.044, 1.0289, -0.0145, 0.9013, -0.0183, 0.0579};

    // Add noise to initial state
    for (int i = 0; i < state_size; ++i) {
        state_ctra_format[i] += noise_ctra_format[i];
    }

    state_t initial_state = TemporalAlignerEKF::format_to_object_model(state_ctra_format);

    TemporalAlignerEKF temporal_alignment(initial_state);

    const float delta_t = 0.4571;
    state_t aligned_state = temporal_alignment.align(delta_t);

    const float checking_precision = 1e-4;
    state_t expected = TemporalAlignerEKF::format_to_object_model({2.6810305, 2.13315666, 1.7530774, 7.75616609, 0.43049895, 4.0579});
    ASSERT_THAT(
        aligned_state,
        testing::Pointwise(testing::FloatNear(checking_precision), expected)
    );

    // ===============================================================================

    // ============================== Check first update =============================

    object_model_msgs::msg::Track measured_track;

    // Fill measured track
    ctra_array_t measured_state_ctra_format = {3.7538, 0.4616, 1.761, 6.6948, 0.4488, 4.0};
    noise_ctra_format = {0.7317, 0.8656, 0.0095, -0.5005, 0.0835, -0.4629};

    // Add noise to measurement
    for (int i = 0; i < ctra_size_t; ++i) {
        measured_state_ctra_format[i] += noise_ctra_format[i];
    }

    state_t measured_state = TemporalAlignerEKF::format_to_object_model(measured_state_ctra_format);

    // |v| = 1, |a| = 0.5
    constexpr float x_variance = 1.5*1.5, y_variance = 1.5*1.5, vx_variance = 0*0, vy_variance = 1*1;
    constexpr float ax_variance = 0*0, ay_variance = 0.5*0.5, yaw_variance = 0.02*0.02, yaw_rate_variance = 0.1*0.1;

    state_squared_t measurement_noise_array;

    // Fill measurement noises
    measurement_noise_matrix_t measurement_noise_matrix;
    measurement_noise_matrix.setZero();
    measurement_noise_matrix.diagonal() << x_variance, y_variance, vx_variance, vy_variance, ax_variance, ay_variance, yaw_variance, yaw_rate_variance;
    float *measurement_carray_ptr = measurement_noise_matrix.data();
    std::copy(measurement_carray_ptr, measurement_carray_ptr+(state_size*state_size), std::begin(measurement_noise_array));

    temporal_alignment.update(measured_state, measurement_noise_array);

    state_t updated_state = temporal_alignment.get_state();

    expected = TemporalAlignerEKF::format_to_object_model({4.20009035106605, 1.4065263332970936, 1.7703242101697527, 6.231636758416404, 0.5331593429620153, 3.5436312687426756});
    ASSERT_THAT(
        updated_state,
        testing::Pointwise(testing::FloatNear(checking_precision), expected)
    );

    // ============================================================================
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
