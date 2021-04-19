#include <vector>
#include <cstring>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "fusion_layer/spatial_alignment.hpp"
#include "fusion_layer/temporal_aligner_ekf.hpp"
#include "object_model_msgs/msg/object_model.hpp"


void spatial_alignment_check(float delta_x, float delta_y, float theta, const state_t& object_state, const state_t& expected) {
    state_t result = spatially_align(delta_x, delta_y, theta, object_state);

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
    spatial_alignment_check(-10, 2, M_PI/2, {1, 1, 2, 2, 3, 3, 4, 4}, {-11, 3, -2, 2, -3, 3, 5.5708, 4});
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
 * It's not testing directly the temporal alignment but internally, it uses the same functions that
 *   are being tested here
 * */
TEST(TestTemporalAlignment, ekfCompleteImplementation) {
    constexpr int state_size = object_model_msgs::msg::Track::STATE_SIZE;

    // ================================ Check first predict =============================

    // Fill track state
    // CTRA = Constant Turn Rate and Acceleration model currently used by the temporal alignment EKF
    ctra_array_t ctra_state = {4, -2, M_PI/2, 5, M_PI/7, 4};
    ctra_array_t ctra_noise = {-1.044, 1.0289, -0.0145, 0.9013, -0.0183, 0.0579};

    // Add noise to initial state
    for (int i = 0; i < state_size; ++i) {
        ctra_state[i] += ctra_noise[i];
    }

    state_t initial_state = TemporalAlignerEKF::format_to_object_model(ctra_state);

    TemporalAlignerEKF temporal_aligner(initial_state);

    float delta_t = 0.4571;
    state_t aligned_state = temporal_aligner.align(delta_t);

    const float checking_precision = 1e-3;
    state_t expected = TemporalAlignerEKF::format_to_object_model({2.68103, 2.13316, 1.75308, 7.75617, 0.4305, 4.0579});
    ASSERT_THAT(
        aligned_state,
        testing::Pointwise(testing::FloatNear(checking_precision), expected)
    );

    // ===============================================================================

    // ============================== Check first update =============================

    object_model_msgs::msg::Track measured_track;

    // Fill measured track
    ctra_array_t ctra_measured_state = {3.7538, 0.4616, 1.761, 6.6948, 0.4488, 4.0};
    ctra_noise = {0.7317, 0.8656, 0.0095, -0.5005, 0.0835, -0.4629};

    // Add noise to measurement
    for (int i = 0; i < ctra_size_t; ++i) {
        ctra_measured_state[i] += ctra_noise[i];
    }

    state_t measured_state = TemporalAlignerEKF::format_to_object_model(ctra_measured_state);

    /*
     * Measurement noise matrix should be 8x8 (object model format), but as, for now, I don't how
     * to transform it from 8x8 to 6x6, I'll be using 6x6 until I figure it out or remodel CTRA.
     */

    constexpr float x_variance = 1.5*1.5, y_variance = 1.5*1.5, v_variance = 1*1;
    constexpr float a_variance = 0.5*0.5, yaw_variance = 0.02*0.02, yaw_rate_variance = 0.1*0.1;

    ctra_squared_t measurement_noise_array;

    // Fill measurement noises
    ctra_matrix_t measurement_noise_matrix;
    measurement_noise_matrix.setZero();
    measurement_noise_matrix.diagonal() << x_variance, y_variance, yaw_variance, v_variance, yaw_rate_variance, a_variance;
    float *measurement_carray_ptr = measurement_noise_matrix.data();
    std::copy(measurement_carray_ptr, measurement_carray_ptr+(ctra_size_t*ctra_size_t), std::begin(measurement_noise_array));

    capable_vector_t sensor_capable;
    for(auto& capable : sensor_capable) {
        capable = true;
    }

    temporal_aligner.update(measured_state, measurement_noise_array, sensor_capable);

    state_t updated_state = temporal_aligner.get_state();

    expected = TemporalAlignerEKF::format_to_object_model({4.20009, 1.40653, 1.77032, 6.23164, 0.53316, 3.54363});
    ASSERT_THAT(
        updated_state,
        testing::Pointwise(testing::FloatNear(checking_precision), expected)
    );

    // ============================================================================

    // ================================ Check second predict ======================
    delta_t = 0.5364;
    aligned_state = temporal_aligner.align(delta_t);
    
    expected = TemporalAlignerEKF::format_to_object_model({2.88776, 5.01469, 2.05631, 8.13244, 0.53316, 3.54363});
    ASSERT_THAT(
        aligned_state,
        testing::Pointwise(testing::FloatNear(checking_precision), expected)
    );
    // ============================================================================

    // ================================ Check second update =======================

    ctra_measured_state = {2.8449, 3.5203, 1.9512, 8.3896, 0.4488, 4.0};
    ctra_noise = {-0.0571, -1.0837, -0.0015, 0.4288, -0.0634, -0.0808};

    for (int i = 0; i < ctra_size_t; ++i) {
        ctra_measured_state[i] += ctra_noise[i];
    }

    measured_state = TemporalAlignerEKF::format_to_object_model(ctra_measured_state);

    temporal_aligner.update(measured_state, measurement_noise_array, sensor_capable);

    updated_state = temporal_aligner.get_state();

    expected = TemporalAlignerEKF::format_to_object_model({2.8853, 3.45836, 1.95632, 8.78508, 0.38746, 3.79022});
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
