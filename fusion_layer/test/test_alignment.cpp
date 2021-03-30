#include <vector>
#include <cstring>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "fusion_layer/spatial_alignment.hpp"
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


int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
