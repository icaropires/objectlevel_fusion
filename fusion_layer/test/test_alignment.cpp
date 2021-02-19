#include <vector>
#include <cstring>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "fusion_layer/spatial_alignment.hpp"


void spatial_alignment_check(double delta_x, double delta_y, double theta, const std::vector<double>& object_state, const std::vector<double>& expected) {
    double result[STATE_SIZE];
    spatially_align(delta_x, delta_y, theta, (double *) object_state.data(), result);

    std::vector<double> result_vector(std::begin(result), std::end(result));

    float precision = 1e-3;
    ASSERT_THAT(
        result_vector,
        testing::Pointwise(testing::FloatNear(precision), expected)
    );
}

TEST(TestSpatialAlignment, spatiallyAlign) {
    // delta x, delta y, theta, object_state, expected
    spatial_alignment_check(1.0, -0.5, 0, std::vector<double>{1, 1, 2, 2, 3, 3, 4, 4}, std::vector<double>{2, 0.5, 2, 2, 3, 3, 4, 4});
    spatial_alignment_check(-10, 2, 0, std::vector<double>{1, 1, 2, 2, 3, 3, 4, 4}, std::vector<double>{-9, 3, 2, 2, 3, 3, 4, 4});
    spatial_alignment_check(-10, 2, 90, std::vector<double>{1, 1, 2, 2, 3, 3, 4, 4}, std::vector<double>{-11.3421, 2.4459, -2.6841, 0.8918, -4.0262, 1.3377, 94, 4});
}


int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
