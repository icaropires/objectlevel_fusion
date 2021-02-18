#include "fusion_layer/spatial_alignment.hpp"
#include <vector>
#include <gtest/gtest.h>

// TEST(TestSpatialAlignment, spatiallyAlign) {
//     double delta_x = -1.0, delta_y = -1.0, theta = -1.0;
//     double object_state[STATE_SIZE], result[STATE_SIZE], expected[STATE_SIZE];
// 
//     delta_x = 1.0, delta_y = -0.5, theta = 90;
//     object_state[] = {1, 1, 2, 2, 3, 3, 4, 4};
//     expected[] = {1, 1, 2, 2, 3, 3, 4, 4};
// 
//     spatially_align(1.0, 0.5, 0, object_state, result);
// 	EXPECT_EQ(std::vector<double>(result), std::vector<double>(expected));
// }

TEST(hey, hou) {
  EXPECT_EQ(1, 1);
}


int main(int argc, char ** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
