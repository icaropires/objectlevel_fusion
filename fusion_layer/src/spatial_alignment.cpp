#include "fusion_layer/spatial_alignment.hpp"

void spatially_align(float delta_x, float delta_y, float theta, const state_t& object_state, state_t& result) {
    theta = fmod(theta, 2.0 * M_PI);

    auto sin_theta = static_cast<float>(sin(theta));
    auto cos_theta = static_cast<float>(cos(theta));

    static constexpr int state_size = object_model_msgs::msg::Track::STATE_SIZE;
    static constexpr int transform_size = state_size + 1;

    float transformation[transform_size][transform_size] = {
        {cos_theta, -sin_theta, 0, 0, 0, 0, 0, 0, delta_x},
        {sin_theta,  cos_theta, 0, 0, 0, 0, 0, 0, delta_y},
        {0, 0, cos_theta, -sin_theta, 0, 0, 0, 0,       0},
        {0, 0, sin_theta,  cos_theta, 0, 0, 0, 0,       0},
        {0, 0, 0, 0, cos_theta, -sin_theta, 0, 0,       0},
        {0, 0, 0, 0, sin_theta,  cos_theta, 0, 0,       0},
        {0, 0, 0, 0,          0,           0, 1, 0, theta},
        {0, 0, 0, 0,          0,           0, 0, 1,     0},
        {0, 0, 0, 0,          0,           0, 0, 0,     1},
    };

    // Multiplying matrix
    for(int k = 0; k < transform_size; ++k) {
        float sum = 0;

        for(int i = 0; i < transform_size; ++i){
            float sensor_attr = (i < state_size)? object_state[i] : 1;

            sum += transformation[k][i] * sensor_attr;
        }

        if (k < state_size) {
            result[k] = sum;
        }
    }
}
