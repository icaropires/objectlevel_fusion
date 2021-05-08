#include "fusion_layer/spatial_alignment.hpp"

state_t spatially_align(float delta_x, float delta_y, float angle, const state_t& object_state) {
    constexpr float two_pi = M_PI*2;
    angle = fmod(fmod(angle + M_PI, two_pi) + two_pi, two_pi) - M_PI;

    float sin_angle = sinf(angle);
    float cos_angle = cosf(angle);

    static constexpr int state_size = object_model_msgs::msg::Track::STATE_SIZE;
    static constexpr int transform_size = state_size + 1;

    state_t result;

    float transformation[transform_size][transform_size] = {
        {cos_angle, -sin_angle, 0, 0, 0, 0, 0, 0, delta_x},
        {sin_angle,  cos_angle, 0, 0, 0, 0, 0, 0, delta_y},
        {0, 0, cos_angle, -sin_angle, 0, 0, 0, 0,       0},
        {0, 0, sin_angle,  cos_angle, 0, 0, 0, 0,       0},
        {0, 0, 0, 0, cos_angle, -sin_angle, 0, 0,       0},
        {0, 0, 0, 0, sin_angle,  cos_angle, 0, 0,       0},
        {0, 0, 0, 0,          0,           0, 1, 0, angle},
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

    return result;
}
