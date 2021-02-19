#include "fusion_layer/spatial_alignment.hpp"

void spatially_align(double delta_x, double delta_y, double theta, double object_state[STATE_SIZE], double result[STATE_SIZE]) {
    double transformation[TRANSFORM_SIZE][TRANSFORM_SIZE] = {
        {cos(theta), -sin(theta), 0, 0, 0, 0, 0, 0, delta_x},
        {sin(theta),  cos(theta), 0, 0, 0, 0, 0, 0, delta_y},
        {0, 0, cos(theta), -sin(theta), 0, 0, 0, 0,       0},
        {0, 0, sin(theta),  cos(theta), 0, 0, 0, 0,       0},
        {0, 0, 0, 0, cos(theta), -sin(theta), 0, 0,       0},
        {0, 0, 0, 0, sin(theta),  cos(theta), 0, 0,       0},
        {0, 0, 0, 0,          0,           0, 1, 0,   theta},
        {0, 0, 0, 0,          0,           0, 0, 1,       0},
        {0, 0, 0, 0,          0,           0, 0, 0,       1},
    };

    // Multiplying matrix
    for(int k = 0; k < TRANSFORM_SIZE; ++k) {
        double sum = 0;

        for(int i = 0; i < TRANSFORM_SIZE; ++i){
            double sensor_attr = (i < STATE_SIZE)? object_state[i] : 1;

            sum += transformation[k][i] * sensor_attr;
        }

        if (k < STATE_SIZE) {
            result[k] = sum;
        }
    }
}
