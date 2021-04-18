#pragma once

#include <string>

#include "types.hpp"
#include "object_model_msgs/msg/track.hpp"

class Sensor {
 private:
    std::string name;
    
    float x;
    float y;
    float angle;

 public:
    const capable_vector_t capable;
    const ctra_squared_t measurement_noise_matrix;

 public:
    Sensor(const std::string& name, float x, float y, float angle,
            const capable_vector_t& capable,
            const ctra_squared_t& measurement_noise_matrix);

    std::string get_name();

    float get_x();

    float get_y();

    float get_angle();
};
