#include "fusion_layer/sensor.hpp"

Sensor::Sensor(const std::string& name, float x, float y, float angle,
            const capable_vector_t& capable,
            const ctra_squared_t& measurement_noise_matrix)
    : name(name), x(x), y(y), angle(angle), capable(capable), measurement_noise_matrix(measurement_noise_matrix) {
}

std::string Sensor::get_name() {
    return name;
}

float Sensor::get_x() {
    return x;
}

float Sensor::get_y() {
    return y;
}

float Sensor::get_angle() {
    return angle;
}
