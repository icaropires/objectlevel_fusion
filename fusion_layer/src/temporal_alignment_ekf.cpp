#include "fusion_layer/temporal_alignment_ekf.hpp"

TemporalAlignmentEKF::TemporalAlignmentEKF(object_model_msgs::msg::Track initial_track) {

}

state_t TemporalAlignmentEKF::align(float delta_t) {
    return {1, 1, 1, 1, 1, 1, 1, 1};
}

void TemporalAlignmentEKF::predict() {
}

void TemporalAlignmentEKF::update(object_model_msgs::msg::Track measurement, state_squared_t measurement_noise_matrix) {
    state = {2, 2, 2, 2, 2, 2, 2};
}

/*
* From [x, y, yaw, v, yaw_rate, a] to [x, y, vx, vy, ax, ay, yaw, yaw_rate] 
*/
state_t TemporalAlignmentEKF::format_to_object_model(std::array<float, 6> state){
    return {
        state[0], state[1], cosf(state[2])*state[3], sinf(state[2])*state[3],
        cosf(state[2])*state[5], sinf(state[2])*state[5], state[2], state[4]
    };
}

/*
* From [x, y, vx, vy, ax, ay, yaw, yaw_rate] to [x, y, yaw, v, yaw_rate, a]
*/
std::array<float, 6> TemporalAlignmentEKF::format_from_object_model(state_t state){
    return {
        state[0], state[1], state[6],
        hypotf(state[2], state[3]), state[7], hypotf(state[4], state[5])
    };
}

state_t TemporalAlignmentEKF::get_state() const {
    return state;
}

std::string temporal_align() {
    using namespace Eigen;
 
    Matrix3f m1;
    m1 << 1, 2, 3,
          4, 5, 6,
          7, 8, 9;
    auto m2 = Matrix3f::Constant(1.5);

    Matrix3f m3 = m1 * m2;

    std::stringstream ss;
    ss << m3;

    return ss.str();
}
