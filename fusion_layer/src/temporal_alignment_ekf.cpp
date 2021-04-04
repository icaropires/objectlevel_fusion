#include "fusion_layer/temporal_alignment_ekf.hpp"

/*
 * TODO: Should object_model_msgs::msg::Track.state be the P matrix?
 */
TemporalAlignmentEKF::TemporalAlignmentEKF(state_t initial_state) {
    state = initial_state;

    state_local_format = format_from_object_model(state);
    x_vector = Eigen::Map<Eigen::Matrix<float, 6, 1>>(state_local_format.data());

    P = Eigen::Matrix<float, 6, 6>::Zero();
    P.diagonal() << 10, 10, 10, 10, 10, 10;  // TODO: Should this be static for each vehicle?

    // Other values are set in predict()
    Q = Eigen::Matrix<float, 6, 6>::Zero();
}

state_t TemporalAlignmentEKF::align(float delta_t) {
    predict(delta_t);

    state = format_to_object_model(state_local_format);
    return state;
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

void TemporalAlignmentEKF::predict(float delta_t) {
    float dt = delta_t;
    float noise_pos = 0.5*8.8*dt*dt;  // assumes 8.8m/s2 as maximum acceleration, forcing the vehicle
    float noise_course = 0.1*dt;  // assumes 0.1rad/s as maximum turn rate for the vehicle
    float noise_velocity= 8.8*dt;  // assumes 8.8m/s2 as maximum acceleration, forcing the vehicle
    float noise_yawrate = 1.0*dt;  // assumes 1.0rad/s2 as the maximum turn rate acceleration for the vehicle
    float noise_accel = 0.5;  // assumes 0.5m/s2

    // TODO: Can be improved and calibratred
    Q.diagonal() << noise_pos*noise_pos, noise_pos*noise_pos, noise_course*noise_course, noise_velocity*noise_velocity, noise_yawrate*noise_yawrate, noise_accel*noise_accel;

    float pos_x = state_local_format[X_IDX], pos_y = state_local_format[Y_IDX], yaw = state_local_format[YAW_IDX];
    float velocity = state_local_format[VELOCITY_IDX], yaw_rate = state_local_format[YAW_RATE_IDX], acceleration = state_local_format[ACCELERATION_IDX];

    if(abs(yaw_rate) <  0.00001) {
        yaw_rate = 0.00001;
    }

    state_local_format[X_IDX] = pos_x + (1 / pow(yaw_rate, 2)) *
        (
            (velocity*yaw_rate + acceleration * yaw_rate * dt) * sinf(yaw + yaw_rate* dt)
            + acceleration * cosf(yaw + yaw_rate * dt)
            - velocity * yaw_rate * sinf(yaw) - acceleration * cosf(yaw)
        );

    state_local_format[Y_IDX] = pos_y + (1 / pow(yaw_rate, 2)) *
        (
            (-velocity*yaw_rate - acceleration * yaw_rate * dt) * cosf(yaw + yaw_rate* dt)
            + acceleration * sinf(yaw + yaw_rate * dt)
            + velocity * yaw_rate * cosf(yaw) - acceleration * sinf(yaw)
        );

    state_local_format[YAW_IDX] = fmod((yaw + yaw_rate * dt + M_PI), (2.0 * M_PI)) - M_PI;
    state_local_format[VELOCITY_IDX] = velocity + acceleration * dt;
    state_local_format[YAW_RATE_IDX] = yaw_rate;
    state_local_format[ACCELERATION_IDX] = acceleration;

    pos_x = state_local_format[X_IDX], pos_y = state_local_format[Y_IDX], yaw = state_local_format[YAW_IDX];
    velocity = state_local_format[VELOCITY_IDX], yaw_rate = state_local_format[YAW_RATE_IDX], acceleration = state_local_format[ACCELERATION_IDX];

    // Calculate the Jacobian of the Dynamic Matrix A
    float a13 = (
      (-yaw_rate*velocity*cosf(yaw) + acceleration*sinf(yaw)
      - acceleration*sinf(dt*yaw_rate + yaw) + (dt*yaw_rate*acceleration + yaw_rate*velocity)*cosf(dt*yaw_rate
      + yaw))/pow(yaw_rate, 2)
    );

    float a14 = (-yaw_rate*sinf(yaw) + yaw_rate*sinf(dt*yaw_rate + yaw))/pow(yaw_rate, 2);

    float a15 = (
      (
        -dt*acceleration*sinf(dt*yaw_rate + yaw) + dt*(dt*yaw_rate*acceleration + yaw_rate*velocity)
        * cosf(dt*yaw_rate + yaw) - velocity*sinf(yaw) + (dt*acceleration + velocity)
        * sinf(dt*yaw_rate + yaw)
      ) / pow(yaw_rate, 2)
      - 2*(
        -yaw_rate*velocity*sinf(yaw) - acceleration
        * cosf(yaw) + acceleration*cosf(dt*yaw_rate + yaw) + (dt*yaw_rate*acceleration + yaw_rate*velocity)
        * sinf(dt*yaw_rate + yaw)
      ) / pow(yaw_rate, 3)
    );

    float a16 = (dt*yaw_rate*sinf(dt*yaw_rate + yaw) - cosf(yaw) + cosf(dt * yaw_rate + yaw))/pow(yaw_rate, 2);

    float a23 = (
      (
        -yaw_rate * velocity * sinf(yaw) - acceleration * cosf(yaw) + acceleration * cosf(dt * yaw_rate + yaw)
        - (-dt * yaw_rate*acceleration - yaw_rate * velocity) * sinf(dt * yaw_rate + yaw)
      ) / pow(yaw_rate, 2)
    );

    float a24 = (yaw_rate * cosf(yaw) - yaw_rate*cosf(dt*yaw_rate + yaw))/pow(yaw_rate, 2);

    float a25 = (
      (
        dt * acceleration*cosf(dt*yaw_rate + yaw) - dt * (-dt*yaw_rate*acceleration - yaw_rate * velocity)
        * sinf(dt * yaw_rate + yaw) + velocity*cosf(yaw) + (-dt*acceleration - velocity)*cosf(dt*yaw_rate + yaw)
      ) / pow(yaw_rate, 2)
      - 2*(
        yaw_rate*velocity*cosf(yaw) - acceleration * sinf(yaw) + acceleration * sinf(dt*yaw_rate + yaw)
        + (-dt * yaw_rate * acceleration - yaw_rate * velocity)*cosf(dt*yaw_rate + yaw)
      ) / pow(yaw_rate, 3)
    );

    float a26 = (-dt*yaw_rate*cosf(dt*yaw_rate + yaw) - sinf(yaw) + sinf(dt*yaw_rate + yaw))/pow(yaw_rate, 2);

    Eigen::Matrix<float, 6, 6> JA;
    JA << 1.0, 0.0, a13, a14, a15, a16,
          0.0, 1.0, a23, a24, a25, a26,
          0.0, 0.0, 1.0, 0.0, dt,  0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, dt,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    // Project the error covariance ahead
    P = JA*P*JA.transpose() + Q;
}

void TemporalAlignmentEKF::update(object_model_msgs::msg::Track measurement, state_squared_t measurement_noise_matrix) {
    state = {2, 2, 2, 2, 2, 2, 2, 2};
}
