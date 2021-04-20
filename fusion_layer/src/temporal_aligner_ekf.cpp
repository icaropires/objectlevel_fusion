#include "fusion_layer/temporal_aligner_ekf.hpp"

TemporalAlignerEKF::TemporalAlignerEKF() 
  : is_initialized(false) {
}

TemporalAlignerEKF::TemporalAlignerEKF(const state_t& initial_state) 
  : is_initialized(true), P(ctra_matrix_t::Zero()), Q(ctra_matrix_t::Zero())  {

    state_array = format_from_object_model(initial_state);

    P.diagonal() << 10, 10, 10, 10, 10, 10;  // TODO: Should this be static for each vehicle?
}

state_t TemporalAlignerEKF::align(float delta_t) {
    if(not is_initialized) {
        throw std::runtime_error("TemporalAlignerEKF must be initialized first!");
    }
    predict(delta_t);

    return get_state();
}

void TemporalAlignerEKF::align(float delta_t, state_t& state, ctra_squared_t& covariation) {
    ctra_array_t ctra_state = format_from_object_model(state);
    ctra_matrix_t covariation_matrix(covariation.data());

    predict(delta_t, ctra_state, covariation_matrix);

    state = format_to_object_model(ctra_state);

    float *covariation_carray_ptr = covariation_matrix.data();
    std::copy(covariation_carray_ptr, covariation_carray_ptr+(ctra_size_t*ctra_size_t), std::begin(covariation));
}

state_t TemporalAlignerEKF::get_state() const {
    return format_to_object_model(state_array);
}

void TemporalAlignerEKF::predict(float delta_t) {
    Q = calculate_process_noise(delta_t);
    state_array = predict_state(delta_t, state_array);
    P = predict_covariation(delta_t, state_array, P, Q);
}

void TemporalAlignerEKF::predict(float delta_t, ctra_array_t& state, ctra_matrix_t& covariation) {
    ctra_matrix_t process_noise = calculate_process_noise(delta_t);
    state = predict_state(delta_t, state);

    covariation = predict_covariation(delta_t, state, covariation, process_noise);
}

ctra_matrix_t TemporalAlignerEKF::calculate_process_noise(float delta_t) {
    const float dt = delta_t;
    const float noise_pos = 0.5*8.8*dt*dt;  // assumes 8.8m/s2 as maximum acceleration, forcing the vehicle
    const float noise_course = 0.1*dt;  // assumes 0.1rad/s as maximum turn rate for the vehicle
    const float noise_velocity= 8.8*dt;  // assumes 8.8m/s2 as maximum acceleration, forcing the vehicle
    const float noise_yawrate = 1.0*dt;  // assumes 1.0rad/s2 as the maximum turn rate acceleration for the vehicle
    const float noise_accel = 0.5;  // assumes 0.5m/s2

    // TODO: Can be improved and calibratred
    ctra_matrix_t process_noise = ctra_matrix_t::Zero();  // It's already initialized with zeros in constructuctor, but this makes the implementation more reliable
    process_noise.diagonal() << noise_pos*noise_pos, noise_pos*noise_pos, noise_course*noise_course, noise_velocity*noise_velocity, noise_yawrate*noise_yawrate, noise_accel*noise_accel;

    return process_noise;
}

ctra_array_t TemporalAlignerEKF::predict_state(float delta_t, const ctra_array_t& state) {
    float dt = delta_t;
    float pos_x = state[X_IDX], pos_y = state[Y_IDX], yaw = state[YAW_IDX];
    float velocity = state[VELOCITY_IDX], yaw_rate = state[YAW_RATE_IDX], acceleration = state[ACCELERATION_IDX];

    ctra_array_t predicted_state;

    if(abs(yaw_rate) <  0.00001) {
        yaw_rate = 0.00001;
    }

    predicted_state[X_IDX] = pos_x + (1 / pow(yaw_rate, 2)) *
        (
            (velocity*yaw_rate + acceleration * yaw_rate * dt) * sinf(yaw + yaw_rate* dt)
            + acceleration * cosf(yaw + yaw_rate * dt)
            - velocity * yaw_rate * sinf(yaw) - acceleration * cosf(yaw)
        );

    predicted_state[Y_IDX] = pos_y + (1 / pow(yaw_rate, 2)) *
        (
            (-velocity*yaw_rate - acceleration * yaw_rate * dt) * cosf(yaw + yaw_rate* dt)
            + acceleration * sinf(yaw + yaw_rate * dt)
            + velocity * yaw_rate * cosf(yaw) - acceleration * sinf(yaw)
        );

    predicted_state[YAW_IDX] = fmod((yaw + yaw_rate * dt + M_PI), (2.0 * M_PI)) - M_PI;
    predicted_state[YAW_IDX] = remainderf(yaw + yaw_rate * dt, 2*M_PI);  // Also keeps angle in [0, 2*pi)
    predicted_state[VELOCITY_IDX] = velocity + acceleration * dt;
    predicted_state[YAW_RATE_IDX] = yaw_rate;
    predicted_state[ACCELERATION_IDX] = acceleration;

    return predicted_state;
}

ctra_matrix_t TemporalAlignerEKF::predict_covariation(float delta_t, const ctra_array_t& state, const ctra_matrix_t& covariation, const ctra_matrix_t& process_noise) {
    ctra_matrix_t JA = gen_ja_matrix(delta_t, state);

    // Project the error covariance ahead
    return JA*covariation*JA.transpose() + process_noise;
}

ctra_matrix_t TemporalAlignerEKF::gen_ja_matrix(float delta_t, const ctra_array_t& state) {
    float dt = delta_t;
    float yaw = state[YAW_IDX], velocity = state[VELOCITY_IDX], yaw_rate = state[YAW_RATE_IDX], acceleration = state[ACCELERATION_IDX];

    // Calculate the Jacobian
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

    ctra_matrix_t JA;
    JA << 1.0, 0.0, a13, a14, a15, a16,
          0.0, 1.0, a23, a24, a25, a26,
          0.0, 0.0, 1.0, 0.0, dt,  0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, dt,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    return JA;
}

void TemporalAlignerEKF::disable_not_capable_attributes(ctra_matrix_t& JH, const capable_vector_t& capable) {
    state_t capable_float;
    for(int i = 0; i < object_model_msgs::msg::Track::STATE_SIZE; ++i) {
        capable_float[i] = static_cast<float>(capable[i]);
    }

    ctra_array_t ctra_cabable = format_from_object_model(capable_float);
    for(int i = 0; i < ctra_size_t; ++i) {
        if(ctra_cabable[i] < 0.001) {  // almost 0
            JH(i, i) = 0.0;
        }
    }
}

void TemporalAlignerEKF::update(const state_t& measurement, const ctra_squared_t& measurement_noise_matrix, const capable_vector_t& capable) {
    if(not is_initialized) {
        throw std::runtime_error("TemporalAlignerEKF must be initialized first!");
    }

    ctra_matrix_t R(measurement_noise_matrix.data());

    Eigen::Map<ctra_vector_t> x(state_array.data());

    ctra_vector_t hx = x;  // Not making any transformation and considering that all the atributes are being measured

    ctra_matrix_t JH;
    JH.setIdentity();

    disable_not_capable_attributes(JH, capable);

    ctra_matrix_t S = JH*P*JH.transpose() + R;
    ctra_matrix_t K = (P*JH.transpose()) * S.inverse();

    // Update the estimate
    auto Z_array = format_from_object_model(measurement);
    Eigen::Map<ctra_vector_t> Z(Z_array.data());
    ctra_vector_t y = Z - hx;  // Innovation or Residual

    x = x + K*y;
    P = (ctra_matrix_t::Identity() - K*JH)*P;
}

/*
* From [x, y, yaw, v, yaw_rate, a] to [x, y, vx, vy, ax, ay, yaw, yaw_rate]
*/
state_t TemporalAlignerEKF::format_to_object_model(const ctra_array_t& state){
    return {
        state[X_IDX],
        state[Y_IDX],
        cosf(state[YAW_IDX])*state[VELOCITY_IDX],
        sinf(state[YAW_IDX])*state[VELOCITY_IDX],
        cosf(state[YAW_IDX])*state[ACCELERATION_IDX],
        sinf(state[YAW_IDX])*state[ACCELERATION_IDX],
        state[YAW_IDX],
        state[YAW_RATE_IDX]
    };
}

/*
* From [x, y, vx, vy, ax, ay, yaw, yaw_rate] to [x, y, yaw, v, yaw_rate, a]
*/
ctra_array_t TemporalAlignerEKF::format_from_object_model(const state_t& state){
    using namespace object_model_msgs::msg;

    return {
        state[Track::STATE_X_IDX],
        state[Track::STATE_Y_IDX],
        state[Track::STATE_YAW_IDX],
        hypotf(state[Track::STATE_VELOCITY_X_IDX], state[Track::STATE_VELOCITY_Y_IDX]),
        state[Track::STATE_YAW_RATE_IDX],
        hypotf(state[Track::STATE_ACCELERATION_X_IDX], state[Track::STATE_ACCELERATION_Y_IDX])
    };
}
