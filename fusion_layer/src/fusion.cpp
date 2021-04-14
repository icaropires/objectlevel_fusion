#include "fusion_layer/fusion.hpp"

Fusion::Fusion()
  : Node("fusion_layer"),
    input_topic("objectlevel_fusion/fusion_layer/fusion/submit"),
    is_first_message(true), time_last_msg(0)
{
    register_sensor_srv_ = create_service<fusion_layer::srv::RegisterSensor>("fusion_layer/register_sensor",
            std::bind(&Fusion::register_sensor, this, std::placeholders::_1, std::placeholders::_2));

    remove_sensor_srv_ = create_service<fusion_layer::srv::RemoveSensor>("fusion_layer/remove_sensor",
            std::bind(&Fusion::remove_sensor, this, std::placeholders::_1, std::placeholders::_2));

    subscription_ = create_subscription<object_model_msgs::msg::ObjectModel>(
        input_topic, 10, std::bind(&Fusion::topic_callback, this, std::placeholders::_1)
    );
}

Fusion::~Fusion() {
    rclcpp::shutdown();
}

void Fusion::register_sensor(const std::shared_ptr<fusion_layer::srv::RegisterSensor::Request> request,
          std::shared_ptr<fusion_layer::srv::RegisterSensor::Response> response)
{
    auto sensor = std::make_shared<Sensor> (request->name, request->x, request->y, request->angle, request->capable, request->measurement_noise_matrix);
    sensors[sensor->get_name()] = sensor;
    response->status = "Sensor '" + sensor->get_name() + "' registered successfully!";

    RCLCPP_INFO(get_logger(), response->status);
}

void Fusion::remove_sensor(const std::shared_ptr<fusion_layer::srv::RemoveSensor::Request> request,
          std::shared_ptr<fusion_layer::srv::RemoveSensor::Response> response) {

    sensors.erase(request->name);

    response->status = "Sensor '" + request->name + "' removed successfully!";

    RCLCPP_INFO(get_logger(), response->status);
}

void Fusion::topic_callback(const object_model_msgs::msg::ObjectModel::SharedPtr msg)
{
    using namespace object_model_msgs::msg;

    state_t received_state = msg->track.state;

    RCLCPP_INFO(get_logger(), "Received message from: %s", msg->header.frame_id.c_str());
    log_state((char *) "\n==> Received state:\n %s", received_state);

    float x = 1.0, y = 1.0, theta = M_PI/4;
    state_t spatially_aligned_state;
    spatially_align(x, y, theta, received_state, spatially_aligned_state);

    log_state((char *) "\n==> State spatially aligned:\n %s", spatially_aligned_state);

    state_t temporally_aligned_state;

    if(not is_first_message) {
        uint64_t delta_t_int = get_timestamp(msg) - time_last_msg;
        float delta_t = static_cast<float>(delta_t_int) / 1e9;  // seconds

        temporally_aligned_state = temporal_aligner.align(delta_t);

        log_state((char *) "\n==> State temporally aligned:\n %s", temporally_aligned_state);
    } else {
        temporal_aligner = TemporalAlignerEKF(spatially_aligned_state);

        is_first_message = false;
        time_last_msg = get_timestamp(msg);
        RCLCPP_INFO(get_logger(), "Temporal Aligner intialized\n\n");
        return;
    }

    // Final state of the object
    // Must be updated in order to have the state after all preprocessing and fusion
    state_t final_state = temporally_aligned_state;

    // Filling measurement_noise_matrix here
    // TODO: remove, once sensor registration is completed
    ctra_matrix_t measurement_noise_matrix;
    ctra_squared_t measurement_noise_array;
    measurement_noise_matrix.setZero();
    measurement_noise_matrix.diagonal() << 2.25, 2.25, 0.0004, 1.0, 0.01, 0.25;
    float *measurement_carray_ptr = measurement_noise_matrix.data();
    std::copy(measurement_carray_ptr, measurement_carray_ptr+(ctra_size_t*ctra_size_t), std::begin(measurement_noise_array));

    temporal_aligner.update(final_state, measurement_noise_array);

    time_last_msg = get_timestamp(msg);
    RCLCPP_INFO(get_logger(), "Number of registered sensors: %d", sensors.size());
    RCLCPP_INFO(get_logger(), "\n\n");
}

void Fusion::state_to_str(const state_t& state, char *c_str) {
    using namespace object_model_msgs::msg;

    sprintf(c_str,
            "X = %0.3f, Y = %0.3f, Vx = %0.3f, Vy = %0.3f, Ax = %0.3f, Ay = %0.3f, Yaw = %0.3f, Yaw Rate = %0.3f",
            state[Track::STATE_X_IDX],
            state[Track::STATE_Y_IDX],
            state[Track::STATE_VELOCITY_X_IDX],
            state[Track::STATE_VELOCITY_Y_IDX],
            state[Track::STATE_ACCELERATION_X_IDX],
            state[Track::STATE_ACCELERATION_Y_IDX],
            state[Track::STATE_YAW_IDX],
            state[Track::STATE_YAW_RATE_IDX]);
}

void Fusion::log_state(char *label, const state_t& state) {
    static constexpr size_t maximum_str_size = 200;
    char c_str[maximum_str_size];

    state_to_str(state, c_str);
    RCLCPP_INFO(get_logger(), label, c_str);
}

/*
 * return: message timestamp in nanoseconds
 */
uint64_t Fusion::get_timestamp(const object_model_msgs::msg::ObjectModel::SharedPtr msg)
{
    auto time = rclcpp::Time(msg->header.stamp);
    return time.nanoseconds();
}

// One main for each node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Fusion>());

    return 0;
}
