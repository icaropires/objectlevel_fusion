#include "fusion_layer/fusion.hpp"

Fusion::Fusion()
  : Node("fusion_layer"),
    object_id_counter(0),
    fusions_counter(0),
    input_topic("objectlevel_fusion/fusion_layer/fusion/submit"),
    time_last_msg(0)
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

void Fusion::log_csv_style(const object_model_msgs::msg::ObjectModel::SharedPtr msg, const state_t& state) {
    using object_model_msgs::msg::Track;

    const std::string timestamp = std::to_string(get_timestamp(msg));
    const std::string sensor_name = msg->header.frame_id;

    // Didn't add \n to add more attributes in other points in code
    fprintf(stdout,
            "%s,%s,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f",
            timestamp.c_str(),
            sensor_name.c_str(),
            state[Track::STATE_X_IDX],
            state[Track::STATE_Y_IDX],
            state[Track::STATE_VELOCITY_X_IDX],
            state[Track::STATE_VELOCITY_Y_IDX],
            state[Track::STATE_ACCELERATION_X_IDX],
            state[Track::STATE_ACCELERATION_Y_IDX],
            state[Track::STATE_YAW_IDX],
            state[Track::STATE_YAW_RATE_IDX]);
}

void Fusion::topic_callback(const object_model_msgs::msg::ObjectModel::SharedPtr msg) {
    using object_model_msgs::msg::Dimensions;

    const std::string sensor_name = msg->header.frame_id;
    RCLCPP_INFO(get_logger(), "Received message from: %s", sensor_name.c_str());

    if(sensors.find(sensor_name) == sensors.end()) {
        RCLCPP_WARN(get_logger(), "Sensor '%s' is not registered, ignoring message..", sensor_name.c_str());
        return;
    }

    if(msg->object_model.size() == 0) {
        RCLCPP_WARN(get_logger(), "Got an empty object list from sensor '%s', ignoring message..", sensor_name.c_str());
        return;
    }

    for(const auto& obj: msg->object_model) {
        const auto dimensions_values = obj.dimensions.values;
        if(dimensions_values[Dimensions::DIMENSIONS_LENGHT_IDX] <= 0 || dimensions_values[Dimensions::DIMENSIONS_WIDTH_IDX] <= 0) {
            RCLCPP_WARN(get_logger(), "Received zero or negative object dimensions from sensor '%s', ignoring message..", sensor_name.c_str());
            return;
        }
    }

    // Keep global objects up to date to be fused with arriving objects
    float delta_t = get_delta_t(msg);
    temporally_align_global_objects(delta_t);

    for(auto& obj: msg->object_model) {
        object_id_counter++;

        state_t received_state = obj.track.state;
        log_state("\n==> Received state", object_id_counter, received_state);

        auto sensor = sensors[sensor_name];

        state_t spatially_aligned_state = spatially_align(sensor->get_x(), sensor->get_y(), sensor->get_angle(), received_state);
        log_state("\n==> Received state spatially aligned", object_id_counter, spatially_aligned_state);

        obj.track.state = spatially_aligned_state;

        log_csv_style(msg, obj.track.state); // Adding state attributes to CSV line

        uint32_t global_idx = SimpleAssociation::associate(obj, global_object_model);

        if(global_idx == global_object_model.size()) {  // It's a new object
            global_object_model[global_idx] = std::make_shared<object_model_msgs::msg::Object>(std::move(obj));
        }
        else {  // Will be fused with existent global object
            fusions_counter++;

            // A real fusion will substitute this assignment. For now, just replacing
            global_object_model[global_idx] = std::make_shared<object_model_msgs::msg::Object>(std::move(obj));
            RCLCPP_INFO(get_logger(), "Fusion: object %u <- %u", global_idx, object_id_counter);
        }

        fprintf(stdout, "\n");  // Ending CSV line
        fflush(stdout);

        RCLCPP_INFO(get_logger(), "\n");
    }

    time_last_msg = get_timestamp(msg);
    RCLCPP_INFO(get_logger(), "Number of registered sensors: %d", sensors.size());
    RCLCPP_INFO(get_logger(), "Number of objects being tracked: %d", global_object_model.size());
    RCLCPP_INFO(get_logger(), "Number of fusions performed: %d", fusions_counter);
    RCLCPP_INFO(get_logger(), "\n\n");
}

void Fusion::temporally_align_global_objects(float delta_t) {
    for(auto& object_pair : global_object_model)  {
        auto global_object = object_pair.second;

        TemporalAlignerEKF::align(delta_t, global_object->track.state, global_object->track.covariation);
    }
}

float Fusion::get_delta_t(const object_model_msgs::msg::ObjectModel::SharedPtr msg) {
    uint64_t delta_t_int = get_timestamp(msg) - time_last_msg;
    float delta_t = static_cast<float>(delta_t_int) / 1e9;  // seconds

    return delta_t;
}

void Fusion::register_sensor(const std::shared_ptr<fusion_layer::srv::RegisterSensor::Request> request,
          std::shared_ptr<fusion_layer::srv::RegisterSensor::Response> response) {
    constexpr int min_name_size = 2, max_name_size = 30;

    try{
        if(request->name.size() < min_name_size || request->name.size() > max_name_size) {
            auto error_msg = "Failed to register sensor! Name should have between " + std::to_string(min_name_size) + " and " + std::to_string(max_name_size) + " characters.";
            throw std::runtime_error(error_msg);
        }

        if(sensors.find(request->name) != sensors.end()) {
            throw std::runtime_error("Sensor '" + request->name + "' already registered! Pick another name.");
        }

        auto sensor = std::make_shared<Sensor> (request->name, request->x, request->y, request->angle, request->capable, request->measurement_noise_matrix);

        sensors[sensor->get_name()] = sensor;

        response->status = "Sensor '" + sensor->get_name() + "' registered successfully!";
    }
    catch (std::runtime_error& e) {
        response->status = e.what();
    }
    catch(std::exception&) {
        response->status = "Unexpected error when registering sensor!";
    }

    RCLCPP_INFO(get_logger(), response->status);
}

void Fusion::remove_sensor(const std::shared_ptr<fusion_layer::srv::RemoveSensor::Request> request,
          std::shared_ptr<fusion_layer::srv::RemoveSensor::Response> response) {

    try {
        if (sensors.find(request->name) == sensors.end()) {
           throw std::runtime_error("Failed to remove sensor! Sensor '" + request->name + "' is not registered.");
        }

        sensors.erase(request->name);
        response->status = "Sensor '" + request->name + "' removed successfully!";
    }
    catch (std::runtime_error& e) {
        response->status = e.what();
    }
    catch(std::exception&) {
        response->status = "Unexpected error when removing sensor!";
    }

    RCLCPP_INFO(get_logger(), response->status);
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

void Fusion::log_state(const std::string& label, uint32_t obj_id, const state_t& state) {
    static constexpr size_t maximum_str_size = 200;
    char state_cstr[maximum_str_size];

    state_to_str(state, state_cstr);
    RCLCPP_INFO(get_logger(), label + " from object %u:\n %s", obj_id, state_cstr);
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
