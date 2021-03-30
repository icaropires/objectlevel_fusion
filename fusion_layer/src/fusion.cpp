#include "fusion_layer/fusion.hpp"

Fusion::Fusion()
  : Node("fusion_layer")
{
    // TODO: Make it attribute
    const std::string topic = "objectlevel_fusion/fusion_layer/fusion/submit";

    subscription_ = this->create_subscription<object_model_msgs::msg::ObjectModel>(
        topic, 10, std::bind(&Fusion::topic_callback, this, _1)
    );

}

std::string Fusion::state_to_str(const state_t& state) {
    using namespace object_model_msgs::msg;

    static constexpr size_t maximum_str_size = 200;
    char c_str[maximum_str_size]; 

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

    return std::string(c_str);
}

void Fusion::topic_callback(const object_model_msgs::msg::ObjectModel::SharedPtr msg) const
{
    using namespace object_model_msgs::msg;

    state_t state = msg->track.state;

    auto state_str = state_to_str(state);
    RCLCPP_INFO(this->get_logger(), "Received message from: %s\nState:\n %s", msg->header.frame_id.c_str(), state_str.c_str());

    float x = 1.0, y = 1.0, theta = 90;
    state_t state_spatially_aligned;
    spatially_align(x, y, theta, state, state_spatially_aligned);
    state_str = state_to_str(state_spatially_aligned);
    RCLCPP_INFO(this->get_logger(), "\nState spatially aligned:\n %s", state_str.c_str());
}

// One main for each node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Fusion>());
    rclcpp::shutdown();

    return 0;
}
