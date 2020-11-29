#include "fusion_layer/fusion.hpp"

Fusion::Fusion()
  : Node("fusion_layer")
{
    // Make it attribute
    const std::string topic = "objectlevel_fusion/fusion_layer/fusion/submit";

    subscription_ = this->create_subscription<object_model_msgs::msg::ObjectModel>(
        topic, 10, std::bind(&Fusion::topic_callback, this, _1)
    );
}

void Fusion::topic_callback(const object_model_msgs::msg::ObjectModel::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Received message from: %s", msg->header.frame_id.c_str());
}

// One main for each node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Fusion>());
    rclcpp::shutdown();

    return 0;
}
