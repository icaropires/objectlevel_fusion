#include "fusion_layer/fusion.hpp"

Fusion::Fusion()
  : Node("fusion_layer")
{
    const std::string topic = "objectlevel_fusion/fusion_layer/fusion";

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        topic, 10, std::bind(&Fusion::topic_callback, this, _1)
    );
}

void Fusion::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

// One main for each node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Fusion>());
    rclcpp::shutdown();

    return 0;
}
