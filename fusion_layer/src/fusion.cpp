#include "fusion_layer/fusion.hpp"
#include <Eigen/Dense> // remove
#include <string> // remove

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
    RCLCPP_INFO(this->get_logger(), "Received message from: \n%s", msg->header.frame_id.c_str());
 
    // TODO: delete this block, letting for now just to let this run through CI to validate lib integration
    {
        using namespace Eigen;
     
        Matrix3f m1;
        m1 << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;
        auto m2 = Matrix3f::Constant(1.5);

        Matrix3f m3 = m1 * m2;

        std::stringstream ss;
        ss << m3;
        RCLCPP_INFO(this->get_logger(), "M = %s", ss.str().c_str());
    }
}

// One main for each node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Fusion>());
    rclcpp::shutdown();

    return 0;
}
