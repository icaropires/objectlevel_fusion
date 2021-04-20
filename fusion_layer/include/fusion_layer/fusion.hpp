#pragma once

#include <functional>
#include <memory>
#include <cstdio>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "fusion_layer/srv/register_sensor.hpp"
#include "fusion_layer/srv/remove_sensor.hpp"
#include "object_model_msgs/msg/object_model.hpp"

#include "fusion_layer/sensor.hpp"
#include "fusion_layer/spatial_alignment.hpp"
#include "fusion_layer/temporal_aligner_ekf.hpp"
#include "types.hpp"

class Fusion : public rclcpp::Node
{
private:
    TemporalAlignerEKF temporal_aligner;

    uint32_t object_id_counter;

    std::map<std::string, std::shared_ptr<Sensor>> sensors;
    std::map<uint32_t, object_model_msgs::msg::Object::SharedPtr> global_object_model;
  
    const std::string input_topic;
  
    uint64_t time_last_msg;
  
    rclcpp::Subscription<object_model_msgs::msg::ObjectModel>::SharedPtr subscription_;

    rclcpp::Service<fusion_layer::srv::RegisterSensor>::SharedPtr register_sensor_srv_;

    rclcpp::Service<fusion_layer::srv::RemoveSensor>::SharedPtr remove_sensor_srv_;

public:
  Fusion();
  ~Fusion();

private:
    void topic_callback(const object_model_msgs::msg::ObjectModel::SharedPtr msg);

    void temporally_align_global_objects(float delta_t);

    float get_delta_t(const object_model_msgs::msg::ObjectModel::SharedPtr msg);

    void register_sensor(const std::shared_ptr<fusion_layer::srv::RegisterSensor::Request> request,
        std::shared_ptr<fusion_layer::srv::RegisterSensor::Response> response);

    void remove_sensor(const std::shared_ptr<fusion_layer::srv::RemoveSensor::Request> request,
        std::shared_ptr<fusion_layer::srv::RemoveSensor::Response> response);
  
    void log_state(const std::string& label, uint32_t obj_id, const state_t& state);
  
    static uint64_t get_timestamp(const object_model_msgs::msg::ObjectModel::SharedPtr msg);
  
    static void state_to_str(const state_t& state, char *c_str);
};
