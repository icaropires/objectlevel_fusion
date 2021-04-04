#pragma once

#include <array>
#include <Eigen/Dense>

#include <object_model_msgs/msg/object_model.hpp>

using state_t = std::array<float, object_model_msgs::msg::Track::STATE_SIZE>;
using state_squared_t = std::array<float, object_model_msgs::msg::Track::STATE_SIZE*object_model_msgs::msg::Track::STATE_SIZE>;

using state_vector_t = Eigen::Matrix<float, object_model_msgs::msg::Track::STATE_SIZE, 1>;
using state_covariance_matrix_t = Eigen::Matrix<float, object_model_msgs::msg::Track::STATE_SIZE, object_model_msgs::msg::Track::STATE_SIZE>;
using measurement_noise_matrix_t = Eigen::Matrix<float, object_model_msgs::msg::Track::STATE_SIZE, object_model_msgs::msg::Track::STATE_SIZE>;
using process_noise_matrix_t = Eigen::Matrix<float, object_model_msgs::msg::Track::STATE_SIZE, object_model_msgs::msg::Track::STATE_SIZE>;
