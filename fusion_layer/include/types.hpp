#pragma once

#include <array>
#include <Eigen/Dense>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Aff_transformation_2.h>

#include <object_model_msgs/msg/object_model.hpp>

using state_t = std::array<float, object_model_msgs::msg::Track::STATE_SIZE>;
using state_squared_t = std::array<float, object_model_msgs::msg::Track::STATE_SIZE*object_model_msgs::msg::Track::STATE_SIZE>;

using state_vector_t = Eigen::Matrix<float, object_model_msgs::msg::Track::STATE_SIZE, 1>;
using state_covariance_matrix_t = Eigen::Matrix<float, object_model_msgs::msg::Track::STATE_SIZE, object_model_msgs::msg::Track::STATE_SIZE>;
using measurement_noise_matrix_t = Eigen::Matrix<float, object_model_msgs::msg::Track::STATE_SIZE, object_model_msgs::msg::Track::STATE_SIZE>;
using process_noise_matrix_t = Eigen::Matrix<float, object_model_msgs::msg::Track::STATE_SIZE, object_model_msgs::msg::Track::STATE_SIZE>;

using capable_vector_t = std::array<bool, object_model_msgs::msg::Track::STATE_SIZE>;

// EKF - Temporal Alignment
constexpr int ctra_size_t = 6;
using ctra_vector_t = Eigen::Matrix<float, ctra_size_t, 1>;
using ctra_matrix_t = Eigen::Matrix<float, ctra_size_t, ctra_size_t>;
using ctra_array_t = std::array<float, 6>;
using ctra_squared_t = std::array<float, ctra_size_t*ctra_size_t>;

// CGAL
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Polygon = CGAL::Polygon_2<Kernel>;
using Point = CGAL::Point_2<Kernel>;
using PolygonWithHoles = CGAL::Polygon_with_holes_2<Kernel>;
using Transformation = CGAL::Aff_transformation_2<Kernel>;
