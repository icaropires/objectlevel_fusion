#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include "object_model_msgs/msg/object_model.hpp"

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Polygon = CGAL::Polygon_2<Kernel>;
using Point = CGAL::Point_2<Kernel>;
using PolygonWithHoles = CGAL::Polygon_with_holes_2<Kernel>;

uint32_t simple_associate(const object_model_msgs::msg::Object& object,
        const std::map<uint32_t, object_model_msgs::msg::Object::SharedPtr>& global_object_model);
