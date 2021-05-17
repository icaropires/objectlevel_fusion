#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Aff_transformation_2.h>

#include "types.hpp"
#include "object_model_msgs/msg/object_model.hpp"

namespace SimpleAssociation {
    uint32_t associate(const object_model_msgs::msg::Object& received,
            const std::map<uint32_t, object_model_msgs::msg::Object::SharedPtr>& global_object_model,
            double break_threshold = 0.9,
            double minimum_score = 0.5);
    
    Polygon obj_to_rectangle(const object_model_msgs::msg::Object& object);
    
    double get_association_score(const Polygon& pol1, const Polygon& pol2);
}
