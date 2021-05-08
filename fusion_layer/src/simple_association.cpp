#include "fusion_layer/simple_association.hpp"

namespace SimpleAssociation {
    uint32_t associate(const object_model_msgs::msg::Object& received,
            const std::map<uint32_t, object_model_msgs::msg::Object::SharedPtr>& global_object_model,
            double break_threshold,
            double minimum_score) {
    
        const Polygon received_rectangle = obj_to_rectangle(received);
    
        uint32_t idx_biggest = global_object_model.size();
        double biggest_score = minimum_score;
    
        for(const auto& global: global_object_model) {
            const Polygon global_rectangle = obj_to_rectangle(*global.second);
    
            const double association_score = get_association_score(received_rectangle, global_rectangle);
            if(association_score > biggest_score){
                idx_biggest = global.first;
                biggest_score = association_score;
    
                if(biggest_score >= break_threshold) {
                    break;
                }
            }
        }

        // Adding attributes to CSV
        fprintf(stdout, ",%d,%0.3f,%d",
                idx_biggest,
                biggest_score,
                idx_biggest == global_object_model.size());

        return idx_biggest;
    }
    
    double get_association_score(const Polygon& pol1, const Polygon& pol2) {
        std::vector<PolygonWithHoles> intersections;
        CGAL::intersection(pol1, pol2, std::back_inserter(intersections));
    
        const double pol1_area = CGAL::to_double(pol1.area());
        const double pol2_area = CGAL::to_double(pol2.area());
    
        if(intersections.size()) {
            const double intersection_area = CGAL::to_double(intersections[0].outer_boundary().area());
            const double union_area = pol1_area + pol2_area - intersection_area;

            const double intersection_over_union = intersection_area / union_area;  // Jaccard index
    
            return intersection_over_union;
        }
    
        return 0;
    }
    
    Polygon obj_to_rectangle(const object_model_msgs::msg::Object& object) {
        using object_model_msgs::msg::Track;
        using object_model_msgs::msg::Dimensions;
    
        constexpr int rectangle_n_points = 4;
    
        const auto& state = object.track.state;
        const auto& dimensions = object.dimensions.values;
    
        const float x = state[Track::STATE_X_IDX];
        const float y = state[Track::STATE_Y_IDX];
        const float half_width = dimensions[Dimensions::DIMENSIONS_WIDTH_IDX]/2;
        const float half_length = dimensions[Dimensions::DIMENSIONS_LENGHT_IDX]/2;
    
        const float yaw = state[Track::STATE_YAW_IDX];
        Transformation rotate(CGAL::ROTATION, sin(yaw), cos(yaw));
    
        std::array<Point, rectangle_n_points> points ({
            Point(x-half_width, y-half_length),
            Point(x+half_width, y-half_length),
            Point(x+half_width, y+half_length),
            Point(x-half_width, y+half_length)
        });
    
        for(auto& point: points) {
            point = rotate(point);
        }
    
        return Polygon(points.cbegin(), points.cend());
    }
}
