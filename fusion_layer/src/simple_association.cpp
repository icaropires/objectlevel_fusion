#include "fusion_layer/simple_association.hpp"

uint32_t simple_associate(const object_model_msgs::msg::Object& object,
        const std::map<uint32_t, object_model_msgs::msg::Object::SharedPtr>& global_object_model) {

    using object_model_msgs::msg::Track;

    constexpr int rectangle_n_points = 4;

    // TODO
    (void) object; 
    (void) global_object_model;

    const std::array<Point, rectangle_n_points> points1{Point(1.3, 2.5), Point(2.7, 2.5), Point(2.7, 5.5), Point(1.3, 5.5)};
    const Polygon polygon1(points1.cbegin(), points1.cend());

    const std::array<Point, rectangle_n_points> points2({Point(1.47, 2.65), Point(2.63, 2.34), Point(3.3, 4.85), Point(2.14, 5.16)});
    const Polygon polygon2(points2.cbegin(), points2.cend());

    std::vector<PolygonWithHoles> intersections;
    CGAL::intersection(polygon1, polygon2, std::back_inserter(intersections));

    std::cout << "Intersection area: " << intersections[0].outer_boundary().area() << std::endl;

    return 1;
}
