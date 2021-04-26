#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Aff_transformation_2.h>

#include <cmath>

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Polygon = CGAL::Polygon_2<Kernel>;
using Point = CGAL::Point_2<Kernel>;
using PolygonWithHoles = CGAL::Polygon_with_holes_2<Kernel>;
using Transformation = CGAL::Aff_transformation_2<Kernel>;

int main() {
  constexpr int rectangle_n_points = 4;

  const std::array<Point, rectangle_n_points> points1 {
      Point(1.3, 2.5), Point(2.7, 2.5),
      Point(2.7, 5.5), Point(1.3, 5.5)
  };
  const Polygon polygon1(points1.cbegin(), points1.cend());

  float angle = -M_PI/12;  // 45º clockwise
  Transformation rotate(CGAL::ROTATION, sin(angle), cos(angle));

  std::array<Point, rectangle_n_points> points2 {
      Point(0.73461, 2.93749), Point(1.93641, 2.93749),
      Point(1.93461, 5.53749), Point(0.73461, 5.53749)
  };

  for(auto& p: points2) {
      p = rotate(p);
  }

  Polygon polygon2(points2.cbegin(), points2.cend());

  std::cout << "Polygon 2 rotated: " << polygon2 << std::endl;

  std::vector<PolygonWithHoles> intersections;
  CGAL::intersection(polygon1, polygon2, std::back_inserter(intersections));

  std::cout << "Intersection area: " << intersections[0].outer_boundary().area() << std::endl;

  return EXIT_SUCCESS;
}
