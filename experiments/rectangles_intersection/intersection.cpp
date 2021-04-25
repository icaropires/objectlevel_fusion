#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Polygon = CGAL::Polygon_2<Kernel>;
using Point = CGAL::Point_2<Kernel>;
using PolygonWithHoles = CGAL::Polygon_with_holes_2<Kernel>;

int main() {
  const std::array<Point, 4> points1{Point(1.3, 2.5), Point(2.7, 2.5), Point(2.7, 5.5), Point(1.3, 5.5)};
	const Polygon polygon1(points1.cbegin(), points1.cend());

  const std::array<Point, 4> points2({Point(1.47, 2.65), Point(2.63, 2.34), Point(3.3, 4.85), Point(2.14, 5.16)});
	const Polygon polygon2(points2.cbegin(), points2.cend());

  std::vector<PolygonWithHoles> intersections;
  CGAL::intersection(polygon1, polygon2, std::back_inserter(intersections));

  std::cout << "Intersection area: " << intersections[0].outer_boundary().area() << std::endl;

  return EXIT_SUCCESS;
}

