#ifndef VIS_WHYATT_H
#define VIS_WHYATT_H

#include <vector>

namespace viswhyatt {
/**
* @brief Implementation of the Visvalingam Whyatt algorithm
*/

struct Point {
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

// Calculate the area of the triangle formed by the three points
double triangleArea(const Point& point1, const Point& point2, const Point& point3);

// Visvalingam-Whyatt function to simplify the polyline
void visWhyatt(const std::vector<Point>& points, double epsilon, std::vector<Point>& outPoints);

// Wrapper function to simplify a polyline
std::vector<Point> simplifyPolyline(const std::vector<Point>& points, std::size_t targetSize);

} // namespace viswhyatt

#endif // VIS_WHYATT_H
