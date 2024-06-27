#ifndef DOUGLAS_PEUCKER_H
#define DOUGLAS_PEUCKER_H

#include <vector>

namespace dougpeuck {
/**
* @brief Implementation of the Douglas Peucker algorithm
*/

// Point structure to represent a point in 2D space
struct Point {
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

// Calculates the perpendicular distance from a point to a line
double perpendicularDistance(const Point& pt, const Point& lineStart, const Point& lineEnd);

// Douglas Peucker recursive function to simplify the polyline
void douglasPeucker(const std::vector<Point>& points, double epsilon, std::vector<Point>& outPoints);

// Wrapper function to simplify a polyline
std::vector<Point> simplifyPolyline(const std::vector<Point>& points, float epsilon);

} // namespace dougpeuck

#endif // DOUGLAS_PEUCKER_H
