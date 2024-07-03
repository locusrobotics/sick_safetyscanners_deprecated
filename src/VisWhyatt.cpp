#include "sick_viz/VisWhyatt.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>


namespace viswhyatt {

double triangleArea(const Point& point1, const Point& point2, const Point& point3){
    double area = 0.5 * std::abs(point1.x * (point2.y - point3.y) +
                                 point2.x * (point3.y - point1.y) +
                                 point3.x * (point1.y - point2.y));
    return area;
}

void visWhyatt(const std::vector<Point>& points, std::size_t targetSize, std::vector<Point>& outPoints) {
    if (points.size() < 2 || targetSize < 2 || targetSize > points.size()) {
        throw std::invalid_argument("Invalid target size for simplification.");
    }

    std::vector<Point> simplifiedPoints {points};
    std::vector<double> areas(simplifiedPoints.size(), std::numeric_limits<double>::max());

    // Calculate initial triangle areas
    for (std::size_t i = 1; i < simplifiedPoints.size() - 1; ++i) {
        areas[i] = triangleArea(simplifiedPoints[i - 1], simplifiedPoints[i], simplifiedPoints[i + 1]);
    }

    while (simplifiedPoints.size() > targetSize) {
        // Find the point with the smallest associated area
        auto minIt = std::min_element(areas.begin() + 1, areas.end() - 1);
        std::size_t minIndex = std::distance(areas.begin(), minIt);

        // Remove the point with the smallest area
        simplifiedPoints.erase(simplifiedPoints.begin() + minIndex);
        areas.erase(areas.begin() + minIndex);

        // Recalculate areas for the affected points
        if (minIndex > 1) {
            areas[minIndex - 1] = triangleArea(simplifiedPoints[minIndex - 2], simplifiedPoints[minIndex - 1], simplifiedPoints[minIndex]);
        }
        if (minIndex < simplifiedPoints.size() - 1) {
            areas[minIndex] = triangleArea(simplifiedPoints[minIndex - 1], simplifiedPoints[minIndex], simplifiedPoints[minIndex + 1]);
        }
    }

    outPoints = simplifiedPoints;
}

std::vector<Point> simplifyPolyline(const std::vector<Point>& points, std::size_t targetSize) {
    std::vector<Point> result;
    if (points.size() < 2 || targetSize < 2 || targetSize > points.size()) {
        return points;
    }
    visWhyatt(points, targetSize, result);
    return result;
}

} // namespace viswhyatt
