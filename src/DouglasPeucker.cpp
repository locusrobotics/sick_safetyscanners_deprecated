#include "sick_viz/DouglasPeucker.h"
#include <cmath>
#include <stdexcept>

namespace dougpeuck {

double perpendicularDistance(const Point& pt, const Point& lineStart, const Point& lineEnd) {
    double dx = lineEnd.x - lineStart.x;
    double dy = lineEnd.y - lineStart.y;

    if (dx == 0 && dy == 0) {
        dx = pt.x - lineStart.x;
        dy = pt.y - lineStart.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double t = ((pt.x - lineStart.x) * dx + (pt.y - lineStart.y) * dy) / (dx * dx + dy * dy);
    if (t < 0) {
        dx = pt.x - lineStart.x;
        dy = pt.y - lineStart.y;
    } else if (t > 0) {
        dx = pt.x - lineEnd.x;
        dy = pt.y - lineEnd.y;
    } else {
        dx = pt.x - (lineStart.x + t * dx);
        dy = pt.y - (lineStart.y + t * dy);
    }

    return std::sqrt(dx * dx + dy * dy);
}

void douglasPeucker(const std::vector<Point>& points, double epsilon, std::vector<Point>& outPoints) {
    if (points.size() < 2) {
        throw std::invalid_argument("Not enough points to simplify");
    }

    double maxDistance = 0;
    size_t index = 0;
    for (size_t i = 1; i < points.size() - 1; ++i) {
        double distance = perpendicularDistance(points[i], points[0], points.back());
        if (distance > maxDistance) {
            maxDistance = distance;
            index = i;
        }
    }

    if (maxDistance > epsilon) {
        std::vector<Point> leftSubVector(points.begin(), points.begin() + index + 1);
        std::vector<Point> rightSubVector(points.begin() + index, points.end());

        std::vector<Point> leftResult, rightResult;
        douglasPeucker(leftSubVector, epsilon, leftResult);
        douglasPeucker(rightSubVector, epsilon, rightResult);

        outPoints.assign(leftResult.begin(), leftResult.end() - 1);
        outPoints.insert(outPoints.end(), rightResult.begin(), rightResult.end());
    } else {
        outPoints.clear();
        outPoints.push_back(points.front());
        outPoints.push_back(points.back());
    }
}

std::vector<Point> simplifyPolyline(const std::vector<Point>& points, float epsilon) {
    std::vector<Point> result;
    if (points.size() < 2) {
        return points;
    }
    douglasPeucker(points, epsilon, result);
    return result;
}

} // namespace dougpeuck
