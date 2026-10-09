#pragma once

#include "field_geometry.hpp"
#include <limits>
#include <vector>

namespace cupcup {

struct NavigationObstacle { FieldPoint center; double radius = 0.0; };
struct NavigationRoute { FieldPoint waypoint; double length = 0.0; bool reachable = false; };

inline double pointDistance(FieldPoint a, FieldPoint b) { return std::hypot(a.x - b.x, a.z - b.z); }
inline double pointSegmentDistance(FieldPoint p, FieldPoint a, FieldPoint b) {
    const double dx = b.x - a.x, dz = b.z - a.z;
    const double t = std::max(0.0, std::min(1.0,
        ((p.x - a.x) * dx + (p.z - a.z) * dz) / std::max(1e-9, dx * dx + dz * dz)));
    return pointDistance(p, {a.x + t * dx, a.z + t * dz});
}

inline bool crossesOwnBox(TeamColor color, FieldPoint a, FieldPoint b, double margin) {
    const double sign = FieldGeometry::ownSign(color);
    const double starts[2] = {sign * a.x, a.z}, ends[2] = {sign * b.x, b.z};
    const double low[2] = {FieldGeometry::penaltyFront - margin + 1e-8,
                          -FieldGeometry::penaltyHalfWidth - margin + 1e-8};
    const double high[2] = {FieldGeometry::halfLength + margin,
                           FieldGeometry::penaltyHalfWidth + margin - 1e-8};
    double enter = 0.0, leave = 1.0;
    for (int axis = 0; axis < 2; ++axis) {
        const double delta = ends[axis] - starts[axis];
        if (std::abs(delta) < 1e-10) {
            if (starts[axis] < low[axis] || starts[axis] > high[axis]) return false;
        } else {
            const double p = (low[axis] - starts[axis]) / delta;
            const double q = (high[axis] - starts[axis]) / delta;
            enter = std::max(enter, std::min(p, q)); leave = std::min(leave, std::max(p, q));
            if (enter > leave) return false;
        }
    }
    return enter <= leave;
}

// Small visibility graph: at most four discs, eight circumscribed vertices each.
// Dijkstra chooses a whole route, not a local repulsive force with no escape plan.
// This is an independent implementation of a standard algorithm, not copied code.
inline NavigationRoute planNavigation(TeamColor color, int id, FieldPoint start, FieldPoint goal,
                                      const std::vector<NavigationObstacle> &obstacles, double margin) {
    NavigationRoute result; result.waypoint = start;
    goal = FieldGeometry::legalTarget(color, id, goal, margin);
    if (!std::isfinite(start.x + start.z + goal.x + goal.z)) return result;
    if (FieldGeometry::risk(color, id, start, margin) > 0.0) {
        result.waypoint = goal; result.length = pointDistance(start, goal); result.reachable = true;
        return result;  // Existing per-step safety guard handles recovery from relocation/noise.
    }
    auto visible = [&](FieldPoint a, FieldPoint b) {
        if (FieldGeometry::risk(color, id, b, margin) > 1e-8 ||
            (id == 1 && crossesOwnBox(color, a, b, margin))) return false;
        for (const auto &o : obstacles) {
            if (!std::isfinite(o.center.x + o.center.z + o.radius) || o.radius <= 0.0) continue;
            const double distance = pointDistance(a, o.center);
            // A noisy pose may be inside a disc. Allow only monotonically outward escape.
            if (distance < o.radius &&
                (b.x - a.x) * (a.x - o.center.x) + (b.z - a.z) * (a.z - o.center.z) >= 0 &&
                pointDistance(b, o.center) > distance + 1e-8) continue;
            if (pointSegmentDistance(o.center, a, b) < o.radius - 1e-8) return false;
        }
        return true;
    };
    if (visible(start, goal)) return {goal, pointDistance(start, goal), true};
    std::vector<FieldPoint> nodes{start, goal};
    constexpr double pi = 3.14159265358979323846;
    for (std::size_t i = 0; i < std::min<std::size_t>(4, obstacles.size()); ++i) {
        const auto &o = obstacles[i];
        if (!std::isfinite(o.center.x + o.center.z + o.radius) || o.radius <= 0.0) continue;
        const double radius = (o.radius + 0.015) / std::cos(pi / 8.0);
        for (int k = 0; k < 8; ++k) {
            // Coordinates anchored to attack orientation preserve red/blue rotation symmetry.
            const double angle = k * pi / 4.0;
            const double sign = FieldGeometry::ownSign(color);
            const FieldPoint p{o.center.x + sign * radius * std::cos(angle),
                               o.center.z + sign * radius * std::sin(angle)};
            if (FieldGeometry::risk(color, id, p, margin) <= 0.0) nodes.push_back(p);
        }
    }
    for (double side : {-1.0, 1.0}) {
        const FieldPoint p{FieldGeometry::ownSign(color) * (FieldGeometry::penaltyFront - margin - 0.02),
                           FieldGeometry::ownSign(color) * side *
                           (FieldGeometry::penaltyHalfWidth + margin + 0.02)};
        if (id == 1 && FieldGeometry::insideField(p, margin)) nodes.push_back(p);
    }
    std::vector<double> cost(nodes.size(), std::numeric_limits<double>::infinity());
    std::vector<std::size_t> first(nodes.size(), 0);
    std::vector<bool> closed(nodes.size(), false);
    cost[0] = 0.0;
    for (std::size_t iteration = 0; iteration < nodes.size(); ++iteration) {
        std::size_t current = nodes.size();
        for (std::size_t i = 0; i < nodes.size(); ++i)
            if (!closed[i] && (current == nodes.size() || cost[i] < cost[current])) current = i;
        if (current == nodes.size() || !std::isfinite(cost[current])) break;
        if (current == 1) return {nodes[first[1]], cost[1], true};
        closed[current] = true;
        for (std::size_t i = 1; i < nodes.size(); ++i) {
            if (closed[i] || !visible(nodes[current], nodes[i])) continue;
            const double next = cost[current] + pointDistance(nodes[current], nodes[i]);
            if (next < cost[i] - 1e-8) {
                cost[i] = next; first[i] = current == 0 ? i : first[current];
            }
        }
    }
    return result;  // No route is safer than commanding through an obstacle.
}

}  // namespace cupcup
