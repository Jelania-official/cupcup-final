#include "navigation.hpp"
#include <cassert>

int main() {
    using namespace cupcup;
    const auto direct = planNavigation(TeamColor::Red, 1, {1, 0}, {-1, 0}, {}, 0.2);
    assert(direct.reachable && direct.waypoint.x == -1 && direct.length == 2);
    const std::vector<NavigationObstacle> obstacle{{{0, 0}, .4}};
    const auto around = planNavigation(TeamColor::Red, 1, {1, 0}, {-1, 0}, obstacle, .2);
    assert(around.reachable && std::abs(around.waypoint.z) > .1 && around.length > 2);
    assert(pointSegmentDistance({0, 0}, {1, 0}, around.waypoint) >= .4 - 1e-8);
    const auto mirror = planNavigation(TeamColor::Blue, 1, {-1, 0}, {1, 0}, obstacle, .2);
    assert(std::abs(around.waypoint.x + mirror.waypoint.x) < 1e-8);
    assert(std::abs(around.waypoint.z + mirror.waypoint.z) < 1e-8);
    assert(std::abs(around.length - mirror.length) < 1e-8);
    const auto trapped = planNavigation(TeamColor::Red, 1, {1, 0}, {0, 0}, obstacle, .2);
    assert(!trapped.reachable && trapped.waypoint.x == 1);
    const auto escape = planNavigation(TeamColor::Red, 1, {.1, 0}, {1, 0}, obstacle, .2);
    assert(escape.reachable && escape.waypoint.x == 1);
    assert(crossesOwnBox(TeamColor::Red, {2, 0}, {4, 3}, .0));
    assert(!crossesOwnBox(TeamColor::Red, {2, 0}, {2, 3}, .0));
    const auto defender = planNavigation(TeamColor::Red, 2, {1, 0}, {-1, 0}, {}, .2);
    assert(defender.reachable && defender.waypoint.x >= .2);
    const auto recover = planNavigation(TeamColor::Red, 1, {3, 0}, {1, 0}, {}, .2);
    assert(recover.reachable && recover.waypoint.x < 2.3);
}
