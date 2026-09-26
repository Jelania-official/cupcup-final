#include "ball_tracker.hpp"

#include <cassert>
#include <cmath>

int main()
{
    cupcup::BallTracker tracker(0.30, 0.60);
    cupcup::BallObservation observation;
    observation.valid = true;
    observation.x = 0.40;
    observation.y = 0.70;
    observation.radius = 0.04;
    observation.score = 0.90;

    auto first = tracker.update(observation, 0.0);
    assert(first.valid);
    assert(std::abs(first.x - 0.40) < 1e-9);

    observation.x = 0.44;
    auto second = tracker.update(observation, 0.10);
    assert(second.valid);
    assert(second.x > first.x);
    assert(second.x < observation.x + 1e-9);

    auto predicted = tracker.update(cupcup::BallObservation(), 0.20);
    assert(predicted.valid);
    assert(predicted.x >= second.x);
    assert(predicted.score < second.score);
    assert(tracker.isPredicted(0.20));

    observation.x = 0.95;
    auto gated = tracker.update(observation, 0.30);
    assert(gated.valid);
    assert(gated.x < 0.70);

    auto expired = tracker.update(cupcup::BallObservation(), 0.95);
    assert(!expired.valid);
    return 0;
}
