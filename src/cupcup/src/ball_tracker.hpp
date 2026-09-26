#pragma once

#include <algorithm>
#include <cmath>

namespace cupcup {

struct BallObservation {
    bool valid = false;
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    double score = 0.0;
};

// A small, platform-independent tracker for a camera observation.
// It smooths noisy detections, keeps a short prediction during a dropped
// frame, and makes prediction expire quickly. It is not a world model.
class BallTracker
{
public:
    BallTracker(double innovationGate = 0.30, double timeout = 0.60)
        : innovationGate_(innovationGate), timeout_(timeout)
    {
    }

    void reset()
    {
        estimate_ = BallObservation();
        vx_ = 0.0;
        vy_ = 0.0;
        vr_ = 0.0;
        lastUpdate_ = -1.0;
        lastMeasurement_ = -1.0;
    }

    BallObservation update(const BallObservation &measurement, double time)
    {
        if (!std::isfinite(time)) return estimate_;

        if (!measurement.valid || !finite(measurement)) {
            predict(time);
            return estimate_;
        }

        if (!estimate_.valid || lastMeasurement_ < 0.0 ||
            time - lastMeasurement_ > timeout_ * 1.5) {
            estimate_ = sanitize(measurement);
            estimate_.valid = true;
            vx_ = 0.0;
            vy_ = 0.0;
            vr_ = 0.0;
            lastUpdate_ = time;
            lastMeasurement_ = time;
            return estimate_;
        }

        const double dt = clamp(time - lastUpdate_, 0.01, 0.30);
        const double predictedX = clamp(estimate_.x + vx_ * dt, 0.0, 1.0);
        const double predictedY = clamp(estimate_.y + vy_ * dt, 0.0, 1.0);
        const double predictedRadius = clamp(estimate_.radius + vr_ * dt, 0.0, 0.5);
        const double innovation = std::hypot(measurement.x - predictedX,
            measurement.y - predictedY);

        // Soften an implausible jump instead of allowing a false positive to
        // teleport the target, while still recovering from a moving ball.
        const double baseAlpha = clamp(0.34 + 0.42 * measurement.score, 0.34, 0.76);
        const double alpha = innovation > innovationGate_ ? baseAlpha * 0.35 : baseAlpha;
        const double correctedX = clamp(predictedX + alpha * (measurement.x - predictedX), 0.0, 1.0);
        const double correctedY = clamp(predictedY + alpha * (measurement.y - predictedY), 0.0, 1.0);
        const double correctedRadius = clamp(
            predictedRadius + alpha * (measurement.radius - predictedRadius), 0.0, 0.5);

        const double measuredVx = (correctedX - estimate_.x) / dt;
        const double measuredVy = (correctedY - estimate_.y) / dt;
        const double measuredVr = (correctedRadius - estimate_.radius) / dt;
        vx_ = clamp(0.65 * vx_ + 0.35 * measuredVx, -1.5, 1.5);
        vy_ = clamp(0.65 * vy_ + 0.35 * measuredVy, -1.5, 1.5);
        vr_ = clamp(0.65 * vr_ + 0.35 * measuredVr, -1.0, 1.0);

        estimate_.valid = true;
        estimate_.x = correctedX;
        estimate_.y = correctedY;
        estimate_.radius = correctedRadius;
        estimate_.score = clamp(0.65 * estimate_.score + 0.35 * measurement.score, 0.0, 1.0);
        lastUpdate_ = time;
        lastMeasurement_ = time;
        return estimate_;
    }

    bool isPredicted(double time) const
    {
        return estimate_.valid && lastMeasurement_ >= 0.0 &&
            time - lastMeasurement_ > 0.01;
    }

private:
    static double clamp(double value, double low, double high)
    {
        return std::max(low, std::min(high, value));
    }

    static bool finite(const BallObservation &observation)
    {
        return std::isfinite(observation.x) && std::isfinite(observation.y) &&
            std::isfinite(observation.radius) && std::isfinite(observation.score);
    }

    static BallObservation sanitize(const BallObservation &observation)
    {
        BallObservation result = observation;
        result.x = clamp(result.x, 0.0, 1.0);
        result.y = clamp(result.y, 0.0, 1.0);
        result.radius = clamp(result.radius, 0.0, 0.5);
        result.score = clamp(result.score, 0.0, 1.0);
        return result;
    }

    void predict(double time)
    {
        if (!estimate_.valid || lastUpdate_ < 0.0) return;
        const double age = time - lastMeasurement_;
        if (age > timeout_) {
            estimate_.valid = false;
            return;
        }

        const double dt = clamp(time - lastUpdate_, 0.0, 0.30);
        estimate_.x = clamp(estimate_.x + vx_ * dt, 0.0, 1.0);
        estimate_.y = clamp(estimate_.y + vy_ * dt, 0.0, 1.0);
        estimate_.radius = clamp(estimate_.radius + vr_ * dt, 0.0, 0.5);
        estimate_.score *= 0.88;
        lastUpdate_ = time;
    }

    BallObservation estimate_;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vr_ = 0.0;
    double innovationGate_ = 0.30;
    double timeout_ = 0.60;
    double lastUpdate_ = -1.0;
    double lastMeasurement_ = -1.0;
};

}  // namespace cupcup
