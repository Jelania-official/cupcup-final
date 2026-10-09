#pragma once

#include "strategy_logic.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace cupcup {

inline FieldPoint projectToField(double originX, double originZ,
                                 double headingDegrees, double range,
                                 double bearingDegrees)
{
    const double angle = (headingDegrees + bearingDegrees) * M_PI / 180.0;
    return {originX + range * std::cos(angle), originZ - range * std::sin(angle)};
}

struct TimedPoint {
    bool valid = false;
    double x = 0.0;
    double z = 0.0;
    double confidence = 0.0;
    double observedAt = -1.0;

    bool fresh(double now, double timeout) const
    {
        return valid && std::isfinite(now) && now >= observedAt &&
            now - observedAt <= timeout;
    }
};

struct PoseEstimate : TimedPoint {
    double yaw = 0.0;  // degrees; 0 faces +x, positive yaw faces toward -z
};

enum class BallPositionSource { Unknown, Local, GroundRay, Radius, Teammate };
inline const char *ballPositionSourceName(BallPositionSource source) {
    switch (source) {
    case BallPositionSource::Local: return "local";
    case BallPositionSource::GroundRay: return "ground_ray";
    case BallPositionSource::Radius: return "radius";
    case BallPositionSource::Teammate: return "teammate";
    default: return "unknown";
    }
}

enum class RobotTeam { Unknown, Red, Blue };
inline const char *robotTeamName(RobotTeam team) {
    return team == RobotTeam::Red ? "red" : team == RobotTeam::Blue ? "blue" : "unknown";
}
enum class RobotPositionSource { Unknown, BoxHeight, NumberSquare };
inline const char *robotPositionSourceName(RobotPositionSource source) {
    return source == RobotPositionSource::BoxHeight ? "box_height" :
        source == RobotPositionSource::NumberSquare ? "number_square" : "unknown";
}
struct RobotObservation {
    FieldPoint position;
    RobotTeam team = RobotTeam::Unknown;
    double confidence = 0.0;
    double uncertainty = 1.0;
    RobotPositionSource source = RobotPositionSource::Unknown;
};
struct RobotCandidate {
    TimedPoint position;
    unsigned trackId = 0;  // local association, NOT a referee jersey/player ID
    RobotTeam team = RobotTeam::Unknown, pendingTeam = RobotTeam::Unknown;
    int colorHits = 0;
    double uncertainty = 1.0;
    RobotPositionSource latestSource = RobotPositionSource::Unknown;
};

// A compact, time-aware field state. It fuses only observations the current
// package actually has; it is not a localization filter or a tactical planner.
class WorldModel
{
public:
    bool updateSelf(double x, double z, double yaw, double time,
                    double confidence = 0.35)
    {
        return updatePose(self_, x, z, yaw, time, confidence, 2.25, 0.40);
    }

    bool updateTeammate(double x, double z, double yaw, double time,
                        double confidence = 0.35)
    {
        return updatePose(teammate_, x, z, yaw, time, confidence, 3.0, 0.55);
    }

    bool updateBall(double x, double z, double time, double confidence,
                    BallPositionSource source = BallPositionSource::Unknown)
    {
        if (!updatePoint(ball_, x, z, time, confidence, 2.5, 0.55)) return false;
        latestBallSource_ = source;
        return true;
    }

    // The current detector reports a generic robot box, not team identity or
    // calibrated depth. Keep this low-confidence estimate for map/debug use;
    // callers must not use it as a confirmed opponent position for control.
    bool updateRobotCandidate(double x, double z, double time, double confidence)
    {
        if (!validCoordinate(x, z) || time < 0.0 || !std::isfinite(time) ||
            !std::isfinite(confidence)) return false;
        updateRobots({{{x, z}, RobotTeam::Unknown, confidence, 1.0}}, time);
        return true;
    }

    void updateRobots(const std::vector<RobotObservation> &observations, double time)
    {
        if (!std::isfinite(time) || time < 0.0 || time <= robotsObservedAt_) return;
        robotsObservedAt_ = time;
        robots_.erase(std::remove_if(robots_.begin(), robots_.end(), [time](const RobotCandidate &r) {
            return time >= r.position.observedAt && !r.position.fresh(time, 0.65);
        }), robots_.end());
        bool used[4] = {false, false, false, false};
        for (const auto &observation : observations) {
            if (!validCoordinate(observation.position.x, observation.position.z) ||
                !std::isfinite(observation.confidence) || !std::isfinite(observation.uncertainty)) continue;
            std::size_t selected = robots_.size();
            double nearest = 0.8;
            for (std::size_t i = 0; i < robots_.size(); ++i) {
                const auto &track = robots_[i];
                if (used[i] || time < track.position.observedAt ||
                    (observation.team != RobotTeam::Unknown && track.team != RobotTeam::Unknown &&
                     observation.team != track.team)) continue;
                const double delta = std::hypot(track.position.x - observation.position.x,
                                                track.position.z - observation.position.z);
                if (delta < nearest) { nearest = delta; selected = i; }
            }
            if (selected == robots_.size()) {
                if (robots_.size() == 4) continue;
                robots_.push_back(RobotCandidate());
                robots_.back().trackId = nextTrackId_++;
            }
            auto &track = robots_[selected];
            if (!updatePoint(track.position, observation.position.x, observation.position.z, time,
                             std::min(0.25, observation.confidence), 3.0, 0.45)) continue;
            used[selected] = true;
            track.uncertainty = std::max(0.5, observation.uncertainty);
            track.latestSource = observation.source;
            if (observation.team != track.pendingTeam) { track.pendingTeam = observation.team; track.colorHits = 0; }
            track.colorHits = std::min(20, track.colorHits + 1);
            if (track.colorHits >= 2) track.team = track.pendingTeam;
        }
    }

    void expire(double now)
    {
        if (!self_.fresh(now, 2.0)) self_.valid = false;
        if (!teammate_.fresh(now, 1.2)) teammate_.valid = false;
        if (!ball_.fresh(now, 0.65)) ball_.valid = false;
        for (auto &robot : robots_) if (!robot.position.fresh(now, 0.65)) robot.position.valid = false;
    }

    void reset()
    {
        self_ = PoseEstimate();
        teammate_ = PoseEstimate();
        ball_ = TimedPoint();
        latestBallSource_ = BallPositionSource::Unknown;
        robots_.clear(); nextTrackId_ = 1; robotsObservedAt_ = -1.0;
    }

    const PoseEstimate &self() const { return self_; }
    const PoseEstimate &teammate() const { return teammate_; }
    const TimedPoint &ball() const { return ball_; }
    // Latest accepted update only; smoothing may contain older/different sources.
    BallPositionSource latestBallSource() const { return latestBallSource_; }
    const std::vector<RobotCandidate> &robots() const { return robots_; }
    const RobotCandidate &robotCandidate() const {
        static const RobotCandidate missing;
        return robots_.empty() ? missing : robots_.front();
    }

private:
    static double clamp(double value, double low, double high)
    {
        return std::max(low, std::min(high, value));
    }

    static double wrapDegrees(double value)
    {
        while (value > 180.0) value -= 360.0;
        while (value < -180.0) value += 360.0;
        return value;
    }

    static bool validCoordinate(double x, double z)
    {
        // Allow modest measurement noise and a ball just beyond the touchline.
        return std::isfinite(x) && std::isfinite(z) &&
            std::abs(x) <= FieldGeometry::halfLength + 1.0 &&
            std::abs(z) <= FieldGeometry::halfWidth + 1.0;
    }

    static bool updatePoint(TimedPoint &estimate, double x, double z,
                            double time, double confidence, double jumpGate,
                            double alpha)
    {
        if (!validCoordinate(x, z) || !std::isfinite(time) || time < 0.0 ||
            !std::isfinite(confidence)) return false;
        confidence = clamp(confidence, 0.0, 1.0);
        if (estimate.valid) {
            if (time < estimate.observedAt) return false;
            const double age = time - estimate.observedAt;
            const double jump = std::hypot(x - estimate.x, z - estimate.z);
            if (age <= 1.5 && jump > jumpGate) return false;
            if (age <= 1.5) {
                estimate.x += alpha * (x - estimate.x);
                estimate.z += alpha * (z - estimate.z);
                estimate.confidence = 0.65 * estimate.confidence + 0.35 * confidence;
            } else {
                estimate.x = x;
                estimate.z = z;
                estimate.confidence = confidence;
            }
        } else {
            estimate.x = x;
            estimate.z = z;
            estimate.confidence = confidence;
        }
        estimate.valid = true;
        estimate.observedAt = time;
        return true;
    }

    static bool updatePose(PoseEstimate &estimate, double x, double z,
                           double yaw, double time, double confidence,
                           double jumpGate, double alpha)
    {
        if (!std::isfinite(yaw)) return false;
        const bool hadEstimate = estimate.valid;
        const double previousTime = estimate.observedAt;
        const double previousYaw = estimate.yaw;
        if (!updatePoint(estimate, x, z, time, confidence, jumpGate, alpha)) return false;
        if (!hadEstimate || time - previousTime > 1.5) {
            estimate.yaw = wrapDegrees(yaw);
        } else {
            estimate.yaw = wrapDegrees(previousYaw + alpha * wrapDegrees(yaw - previousYaw));
        }
        return true;
    }

    PoseEstimate self_;
    PoseEstimate teammate_;
    TimedPoint ball_;
    BallPositionSource latestBallSource_ = BallPositionSource::Unknown;
    std::vector<RobotCandidate> robots_;
    unsigned nextTrackId_ = 1;
    double robotsObservedAt_ = -1.0;
};

}  // namespace cupcup
