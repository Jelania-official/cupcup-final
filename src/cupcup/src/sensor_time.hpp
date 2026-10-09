#pragma once

#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>

namespace cupcup {

inline bool sameHeadTarget(double yaw, double pitch, double otherYaw, double otherPitch) {
    return std::isfinite(yaw + pitch + otherYaw + otherPitch) &&
        std::abs(yaw - otherYaw) <= 1.0 && std::abs(pitch - otherPitch) <= 1.0;
}

// Receipt-clock dwell is a conservative target-stability guard, not measured
// neck feedback or an exposure timestamp. Compare against an anchor so a series
// of small target changes cannot masquerade as a stationary head.
class HeadTargetStability {
public:
    void reset() { since_ = -1.0; }
    void observe(double yaw, double pitch, double time) {
        if (!std::isfinite(yaw + pitch + time)) { reset(); return; }
        if (since_ < 0 || time < since_ || std::abs(yaw - yaw_) > 1.0 ||
            std::abs(pitch - pitch_) > 1.0) {
            yaw_ = yaw; pitch_ = pitch; since_ = time;
        }
    }
    double age(double time) const {
        return since_ >= 0 && std::isfinite(time) && time >= since_ ? time - since_ : 0.0;
    }
private:
    double yaw_ = 0, pitch_ = 0, since_ = -1;
};

inline bool usableReceiptPose(double headAge, double imuAge) {
    return std::isfinite(headAge) && std::isfinite(imuAge) &&
        headAge >= 0 && headAge <= .25 && imuAge >= 0 && imuAge <= .25;
}

inline bool usableBallReceiptGeometry(double headAge, double imuAge, double targetDwell) {
    return usableReceiptPose(headAge, imuAge) && std::isfinite(targetDwell) && targetDwell >= .8;
}

// The official Location has only x/z. Optional diagnostics must compile
// against that interface without requiring an experimental common rebuild.
template<typename T>
auto optionalStamp(const T &message, int = 0) -> decltype(message.stamp)
{
    return message.stamp;
}

inline uint32_t optionalStamp(...) { return 0U; }

inline bool imageTimeMilliseconds(int32_t seconds, uint32_t nanoseconds,
                                 uint32_t &milliseconds)
{
    if (seconds < 0 || nanoseconds >= 1000000000U) return false;
    const uint64_t total = static_cast<uint64_t>(seconds) * 1000U +
        nanoseconds / 1000000U;
    if (total == 0U || total > std::numeric_limits<uint32_t>::max()) return false;
    milliseconds = static_cast<uint32_t>(total);
    return true;
}

// Return the signed age (reference - sample) across uint32 millisecond wrap.
inline double timestampAgeSeconds(uint32_t reference, uint32_t sample)
{
    const uint32_t modular = reference - sample;
    const int64_t signedMilliseconds = modular <= 0x7fffffffU ?
        static_cast<int64_t>(modular) :
        static_cast<int64_t>(modular) - 0x100000000LL;
    return static_cast<double>(signedMilliseconds) / 1000.0;
}

// Small capture-time lookup; no extrapolation and no unbounded message queue.
// Selection happens when the frame is processed, so same-stamp head/IMU
// callbacks arriving just after the image callback can still be used.
template<typename T>
class SensorHistory
{
public:
    void push(uint32_t stamp, const T &value)
    {
        if (stamp == 0U) return;
        if (!samples_.empty()) {
            const double delta = timestampAgeSeconds(stamp, samples_.back().stamp);
            if (delta < -1.0) samples_.clear();  // simulation clock restarted
            else if (delta < 0.0) return;       // late/out-of-order callback
        }
        samples_.push_back({stamp, value});
        if (samples_.size() > 32U) samples_.pop_front();
    }

    bool nearest(uint32_t reference, T &value, double &age,
                 double maxAge = 0.04) const
    {
        if (reference == 0U) return false;
        bool found = false;
        double best = maxAge + 1e-9;
        for (const auto &sample : samples_) {
            const double candidateAge = timestampAgeSeconds(reference, sample.stamp);
            if (std::abs(candidateAge) < best) {
                value = sample.value;
                age = candidateAge;
                best = std::abs(candidateAge);
                found = true;
            }
        }
        return found;
    }

private:
    struct Sample { uint32_t stamp; T value; };
    std::deque<Sample> samples_;
};

}  // namespace cupcup
