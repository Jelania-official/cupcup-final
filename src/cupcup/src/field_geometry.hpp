#pragma once

#include <algorithm>
#include <cmath>

namespace cupcup {

enum class TeamColor { Red, Blue };
enum class PlayerRole { Forward, Defender };
struct FieldPoint { double x = 0.0; double z = 0.0; };

// Field z points opposite to positive body yaw on the supplied platform.
inline double fieldHeading(FieldPoint origin, FieldPoint target) {
    return std::atan2(origin.z - target.z, target.x - origin.x) *
        180.0 / 3.14159265358979323846;
}

// From the release ZIP, not the unused supervisor forbidArea constants.
// Documented restrictions are conservative supersets of referee tolerances.
struct FieldGeometry {
    static constexpr double halfLength = 4.5;
    static constexpr double halfWidth = 3.0;
    static constexpr double goalHalfWidth = 1.3;
    static constexpr double penaltyFront = 2.5;
    static constexpr double penaltyHalfWidth = 2.5;

    static double ownSign(TeamColor color) { return color == TeamColor::Red ? 1.0 : -1.0; }
    static double ownGoalX(TeamColor color) { return ownSign(color) * halfLength; }
    static double attackGoalX(TeamColor color) { return -ownGoalX(color); }

    static bool insideField(const FieldPoint &point, double margin = 0.0)
    {
        return std::isfinite(point.x) && std::isfinite(point.z) &&
            std::abs(point.x) <= halfLength - std::max(0.0, margin) &&
            std::abs(point.z) <= halfWidth - std::max(0.0, margin);
    }

    static bool insideGoalMouth(double z, double margin = 0.0)
    {
        return std::isfinite(z) && std::abs(z) <= goalHalfWidth - std::max(0.0, margin);
    }

    // Signed penetration: positive means unsafe, negative means clearance.
    static double risk(TeamColor color, int id, FieldPoint p, double margin = 0.0)
    {
        const double field = std::max(std::abs(p.x) - halfLength + margin,
                                      std::abs(p.z) - halfWidth + margin);
        const double ownX = ownSign(color) * p.x;
        const double role = id == 1 ?
            std::min(ownX - penaltyFront + margin, penaltyHalfWidth + margin - std::abs(p.z)) :
            margin - ownX;
        return std::max(field, role);
    }

    static bool refereeViolation(TeamColor color, int id, FieldPoint p)
    {
        const double ownX = ownSign(color) * p.x;
        return std::abs(p.x) > 4.75 || std::abs(p.z) > 3.25 ||
            (id == 1 ? (ownX > 2.7 && std::abs(p.z) < 2.0) : ownX < -0.2);
    }

    static FieldPoint legalTarget(TeamColor color, int id, FieldPoint p, double margin = 0.20)
    {
        p.x = std::max(-halfLength + margin, std::min(halfLength - margin, p.x));
        p.z = std::max(-halfWidth + margin, std::min(halfWidth - margin, p.z));
        const double sign = ownSign(color);
        if (id == 2 && sign * p.x < margin) p.x = sign * margin;
        if (id == 1 && sign * p.x >= penaltyFront - margin &&
            std::abs(p.z) <= penaltyHalfWidth + margin)
            p.x = sign * (penaltyFront - margin - 0.01);
        return p;
    }

    static FieldPoint localDisplacement(double yawDegrees, double forward, double left)
    {
        const double yaw = yawDegrees * 3.14159265358979323846 / 180.0;
        return {forward * std::cos(yaw) - left * std::sin(yaw),
                -forward * std::sin(yaw) - left * std::cos(yaw)};
    }

    static bool safeMotion(TeamColor color, int id, FieldPoint p, double yaw,
                           double forward, double left, double margin)
    {
        if (!std::isfinite(p.x + p.z + yaw + forward + left)) return false;
        const FieldPoint delta = localDisplacement(yaw, forward, left);
        const double before = risk(color, id, p, margin);
        const double after = risk(color, id, {p.x + delta.x, p.z + delta.z}, margin);
        // Kickoff/penalty relocation may already be outside buffered bounds.
        // Permit recovery, but never deepen a violation or enter from safety.
        return after <= 0.0 || after < before - 1e-6 ||
            (std::abs(forward) + std::abs(left) < 1e-9);
    }
};

}  // namespace cupcup
