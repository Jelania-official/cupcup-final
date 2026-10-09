#pragma once

#include "world_model.hpp"
#include "navigation.hpp"
#include <vector>

namespace cupcup {

// The same input/intent boundary is used by the ROS adapter and 2D adapter.
// Coordinates are beliefs. Simulator truth is deliberately not a member.
struct ObservedObstacle {
    TimedPoint position;
    double radius = 0.20;
    double uncertainty = 0.50;
    RobotTeam team = RobotTeam::Unknown;
    bool fromTeammate = false;
};

struct MatchState {
    TeamColor color = TeamColor::Red;
    int id = 1;
    double now = 0.0;
    bool healthy = false;
    bool localBallVisible = false;
    WorldModel map;
    TeamStatus teammate;
    double teammateMessageAge = 99.0;
    std::vector<ObservedObstacle> obstacles;
};

struct PolicyConfig {
    double supportX = 0.40;
    double defenderHomeX = 1.55;
    double defenderHomeZ = 0.0;
    double defenderAnchorGain = 0.45;
    double boundaryMargin = 0.75;
    double claimStaleAfter = 1.20;
    double takeoverAfter = 2.50;
    double kickTravel = 1.50;  // Nominal short kick, not a commanded physical distance.
    bool defenderClearEnabled = true;
    double kickDirectionHysteresis = 0.25;
    bool defenderLineAnchor = true;
};

// One committed direction, not another behavior state machine. Re-evaluate
// every frame, but require a meaningful improvement before changing a plan.
struct PolicyMemory {
    bool valid = false;
    TeamColor color = TeamColor::Red;
    int id = 0;
    double lastPlanAt = -99.0;
    FieldPoint ballAtChoice, direction;
    void reset() { valid = false; }
};

struct MatchIntent {
    TacticalDecision tactical;
    FieldPoint target;
    FieldPoint kickTarget;
    bool targetValid = false;
    bool kickTargetValid = false;
    const char *reason = "unhealthy";
};

inline bool usableObstacle(const ObservedObstacle &o, double now) {
    return o.position.fresh(now, 0.65) && std::isfinite(o.position.x + o.position.z +
        o.position.confidence + o.radius + o.uncertainty) && o.position.confidence > 0.0 &&
        o.position.confidence <= 1.0 && o.radius >= 0.0 && o.uncertainty >= 0.0;
}

// Only the measured front-number geometry has controlled relative-position
// evidence on this platform. Preserve its low trust: these candidates may bias
// kick scoring, never become confirmed opponents or hard navigation discs.
inline std::vector<ObservedObstacle> mapRobotObstacles(const WorldModel &map, double now) {
    std::vector<ObservedObstacle> result;
    for (const auto &robot : map.robots()) {
        if (robot.latestSource != RobotPositionSource::NumberSquare) continue;
        ObservedObstacle obstacle{robot.position, 0.20, robot.uncertainty};
        obstacle.team = robot.team;
        obstacle.position.confidence = std::min(0.25, obstacle.position.confidence);
        if (usableObstacle(obstacle, now)) result.push_back(obstacle);
    }
    return result;
}

// One-hop soft sharing, not track fusion. Prefer local geometry when centres
// overlap within 20 cm; do not average coordinates or increase confidence.
// Peer observations never enter WorldModel::robots(), hence cannot be relayed.
inline void appendTeammateObstacles(MatchState &state) {
    const auto &peer = state.teammate;
    const double elapsed = state.teammateMessageAge;
    if (!peer.valid || !peer.healthy || peer.id != 3 - state.id ||
        !std::isfinite(elapsed) || elapsed < 0.0 || elapsed > 0.65) return;
    const auto enemy = state.color == TeamColor::Red ? RobotTeam::Blue : RobotTeam::Red;
    for (const auto &robot : peer.robots) {
        if (robot.team != robotTeamName(enemy) || robot.source != "number_square" ||
            !std::isfinite(robot.x) || !std::isfinite(robot.z) ||
            !std::isfinite(robot.age) || robot.age < 0.0 ||
            std::abs(robot.x) > 6.0 || std::abs(robot.z) > 4.0 ||
            !std::isfinite(robot.confidence) || robot.confidence <= 0.0 || robot.confidence > 1.0 ||
            !std::isfinite(robot.uncertainty) || robot.uncertainty < 0.0) continue;
        ObservedObstacle candidate{{true, robot.x, robot.z, std::min(.25, robot.confidence),
            state.now - elapsed - robot.age}, .20, robot.uncertainty, enemy, true};
        if (!usableObstacle(candidate, state.now)) continue;
        const bool duplicate = std::any_of(state.obstacles.begin(), state.obstacles.end(),
            [&](const ObservedObstacle &local) {
                return usableObstacle(local, state.now) &&
                    (local.team == enemy || local.team == RobotTeam::Unknown) &&
                    std::hypot(local.position.x - robot.x, local.position.z - robot.z) <= .20;
            });
        if (!duplicate) state.obstacles.push_back(candidate);
    }
}

inline double segmentDistance(FieldPoint p, FieldPoint a, FieldPoint b)
{
    return pointSegmentDistance(p, a, b);
}

inline FieldPoint kickStagingPoint(FieldPoint ball, FieldPoint target) {
    const double range = std::max(0.01, pointDistance(ball, target));
    return {ball.x - 0.22 * (target.x - ball.x) / range,
            ball.z - 0.22 * (target.z - ball.z) / range};
}

inline MatchIntent planMatch(const MatchState &state, const PolicyConfig &config = {},
                             PolicyMemory *memory = nullptr)
{
    MatchIntent out;
    out.kickTarget = {FieldGeometry::attackGoalX(state.color), 0.0};
    const auto &self = state.map.self();
    const auto &ball = state.map.ball();
    if (!state.healthy || !self.fresh(state.now, 2.0)) {
        if (memory) memory->reset();
        return out;
    }
    const bool knownBall = ball.fresh(state.now, 0.65);
    TacticalInput input;
    input.color = state.color;
    input.id = state.id;
    input.selfHealthy = state.healthy;
    input.selfLocationFresh = true;
    input.selfBall = knownBall && state.localBallVisible &&
        (state.id == 1 || config.defenderClearEnabled);
    input.selfBallX = ball.x;
    input.selfBallZ = ball.z;
    input.selfBallScore = ball.confidence;
    input.selfBallAge = knownBall ? state.now - ball.observedAt : 99.0;
    input.selfBallDistance = knownBall ? std::hypot(ball.x - self.x, ball.z - self.z) : 99.0;
    input.teammate = state.teammate;
    input.teammateMessageAge = state.teammateMessageAge;
    input.boundaryMargin = config.boundaryMargin;
    out.tactical = decideTactics(input, config.claimStaleAfter, config.takeoverAfter);
    const double sign = FieldGeometry::ownSign(state.color);
    const FieldPoint ballPoint{ball.x, ball.z};
    const bool mayCommit = knownBall && state.localBallVisible &&
        isBallAction(out.tactical.action) && pointDistance({self.x, self.z}, ballPoint) <= 1.0;
    if (memory && (!mayCommit || memory->id != state.id || memory->color != state.color ||
        state.now < memory->lastPlanAt || state.now - memory->lastPlanAt > 0.65 ||
        pointDistance(memory->ballAtChoice, ballPoint) > 0.35)) memory->reset();
    out.kickTargetValid = knownBall;
    if (knownBall) {
        const FieldPoint goal{FieldGeometry::attackGoalX(state.color), 0.0};
        const double range = std::max(0.01, pointDistance(goal, ballPoint));
        const double dx = (goal.x - ball.x) / range, dz = (goal.z - ball.z) / range;
        const double travel = std::max(0.5, std::min(2.0, config.kickTravel));
        auto pointFor = [&](FieldPoint direction, double degrees) {
            FieldPoint candidate{ball.x + travel * direction.x, ball.z + travel * direction.z};
            candidate.z = std::max(-2.65, std::min(2.65, candidate.z));
            if (range <= travel && std::abs(degrees) < 1.0) candidate = goal;
            else candidate.x = std::max(-4.35, std::min(4.35, candidate.x));
            return candidate;
        };
        auto scoreFor = [&](FieldPoint candidate, double degrees) {
            double score = range - pointDistance(goal, candidate) - 0.08 * std::abs(degrees) / 90.0;
            const FieldPoint staging = kickStagingPoint(ballPoint, candidate);
            for (const auto &obstacle : state.obstacles) {
                if (!usableObstacle(obstacle, state.now)) continue;
                // A player behind the ball does not block its flight, but may
                // occupy the pose needed to execute that kick. The same trust
                // gate as navigation excludes unvalidated map candidates;
                // even trusted observations receive a soft cost, not a veto.
                if (obstacle.position.confidence >= 0.45) {
                    const double poseOverlap = std::max(0.0, obstacle.radius + 0.18 -
                        pointDistance(staging, {obstacle.position.x, obstacle.position.z}));
                    score -= 5.0 * poseOverlap * obstacle.position.confidence;
                }
                const double ahead = (obstacle.position.x - ball.x) * (candidate.x - ball.x) +
                    (obstacle.position.z - ball.z) * (candidate.z - ball.z);
                if (ahead <= 0.0) continue;  // A player behind the kick is not blocking its flight.
                const double clearance = segmentDistance(
                    {obstacle.position.x, obstacle.position.z}, ballPoint, candidate) -
                    obstacle.radius - 0.07 - std::min(0.20, std::max(0.0, obstacle.uncertainty) * 0.25);
                // The old behind/ahead step gave a full safety penalty when a
                // lateral observation moved just millimetres ahead of the ball.
                // Fade the soft buffer continuously; actual overlap stays costly.
                const double front = std::min(1.0, ahead / std::max(0.01,
                    pointDistance(ballPoint, candidate) * pointDistance(ballPoint,
                        {obstacle.position.x, obstacle.position.z})));
                const double risk = std::max(0.0, -clearance) +
                    std::max(0.0, 0.40 - std::max(0.0, clearance)) * front;
                score -= 5.0 * risk * obstacle.position.confidence;
            }
            return score;
        };
        double best = -1e9;
        FieldPoint direction;
        for (double degrees : {0.0, -30.0, 30.0, -60.0, 60.0, -90.0, 90.0}) {
            const double angle = degrees * 3.14159265358979323846 / 180.0;
            const FieldPoint choice{dx * std::cos(angle) - dz * std::sin(angle),
                                    dx * std::sin(angle) + dz * std::cos(angle)};
            const FieldPoint candidate = pointFor(choice, degrees);
            const double score = scoreFor(candidate, degrees);
            // Symmetric left/right candidates can have exactly equal geometric
            // utility; numerical roundoff must not choose opposite lanes on swap.
            if (score > best + 1e-8) { best = score; out.kickTarget = candidate; direction = choice; }
        }
        bool retained = false;
        if (memory && memory->valid && mayCommit) {
            const auto &previous = memory->direction;
            const double degrees = std::atan2(dx * previous.z - dz * previous.x,
                dx * previous.x + dz * previous.z) * 180.0 / 3.14159265358979323846;
            const FieldPoint candidate = pointFor(previous, degrees);
            if (scoreFor(candidate, degrees) + config.kickDirectionHysteresis + 1e-8 >= best) {
                out.kickTarget = candidate;
                direction = previous;
                retained = true;
            }
        }
        if (memory && mayCommit) {
            if (!retained) memory->ballAtChoice = ballPoint;
            memory->valid = true;
            memory->id = state.id; memory->color = state.color;
            memory->lastPlanAt = state.now; memory->direction = direction;
        }
    }
    switch (out.tactical.action) {
    case TacticalAction::Chase:
    case TacticalAction::Clear: {
        if (!knownBall) { out.reason = "search-no-ball"; return out; }
        out.target = kickStagingPoint(ballPoint, out.kickTarget);
        out.reason = out.tactical.action == TacticalAction::Clear ? "own-half-clear" : "ball-owner";
        break;
    }
    case TacticalAction::Support:
        out.target = {-sign * config.supportX,
            knownBall ? std::max(-1.4, std::min(1.4, ball.z * 0.35)) : 0.0};
        out.reason = knownBall && !ballWithinRoleReach(state.color, state.id, ballPoint,
            config.boundaryMargin) ? "ball-outside-role-reach" : "yield-to-teammate";
        break;
    case TacticalAction::Defend:
        out.target = {sign * config.defenderHomeX,
            knownBall ? std::max(-1.45, std::min(1.45,
                config.defenderHomeZ + config.defenderAnchorGain * (ball.z - config.defenderHomeZ))) :
                config.defenderHomeZ};
        out.reason = "protect-own-half";
        if (config.defenderLineAnchor && knownBall && sign * ball.x > 0.0) {
            // Protect the goal side of a threatening ball, not a fixed point
            // that may already be behind the opponent's advance.
            const double ownBallX = sign * ball.x;
            const double ownAnchorX = std::min(FieldGeometry::halfLength - config.boundaryMargin,
                std::max(config.defenderHomeX, ownBallX + 0.60));
            const double fraction = std::max(0.0, std::min(1.0,
                (FieldGeometry::halfLength - ownAnchorX) /
                std::max(0.01, FieldGeometry::halfLength - ownBallX)));
            out.target = {sign * ownAnchorX, std::max(-1.45, std::min(1.45, ball.z * fraction))};
            out.reason = "cover-ball-goal-line";
        }
        break;
    default: return out;
    }
    if (needsBodySearch(out.tactical.action, knownBall)) out.reason = "search-no-map";
    out.target = FieldGeometry::legalTarget(state.color, state.id, out.target, config.boundaryMargin);
    // Global disc routing is for travel, not a substitute for the image-based
    // ball approach/orbit controller. Contact is legal; an uncertain near-ball
    // obstacle must not turn every kick preparation into an unreachable goal.
    if (knownBall && (out.tactical.action == TacticalAction::Chase ||
        out.tactical.action == TacticalAction::Clear) &&
        pointDistance({self.x, self.z}, ballPoint) <= 1.0) {
        out.targetValid = true;
        return out;
    }
    std::vector<NavigationObstacle> obstacles;
    for (const auto &o : state.obstacles) {
        if (!usableObstacle(o, state.now) || o.position.confidence < 0.45) continue;
        obstacles.push_back({{o.position.x, o.position.z}, o.radius + 0.18 +
            0.10 * std::max(0.0, std::min(0.5, o.uncertainty))});
    }
    const auto route = planNavigation(state.color, state.id, {self.x, self.z}, out.target,
                                     obstacles, config.boundaryMargin);
    out.target = route.waypoint;
    if (!route.reachable) out.reason = "no-clear-route";
    out.targetValid = true;
    return out;
}

}  // namespace cupcup
