#pragma once

#include "field_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace cupcup {

// Non-ball-owner view: retain the ball below centre so robot upper bodies can
// remain visible above it. A deadband lets the existing head-stability gate
// settle; this is receipt-target feedback, not measured neck control.
inline double defenderTrackingPitch(double pitch, double ballY) {
    if (!std::isfinite(pitch + ballY) || ballY < 0.0 || ballY > 1.0) return pitch;
    if (ballY >= .55 && ballY <= .69) return pitch;
    return std::max(8.0, std::min(60.0,
        pitch + std::max(-1.8, std::min(1.8, (ballY - .62) * 4.0))));
}

// Count observations, not timer ticks. Two outliers tolerate image jitter;
// persistent misalignment must return to adjustment, even inside the hold window.
inline bool updateKickAlignment(bool freshImage, bool linedUp, bool holdPose,
                                int &stableFrames, int &missedFrames)
{
    if (!holdPose) {
        stableFrames = missedFrames = 0;
        return false;
    }
    if (freshImage) {
        if (linedUp) {
            stableFrames = std::min(stableFrames + 1, 20);
            missedFrames = 0;
        } else {
            missedFrames = std::min(missedFrames + 1, 3);
            if (missedFrames >= 3) stableFrames = 0;
        }
    }
    return missedFrames < 3;
}

// Original images are sampled at 10 Hz but have zero capture timestamps.
// Require actual distinct stable observations as well as a wall-time minimum:
// fast wall timers alone do not let an outstanding gait/stop queue finish.
// Callers count fresh images, never control-loop ticks. This is not an ACK.
inline bool settledForKick(double stoppedWallSeconds, int stableFrames, int requiredFrames = 16)
{
    return std::isfinite(stoppedWallSeconds) && stoppedWallSeconds > 0.8 &&
        requiredFrames >= 3 && requiredFrames <= 20 && stableFrames >= requiredFrames;
}

// These helpers contain the parts of the strategy that must remain stable when
// the ROS/Webots adapter or the detector is changed.  They deliberately have
// no ROS dependency so they can be replayed from logs in a unit test.

// Direct observations only. Track IDs are local and are not player identities.
struct PeerRobotObservation {
    double x = 0.0, z = 0.0, confidence = 0.0, uncertainty = 1.0, age = 99.0;
    std::string team, source;
};

struct TeamStatus {
    bool valid = false;
    int id = 0;
    PlayerRole role = PlayerRole::Forward;
    bool ball = false;
    bool active = false;
    bool kick = false;
    bool claim = false;
    bool healthy = true;
    double ballAge = 99.0;
    double ballMapAge = 99.0;
    double ballScore = 0.0;
    double ballDistance = 99.0;
    double ballX = 0.0;
    double ballZ = 0.0;
    bool hasBallPosition = false;
    bool hasPose = false;
    double poseX = 0.0;
    double poseZ = 0.0;
    double poseYaw = 0.0;
    double poseAge = 99.0;
    std::string state;
    std::vector<PeerRobotObservation> robots;
};

inline bool parseBool(const std::string &value)
{
    return value == "1" || value == "true" || value == "TRUE";
}

inline TeamStatus parseTeamStatus(const std::string &wire)
{
    TeamStatus status;
    if (wire.compare(0, 7, "cupcup|") != 0) return status;
    std::map<std::string, std::string> fields;
    std::stringstream stream(wire);
    std::string field;
    while (std::getline(stream, field, '|')) {
        const std::size_t equals = field.find('=');
        if (equals != std::string::npos) {
            fields[field.substr(0, equals)] = field.substr(equals + 1);
        }
    }
    const auto id = fields.find("id");
    const auto role = fields.find("role");
    if (id == fields.end() || role == fields.end()) return status;
    char *end = nullptr;
    const long parsedId = std::strtol(id->second.c_str(), &end, 10);
    if (end == id->second.c_str() || *end != '\0' || parsedId < 1 || parsedId > 2) return status;
    status.id = static_cast<int>(parsedId);
    status.role = role->second == "defender" ? PlayerRole::Defender : PlayerRole::Forward;
    const auto ball = fields.find("ball");
    const auto active = fields.find("active");
    const auto kick = fields.find("kick");
    const auto claim = fields.find("claim");
    const auto healthy = fields.find("healthy");
    const auto age = fields.find("age");
    const auto ballMapAge = fields.find("bmap_age");
    const auto score = fields.find("conf");
    const auto distance = fields.find("bdist");
    const auto ballX = fields.find("bx");
    const auto ballZ = fields.find("bz");
    const auto poseX = fields.find("px");
    const auto poseZ = fields.find("pz");
    const auto poseYaw = fields.find("pyaw");
    const auto poseAge = fields.find("pose_age");
    status.ball = ball != fields.end() && parseBool(ball->second);
    status.active = active != fields.end() && parseBool(active->second);
    status.kick = kick != fields.end() && parseBool(kick->second);
    status.claim = claim != fields.end() && parseBool(claim->second);
    status.healthy = healthy == fields.end() || parseBool(healthy->second);
    if (age != fields.end()) {
        char *ageEnd = nullptr;
        const double parsedAge = std::strtod(age->second.c_str(), &ageEnd);
        if (ageEnd != age->second.c_str() && std::isfinite(parsedAge)) {
            status.ballAge = std::max(0.0, parsedAge);
        }
    }
    auto parseNumber = [](const std::string &value, double fallback) {
        char *numberEnd = nullptr;
        const double parsed = std::strtod(value.c_str(), &numberEnd);
        return numberEnd != value.c_str() && std::isfinite(parsed) ? parsed : fallback;
    };
    status.ballMapAge = ballMapAge == fields.end() ? status.ballAge :
        std::max(0.0, parseNumber(ballMapAge->second, 99.0));
    if (score != fields.end()) status.ballScore = std::max(0.0, parseNumber(score->second, 0.0));
    if (distance != fields.end()) status.ballDistance = std::max(0.0, parseNumber(distance->second, 99.0));
    if (ballX != fields.end()) status.ballX = parseNumber(ballX->second, 1000.0);
    if (ballZ != fields.end()) status.ballZ = parseNumber(ballZ->second, 1000.0);
    status.hasBallPosition = ballX != fields.end() && ballZ != fields.end() &&
        std::abs(status.ballX) <= 6.0 && std::abs(status.ballZ) <= 4.0;
    if (poseX != fields.end() && poseZ != fields.end() && poseYaw != fields.end()) {
        status.poseX = parseNumber(poseX->second, 1000.0);
        status.poseZ = parseNumber(poseZ->second, 1000.0);
        status.poseYaw = parseNumber(poseYaw->second, 1000.0);
        status.hasPose = std::abs(status.poseX) <= 6.0 && std::abs(status.poseZ) <= 4.0 &&
            std::abs(status.poseYaw) <= 360.0;
    }
    if (poseAge != fields.end()) {
        status.poseAge = std::max(0.0, parseNumber(poseAge->second, 99.0));
    }
    const auto state = fields.find("state");
    if (state != fields.end()) status.state = state->second;
    // Sharing is stricter than legacy status parsing: missing health, invalid
    // numbers, negative age and partial numeric strings never make obstacles.
    auto strictNumber = [&](const std::string &key, double &value) {
        const auto found = fields.find(key);
        if (found == fields.end()) return false;
        char *tail = nullptr;
        value = std::strtod(found->second.c_str(), &tail);
        return tail != found->second.c_str() && *tail == '\0' && std::isfinite(value);
    };
    double count = 0.0;
    if (healthy != fields.end() && parseBool(healthy->second) &&
        strictNumber("robots", count) && count >= 0.0 && count <= 4.0 && std::floor(count) == count) {
        for (int i = 0; i < static_cast<int>(count); ++i) {
            const auto prefix = "r" + std::to_string(i);
            PeerRobotObservation robot;
            if (!strictNumber(prefix + "x", robot.x) || !strictNumber(prefix + "z", robot.z) ||
                !strictNumber(prefix + "conf", robot.confidence) ||
                !strictNumber(prefix + "unc", robot.uncertainty) ||
                !strictNumber(prefix + "age", robot.age) ||
                std::abs(robot.x) > 6.0 || std::abs(robot.z) > 4.0 ||
                robot.confidence <= 0.0 || robot.confidence > 1.0 ||
                robot.uncertainty < 0.0 || robot.age < 0.0 || robot.age > 0.65) continue;
            robot.team = fields[prefix + "team"];
            robot.source = fields[prefix + "source"];
            if ((robot.team == "red" || robot.team == "blue") && robot.source == "number_square")
                status.robots.push_back(robot);
        }
    }
    status.valid = true;
    return status;
}

enum class TacticalAction { Hold, Chase, Support, Defend, Clear };
inline bool isBallAction(TacticalAction action) {
    return action == TacticalAction::Chase || action == TacticalAction::Clear;
}
inline bool needsBodySearch(TacticalAction action, bool freshMappedBall) {
    return !freshMappedBall &&
        (action == TacticalAction::Defend || action == TacticalAction::Support);
}

struct TacticalInput {
    TeamColor color = TeamColor::Red;
    int id = 1;
    bool selfHealthy = false;
    bool selfBall = false;
    bool selfLocationFresh = false;
    double selfBallScore = 0.0;
    double selfBallDistance = 99.0;
    double selfBallAge = 99.0;
    double selfBallX = 0.0;
    double selfBallZ = 0.0;
    TeamStatus teammate;
    double teammateMessageAge = 99.0;
    double boundaryMargin = 0.75;
};

struct TacticalDecision {
    TacticalAction action = TacticalAction::Hold;
    bool claim = false;
    bool takeover = false;
    bool ownHalfBall = false;
    double selfUtility = -100.0;
    double teammateUtility = -100.0;
};

inline bool ballInOwnHalf(TeamColor color, double x)
{
    return color == TeamColor::Red ? x >= 0.0 : x <= 0.0;
}

// Claim only balls that can be touched from a legal robot-centre position.
// Number restrictions remain fixed even when the team changes ball ownership.
inline bool ballWithinRoleReach(TeamColor color, int id, FieldPoint ball,
                               double margin, double reach = 0.32)
{
    if ((id != 1 && id != 2) || !std::isfinite(ball.x + ball.z + margin + reach) ||
        margin < 0.0 || reach < 0.0)
        return false;
    const auto target = FieldGeometry::legalTarget(color, id, ball, margin);
    double nearest = FieldGeometry::risk(color, id, target, margin) <= 0.0 ?
        std::hypot(target.x - ball.x, target.z - ball.z) : 99.0;
    if (id == 1) for (const double side : {-1.0, 1.0}) {
        const FieldPoint corner{target.x, side * (FieldGeometry::penaltyHalfWidth + margin + 0.01)};
        const FieldPoint edge{std::max(-FieldGeometry::halfLength + margin,
            std::min(FieldGeometry::halfLength - margin, ball.x)), corner.z};
        if (FieldGeometry::risk(color, id, edge, margin) <= 0.0)
            nearest = std::min(nearest, std::hypot(edge.x - ball.x, edge.z - ball.z));
    }
    return nearest <= reach;
}

inline TacticalDecision decideTactics(const TacticalInput &input,
    double staleAfter = 1.20, double takeoverAfter = 2.50)
{
    TacticalDecision result;
    if (!input.selfHealthy) return result;
    const bool selfReachable = input.selfBall && ballWithinRoleReach(input.color, input.id,
        {input.selfBallX, input.selfBallZ}, input.boundaryMargin);
    const bool teammateFresh = input.teammate.valid && input.teammateMessageAge >= 0.0 &&
        input.teammateMessageAge <= staleAfter && input.teammate.healthy;
    const bool teammateReachable = !input.teammate.hasBallPosition ||
        (input.teammate.ballMapAge <= 0.65 && ballWithinRoleReach(input.color,
            input.teammate.id, {input.teammate.ballX, input.teammate.ballZ}, input.boundaryMargin));
    const bool teammateClaims = teammateFresh && teammateReachable && input.teammate.claim;
    result.ownHalfBall = input.selfBall && ballInOwnHalf(input.color, input.selfBallX);

    // The utility is intentionally simple and monotonic.  A closer, fresher,
    // more confident observation wins, while the defender has a small penalty
    // for leaving its home role.  This gives deterministic arbitration without
    // requiring a central referee or a new ROS message.
    result.selfUtility = selfReachable ?
        input.selfBallScore - 0.12 * input.selfBallDistance - 0.08 * input.selfBallAge : -100.0;
    result.teammateUtility = teammateFresh && teammateReachable && input.teammate.ball ?
        input.teammate.ballScore - 0.12 * input.teammate.ballDistance -
        0.08 * input.teammate.ballAge : -100.0;

    const double handoffMargin = 0.25;
    bool selfWins = result.selfUtility >= result.teammateUtility + handoffMargin;
    if (teammateClaims && input.selfBall && input.teammate.ball &&
        std::abs(result.selfUtility - result.teammateUtility) < handoffMargin) {
        // Simultaneous observations must converge even when the messages
        // arrive in different DDS cycles.  Lower robot id owns a tie; a
        // defender can still take over when it is clearly closer to a ball in
        // its own half.
        selfWins = input.id < input.teammate.id;
    }
    const bool teammateWins = teammateClaims && !selfWins;
    const bool teammateStale = !teammateFresh || input.teammateMessageAge > takeoverAfter;

    if (input.selfBall && !selfReachable) {
        result.action = input.id == 1 ? TacticalAction::Support : TacticalAction::Defend;
    } else if (input.id == 1) {
        if (input.selfBall && (!teammateWins || selfWins)) {
            result.action = TacticalAction::Chase;
            result.claim = true;
        } else if (teammateWins) {
            result.action = TacticalAction::Support;
        } else {
            result.action = TacticalAction::Chase;
        }
    } else {
        const bool safeToLeaveHome = result.ownHalfBall && input.selfBall &&
            (!teammateWins || teammateStale) && input.selfBallScore >= 0.30;
        if (safeToLeaveHome) {
            result.action = TacticalAction::Clear;
            result.claim = true;
            result.takeover = teammateStale || !teammateClaims;
        } else if (teammateWins) {
            result.action = TacticalAction::Defend;
        } else if (input.selfBall && result.ownHalfBall && selfWins) {
            result.action = TacticalAction::Clear;
            result.claim = true;
        } else {
            result.action = TacticalAction::Defend;
        }
    }
    return result;
}

struct ApproachMotion {
    double forward = 0.0;
    double turn = 0.0;
};

struct KickPoseControl {
    bool valid = false, linedUp = false, holdPose = false;
    double forward = 0.0, lateral = 0.0;
};

// Body-relative ball targets from the released-platform controlled kick trials:
// left (.18, .08), right (.18, -.04) metres. Do not mirror asymmetric feet.
inline KickPoseControl metricKickPose(double fieldX, double fieldZ, double yaw, bool leftFoot)
{
    KickPoseControl result;
    if (!std::isfinite(fieldX + fieldZ + yaw) || std::hypot(fieldX, fieldZ) > .6) return result;
    const double radians = yaw * 3.14159265358979323846 / 180.0;
    const double forward = std::cos(radians) * fieldX - std::sin(radians) * fieldZ;
    const double left = -std::sin(radians) * fieldX - std::cos(radians) * fieldZ;
    if (forward <= 0.0) return result;
    const double forwardError = forward - .18, leftError = left - (leftFoot ? .08 : -.04);
    result.valid = true;
    result.linedUp = std::abs(forwardError) < .025 && std::abs(leftError) < .020;
    result.holdPose = std::abs(forwardError) < .045 && std::abs(leftError) < .035;
    result.forward = std::max(-.018, std::min(.025, forwardError * .5));
    result.lateral = std::max(-.028, std::min(.028, leftError * .5));
    return result;
}

inline ApproachMotion searchMotion(double scanAge, double unseenAge, bool locationFresh)
{
    // First cover both head elevations without moving. A stationary head sweep
    // cannot cover the rear hemisphere; thereafter use the same small crawl
    // and turn budget as a confirmed side-ball approach, not a pure yaw task.
    if (!locationFresh || !std::isfinite(scanAge + unseenAge) ||
        scanAge < 6.0 || unseenAge < 6.0) return {};
    return {0.008, 4.0};
}

inline ApproachMotion approachMotion(double bodyBearing, double radius)
{
    ApproachMotion motion;
    if (!std::isfinite(bodyBearing) || !std::isfinite(radius) || radius <= 0.0)
        return motion;
    // Body bearing includes the head angle: a centered image can still require
    // a large body turn. Do not reset visual search for a confirmed side ball.
    const double speed = radius < 0.025 ? 0.05 : (radius < 0.045 ? 0.04 : 0.032);
    motion.forward = std::abs(bodyBearing) > 24.0 ? 0.008 : speed;
    motion.turn = std::max(-4.0, std::min(4.0, bodyBearing * 0.18));
    return motion;
}

// Orbit establishes a stable, usable near-ball view. ALIGN then corrects the
// exact calibrated pixel target; the coarse image-height gate prevents handing
// off while the ball is still at the edge of the current camera view.
inline bool readyForAlignment(double headingError, double ballBearing,
                              double headYaw, double headPitch,
                              double kickPitch, double ballRadius, double ballY)
{
    return std::isfinite(headingError) && std::isfinite(ballBearing) &&
        std::isfinite(headYaw) && std::isfinite(headPitch) &&
        std::isfinite(kickPitch) && std::isfinite(ballRadius) && std::isfinite(ballY) &&
        std::abs(headingError) < 15.0 && std::abs(ballBearing) < 22.0 &&
        std::abs(headYaw) < 8.0 && headPitch > kickPitch - 8.0 &&
        ballRadius > 0.050 && ballRadius < 0.12 && ballY > 0.30;
}

inline double headRecenteringCommand(double currentYaw, double ballX,
                                     double ballRadius, bool orbiting)
{
    if (!orbiting || !std::isfinite(currentYaw) || !std::isfinite(ballX) ||
        !std::isfinite(ballRadius) || std::abs(currentYaw) <= 6.0 ||
        std::abs(ballX - 0.5) >= 0.20 || ballRadius <= 0.045) {
        return currentYaw;
    }
    return currentYaw - std::max(-0.5, std::min(0.5, currentYaw * 0.12));
}

enum class LifecycleEvent { None, Reset, PlayStarted, Restarted };

class MatchLifecycle
{
public:
    LifecycleEvent observe(int state, int score, double time)
    {
        if (!initialized_) {
            initialized_ = true;
            previousState_ = state;
            previousScore_ = score;
            if (state == playState_) restartUntil_ = time + restartGrace_;
            return LifecycleEvent::Reset;
        }
        const bool stateChanged = state != previousState_;
        const bool scoreChanged = score != previousScore_;
        const bool started = state == playState_ && previousState_ != playState_;
        LifecycleEvent event = LifecycleEvent::None;
        if (stateChanged || scoreChanged) {
            event = scoreChanged && state == playState_ ? LifecycleEvent::Restarted : LifecycleEvent::Reset;
            if (started) event = LifecycleEvent::PlayStarted;
            if (state == playState_) restartUntil_ = time + restartGrace_;
        }
        previousState_ = state;
        previousScore_ = score;
        return event;
    }

    bool canPlay(int state, double time) const
    {
        return state == playState_ && time >= restartUntil_;
    }

    void reset(double time)
    {
        restartUntil_ = time + restartGrace_;
    }

    void setRestartGrace(double seconds)
    {
        restartGrace_ = std::max(0.0, seconds);
    }

private:
    int previousState_ = -1;
    int previousScore_ = 0;
    const int playState_ = 2;  // common::msg::GameData::STATE_PLAY
    double restartUntil_ = 0.0;
    double restartGrace_ = 1.50;
    bool initialized_ = false;
};

struct BoundaryDecision {
    bool stopWalking = false;
    bool forwardBoundary = false;
    bool defenderBoundary = false;
};

class BoundaryGuard
{
public:
    BoundaryDecision update(TeamColor color, PlayerRole role, double x, double z, double margin)
    {
        const double safeMargin = std::max(0.20, std::min(1.20, margin));
        const bool red = color == TeamColor::Red;
        if (role == PlayerRole::Forward) {
            // The document excludes the LARGE penalty area. Keep geometry
            // shared with the map and sandbox, not unused referee constants.
            const double ownGoalX = red ? x : -x;
            const double enterX = FieldGeometry::penaltyFront - safeMargin;
            const double enterZ = FieldGeometry::penaltyHalfWidth + safeMargin;
            const double exitX = enterX - 0.18;
            const double exitZ = enterZ + 0.18;
            const bool beyond = ownGoalX > enterX && std::abs(z) <= enterZ;
            const bool clear = ownGoalX < exitX || std::abs(z) > exitZ;
            if (!forwardBlocked_ && beyond) forwardBlocked_ = true;
            else if (forwardBlocked_ && clear) forwardBlocked_ = false;
        } else {
            const double centerEnter = safeMargin;
            const double centerExit = std::max(0.10, safeMargin - 0.18);
            const bool beyond = red ? x < centerEnter : x > -centerEnter;
            const bool clear = red ? x > centerExit : x < -centerExit;
            if (!defenderBlocked_ && beyond) defenderBlocked_ = true;
            else if (defenderBlocked_ && clear) defenderBlocked_ = false;
        }
        return {forwardBlocked_ || defenderBlocked_, forwardBlocked_, defenderBlocked_};
    }

    void reset()
    {
        forwardBlocked_ = false;
        defenderBlocked_ = false;
    }

    BoundaryDecision update(TeamColor color, PlayerRole role, double x, double margin)
    {
        return update(color, role, x, 0.0, margin);
    }

private:
    bool forwardBlocked_ = false;
    bool defenderBlocked_ = false;
};

}  // namespace cupcup
