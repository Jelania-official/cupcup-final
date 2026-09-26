#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>

namespace cupcup {

// These helpers contain the parts of the strategy that must remain stable when
// the ROS/Webots adapter or the detector is changed.  They deliberately have
// no ROS dependency so they can be replayed from logs in a unit test.
enum class TeamColor { Red, Blue };
enum class PlayerRole { Forward, Defender };

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
    double ballScore = 0.0;
    double ballDistance = 99.0;
    double ballX = 0.0;
    double ballZ = 0.0;
    std::string state;
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
    if (end == id->second.c_str() || parsedId < 1 || parsedId > 2) return status;
    status.id = static_cast<int>(parsedId);
    status.role = role->second == "defender" ? PlayerRole::Defender : PlayerRole::Forward;
    const auto ball = fields.find("ball");
    const auto active = fields.find("active");
    const auto kick = fields.find("kick");
    const auto claim = fields.find("claim");
    const auto healthy = fields.find("healthy");
    const auto age = fields.find("age");
    const auto score = fields.find("conf");
    const auto distance = fields.find("bdist");
    const auto ballX = fields.find("bx");
    const auto ballZ = fields.find("bz");
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
    if (score != fields.end()) status.ballScore = std::max(0.0, parseNumber(score->second, 0.0));
    if (distance != fields.end()) status.ballDistance = std::max(0.0, parseNumber(distance->second, 99.0));
    if (ballX != fields.end()) status.ballX = parseNumber(ballX->second, 0.0);
    if (ballZ != fields.end()) status.ballZ = parseNumber(ballZ->second, 0.0);
    const auto state = fields.find("state");
    if (state != fields.end()) status.state = state->second;
    status.valid = true;
    return status;
}

struct TeammateDecision {
    bool messageFresh = false;
    bool forwardBusy = false;
    bool allowDefenderClear = true;
    bool teammateClaimsBall = false;
    bool selfShouldClaim = false;
};

enum class TacticalAction { Hold, Chase, Support, Defend, Clear };

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

inline TacticalDecision decideTactics(const TacticalInput &input,
    double staleAfter = 1.20, double takeoverAfter = 2.50)
{
    TacticalDecision result;
    const bool teammateFresh = input.teammate.valid && input.teammateMessageAge >= 0.0 &&
        input.teammateMessageAge <= staleAfter && input.teammate.healthy;
    const bool teammateClaims = teammateFresh && input.teammate.claim;
    result.ownHalfBall = input.selfBall && ballInOwnHalf(input.color, input.selfBallX);

    // The utility is intentionally simple and monotonic.  A closer, fresher,
    // more confident observation wins, while the defender has a small penalty
    // for leaving its home role.  This gives deterministic arbitration without
    // requiring a central referee or a new ROS message.
    result.selfUtility = input.selfHealthy && input.selfBall ?
        input.selfBallScore - 0.12 * input.selfBallDistance - 0.08 * input.selfBallAge : -100.0;
    result.teammateUtility = teammateFresh && input.teammate.ball ?
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

    if (input.id == 1) {
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

struct DefenderClearInput {
    TeamColor color = TeamColor::Red;
    bool ballVisible = false;
    bool locationFresh = false;
    bool forwardBusy = false;
    double locationX = 0.0;
    double ballRadius = 0.0;
    double bearing = 0.0;
    double headingError = 0.0;
};

inline bool defenderOwnHalf(TeamColor color, double locationX, double safeMargin)
{
    const double margin = std::max(0.20, std::min(1.20, safeMargin));
    return color == TeamColor::Red ? locationX >= margin : locationX <= -margin;
}

inline bool shouldDefenderClear(const DefenderClearInput &input,
    double minimumRadius = 0.075, double maximumBearing = 28.0,
    double maximumHeadingError = 18.0, double safeMargin = 0.75)
{
    return input.ballVisible && input.locationFresh && !input.forwardBusy &&
        defenderOwnHalf(input.color, input.locationX, safeMargin) &&
        input.ballRadius >= minimumRadius && std::abs(input.bearing) <= maximumBearing &&
        std::abs(input.headingError) <= maximumHeadingError;
}

inline TeammateDecision arbitrate(const TeamStatus &teammate, double messageAge,
                                  double staleAfter = 1.20)
{
    TeammateDecision result;
    result.messageFresh = teammate.valid && messageAge >= 0.0 && messageAge <= staleAfter;
    result.forwardBusy = result.messageFresh && teammate.role == PlayerRole::Forward &&
        (teammate.active || teammate.kick);
    result.allowDefenderClear = !result.forwardBusy;
    result.teammateClaimsBall = result.messageFresh && teammate.claim;
    return result;
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
    BoundaryDecision update(TeamColor color, PlayerRole role, double x, double margin)
    {
        const double safeMargin = std::max(0.20, std::min(1.20, margin));
        const double enter = 3.55 - safeMargin;
        const double exit = enter - 0.18;
        const bool red = color == TeamColor::Red;
        if (role == PlayerRole::Forward) {
            const bool beyond = red ? x > enter : x < -enter;
            const bool clear = red ? x < exit : x > -exit;
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

private:
    bool forwardBlocked_ = false;
    bool defenderBlocked_ = false;
};

}  // namespace cupcup
