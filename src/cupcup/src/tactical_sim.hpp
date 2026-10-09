#pragma once

#include "match_policy.hpp"
#include <array>
#include <deque>
#include <random>
#include <string>

namespace cupcup { namespace sim {

constexpr double pi = 3.14159265358979323846;
constexpr double stepSeconds = 0.05;
using Point = FieldPoint;
inline double limit(double v, double a, double b) { return std::max(a, std::min(b, v)); }
inline double distance(Point a, Point b) { return std::hypot(a.x - b.x, a.z - b.z); }
inline double wrap(double a) {
    while (a > 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}
inline double heading(Point a, Point b) { return fieldHeading(a, b); }
inline bool active(TacticalAction a) { return isBallAction(a); }
inline const char *actionName(TacticalAction a) {
    switch (a) {
    case TacticalAction::Chase: return "CHASE";
    case TacticalAction::Clear: return "CLEAR";
    case TacticalAction::Support: return "SUPPORT";
    case TacticalAction::Defend: return "DEFEND";
    default: return "HOLD";
    }
}

// Bounds are explicit experimental inputs, not calibrated robot capabilities.
struct Capabilities {
    double speed = 0.4, turn = 150.0, acceleration = 0.9;
    double kickSpeed = 0.9, kickRange = 0.32, setup = 1.0, cooldown = 2.0;
};
struct Config {
    unsigned seed = 7;
    double duration = 45.0, noise = 0.2, selfNoise = 0.0;
    double delay = 0.1, dropout = 0.0, ballDecay = 0.9;
    bool oracle = false;
    bool headPanVision = true;
    double kickDirectionHysteresis = 0.25;
    bool defenderLineAnchor = true;
    TeamColor cupcupColor = TeamColor::Red;
    std::string opponent = "block", scenario = "kickoff";
    Capabilities own, other;
    bool customInitialState = false;
    // red1/red2/blue1/blue2 (x,z,yaw), then ball (x,z,vx,vz).
    std::array<double, 16> initialState{};
};
struct Observation {
    double capturedAt = -99.0;
    Point self, ball, relativeBall;  // relative displacement expressed in field axes
    double yaw = 0.0, confidence = 0.0;
    bool seesBall = false;
    std::vector<ObservedObstacle> obstacles;
};
// Original camera horizontal FOV is 1.3613 radians. Pan limits follow the
// player controller; 90 deg/s is provisional, NOT a measured servo capability.
constexpr double cameraHalfFov = 1.3613 * 180.0 / pi / 2.0;
constexpr double headPanSpeed = 90.0;
inline bool horizontallyVisible(Point self, double bodyYaw, double pan, Point target) {
    return distance(self, target) <= 5.5 &&
        std::abs(wrap(heading(self, target) - bodyYaw - pan)) <= cameraHalfFov;
}
inline double headPanTarget(const Observation &delivered, double now) {
    if (delivered.seesBall && now >= delivered.capturedAt && now - delivered.capturedAt <= .65)
        return limit(wrap(heading({}, delivered.relativeBall) - delivered.yaw), -60.0, 60.0);
    static const double scan[] = {-45.0, 0.0, 45.0, 45.0, 0.0, -45.0};
    return scan[static_cast<unsigned>(std::max(0.0, now) / 2.0) % 6];
}
struct Agent {
    int id = 1;
    Point position, target, kickTarget;
    double yaw = 0.0, speed = 0.0, preparing = 0.0, kickReadyAt = 0.0;
    double headPan = 0.0;
    double penaltyRemaining = 0.0;
    TacticalAction action = TacticalAction::Hold;
    const char *reason = "initial";
    Observation belief;
    std::vector<ObservedObstacle> sharedObstacles;  // diagnostic policy inputs, not sensor data
    WorldModel map;
    PolicyMemory policy;
    double mappedAt = -99.0, teammateMappedAt = -99.0;
    std::deque<Observation> pending;
    std::mt19937 random;
};

inline Point localStagingError(Point relativeBall, Point mappedBall, Point stagingTarget) {
    return {relativeBall.x + stagingTarget.x - mappedBall.x,
            relativeBall.z + stagingTarget.z - mappedBall.z};
}

inline bool ballTranslationBlocked(Point robot, Point before, Point after, double radius) {
    if (distance(robot, before) >= radius)
        return segmentDistance(robot, before, after) < radius;
    // Existing overlap may escape, but cannot move farther into the disc.
    // Checking the initial direction also catches steps that cross the centre
    // and end outside again. A ball exactly at the centre may escape freely.
    return (before.x - robot.x) * (after.x - before.x) +
        (before.z - robot.z) * (after.z - before.z) < -1e-12;
}
struct Team {
    TeamColor color = TeamColor::Red;
    std::array<Agent, 2> players;
    int score = 0, kicks = 0, penalties = 0;
    double ballPositionIntegral = 0.0, territorySeconds = 0.0, threatSeconds = 0.0;
    double accessSeconds = 0.0, contestedSeconds = 0.0, blockedSeconds = 0.0;
    double kickReachSeconds = 0.0, kickFacingSeconds = 0.0, kickAimSeconds = 0.0;
    double kickReadySeconds = 0.0, maxPreparingSeconds = 0.0;
    int setupResets = 0;
};
struct World {
    Point ball, velocity;
    std::array<Team, 2> teams;
    double time = 0.001, playTime = 0.0, pauseRemaining = 0.0, stillFor = 0.0;
    int restarts = 0, collisions = 0;
    double stationaryBallSeconds = 0.0;
    const char *event = "kickoff";
};

// Keep or reject commanded translations; never shove a robot beyond its speed
// budget. Contact forbids closing motion, not motion that separates the pair.
// Recheck all pairs after simultaneous rejection: a stopped robot can otherwise
// become an obstacle to a third robot's previously safe proposal.
inline void resolveRobotTranslations(const World &before, World &after) {
    std::array<bool, 4> blocked{{false, false, false, false}};
    std::array<std::array<bool, 4>, 4> counted{};
    std::array<Point, 4> proposed;
    for (int a = 0; a < 4; ++a) proposed[a] = after.teams[a / 2].players[a % 2].position;
    for (int pass = 0; pass < 5; ++pass) {
        auto next = blocked;
        for (int a = 0; a < 4; ++a) for (int b = a + 1; b < 4; ++b) {
            const auto &pa = after.teams[a / 2].players[a % 2];
            const auto &pb = after.teams[b / 2].players[b % 2];
            if (pa.penaltyRemaining > 0.0 || pb.penaltyRemaining > 0.0) continue;
            const Point oldA = before.teams[a / 2].players[a % 2].position;
            const Point oldB = before.teams[b / 2].players[b % 2].position;
            const double oldDistance = distance(oldA, oldB);
            if (distance(pa.position, pb.position) + 1e-10 >= std::min(0.36, oldDistance)) continue;
            if (!counted[a][b]) { counted[a][b] = true; ++after.collisions; }
            const Point normal{oldB.x - oldA.x, oldB.z - oldA.z};
            const auto closing = [&](Point start, Point end) {
                return (end.x - start.x) * normal.x + (end.z - start.z) * normal.z;
            };
            if (closing(oldA, proposed[a]) > 0.0) next[a] = true;
            if (closing(oldB, proposed[b]) < 0.0) next[b] = true;
        }
        if (next == blocked) break;
        blocked = next;
        for (int a = 0; a < 4; ++a) if (blocked[a]) {
            auto &p = after.teams[a / 2].players[a % 2];
            p.position = before.teams[a / 2].players[a % 2].position; p.speed = 0.0;
        }
    }
    for (int t = 0; t < 2; ++t) if (blocked[t * 2] || blocked[t * 2 + 1])
        after.teams[t].blockedSeconds += stepSeconds;
}

class Simulation {
public:
    explicit Simulation(Config config) : config_(std::move(config)), random_(config_.seed) {
        world.teams[0].color = TeamColor::Red;
        world.teams[1].color = TeamColor::Blue;
        for (auto &team : world.teams) for (int i = 0; i < 2; ++i) {
            auto &p = team.players[i];
            p.id = i + 1;
            // Streams follow cupcup/opponent identity, not red/blue iteration order.
            p.random.seed(config_.seed + 1009U * i +
                (team.color == config_.cupcupColor ? 23U : 7919U));
        }
        resetPlayers(false);
        const double sign = FieldGeometry::ownSign(config_.cupcupColor);
        if (config_.scenario == "own-half") world.ball = {sign * 2.0, sign * 1.0};
        if (config_.scenario == "sideline") {
            world.ball = {0.0, sign * 2.97}; world.velocity = {0.0, sign * 0.8};
        }
        if (config_.scenario == "incoming") {
            world.ball = {sign * 0.6, sign * 0.2}; world.velocity = {sign * 1.2, 0.0};
        }
        if (config_.scenario == "shot") {
            world.ball = {-sign * 4.35, 0.0}; world.velocity = {-sign * 1.2, 0.0};
        }
        if (config_.customInitialState) {
            const auto &s = config_.initialState;
            for (int i = 0; i < 4; ++i) {
                auto &p = world.teams[i / 2].players[i % 2];
                p.position = p.target = {s[3 * i], s[3 * i + 1]};
                p.yaw = s[3 * i + 2];
            }
            world.ball = {s[12], s[13]}; world.velocity = {s[14], s[15]};
        }
    }
    World world;
    const Config &config() const { return config_; }

    void tick() {
        world.time += stepSeconds;
        if (world.pauseRemaining > 0.0) {
            world.pauseRemaining = std::max(0.0, world.pauseRemaining - stepSeconds);
            return;
        }
        world.playTime += stepSeconds;
        // All observations/actions refer to one frozen world. Neither team can
        // see its opponent's action from this same tick.
        const World snapshot = world;
        recordMetrics(snapshot);
        std::array<std::array<MatchIntent, 2>, 2> intents;
        for (int t = 0; t < 2; ++t) {
            for (int p = 0; p < 2; ++p) sense(t, p, snapshot);
            const Team delivered = world.teams[t];
            for (int p = 0; p < 2; ++p) intents[t][p] = decide(t, p, snapshot, delivered);
        }
        for (int t = 0; t < 2; ++t) for (int p = 0; p < 2; ++p) {
            auto &agent = world.teams[t].players[p];
            const auto &intent = intents[t][p];
            agent.action = intent.tactical.action;
            // A missing intent means hold at the estimated self position.
            // Feeding the true position here leaks perfect feedback under noise.
            agent.target = intent.targetValid ? intent.target :
                Point{agent.map.self().x, agent.map.self().z};
            agent.kickTarget = intent.kickTarget;
            agent.reason = intent.reason;
            if (agent.penaltyRemaining > 0.0) {
                agent.penaltyRemaining = std::max(0.0, agent.penaltyRemaining - stepSeconds);
                agent.speed = agent.preparing = 0.0;
            } else move(agent, world.teams[t].color, capabilities(t));
        }
        resolveContacts(snapshot);
        kick();
        advanceBall();
        referee();
    }

private:
    Config config_;
    std::mt19937 random_;
    void recordMetrics(const World &snapshot) {
        bool nearby[2] = {false, false};
        for (int t = 0; t < 2; ++t) for (const auto &p : snapshot.teams[t].players)
            if (p.penaltyRemaining <= 0.0 && distance(p.position, snapshot.ball) <= 0.6) nearby[t] = true;
        if (std::hypot(snapshot.velocity.x, snapshot.velocity.z) < 0.02)
            world.stationaryBallSeconds += stepSeconds;
        for (int t = 0; t < 2; ++t) {
            auto &team = world.teams[t];
            const double sign = FieldGeometry::ownSign(team.color);
            team.ballPositionIntegral += -sign * snapshot.ball.x * stepSeconds;
            // The centre line is neutral. Rotation can leave opposite signed
            // roundoff near zero; do not count it as territorial progress.
            if (sign * snapshot.ball.x < -1e-9) team.territorySeconds += stepSeconds;
            if (sign * snapshot.ball.x > 2.5 && std::abs(snapshot.ball.z) < 2.5)
                team.threatSeconds += stepSeconds;
            if (nearby[t] && !nearby[1 - t]) team.accessSeconds += stepSeconds;
            if (nearby[t] && nearby[1 - t]) team.contestedSeconds += stepSeconds;
        }
    }
    const Capabilities &capabilities(int t) const {
        return world.teams[t].color == config_.cupcupColor ? config_.own : config_.other;
    }
    void resetPlayers(bool goal) {
        for (auto &team : world.teams) {
            const double sign = FieldGeometry::ownSign(team.color);
            for (auto &p : team.players) {
                p.position = p.id == 1 ? Point{sign * (goal ? 0.9 : 1.5), -sign * 0.55} :
                    Point{sign * (goal ? 3.0 : 2.1), sign * 0.55};
                p.yaw = sign > 0.0 ? 180.0 : 0.0;
                p.target = p.position;
                p.speed = p.preparing = p.penaltyRemaining = 0.0;
                p.headPan = 0.0;
                p.kickReadyAt = world.time + 0.5;
                p.action = TacticalAction::Hold;
                p.belief = Observation(); p.pending.clear(); p.map.reset();
                p.sharedObstacles.clear();
                p.policy.reset();
                p.mappedAt = p.teammateMappedAt = -99.0;
            }
        }
    }
    void sense(int t, int index, const World &snapshot) {
        Agent &agent = world.teams[t].players[index];
        const auto &truth = snapshot.teams[t].players[index];
        const double sign = FieldGeometry::ownSign(snapshot.teams[t].color);
        std::normal_distribution<double> ballNoise(0.0, config_.oracle ? 0.0 : config_.noise);
        std::normal_distribution<double> poseNoise(0.0, config_.oracle ? 0.0 : config_.selfNoise);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        Observation obs;
        obs.capturedAt = snapshot.time;
        obs.self = {truth.position.x + sign * poseNoise(agent.random),
                    truth.position.z + sign * poseNoise(agent.random)};
        obs.yaw = truth.yaw;
        const bool scans = truth.action == TacticalAction::Hold || !truth.belief.seesBall;
        if (config_.headPanVision) agent.headPan += limit(
            headPanTarget(truth.belief, snapshot.time) - agent.headPan,
            -headPanSpeed * stepSeconds, headPanSpeed * stepSeconds);
        const bool visible = config_.headPanVision ?
            horizontallyVisible(truth.position, truth.yaw, agent.headPan, snapshot.ball) :
            distance(truth.position, snapshot.ball) <= 5.5 &&
            (scans || std::abs(wrap(heading(truth.position, snapshot.ball) - truth.yaw)) < 95.0);
        obs.seesBall = config_.oracle || (visible && unit(agent.random) >= config_.dropout);
        if (obs.seesBall) {
            obs.relativeBall = {snapshot.ball.x - truth.position.x + sign * ballNoise(agent.random),
                                snapshot.ball.z - truth.position.z + sign * ballNoise(agent.random)};
            // Global projection shares the observer's pose error. The local
            // measurement does not acquire that error merely by being mapped.
            obs.ball = {obs.self.x + obs.relativeBall.x, obs.self.z + obs.relativeBall.z};
            obs.confidence = config_.oracle ? 1.0 : limit(1.0 - distance(obs.self, obs.ball) / 9.0, 0.15, 0.95);
        }
        for (int slot = 0; slot < 2; ++slot) for (int other = 0; other < 2; ++other) {
            const int otherTeam = config_.cupcupColor == TeamColor::Red ? slot : 1 - slot;
            if (otherTeam == t && other == index) continue;
            const auto &robot = snapshot.teams[otherTeam].players[other];
            if (robot.penaltyRemaining > 0.0) continue;
            const bool robotVisible = config_.headPanVision ?
                horizontallyVisible(truth.position, truth.yaw, agent.headPan, robot.position) :
                distance(truth.position, robot.position) <= 5.5 &&
                std::abs(wrap(heading(truth.position, robot.position) - truth.yaw)) <= 95.0;
            if (!config_.oracle && (!robotVisible || unit(agent.random) < config_.dropout)) continue;
            ObservedObstacle obstacle;
            obstacle.team = snapshot.teams[otherTeam].color == TeamColor::Red ?
                RobotTeam::Red : RobotTeam::Blue;
            obstacle.position.valid = true;
            obstacle.position.x = obs.self.x + robot.position.x - truth.position.x + sign * ballNoise(agent.random);
            obstacle.position.z = obs.self.z + robot.position.z - truth.position.z + sign * ballNoise(agent.random);
            obstacle.position.observedAt = snapshot.time;
            obstacle.position.confidence = config_.oracle ? 1.0 : 0.25;
            obstacle.radius = 0.18;
            obstacle.uncertainty = config_.oracle ? 0.0 :
                1.0 + distance(obs.self, {obstacle.position.x, obstacle.position.z}) * 0.35;
            obs.obstacles.push_back(obstacle);
        }
        agent.pending.push_back(obs);
        const double delay = config_.oracle ? 0.0 : config_.delay;
        while (!agent.pending.empty() && agent.pending.front().capturedAt <= world.time - delay + 1e-8) {
            agent.belief = agent.pending.front(); agent.pending.pop_front();
        }
        agent.map.expire(world.time);
        if (agent.belief.capturedAt > agent.mappedAt) {
            const auto &delivered = agent.belief;
            agent.map.updateSelf(delivered.self.x, delivered.self.z, delivered.yaw, delivered.capturedAt);
            if (delivered.seesBall)
                agent.map.updateBall(delivered.ball.x, delivered.ball.z, delivered.capturedAt,
                                     delivered.confidence, BallPositionSource::Local);
            agent.mappedAt = delivered.capturedAt;
        }
    }
    MatchIntent decide(int t, int index, const World &snapshot, const Team &delivered) {
        const auto &team = snapshot.teams[t];
        auto &agent = world.teams[t].players[index];
        // Mate's delivered observation is available by Talk; its action is from
        // the previous tick, avoiding instantaneous action-order arbitration.
        const auto &mate = delivered.players[1 - index];
        const auto &mateBefore = team.players[1 - index];
        MatchState input;
        input.color = team.color; input.id = agent.id; input.now = world.time;
        input.healthy = agent.penaltyRemaining <= 0.0;
        const auto &obs = agent.belief;
        if (mate.belief.capturedAt > agent.teammateMappedAt) {
            const auto &mateBall = mate.map.ball();
            if (mate.belief.seesBall && mateBall.fresh(world.time, 0.45))
                agent.map.updateBall(mateBall.x, mateBall.z, mateBall.observedAt,
                                     mateBall.confidence * 0.5, BallPositionSource::Teammate);
            agent.map.updateTeammate(mate.belief.self.x, mate.belief.self.z,
                mate.belief.yaw, mate.belief.capturedAt);
            agent.teammateMappedAt = mate.belief.capturedAt;
        }
        input.map = agent.map;
        input.localBallVisible = obs.seesBall && world.time - obs.capturedAt <= 0.65;
        input.obstacles = obs.obstacles;
        auto &status = input.teammate;
        status.valid = true; status.id = mate.id;
        status.role = mate.id == 1 ? PlayerRole::Forward : PlayerRole::Defender;
        status.healthy = mate.penaltyRemaining <= 0.0;
        status.ball = mate.belief.seesBall;
        status.claim = status.active = active(mateBefore.action);
        status.ballScore = mate.map.ball().confidence;
        status.ballDistance = distance({}, mate.belief.relativeBall);
        status.ballAge = world.time - mate.belief.capturedAt;
        status.ballMapAge = world.time - mate.map.ball().observedAt;
        status.hasBallPosition = mate.map.ball().fresh(world.time, 0.65);
        status.ballX = mate.map.ball().x; status.ballZ = mate.map.ball().z;
        input.teammateMessageAge = world.time - mate.belief.capturedAt;
        // Share delivered observations, never the snapshot's robot positions.
        // Number-square here abstracts a qualified measurement, not a claim
        // that the 2D sensor actually runs image geometry or colour recognition.
        for (const auto &robot : mate.belief.obstacles) {
            status.robots.push_back({robot.position.x, robot.position.z,
                robot.position.confidence, robot.uncertainty,
                mate.belief.capturedAt - robot.position.observedAt,
                robotTeamName(robot.team), "number_square"});
        }
        appendTeammateObstacles(input);
        if (team.color == config_.cupcupColor || config_.opponent == "shared") {
            agent.sharedObstacles.clear();
            for (const auto &obstacle : input.obstacles)
                if (input.healthy && obstacle.fromTeammate) agent.sharedObstacles.push_back(obstacle);
            PolicyConfig policy;
            policy.kickDirectionHysteresis = config_.kickDirectionHysteresis;
            policy.defenderLineAnchor = config_.defenderLineAnchor;
            return planMatch(input, policy,
                config_.kickDirectionHysteresis > 0.0 ? &agent.policy : nullptr);
        }
        MatchIntent intent;
        agent.sharedObstacles.clear();  // abstract opponents do not use shared policy scores
        if (!input.healthy) return intent;
        const auto &ball = input.map.ball();
        if (!ball.fresh(world.time, 0.65)) { intent.reason = "opponent-search"; return intent; }
        const double sign = FieldGeometry::ownSign(team.color);
        const bool chase = agent.id == 1 || config_.opponent == "press";
        intent.tactical.action = chase ? TacticalAction::Chase : TacticalAction::Defend;
        const Point goal{FieldGeometry::attackGoalX(team.color), 0.0};
        const double range = std::max(0.01, distance({ball.x, ball.z}, goal));
        // A bounded abstract attacker must stage behind the actual shot line,
        // not the x axis. Otherwise wide balls artificially disable its kick.
        intent.target = chase ? Point{ball.x - .22 * (goal.x - ball.x) / range,
                                      ball.z - .22 * (goal.z - ball.z) / range} :
            Point{sign * (config_.opponent == "keeper" ? 3.0 : 1.65), limit(ball.z * 0.42, -1.45, 1.45)};
        intent.target = FieldGeometry::legalTarget(team.color, agent.id, intent.target, 0.2);
        intent.targetValid = intent.kickTargetValid = true;
        intent.kickTarget = goal;
        intent.reason = "abstract-opponent";
        return intent;
    }
    void move(Agent &p, TeamColor color, const Capabilities &cap) {
        const auto &self = p.map.self();
        if (!self.fresh(world.time, 2.0)) { p.speed = 0.0; return; }
        const Point estimatedSelf{self.x, self.z};
        // The executor also uses delivered estimates. Perfect pose feedback in
        // realistic mode would silently bypass localization error and delay.
        Point error{p.target.x - estimatedSelf.x, p.target.z - estimatedSelf.z};
        const double localRange = distance({}, p.belief.relativeBall);
        const bool nearBall = p.belief.seesBall && localRange < cap.kickRange + 0.08;
        const auto &mappedBall = p.map.ball();
        const bool searchesBody = needsBodySearch(p.action, mappedBall.fresh(world.time, .65));
        const bool localApproach = active(p.action) && p.belief.seesBall &&
            world.time - p.belief.capturedAt <= 0.65 &&
            mappedBall.fresh(world.time, 0.65) && localRange <= 1.0;
        if (localApproach)
            error = localStagingError(p.belief.relativeBall, {mappedBall.x, mappedBall.z}, p.target);
        const double range = distance({}, error);
        double desired = range > 0.05 ? heading({}, error) : p.belief.yaw;
        if (localApproach) desired = heading({mappedBall.x, mappedBall.z}, p.kickTarget);
        if ((p.action == TacticalAction::Hold && !p.belief.seesBall) || searchesBody)
            desired = p.belief.yaw + 35.0 * stepSeconds;
        const double yawError = wrap(desired - p.belief.yaw);
        p.yaw = wrap(p.yaw + limit(yawError, -cap.turn * stepSeconds, cap.turn * stepSeconds));
        const double wanted = (range < 0.05 || p.action == TacticalAction::Hold || searchesBody ||
            (nearBall && range < 0.12)) ? 0.0 :
            std::min(cap.speed * (localApproach ? 0.25 : 1.0), range * 1.8) *
                (localApproach ? 1.0 : std::max(0.0, std::cos(yawError * pi / 180.0)));
        p.speed += limit(wanted - p.speed, -cap.acceleration * stepSeconds, cap.acceleration * stepSeconds);
        const double rad = p.yaw * pi / 180.0;
        // Abstract the real lateral/backward image servo, within the same
        // translation budget. The 0.25 near-ball factor remains provisional;
        // no true ball position enters this executor.
        const Point delta = localApproach && range > 1e-8 ?
            Point{p.speed * error.x / range * stepSeconds,
                  p.speed * error.z / range * stepSeconds} :
            Point{p.speed * std::cos(rad) * stepSeconds, -p.speed * std::sin(rad) * stepSeconds};
        const Point estimatedNext{estimatedSelf.x + delta.x, estimatedSelf.z + delta.z};
        const double margin = color == config_.cupcupColor || config_.opponent == "shared" ?
            PolicyConfig().boundaryMargin : 0.20;
        if (FieldGeometry::risk(color, p.id, estimatedNext, margin) > 0.0 &&
            FieldGeometry::risk(color, p.id, estimatedNext, margin) >=
            FieldGeometry::risk(color, p.id, estimatedSelf, margin)) { p.speed = 0.0; return; }
        p.position.x += delta.x; p.position.z += delta.z;
    }
    void resolveContacts(const World &before) {
        resolveRobotTranslations(before, world);
    }
    void kick() {
        std::vector<int> contacts;
        for (int t = 0; t < 2; ++t) for (int i = 0; i < 2; ++i) {
            auto &p = world.teams[t].players[i];
            const auto &cap = capabilities(t);
            const auto &mappedBall = p.map.ball();
            const bool available = p.penaltyRemaining <= 0.0 && active(p.action) && p.belief.seesBall &&
                mappedBall.fresh(world.time, 0.65) &&
                world.time - p.belief.capturedAt <= 0.65 && world.time >= p.kickReadyAt;
            const bool reach = available && distance(p.position, world.ball) <= cap.kickRange;
            const bool facing = reach && std::abs(wrap(heading(p.position, world.ball) - p.yaw)) <= 35.0;
            const bool aim = facing &&
                std::abs(wrap(heading({mappedBall.x, mappedBall.z}, p.kickTarget) - p.yaw)) <= 20.0;
            const bool ready = aim && p.speed <= 0.06;
            // Offline gate diagnostics. No truth-derived counter feeds policy.
            auto &team = world.teams[t];
            team.kickReachSeconds += reach ? stepSeconds : 0.0;
            team.kickFacingSeconds += facing ? stepSeconds : 0.0;
            team.kickAimSeconds += aim ? stepSeconds : 0.0;
            team.kickReadySeconds += ready ? stepSeconds : 0.0;
            if (!ready && p.preparing > 0.0) ++team.setupResets;
            p.preparing = ready ? p.preparing + stepSeconds : 0.0;
            team.maxPreparingSeconds = std::max(team.maxPreparingSeconds, p.preparing);
            if (ready && p.preparing + 1e-8 >= cap.setup) contacts.push_back(t * 2 + i);
        }
        if (contacts.empty()) return;
        // Uniform choice among the full candidate set, in canonical team order.
        if (config_.cupcupColor == TeamColor::Blue)
            std::sort(contacts.begin(), contacts.end(), [](int a, int b) { return (a + 2) % 4 < (b + 2) % 4; });
        std::uniform_int_distribution<std::size_t> choose(0, contacts.size() - 1);
        const int winner = contacts[choose(random_)];
        auto &p = world.teams[winner / 2].players[winner % 2];
        const auto &cap = capabilities(winner / 2);
        const auto &ball = p.map.ball();
        const double angle = heading({ball.x, ball.z}, p.kickTarget) * pi / 180.0;
        world.velocity = {cap.kickSpeed * std::cos(angle), -cap.kickSpeed * std::sin(angle)};
        p.kickReadyAt = world.time + cap.cooldown; p.preparing = 0.0;
        p.policy.reset();
        ++world.teams[winner / 2].kicks;
    }
    void restart(const char *event, bool goal = false) {
        world.event = event; ++world.restarts; world.pauseRemaining = 3.0;
        world.ball = {}; world.velocity = {}; world.stillFor = 0.0;
        resetPlayers(goal);
    }
    void advanceBall() {
        const Point before = world.ball;
        world.ball.x += world.velocity.x * stepSeconds;
        world.ball.z += world.velocity.z * stepSeconds;
        const double decay = std::exp(-config_.ballDecay * stepSeconds);
        world.velocity.x *= decay; world.velocity.z *= decay;
        // A simple bounded disc collision represents a block, not a kick. It
        // dissipates motion and never accelerates the ball beyond its input speed.
        for (const auto &team : world.teams) for (const auto &p : team.players) {
            if (p.penaltyRemaining > 0.0) continue;
            if (ballTranslationBlocked(p.position, before, world.ball, 0.22)) {
                world.ball = before; world.velocity.x *= -0.25; world.velocity.z *= -0.25;
                ++world.collisions;
            }
        }
        if (std::abs(world.ball.x) > 4.55 && std::abs(world.ball.z) < 1.35) {
            ++world.teams[world.ball.x < 0.0 ? 0 : 1].score; restart("goal", true);
        } else if (std::abs(world.ball.z) > 3.05) restart("sideline");
        else if (std::abs(world.ball.x) > 4.55) restart("endline");
        else {
            world.stillFor = distance(before, world.ball) < 0.001 ? world.stillFor + stepSeconds : 0.0;
            if (world.stillFor > 100.0) restart("no-movement");
        }
    }
    void referee() {
        for (auto &team : world.teams) for (auto &p : team.players) {
            if (p.penaltyRemaining > 0.0 || !FieldGeometry::refereeViolation(team.color, p.id, p.position)) continue;
            ++team.penalties;
            p.penaltyRemaining = 31.0;
            p.position = {FieldGeometry::ownSign(team.color) * (p.id == 1 ? 1.5 : 2.3), p.id == 1 ? -2.82 : 2.82};
            p.speed = p.preparing = 0.0; p.action = TacticalAction::Hold;
        }
    }
};

}} // namespace cupcup::sim
