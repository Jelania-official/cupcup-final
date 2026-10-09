#include "tactical_sim.hpp"
#include <cassert>
#include <iostream>

using namespace cupcup;
using namespace cupcup::sim;

int main() {
    assert(horizontallyVisible({}, 0, 0, {1, 0}));
    assert(!horizontallyVisible({}, 0, 0, {0, -1}));
    assert(horizontallyVisible({}, 0, 60, {0, -1}));
    assert(!horizontallyVisible({}, 0, -60, {0, -1}));
    assert(!horizontallyVisible({}, 0, 0, {-1, 0}));
    assert(horizontallyVisible({}, 180, 60, {0, 1}));
    assert(!horizontallyVisible({}, 0, 0, {6, 0}));
    Observation deliveredView;
    deliveredView.seesBall = true;
    deliveredView.capturedAt = 1;
    deliveredView.relativeBall = {0, -1};
    assert(headPanTarget(deliveredView, 1.1) == 60);
    assert(headPanTarget(deliveredView, 1.7) == -45);  // expired: scan, not true-ball tracking
    deliveredView.yaw = 90;
    assert(headPanTarget(deliveredView, 1.1) == 0);
    Config narrowView;
    narrowView.delay = narrowView.noise = 0;
    Simulation unseenBehind(narrowView), seenAhead(narrowView);
    for (auto *s : {&unseenBehind, &seenAhead}) {
        s->world.teams[0].players[0].position = {};
        s->world.teams[0].players[0].yaw = 0;
    }
    unseenBehind.world.ball = {-1, 0};
    seenAhead.world.ball = {1, 0};
    unseenBehind.tick(); seenAhead.tick();
    const auto &blindObserver = unseenBehind.world.teams[0].players[0];
    const auto &frontObserver = seenAhead.world.teams[0].players[0];
    assert(blindObserver.belief.capturedAt == unseenBehind.world.time);
    assert(!blindObserver.belief.seesBall && frontObserver.belief.seesBall);
    assert(blindObserver.headPan == frontObserver.headPan);  // target cannot use unseen truth
    assert(std::abs(blindObserver.headPan) <= headPanSpeed * stepSeconds);
    Config rearSearch;
    rearSearch.delay = rearSearch.noise = 0;
    rearSearch.own.speed = rearSearch.other.speed = 0;
    rearSearch.own.turn = rearSearch.other.turn = 30;
    Simulation rear(rearSearch);
    rear.world.ball = {3, 0};
    rear.world.teams[0].players[0].position = {-3, -2};
    rear.world.teams[0].players[1].position = {2, 0};
    rear.world.teams[0].players[1].yaw = 180;
    bool defenderAcquiredRearBall = false;
    for (int i = 0; i < 300; ++i) {
        rear.tick();
        const auto &defender = rear.world.teams[0].players[1];
        defenderAcquiredRearBall |= defender.belief.seesBall;
        assert(distance(defender.position, {2, 0}) < 1e-9);
    }
    assert(defenderAcquiredRearBall);  // bounded scan must cover the rear, no true-ball steering
    Config sharingConfig;
    sharingConfig.seed = 101;
    sharingConfig.noise = .2;
    sharingConfig.dropout = 0;
    Simulation sharing(sharingConfig);
    bool sawSharedGeometry = false;
    for (int i = 0; i < 100; ++i) {
        sharing.tick();
        for (const auto &team : sharing.world.teams) for (const auto &p : team.players) {
            for (const auto &o : p.belief.obstacles) assert(!o.fromTeammate);
            for (const auto &o : p.sharedObstacles) {
                sawSharedGeometry = true;
                assert(o.fromTeammate && usableObstacle(o, sharing.world.time));
                assert(o.position.confidence <= .25);
                assert(o.team == (team.color == TeamColor::Red ? RobotTeam::Blue : RobotTeam::Red));
            }
        }
    }
    assert(sawSharedGeometry);
    assert(ballTranslationBlocked({}, {.21, 0}, {.20, 0}, .22));
    assert(ballTranslationBlocked({}, {.21, 0}, {-.30, 0}, .22));
    assert(ballTranslationBlocked({}, {.30, 0}, {-.30, 0}, .22));
    assert(ballTranslationBlocked({}, {-.21, 0}, {-.20, 0}, .22));
    assert(!ballTranslationBlocked({}, {.21, 0}, {.30, 0}, .22));
    assert(!ballTranslationBlocked({}, {.21, 0}, {.21, .05}, .22));
    assert(!ballTranslationBlocked({}, {}, {.05, 0}, .22));
    assert(!ballTranslationBlocked({}, {.30, 0}, {.40, 0}, .22));
    Config frozen;
    frozen.oracle = true;
    frozen.own.speed = frozen.other.speed = 0;
    Simulation overlappingBall(frozen);
    overlappingBall.world.teams[0].players[0].position = {1, 2};
    overlappingBall.world.teams[0].players[1].position = {2, 2};
    overlappingBall.world.teams[1].players[0].position = {-.21, 0};
    overlappingBall.world.teams[1].players[1].position = {-2, 2};
    overlappingBall.world.ball = {0, 0};
    overlappingBall.world.velocity = {-1, 0};
    overlappingBall.tick();
    // Already in the abstract disc must not mean free passage deeper inside.
    assert(distance(overlappingBall.world.ball, {}) < 1e-9);
    assert(overlappingBall.world.velocity.x > 0);
    Simulation escapingBall(frozen);
    escapingBall.world = overlappingBall.world;
    escapingBall.world.ball = {};
    escapingBall.world.velocity = {1, 0};
    escapingBall.tick();
    assert(std::abs(escapingBall.world.ball.x - stepSeconds) < 1e-9);

    for (double x : {-1e-15, 0.0, 1e-15}) {
        Simulation neutral{Config{}};
        neutral.world.ball = {x, 0};
        neutral.tick();
        assert(neutral.world.teams[0].territorySeconds == 0);
        assert(neutral.world.teams[1].territorySeconds == 0);
    }
    Simulation blueTerritory{Config{}};
    blueTerritory.world.ball = {-0.01, 0};
    blueTerritory.tick();
    assert(blueTerritory.world.teams[0].territorySeconds == stepSeconds);
    assert(blueTerritory.world.teams[1].territorySeconds == 0);
    const Point localError = localStagingError({-0.30, 0.04}, {0.80, 0.50}, {1.02, 0.50});
    const Point shiftedError = localStagingError({-0.30, 0.04}, {1.80, -0.50}, {2.02, -0.50});
    assert(distance(localError, shiftedError) < 1e-9);
    assert(std::abs(localError.x + 0.08) < 1e-9 && std::abs(localError.z - 0.04) < 1e-9);
    Config poseError;
    poseError.oracle = false; poseError.selfNoise = 0.4;
    poseError.noise = 0.0; poseError.delay = 0.0;
    Simulation projected(poseError);
    const Point initialSelf = projected.world.teams[0].players[0].position;
    const Point initialBall = projected.world.ball;
    projected.tick();
    const auto &measurement = projected.world.teams[0].players[0].belief;
    assert(measurement.seesBall);
    assert(distance(measurement.relativeBall,
                    {initialBall.x - initialSelf.x, initialBall.z - initialSelf.z}) < 1e-9);
    assert(distance(measurement.ball,
                    {measurement.self.x + measurement.relativeBall.x,
                     measurement.self.z + measurement.relativeBall.z}) < 1e-9);
    assert(distance(measurement.self, initialSelf) > 1e-6);

    World contactBefore;
    for (int a = 0; a < 4; ++a)
        contactBefore.teams[a / 2].players[a % 2].position = {10.0 * a, 0};
    contactBefore.teams[0].players[0].position = {0, 0};
    contactBefore.teams[1].players[0].position = {0.36, 0};
    World contactAfter = contactBefore;
    contactAfter.teams[0].players[0].position.x = -0.002;  // leading robot escapes
    contactAfter.teams[1].players[0].position.x = 0.35;    // pursuer closes too fast
    resolveRobotTranslations(contactBefore, contactAfter);
    assert(contactAfter.teams[0].players[0].position.x == -0.002);
    assert(contactAfter.teams[1].players[0].position.x == 0.36);
    assert(contactAfter.teams[0].blockedSeconds == 0);
    assert(contactAfter.teams[1].blockedSeconds == stepSeconds);
    contactAfter = contactBefore;
    contactAfter.teams[0].players[0].position.x = 0.01;
    contactAfter.teams[1].players[0].position.x = 0.35;
    resolveRobotTranslations(contactBefore, contactAfter);
    assert(contactAfter.teams[0].players[0].position.x == 0);
    assert(contactAfter.teams[1].players[0].position.x == 0.36);
    // Existing overlap may separate gradually, but cannot get deeper.
    contactBefore.teams[1].players[0].position.x = 0.30;
    contactAfter = contactBefore;
    contactAfter.teams[0].players[0].position.x = -0.01;
    resolveRobotTranslations(contactBefore, contactAfter);
    assert(contactAfter.teams[0].players[0].position.x == -0.01);
    contactAfter = contactBefore;
    contactAfter.teams[0].players[0].position.x = 0.01;
    resolveRobotTranslations(contactBefore, contactAfter);
    assert(contactAfter.teams[0].players[0].position.x == 0);

    // Rejection of the middle robot must be rechecked against its follower.
    contactBefore.teams[0].players[1].position = {0.36, 0};
    contactBefore.teams[1].players[0].position = {0.72, 0};
    contactAfter = contactBefore;
    contactAfter.teams[0].players[0].position.x = 0.01;
    contactAfter.teams[0].players[1].position.x = 0.37;
    contactAfter.teams[1].players[0].position.x = 0.71;
    resolveRobotTranslations(contactBefore, contactAfter);
    for (int a = 0; a < 3; ++a)
        assert(distance(contactBefore.teams[a / 2].players[a % 2].position,
                        contactAfter.teams[a / 2].players[a % 2].position) == 0);
    assert(contactAfter.collisions == 2);

    std::mt19937 contactRandom(53101);
    std::uniform_real_distribution<double> coordinate(-0.5, 0.5), delta(-0.015, 0.015);
    for (int trial = 0; trial < 2000; ++trial) {
        World old, proposed;
        for (int a = 0; a < 4; ++a) {
            const Point start{coordinate(contactRandom), coordinate(contactRandom)};
            old.teams[a / 2].players[a % 2].position = start;
            proposed.teams[a / 2].players[a % 2].position =
                {start.x + delta(contactRandom), start.z + delta(contactRandom)};
        }
        const World commanded = proposed;
        resolveRobotTranslations(old, proposed);
        for (int a = 0; a < 4; ++a) {
            const Point start = old.teams[a / 2].players[a % 2].position;
            const Point end = proposed.teams[a / 2].players[a % 2].position;
            const Point command = commanded.teams[a / 2].players[a % 2].position;
            assert(distance(start, end) == 0 || distance(command, end) == 0);
            for (int b = a + 1; b < 4; ++b)
                assert(distance(end, proposed.teams[b / 2].players[b % 2].position) + 1e-9 >=
                    std::min(0.36, distance(start, old.teams[b / 2].players[b % 2].position)));
        }
    }

    Config config;
    config.oracle = true;
    config.own.setup = 0.5;
    Simulation prep(config);
    prep.world.ball = {0.0, 0.0};
    prep.world.teams[0].players[0].position = {0.25, 0.0};
    prep.world.teams[1].players[0].position = {-3.0, 2.0};
    prep.world.teams[1].players[1].position = {-3.0, -2.0};
    for (int i = 0; i < 8; ++i) prep.tick();
    assert(prep.world.teams[0].kicks == 0);
    for (int i = 0; i < 20; ++i) prep.tick();
    if (prep.world.teams[0].kicks == 0) {
        const auto &p = prep.world.teams[0].players[0];
        std::cerr << "prep failed: " << p.reason << " position=" << p.position.x << ',' << p.position.z
            << " yaw=" << p.yaw << " target=" << p.target.x << ',' << p.target.z
            << " kick=" << p.kickTarget.x << ',' << p.kickTarget.z
            << " preparing=" << p.preparing << " ball=" << prep.world.ball.x << ',' << prep.world.ball.z << '\n';
    }
    assert(prep.world.teams[0].kicks >= 1);
    const auto &gates = prep.world.teams[0];
    assert(gates.kickReadySeconds <= gates.kickAimSeconds);
    assert(gates.kickAimSeconds <= gates.kickFacingSeconds);
    assert(gates.kickFacingSeconds <= gates.kickReachSeconds);
    assert(gates.maxPreparingSeconds >= config.own.setup - 1e-8);

    Simulation goal(config);
    goal.world.ball = {-4.54, 0.0}; goal.world.velocity = {-1.0, 0.0};
    goal.tick();
    assert(goal.world.teams[0].score == 1);
    assert(goal.world.restarts == 1 && goal.world.pauseRemaining > 0.0);
    const double pausedPlay = goal.world.playTime;
    const Point pausedRobot = goal.world.teams[0].players[0].position;
    for (int i = 0; i < 10; ++i) goal.tick();
    assert(goal.world.playTime == pausedPlay);
    assert(goal.world.stationaryBallSeconds == 0.0);
    assert(distance(goal.world.teams[0].players[0].position, pausedRobot) == 0.0);

    Simulation side(config);
    side.world.ball = {0.0, 3.04}; side.world.velocity = {0.0, 1.0}; side.tick();
    assert(side.world.restarts == 1 && std::string(side.world.event) == "sideline");
    assert(distance(side.world.ball, {}) == 0.0 && distance(side.world.velocity, {}) == 0.0);

    Simulation penalty(config);
    penalty.world.teams[0].players[1].position = {-0.3, 1.0}; penalty.tick();
    assert(penalty.world.teams[0].penalties == 1);
    assert(penalty.world.teams[0].players[1].penaltyRemaining > 30.0);
    const Point waiting = penalty.world.teams[0].players[1].position;
    for (int i = 0; i < 20; ++i) penalty.tick();
    assert(distance(waiting, penalty.world.teams[0].players[1].position) == 0.0);

    Config missing = config;
    missing.oracle = false; missing.dropout = 1.0;
    Simulation unseen(missing);
    for (int i = 0; i < 200; ++i) unseen.tick();
    assert(unseen.world.teams[0].kicks == 0 && unseen.world.teams[1].kicks == 0);
    for (const auto &team : unseen.world.teams) for (const auto &p : team.players)
        assert(!p.belief.seesBall);
    assert(std::abs(unseen.world.stationaryBallSeconds - unseen.world.playTime) < 1e-8);
    for (const auto &team : unseen.world.teams) {
        assert(team.accessSeconds + team.contestedSeconds <= unseen.world.playTime + 1e-8);
        assert(team.blockedSeconds <= unseen.world.playTime + 1e-8);
    }
    missing.selfNoise = 0.7;
    Simulation blindPose(missing);
    const Point blindStart = blindPose.world.teams[0].players[0].position;
    for (int i = 0; i < 100; ++i) blindPose.tick();
    assert(distance(blindStart, blindPose.world.teams[0].players[0].position) < 1e-9);

    Simulation wide(config);
    wide.world.ball = {-1.0, 2.0};
    wide.tick();
    const auto &attacker = wide.world.teams[1].players[0];
    const auto &wideBall = attacker.map.ball();
    const Point wideGoal{4.5, 0.0};
    const Point wideDirection{wideGoal.x - wideBall.x, wideGoal.z - wideBall.z};
    const Point behind{wideBall.x - attacker.target.x, wideBall.z - attacker.target.z};
    assert(std::abs(behind.x * wideDirection.z - behind.z * wideDirection.x) < 1e-8);
    assert(behind.x * wideDirection.x + behind.z * wideDirection.z > 0.0);

    // The adapter must retain the production map, not reconstruct it per tick.
    Config delayed = config;
    delayed.oracle = false; delayed.delay = 0.20; delayed.noise = 0.0;
    Simulation history(delayed);
    for (int i = 0; i < 10; ++i) history.tick();
    auto &observer = history.world.teams[0].players[0];
    const double acceptedAt = observer.map.ball().observedAt;
    const double beforeX = observer.map.ball().x;
    history.world.ball = {-0.5, 0.0};
    for (int i = 0; i < 4; ++i) history.tick();
    assert(std::abs(observer.map.ball().x - beforeX) < 1e-8);  // not delivered yet
    history.tick();
    assert(observer.map.ball().observedAt > acceptedAt);
    assert(observer.map.ball().x < beforeX && observer.map.ball().x > -0.5);
    assert(observer.map.self().observedAt <= history.world.time - 0.19);
    observer.penaltyRemaining = 1.0;
    observer.belief.seesBall = false;
    observer.pending.clear();
    observer.map.expire(history.world.time + 3.0);
    assert(!observer.map.self().valid && !observer.map.ball().valid);

    for (int i = 0; i < 1000; ++i) {
        const World before = prep.world;
        prep.tick();
        if (prep.world.restarts != before.restarts) continue;
        for (int t = 0; t < 2; ++t) for (int p = 0; p < 2; ++p) {
            const auto &robot = prep.world.teams[t].players[p];
            const auto &old = before.teams[t].players[p];
            assert(distance(robot.position, old.position) <= 0.4 * stepSeconds + 1e-8);
            assert(std::abs(wrap(robot.yaw - old.yaw)) <= 150.0 * stepSeconds + 1e-8);
            assert(FieldGeometry::risk(prep.world.teams[t].color, robot.id, robot.position, 0.0) <= 0.0);
        }
    }
    std::cout << "tactical simulation tests passed\n";
}
