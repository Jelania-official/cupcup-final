#include "match_policy.hpp"
#include <cassert>
#include <cmath>

int main()
{
    using namespace cupcup;
    MatchState red;
    red.now = 10;
    red.healthy = true;
    red.localBallVisible = true;
    red.map.updateSelf(1, 0, 180, 10);
    red.map.updateBall(0, 0, 10, 0.9);
    auto plan = planMatch(red);
    assert(plan.targetValid && plan.kickTargetValid && plan.tactical.claim);
    assert(plan.target.x > 0 && plan.kickTarget.x < 0);
    assert(pointDistance(plan.kickTarget, {0, 0}) <= PolicyConfig().kickTravel + 1e-8);
    assert(FieldGeometry::risk(TeamColor::Red, 1, plan.target) <= 0);
    MatchState blue = red;
    blue.color = TeamColor::Blue;
    blue.map.reset();
    blue.map.updateSelf(-1, 0, 0, 10);
    blue.map.updateBall(0, 0, 10, 0.9);
    const auto mirror = planMatch(blue);
    assert(std::abs(plan.target.x + mirror.target.x) < 1e-9);
    assert(plan.tactical.action == mirror.tactical.action);
    assert(!planMatch(MatchState{}).targetValid);
    red.obstacles.push_back({{true, -0.50, 0, 1.0, 10}, 0.18, 0.0});
    const auto blockedShot = planMatch(red);
    assert(std::abs(blockedShot.kickTarget.z) > 0.5);
    red.obstacles[0].position.observedAt = 8;
    assert(std::abs(planMatch(red).kickTarget.z) < 1e-8);
    red.obstacles.clear();
    // The same sharing adapter is used in ROS and the sandbox. Exercise the
    // actual Talk decoder, receiver elapsed time and colour symmetry.
    const std::string peerWire = "cupcup|id=2|role=defender|healthy=1|robots=1"
        "|r0x=-0.5|r0z=0|r0conf=0.9|r0unc=0.5|r0age=0.1"
        "|r0team=blue|r0source=number_square";
    MatchState shared = red;
    shared.teammate = parseTeamStatus(peerWire);
    shared.teammateMessageAge = .2;
    appendTeammateObstacles(shared);
    assert(shared.obstacles.size() == 1 && shared.obstacles[0].fromTeammate);
    assert(shared.obstacles[0].position.confidence == .25);
    assert(std::abs(shared.obstacles[0].position.observedAt - 9.7) < 1e-9);
    assert(std::abs(planMatch(shared).kickTarget.z) > .5);
    assert(shared.map.robots().empty());  // no relay through local map
    appendTeammateObstacles(shared);
    assert(shared.obstacles.size() == 1);  // no double scoring
    shared.obstacles[0].fromTeammate = false;
    shared.obstacles[0].position.x = -.45;
    appendTeammateObstacles(shared);
    assert(shared.obstacles.size() == 1 && shared.obstacles[0].position.x == -.45);
    shared.obstacles.clear();
    shared.teammateMessageAge = .56;
    appendTeammateObstacles(shared);
    assert(shared.obstacles.empty());  // sender age + receiver elapsed
    shared.teammateMessageAge = -.01;
    appendTeammateObstacles(shared);
    assert(shared.obstacles.empty());
    shared.teammateMessageAge = .2;
    shared.teammate.id = 1;
    appendTeammateObstacles(shared);
    assert(shared.obstacles.empty());
    shared.teammate = parseTeamStatus(peerWire);
    shared.teammate.healthy = false;
    appendTeammateObstacles(shared);
    assert(shared.obstacles.empty());
    shared.teammate.healthy = true;
    shared.teammate.robots[0].team = "red";
    appendTeammateObstacles(shared);
    assert(shared.obstacles.empty());  // peer's view of us is not an enemy
    shared.teammate.robots[0].team = "unknown";
    appendTeammateObstacles(shared);
    assert(shared.obstacles.empty());
    shared.color = TeamColor::Blue;
    shared.teammate.robots[0].team = "red";
    shared.teammate.robots[0].x = .5;
    appendTeammateObstacles(shared);
    assert(shared.obstacles.size() == 1 && shared.obstacles[0].team == RobotTeam::Red);
    for (const auto &suffix : {"|r0age=-0.1", "|r0age=nan", "|r0x=inf", "|r0x=1junk",
                              "|r0conf=1.1", "|r0unc=-1", "|robots=5", "|robots=1.5",
                              "|healthy=0", "|r0source=box_height", "|r0team=unknown"})
        assert(parseTeamStatus(peerWire + suffix).robots.empty());
    assert(!parseTeamStatus("cupcup|id=2junk|role=defender").valid);
    assert(parseTeamStatus("cupcup|id=2|role=defender|healthy=1|peer_obstacles=1"
        "|pobs0x=-1|pobs0z=0|pobs0conf=0.25|pobs0unc=1|pobs0age=0.1|pobs0team=blue")
        .robots.empty());  // effective diagnostics must never be relayed
    assert(parseTeamStatus("cupcup|id=2|role=defender|robots=1").robots.empty());
    // The ROS map adapter uses only measured-number candidates, remains soft,
    // and drops stale/future samples. It must not turn box-height guesses into
    // hard obstacles or claim that a local track ID identifies an opponent.
    WorldModel robotMap;
    robotMap.updateRobots({{{-.5, 0}, RobotTeam::Unknown, .9, .5,
                           RobotPositionSource::NumberSquare},
                          {{-2, 1}, RobotTeam::Blue, .9, .5,
                           RobotPositionSource::BoxHeight}}, 10);
    const auto candidates = mapRobotObstacles(robotMap, 10);
    assert(candidates.size() == 1 && candidates[0].position.confidence == .25);
    assert(mapRobotObstacles(robotMap, 9).empty());
    assert(mapRobotObstacles(robotMap, 10.66).empty());
    MatchState withCandidate = red;
    withCandidate.obstacles = candidates;
    const auto softPlan = planMatch(withCandidate);
    assert(softPlan.targetValid && std::abs(softPlan.kickTarget.z) > .5);
    assert(std::abs(planMatch(red).kickTarget.z) < 1e-8);
    // Ball flight is clear, but the straight kick pose is occupied from behind.
    MatchState poseBlocked = red;
    poseBlocked.obstacles.push_back({{true, 0.22, 0, 1.0, 10}, 0.18, 0.0});
    const auto occupied = planMatch(poseBlocked);
    assert(std::abs(occupied.kickTarget.z) > 0.5);
    assert(pointDistance(occupied.target, {0.22, 0}) > 0.1);
    poseBlocked.obstacles[0].position.observedAt = 8;
    assert(std::abs(planMatch(poseBlocked).kickTarget.z) < 1e-8);
    poseBlocked.obstacles[0].position.observedAt = 10;
    poseBlocked.obstacles[0].position.confidence = 0.25;
    assert(std::abs(planMatch(poseBlocked).kickTarget.z) < 1e-8);
    MatchState bluePose = blue;
    bluePose.obstacles.push_back({{true, -0.22, 0, 1.0, 10}, 0.18, 0.0});
    const auto mirroredPose = planMatch(bluePose);
    assert(std::abs(occupied.kickTarget.x + mirroredPose.kickTarget.x) < 1e-8);
    assert(std::abs(occupied.kickTarget.z + mirroredPose.kickTarget.z) < 1e-8);
    // Small fluctuations should not reverse a slow, already-chosen kick.
    MatchState noisy = red;
    noisy.map.reset();
    noisy.map.updateSelf(0.7, 0, 180, 10);
    noisy.map.updateBall(0, 0, 10, 0.9);
    noisy.obstacles.push_back({{true, -0.5, 0.01, 1.0, 10}, 0.18, 0.0});
    PolicyMemory memory;
    const auto initial = planMatch(noisy, {}, &memory);
    assert(memory.valid && std::abs(initial.kickTarget.z) > 0.5);
    noisy.now = 10.05;
    noisy.obstacles[0].position.z = -0.01;
    const auto fluctuating = planMatch(noisy);
    assert(fluctuating.kickTarget.z * initial.kickTarget.z < 0.0);
    const auto stable = planMatch(noisy, {}, &memory);
    assert(pointDistance(stable.kickTarget, initial.kickTarget) < 1e-8);
    // A genuinely blocked direction must still be abandoned.
    noisy.obstacles.push_back({{true, 0.3 * memory.direction.x,
        0.3 * memory.direction.z, 1.0, noisy.now}, 0.18, 0.0});
    const auto unsafe = planMatch(noisy, {}, &memory);
    assert(pointDistance(unsafe.kickTarget, stable.kickTarget) > 0.2);
    noisy.healthy = false;
    assert(!planMatch(noisy, {}, &memory).targetValid && !memory.valid);
    noisy.healthy = true;
    planMatch(noisy, {}, &memory);
    assert(memory.valid);
    noisy.localBallVisible = false;
    planMatch(noisy, {}, &memory);
    assert(!memory.valid);
    noisy.localBallVisible = true;
    planMatch(noisy, {}, &memory);
    noisy.now = 11;
    planMatch(noisy, {}, &memory);
    assert(!memory.valid);  // stale observation never refreshes a commitment
    red.now = 11;
    assert(!planMatch(red).kickTargetValid);
    assert(!planMatch(red).targetValid);  // no fake centre ball on loss
    red.now = 10;
    red.id = 2;
    red.map.updateBall(-2, 0, 12, 0.9);
    red.now = 12;
    red.map.updateSelf(1, 0, 180, 12);
    plan = planMatch(red);
    assert(plan.tactical.action == TacticalAction::Defend);
    assert(plan.target.x >= 0.20);
    red.map.reset();
    red.map.updateSelf(1, 0, 180, 12);
    red.map.updateBall(1.2, 0, 12, 0.9);
    assert(planMatch(red).tactical.action == TacticalAction::Clear);
    PolicyConfig disabled;
    disabled.defenderClearEnabled = false;
    assert(planMatch(red, disabled).tactical.action == TacticalAction::Defend);
    assert(!planMatch(red, disabled).tactical.claim);
    red.localBallVisible = false;  // shared ball while the defender scans
    red.map.reset();
    red.map.updateSelf(2, 0, 180, 12);
    red.map.updateBall(3, 1, 12, 0.9);
    const auto cover = planMatch(red);
    assert(cover.targetValid && cover.tactical.action == TacticalAction::Defend);
    assert(cover.target.x > 3 && cover.target.x <= 3.75);
    assert(cover.target.z > 0 && cover.target.z < 1);
    assert(FieldGeometry::risk(TeamColor::Red, 2, cover.target, 0.75) <= 0.0);
    MatchState mirroredCover = red;
    mirroredCover.color = TeamColor::Blue;
    mirroredCover.map.reset();
    mirroredCover.map.updateSelf(-2, 0, 0, 12);
    mirroredCover.map.updateBall(-3, -1, 12, 0.9);
    const auto coverBlue = planMatch(mirroredCover);
    assert(std::abs(cover.target.x + coverBlue.target.x) < 1e-8);
    assert(std::abs(cover.target.z + coverBlue.target.z) < 1e-8);
    red.localBallVisible = true;
    red.id = 1;
    red.map.reset();
    red.map.updateSelf(1, 0, 180, 12);
    red.map.updateBall(3, 0, 12, 0.9);
    plan = planMatch(red);
    assert(!plan.tactical.claim && plan.tactical.action == TacticalAction::Support);
    assert(plan.target.x < FieldGeometry::penaltyFront - 0.20);
    assert(FieldGeometry::refereeViolation(TeamColor::Red, 1, {2.71, 1.99}));
    assert(!FieldGeometry::refereeViolation(TeamColor::Red, 1, {2.70, 0}));
    assert(FieldGeometry::risk(TeamColor::Red, 1, {2.6, 2.4}) > 0);
    assert(FieldGeometry::safeMotion(TeamColor::Red, 1, {2.4, 0}, 180, 0.05, 0, 0.1));
    assert(!FieldGeometry::safeMotion(TeamColor::Red, 1, {2.4, 0}, 0, 0.05, 0, 0.1));
    assert(!FieldGeometry::safeMotion(TeamColor::Red, 2, {0.21, 0}, 180, 0.05, 0, 0.2));
    assert(FieldGeometry::safeMotion(TeamColor::Red, 2, {0.21, 0}, 0, 0.05, 0, 0.2));
    const auto left = FieldGeometry::localDisplacement(0, 0, 1);
    assert(left.x == 0 && left.z == -1);
    const auto rotated = FieldGeometry::localDisplacement(90, 1, 0);
    assert(std::abs(rotated.x) < 1e-9 && rotated.z == -1);
    return 0;
}
