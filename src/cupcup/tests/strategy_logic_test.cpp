#include "strategy_logic.hpp"

#include <cassert>
#include <cmath>

int main()
{
    assert(cupcup::needsBodySearch(cupcup::TacticalAction::Defend, false));
    assert(cupcup::needsBodySearch(cupcup::TacticalAction::Support, false));
    assert(!cupcup::needsBodySearch(cupcup::TacticalAction::Defend, true));
    assert(!cupcup::needsBodySearch(cupcup::TacticalAction::Chase, false));
    assert(std::abs(cupcup::fieldHeading({0, 0}, {1, 0})) < 1e-9);
    assert(std::abs(cupcup::fieldHeading({0, 0}, {0, -1}) - 90) < 1e-9);
    assert(std::abs(cupcup::fieldHeading({0, 0}, {0, 1}) + 90) < 1e-9);
    assert(std::abs(std::abs(cupcup::fieldHeading({0, 0}, {-1, 0})) - 180) < 1e-9);
    assert(std::abs(cupcup::fieldHeading({2, 3}, {3, 2}) - 45) < 1e-9);
    using namespace cupcup;

    assert(defenderTrackingPitch(45, .2) < 45);
    assert(defenderTrackingPitch(20, .8) > 20);
    assert(defenderTrackingPitch(25, .62) == 25);
    assert(defenderTrackingPitch(25, .55) == 25);
    assert(defenderTrackingPitch(25, .69) == 25);
    assert(defenderTrackingPitch(8, .1) == 8);
    assert(defenderTrackingPitch(60, .9) == 60);
    assert(defenderTrackingPitch(25, NAN) == 25);
    assert(defenderTrackingPitch(25, 1.1) == 25);

    assert(metricKickPose(.18, -.08, 0, true).linedUp);
    assert(metricKickPose(.18, .04, 0, false).linedUp);
    assert(!metricKickPose(.18, -.08, 0, false).linedUp);
    const auto tooClose = metricKickPose(.1286, -.0713, 0, true);
    assert(tooClose.valid && !tooClose.linedUp && tooClose.forward < 0);
    assert(!metricKickPose(NAN, 0, 0, true).valid);
    assert(!metricKickPose(.7, 0, 0, true).valid);
    assert(!metricKickPose(-.18, 0, 0, true).valid);
    const auto rotated = metricKickPose(-.18, .08, 180, true);
    assert(rotated.linedUp);
    assert(metricKickPose(.18, -.12, 0, true).lateral > 0);

    assert(searchMotion(5.99, 20, true).forward == 0.0);
    assert(searchMotion(10, 5.99, true).turn == 0.0);
    assert(searchMotion(10, 20, false).forward == 0.0);
    assert(searchMotion(NAN, 20, true).turn == 0.0);
    assert(searchMotion(10, INFINITY, true).forward == 0.0);
    const auto searching = searchMotion(6, 6, true);
    assert(searching.forward == 0.008 && searching.turn == 4.0);

    assert(!settledForKick(2.0, 3));  // a long wall wait is not enough sensor evidence
    assert(!settledForKick(0.7, 16));
    assert(settledForKick(2.0, 16));
    assert(settledForKick(0.81, 3, 3));  // explicit pre-change control for A/B
    assert(!settledForKick(NAN, 16));
    assert(!settledForKick(2.0, 20, 21));
    assert(!settledForKick(2.0, 20, 2));

    int stable = 16, missed = 0;
    assert(updateKickAlignment(true, false, true, stable, missed));
    assert(stable == 16 && missed == 1);
    for (int tick = 0; tick < 100; ++tick)
        assert(updateKickAlignment(false, false, true, stable, missed));
    assert(missed == 1);  // duplicate image/control ticks cannot consume tolerance
    assert(updateKickAlignment(true, true, true, stable, missed));
    assert(stable == 17 && missed == 0);
    assert(updateKickAlignment(true, false, true, stable, missed));
    assert(updateKickAlignment(true, false, true, stable, missed));
    assert(!updateKickAlignment(true, false, true, stable, missed));
    assert(stable == 0 && missed == 3);
    for (int frame = 0; frame < 100; ++frame)
        assert(!updateKickAlignment(true, false, true, stable, missed));
    assert(!updateKickAlignment(false, false, false, stable, missed));
    assert(stable == 0 && missed == 0);
    assert(updateKickAlignment(true, true, true, stable, missed));
    assert(stable == 1 && missed == 0);

    const TeamStatus parsed = parseTeamStatus(
        "cupcup|id=1|role=forward|state=ALIGN|ball=1|active=1|kick=0|claim=1|healthy=1|conf=0.8|bdist=0.7|bx=-1.2|bz=0.4|age=0.2|px=0.6|pz=-0.4|pyaw=45|pose_age=0.1");
    assert(parsed.valid && parsed.id == 1 && parsed.role == PlayerRole::Forward);
    assert(parsed.ball && parsed.active && !parsed.kick && std::abs(parsed.ballAge - 0.2) < 1e-9);
    assert(parsed.claim && parsed.healthy && std::abs(parsed.ballScore - 0.8) < 1e-9);
    assert(std::abs(parsed.ballDistance - 0.7) < 1e-9 && std::abs(parsed.ballX + 1.2) < 1e-9);
    assert(parsed.hasBallPosition && parsed.hasPose);
    assert(std::abs(parsed.ballMapAge - 0.2) < 1e-9);
    assert(std::abs(parsed.poseX - 0.6) < 1e-9 && std::abs(parsed.poseZ + 0.4) < 1e-9);
    assert(std::abs(parsed.poseYaw - 45.0) < 1e-9 && std::abs(parsed.poseAge - 0.1) < 1e-9);
    assert(!parseTeamStatus("other|id=1").valid);
    const auto diagnosticMap = parseTeamStatus(
        "cupcup|id=2|ball=0|mbx=.2|mbz=.3|mbage=.1|mbsource=teammate");
    assert(!diagnosticMap.ball && !diagnosticMap.hasBallPosition);
    const TeamStatus legacy = parseTeamStatus("cupcup|id=2|role=defender|ball=0");
    assert(legacy.valid && !legacy.hasPose && !legacy.hasBallPosition);
    const TeamStatus distinctAges = parseTeamStatus(
        "cupcup|id=2|role=defender|ball=1|bx=0.2|bz=0.1|age=0.01|bmap_age=0.8");
    assert(distinctAges.ballAge < 0.02 && distinctAges.ballMapAge == 0.8);
    const TeamStatus malformedPosition = parseTeamStatus(
        "cupcup|id=2|role=defender|ball=1|bx=nan|bz=0");
    assert(!malformedPosition.hasBallPosition);

    const auto sideBall = approachMotion(59.0, 0.06);
    const auto mirroredSideBall = approachMotion(-59.0, 0.06);
    assert(sideBall.forward > 0.0 && sideBall.forward < 0.01 && sideBall.turn == 4.0);
    assert(mirroredSideBall.forward == sideBall.forward && mirroredSideBall.turn == -4.0);
    assert(approachMotion(0.0, 0.02).forward == 0.05);
    assert(approachMotion(0.0, 0.04).forward == 0.04);
    assert(approachMotion(0.0, 0.06).forward == 0.032);
    assert(approachMotion(NAN, 0.06).forward == 0.0);
    assert(approachMotion(0.0, -1.0).turn == 0.0);

    assert(readyForAlignment(14.9, -21.9, 7.9, 53.0, 60.0, 0.051, 0.31));
    assert(!readyForAlignment(15.0, 0.0, 0.0, 60.0, 60.0, 0.06, 0.78));
    assert(!readyForAlignment(0.0, 0.0, 0.0, 52.0, 60.0, 0.06, 0.78));
    assert(!readyForAlignment(0.0, 0.0, 0.0, 60.0, 60.0, 0.12, 0.78));
    const double recentered = headRecenteringCommand(15.0, 0.41, 0.08, true);
    assert(recentered < 15.0 && recentered > 14.4);
    assert(headRecenteringCommand(15.0, 0.41, 0.04, true) == 15.0);
    assert(headRecenteringCommand(15.0, 0.71, 0.08, true) == 15.0);
    assert(!readyForAlignment(0.0, 0.0, 0.0, 60.0, 60.0, 0.06, 0.30));

    MatchLifecycle lifecycle;
    assert(lifecycle.observe(0, 0, 0.0) == LifecycleEvent::Reset);
    assert(lifecycle.observe(2, 0, 1.0) == LifecycleEvent::PlayStarted);
    assert(!lifecycle.canPlay(2, 1.1));
    assert(lifecycle.canPlay(2, 2.6));
    assert(lifecycle.observe(2, 1, 3.0) == LifecycleEvent::Restarted);
    assert(!lifecycle.canPlay(2, 3.1));
    assert(lifecycle.observe(3, 1, 4.0) == LifecycleEvent::Reset);

    BoundaryGuard redForward;
    auto decision = redForward.update(TeamColor::Red, PlayerRole::Forward, 1.70, 0.70);
    assert(!decision.stopWalking);
    decision = redForward.update(TeamColor::Red, PlayerRole::Forward, 1.90, 0.70);
    assert(decision.forwardBoundary && decision.stopWalking);
    decision = redForward.update(TeamColor::Red, PlayerRole::Forward, 1.78, 0.70);
    assert(decision.forwardBoundary);  // hysteresis holds at the noisy edge
    decision = redForward.update(TeamColor::Red, PlayerRole::Forward, 1.60, 0.70);
    assert(!decision.forwardBoundary);

    BoundaryGuard redForwardPenalty;
    decision = redForwardPenalty.update(TeamColor::Red, PlayerRole::Forward,
        3.00, 3.60, 0.75);
    assert(!decision.forwardBoundary);  // outside the buffered penalty rectangle
    decision = redForwardPenalty.update(TeamColor::Red, PlayerRole::Forward,
        3.00, 1.00, 0.75);
    assert(decision.forwardBoundary);
    decision = redForwardPenalty.update(TeamColor::Red, PlayerRole::Forward,
        3.30, 3.35, 0.75);
    assert(decision.forwardBoundary);  // hysteresis prevents boundary chatter
    decision = redForwardPenalty.update(TeamColor::Red, PlayerRole::Forward,
        3.30, 3.50, 0.75);
    assert(!decision.forwardBoundary);

    BoundaryGuard blueForwardPenalty;
    decision = blueForwardPenalty.update(TeamColor::Blue, PlayerRole::Forward,
        -3.00, -1.00, 0.75);
    assert(decision.forwardBoundary);
    decision = blueForwardPenalty.update(TeamColor::Blue, PlayerRole::Forward,
        -3.00, -3.60, 0.75);
    assert(!decision.forwardBoundary);

    BoundaryGuard blueDefender;
    decision = blueDefender.update(TeamColor::Blue, PlayerRole::Defender, -0.60, 0.70);
    assert(decision.defenderBoundary);
    decision = blueDefender.update(TeamColor::Blue, PlayerRole::Defender, -0.89, 0.70);
    assert(!decision.defenderBoundary);

    TacticalInput forwardInput;
    forwardInput.color = TeamColor::Red;
    forwardInput.id = 1;
    forwardInput.selfHealthy = true;
    forwardInput.selfBall = true;
    forwardInput.selfBallScore = 0.9;
    forwardInput.selfBallDistance = 0.8;
    forwardInput.selfBallAge = 0.1;
    forwardInput.selfBallX = -1.0;
    forwardInput.teammate = parsed;
    forwardInput.teammate.id = 2;
    forwardInput.teammateMessageAge = 0.2;
    const TacticalDecision forwardClaim = decideTactics(forwardInput);
    assert(forwardClaim.action == TacticalAction::Chase && forwardClaim.claim);

    TacticalInput defenderInput = forwardInput;
    defenderInput.id = 2;
    defenderInput.selfBall = true;
    defenderInput.selfBallScore = 0.7;
    defenderInput.selfBallDistance = 0.5;
    defenderInput.selfBallX = 1.0;
    defenderInput.teammate = TeamStatus();
    defenderInput.teammateMessageAge = 9.0;
    const TacticalDecision defenderClear = decideTactics(defenderInput);
    assert(defenderClear.action == TacticalAction::Clear && defenderClear.claim);

    TacticalInput supportInput = forwardInput;
    supportInput.selfBall = false;
    supportInput.teammate.claim = true;
    supportInput.teammate.ballX = 1.0;  // a legally reachable defender claim
    const TacticalDecision support = decideTactics(supportInput);
    assert(support.action == TacticalAction::Support && !support.claim);

    TacticalInput simultaneous = forwardInput;
    simultaneous.teammate.id = 2;
    simultaneous.teammate.claim = true;
    simultaneous.teammate.ball = true;
    simultaneous.teammate.ballScore = 0.88;
    simultaneous.teammate.ballDistance = 0.78;
    const TacticalDecision simultaneousForward = decideTactics(simultaneous);
    assert(simultaneousForward.claim);
    simultaneous.id = 2;
    simultaneous.teammate.id = 1;
    simultaneous.selfBallX = 1.0;
    const TacticalDecision simultaneousDefender = decideTactics(simultaneous);
    assert(!simultaneousDefender.claim && simultaneousDefender.action == TacticalAction::Defend);

    // An unreachable front player must not reserve a ball inside its own box.
    for (const auto color : {TeamColor::Red, TeamColor::Blue}) {
        const double sign = FieldGeometry::ownSign(color);
        TacticalInput forward = forwardInput;
        forward.color = color;
        forward.selfBallX = sign * 3.0;
        forward.selfBallZ = 0.0;
        forward.teammate = TeamStatus();
        assert(!decideTactics(forward).claim);
        assert(decideTactics(forward).action == TacticalAction::Support);
        TacticalInput defender = forward;
        defender.id = 2;
        defender.teammate.valid = defender.teammate.healthy = true;
        defender.teammate.id = 1;
        defender.teammate.claim = defender.teammate.ball = true;
        defender.teammate.hasBallPosition = true;
        defender.teammate.ballX = forward.selfBallX;
        defender.teammate.ballMapAge = 0.1;
        defender.teammateMessageAge = 0.1;
        assert(decideTactics(defender).claim);
        defender.selfBallX = -sign;
        assert(!decideTactics(defender).claim);
        assert(decideTactics(defender).action == TacticalAction::Defend);
    }
    assert(!ballWithinRoleReach(TeamColor::Red, 1, {NAN, 0}, 0.75));
    assert(!ballWithinRoleReach(TeamColor::Red, 1, {0, 0}, 0.75, NAN));
    assert(ballWithinRoleReach(TeamColor::Red, 1, {3, 2.45}, 0.0));
    assert(ballWithinRoleReach(TeamColor::Blue, 1, {-3, -2.45}, 0.0));
    assert(!ballWithinRoleReach(TeamColor::Red, 1, {3, 2.45}, 0.75));
    forwardInput.selfHealthy = false;
    assert(decideTactics(forwardInput).action == TacticalAction::Hold);
    assert(isBallAction(TacticalAction::Chase) && isBallAction(TacticalAction::Clear));
    assert(!isBallAction(TacticalAction::Hold) && !isBallAction(TacticalAction::Support) &&
        !isBallAction(TacticalAction::Defend));

    return 0;
}
