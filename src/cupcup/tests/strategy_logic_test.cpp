#include "strategy_logic.hpp"

#include <cassert>
#include <cmath>

int main()
{
    using namespace cupcup;

    const TeamStatus parsed = parseTeamStatus(
        "cupcup|id=1|role=forward|state=ALIGN|ball=1|active=1|kick=0|claim=1|healthy=1|conf=0.8|bdist=0.7|bx=-1.2|bz=0.4|age=0.2");
    assert(parsed.valid && parsed.id == 1 && parsed.role == PlayerRole::Forward);
    assert(parsed.ball && parsed.active && !parsed.kick && std::abs(parsed.ballAge - 0.2) < 1e-9);
    assert(parsed.claim && parsed.healthy && std::abs(parsed.ballScore - 0.8) < 1e-9);
    assert(std::abs(parsed.ballDistance - 0.7) < 1e-9 && std::abs(parsed.ballX + 1.2) < 1e-9);
    assert(!parseTeamStatus("other|id=1").valid);

    const TeammateDecision busy = arbitrate(parsed, 0.2);
    assert(busy.messageFresh && busy.forwardBusy && !busy.allowDefenderClear);
    const TeammateDecision stale = arbitrate(parsed, 2.0);
    assert(!stale.messageFresh && stale.allowDefenderClear);

    MatchLifecycle lifecycle;
    assert(lifecycle.observe(0, 0, 0.0) == LifecycleEvent::Reset);
    assert(lifecycle.observe(2, 0, 1.0) == LifecycleEvent::PlayStarted);
    assert(!lifecycle.canPlay(2, 1.1));
    assert(lifecycle.canPlay(2, 2.6));
    assert(lifecycle.observe(2, 1, 3.0) == LifecycleEvent::Restarted);
    assert(!lifecycle.canPlay(2, 3.1));
    assert(lifecycle.observe(3, 1, 4.0) == LifecycleEvent::Reset);

    BoundaryGuard redForward;
    auto decision = redForward.update(TeamColor::Red, PlayerRole::Forward, 2.80, 0.70);
    assert(!decision.stopWalking);
    decision = redForward.update(TeamColor::Red, PlayerRole::Forward, 2.90, 0.70);
    assert(decision.forwardBoundary && decision.stopWalking);
    decision = redForward.update(TeamColor::Red, PlayerRole::Forward, 2.78, 0.70);
    assert(decision.forwardBoundary);  // hysteresis holds at the noisy edge
    decision = redForward.update(TeamColor::Red, PlayerRole::Forward, 2.60, 0.70);
    assert(!decision.forwardBoundary);

    BoundaryGuard blueDefender;
    decision = blueDefender.update(TeamColor::Blue, PlayerRole::Defender, -0.60, 0.70);
    assert(decision.defenderBoundary);
    decision = blueDefender.update(TeamColor::Blue, PlayerRole::Defender, -0.89, 0.70);
    assert(!decision.defenderBoundary);

    DefenderClearInput clear;
    clear.color = TeamColor::Red;
    clear.ballVisible = true;
    clear.locationFresh = true;
    clear.locationX = 1.10;
    clear.ballRadius = 0.08;
    clear.bearing = 12.0;
    clear.headingError = 10.0;
    assert(shouldDefenderClear(clear));
    clear.forwardBusy = true;
    assert(!shouldDefenderClear(clear));
    clear.forwardBusy = false;
    clear.locationX = 0.60;
    assert(!shouldDefenderClear(clear));
    clear.locationX = 1.10;
    clear.bearing = 35.0;
    assert(!shouldDefenderClear(clear));

    clear = DefenderClearInput();
    clear.color = TeamColor::Blue;
    clear.ballVisible = true;
    clear.locationFresh = true;
    clear.locationX = -1.10;
    clear.ballRadius = 0.08;
    clear.bearing = -12.0;
    clear.headingError = -10.0;
    assert(shouldDefenderClear(clear));

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

    return 0;
}
