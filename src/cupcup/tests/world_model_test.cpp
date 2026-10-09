#include "world_model.hpp"

#include <cassert>
#include <cmath>

int main()
{
    using namespace cupcup;

    assert(FieldGeometry::ownGoalX(TeamColor::Red) == 4.5);
    assert(FieldGeometry::attackGoalX(TeamColor::Red) == -4.5);
    assert(FieldGeometry::ownGoalX(TeamColor::Blue) == -4.5);
    assert(FieldGeometry::attackGoalX(TeamColor::Blue) == 4.5);
    assert(FieldGeometry::insideField({4.5, 3.0}));
    assert(!FieldGeometry::insideField({4.51, 0.0}));
    assert(FieldGeometry::insideField({4.0, 2.5}, 0.4));
    assert(!FieldGeometry::insideField({4.2, 0.0}, 0.4));
    assert(FieldGeometry::insideGoalMouth(1.3));
    assert(!FieldGeometry::insideGoalMouth(1.31));
    assert(!FieldGeometry::insideGoalMouth(1.0, 0.4));
    const FieldPoint redProjection = projectToField(0.0, 0.0, 0.0, 1.0, 0.0);
    const FieldPoint blueProjection = projectToField(0.0, 0.0, 180.0, 1.0, 0.0);
    assert(std::abs(redProjection.x - 1.0) < 1e-9 && std::abs(redProjection.z) < 1e-9);
    assert(std::abs(blueProjection.x + 1.0) < 1e-9 && std::abs(blueProjection.z) < 1e-9);
    const FieldPoint quarterTurn = projectToField(0.0, 0.0, 90.0, 1.0, 0.0);
    assert(std::abs(quarterTurn.x) < 1e-9 && std::abs(quarterTurn.z + 1.0) < 1e-9);
    WorldModel world;
    WorldModel sourcedBall;
    assert(sourcedBall.latestBallSource() == BallPositionSource::Unknown);
    assert(sourcedBall.updateBall(0, 0, 1, .4, BallPositionSource::Teammate));
    assert(!sourcedBall.updateBall(4, 0, 1.1, .9, BallPositionSource::GroundRay));
    assert(sourcedBall.latestBallSource() == BallPositionSource::Teammate);
    assert(sourcedBall.updateBall(.1, 0, 1.2, .9, BallPositionSource::GroundRay));
    assert(sourcedBall.latestBallSource() == BallPositionSource::GroundRay);
    assert(!sourcedBall.updateBall(0, 0, .9, .8, BallPositionSource::Radius));
    assert(sourcedBall.latestBallSource() == BallPositionSource::GroundRay);
    sourcedBall.reset();
    assert(sourcedBall.latestBallSource() == BallPositionSource::Unknown);
    assert(world.updateSelf(0.0, 0.0, 179.0, 1.0));
    assert(world.updateSelf(0.2, 0.1, -179.0, 1.1));
    assert(std::abs(world.self().x - 0.08) < 1e-9);
    assert(std::abs(world.self().yaw - 179.8) < 1e-9);
    assert(world.self().fresh(2.0, 2.0));
    assert(!world.updateSelf(4.0, 0.0, 0.0, 1.2));  // reject a localization jump
    assert(std::abs(world.self().x - 0.08) < 1e-9);
    assert(!world.updateSelf(0.0, 0.0, 0.0, 1.0));  // reject time reversal

    WorldModel filteredPose;
    assert(filteredPose.updateSelf(1.1, 0.0, 0.0, 0.0));
    assert(filteredPose.updateSelf(1.9, 0.0, 0.0, 0.1));
    assert(filteredPose.updateSelf(0.3, 0.0, 0.0, 0.2));
    assert(filteredPose.updateSelf(1.9, 0.0, 0.0, 0.3));
    assert(filteredPose.updateSelf(0.3, 0.0, 0.0, 0.4));
    assert(std::abs(filteredPose.self().x - 1.1) < 0.30);

    assert(world.updateTeammate(-1.0, 0.5, 0.0, 1.2, 0.8));
    assert(world.teammate().fresh(2.0, 1.2));
    assert(world.updateBall(1.0, -0.5, 1.3, 0.9));
    assert(world.updateBall(1.2, -0.5, 1.4, 0.7));
    assert(std::abs(world.ball().x - 1.11) < 1e-9);
    assert(world.ball().fresh(1.8, 0.65));

    assert(world.updateRobotCandidate(0.0, -1.0, 1.4, 0.9));
    assert(world.robotCandidate().position.confidence <= 0.25);
    assert(!world.robotCandidate().position.fresh(2.1, 0.65));

    WorldModel sources;
    sources.updateRobots({{{1, 0}, RobotTeam::Red, .9, .01,
                          RobotPositionSource::NumberSquare}}, 1);
    assert(sources.robotCandidate().latestSource == RobotPositionSource::NumberSquare);
    assert(sources.robotCandidate().position.confidence <= .25);
    assert(sources.robotCandidate().uncertainty >= .5);
    sources.updateRobots({{{1, 0}, RobotTeam::Red, .9, 1,
                          RobotPositionSource::BoxHeight}}, 1);
    assert(sources.robotCandidate().latestSource == RobotPositionSource::NumberSquare);
    sources.updateRobots({{{1.1, 0}, RobotTeam::Red, .9, 1,
                          RobotPositionSource::BoxHeight}}, 1.1);
    assert(sources.robots().size() == 1);
    assert(sources.robotCandidate().latestSource == RobotPositionSource::BoxHeight);
    assert(sources.robotCandidate().team == RobotTeam::Red);

    world.expire(2.6);
    assert(!world.ball().valid);
    assert(!world.teammate().valid);
    assert(world.self().valid);
    world.reset();
    assert(!world.self().valid && !world.teammate().valid && !world.ball().valid);
    assert(!world.robotCandidate().position.valid);
    return 0;
}
