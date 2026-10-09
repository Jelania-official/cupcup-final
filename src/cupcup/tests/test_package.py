#!/usr/bin/env python3
"""Fast, dependency-free contract checks for the independent finals package."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src" / "cupcup"


def main() -> int:
    required = [
        PACKAGE / "package.xml",
        PACKAGE / "CMakeLists.txt",
        PACKAGE / "src" / "player.cpp",
        PACKAGE / "launch" / "player_launch.py",
        PACKAGE / "config" / "strategy.yaml",
        PACKAGE / "tests" / "mock_gamectrl.py",
        PACKAGE / "src" / "ball_tracker.hpp",
        PACKAGE / "src" / "ball_candidate_verifier.hpp",
        PACKAGE / "tests" / "ball_tracker_test.cpp",
        PACKAGE / "src" / "strategy_logic.hpp",
        PACKAGE / "src" / "world_model.hpp",
        PACKAGE / "tests" / "strategy_logic_test.cpp",
        PACKAGE / "tests" / "world_model_test.cpp",
        PACKAGE / "tests" / "analyze_world_trace.py",
        PACKAGE / "tests" / "world_trace_test.py",
        PACKAGE / "models" / "bitbots-2026" / "conversion_validation.json",
        PACKAGE / "tests" / "TEST_REPORT.md",
        ROOT / "docs" / "WORLD_MODEL.md",
        ROOT / "docs" / "TACTICAL_SIMULATION.md",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise SystemExit("missing package files: " + ", ".join(missing))

    source = (PACKAGE / "src" / "player.cpp").read_text()
    forbidden = "KidsizeRobot-Cupcup-team-dev"
    if forbidden in source:
        raise SystemExit("finals package contains an initial-round runtime path")

    launch = (PACKAGE / "launch" / "player_launch.py").read_text()
    runner = (PACKAGE / "tests" / "run_match.py").read_text()
    opponent_script = (PACKAGE / "tests" / "scripted_opponent.py").read_text()
    supervisor_path = ROOT / "src" / "simulation" / "controller" / "src" / "supervisor.cpp"
    supervisor = supervisor_path.read_text()
    dynamic_team = 'DeclareLaunchArgument("team_name"' in launch
    both_robots = "'_1'" in launch and "'_2'" in launch
    if not dynamic_team or not both_robots:
        raise SystemExit("launch file does not start both finals robots")

    teams = (ROOT / "teams.cfg").read_text().split()
    if "cupcup" not in teams:
        raise SystemExit("cupcup is not registered in the current finals teams.cfg")

    checks = {
        "shared kick direction is not biased a second time": (
            "const bool planned = hasSharedKickTarget(nowSeconds())" in source
            and "(planned ? 0.0 : shotYawOffset_)" in source
            and "if (hasSharedKickTarget(nowSeconds()))" in source
        ),
        "robot map fallback requires fresh receipt pose": (
            "const bool robotPoseUsable = self.fresh(time, 2.0) && imageImuFall_ == 0" in source
            and "cupcup::usableReceiptPose(imageHeadAge_, imageImuAge_)" in source
            and "const bool markerGeometryUsable = robotPoseUsable" in source
            and ("if (robotPoseUsable && !frame_.empty()) "
                 "for (const auto &robot : robotDetections_)") in source
        ),
        "normal 2v2 role split": "input.id = id_" in source and "runDefender" in source,
        "kick pulse": 'body.actname = leftFoot_ ? "left_kick" : "right_kick"' in source,
        "teammate communication": "|active=" in source and "Talk" in source,
        "safety guards": (
            "forwardBoundaryMargin_" in source
            and "defenderBoundaryMargin_" in source
        ),
        "model fallback": "detectTraditional" in source,
        "optional verifier defaults off and restores baseline on failure": (
            'declare_parameter<std::string>("ball_verifier_model", "")' in source
            and 'DeclareLaunchArgument("ball_verifier_model", default_value="")' in launch
            and "ballVerifier_.disable()" in source
            and "baseline restored" in source
        ),
        "validated model threshold": '"ball_min_score", 0.30' in source,
        "active ball confirmation and stale-observation rejection": (
            "BallTracker" not in source
            and "ballConfirmHits_" in source
            and "confirmationGate" in source
            and "time - ballSeenAt_ > 1.0" in source
        ),
        "typed confirmation parameter": 'declare_parameter<int>("ball_confirm_hits"' in source,
        "fresh-frame stop-to-kick preparation": (
            'declare_parameter<int>("kick_settle_frames"' in source
            and "settledForKick(stateAge, stableFrames_, kickSettleFrames_)" in source
            and "cupcup::updateKickAlignment(" in source
            and "freshImage_, linedUp, holdPose, stableFrames_, missedAlignmentFrames_" in source
            and "else if (linedUp &&" in source
        ),
        "game link timeout": 'declare_parameter<double>("game_timeout"' in source,
        "validated motion timing": (
            'declare_parameter<double>("approach_timeout"' in source
            and 'declare_parameter<double>("orbit_timeout"' in source
            and "targetYaw_" in source
        ),
        "active head reacquisition": "active-head-search pattern" in source,
        "role-aware talk": "|role=" in source and "|age=" in source,
        "lifecycle recovery": "lifecycle_.observe" in source and "const int score" in source,
        "structured teammate arbitration": "parseTeamStatus" in source and "planMatch" in source,
        "single kick execution path": (
            "defenderClearIssued_" not in source and "isBallAction" in source
        ),
        "shared match policy": "planMatch" in source and "MatchState" in source,
        "phase-separated orbit and align gates": "readyForAlignment" in source,
        "world ball communication": "ballDistanceEstimate" in source and "|bx=" in source,
        "separate visual/map observation ages": (
            "bmap_age" in source
            and "ballMapAge" in source
            and "teammateStatus_.ballMapAge <= 0.45" in source
        ),
        "trace-only camera geometry diagnostics": (
            'std::getenv("CUPCUP_TRACE_PATH")' in source
            and '"|iu="' in source
            and '"|ihy="' in source
            and '"|iiy="' in source
            and '"|fa="' in source
            and "frame_.cols" in source
            and "frame_.rows" in source
        ),
        "defensive anchor": "defenderAnchorGain_" in source and "navigateTo" in source,
        "obstacle avoidance": "obstacleAhead" in source and "side" in source,
        "boundary hysteresis": "BoundaryGuard" in source and "reset()" in source,
        "shared time-aware world model": (
            "WorldModel worldModel_" in source
            and "updateWorldModel(time" in source
            and "world_model.hpp" in source
        ),
        "teammate pose exchange": "|pyaw=" in source and "pose_age" in source,
        "unknown robot estimate is non-tactical": (
            "updateRobots" in source
            and "struct RobotCandidate" in (PACKAGE / "src" / "world_model.hpp").read_text()
        ),
        "filtered pose guards rectangular penalty area": (
            "selfPosition(nowSeconds())" in source
            and "std::abs(z) <= enterZ" in (PACKAGE / "src" / "strategy_logic.hpp").read_text()
        ),
        "repeatable simulation noise seed": (
            "CUPCUP_SIM_SEED" in runner and "CUPCUP_SIM_SEED" in supervisor
        ),
        "opt-in truth/estimate trace is offline-only": (
            '"--trace"' in runner
            and "CUPCUP_TRACE_PATH" in runner
            and "CUPCUP_TRACE_PATH" in supervisor
            and "TalkCapture" in supervisor
            and "create_publisher<common::msg::Talk>" not in supervisor
        ),
        "multiple regression opponent styles": all(
            style in runner for style in ("unirobot", "cupcup", "rush", "wall", "keeper")
        ),
        "clean scripted opponent shutdown": "ExternalShutdownException" in opponent_script,
    }
    failed = [name for name, ok in checks.items() if not ok]
    if failed:
        raise SystemExit("failed checks: " + ", ".join(failed))

    for name in checks:
        print("PASS", name)
    print("PASS independent cupcup package contract")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
