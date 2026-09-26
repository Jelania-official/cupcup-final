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
        PACKAGE / "tests" / "ball_tracker_test.cpp",
        PACKAGE / "src" / "strategy_logic.hpp",
        PACKAGE / "tests" / "strategy_logic_test.cpp",
        PACKAGE / "models" / "bitbots-2026" / "conversion_validation.json",
        PACKAGE / "tests" / "TEST_REPORT.md",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise SystemExit("missing package files: " + ", ".join(missing))

    source = (PACKAGE / "src" / "player.cpp").read_text()
    forbidden = "KidsizeRobot-Cupcup-team-dev"
    if forbidden in source:
        raise SystemExit("finals package contains an initial-round runtime path")

    launch = (PACKAGE / "launch" / "player_launch.py").read_text()
    dynamic_team = 'DeclareLaunchArgument("team_name"' in launch
    both_robots = "'_1'" in launch and "'_2'" in launch
    if not dynamic_team or not both_robots:
        raise SystemExit("launch file does not start both finals robots")

    teams = (ROOT / "teams.cfg").read_text().split()
    if "cupcup" not in teams:
        raise SystemExit("cupcup is not registered in the current finals teams.cfg")

    checks = {
        "normal 2v2 role split": "if (id_ == 1)" in source and "runDefender" in source,
        "kick pulse": 'body.actname = leftFoot_ ? "left_kick" : "right_kick"' in source,
        "teammate communication": "|active=" in source and "Talk" in source,
        "safety guards": (
            "forwardBoundaryMargin_" in source
            and "defenderBoundaryMargin_" in source
        ),
        "model fallback": "detectTraditional" in source,
        "validated model threshold": '"ball_min_score", 0.30' in source,
        "temporal ball tracking": (
            "BallTracker" in source
            and "ball_track_innovation_gate" in source
            and "ball_track_timeout" in source
            and "ballConfirmHits_" in source
        ),
        "typed confirmation parameter": 'declare_parameter<int>("ball_confirm_hits"' in source,
        "game link timeout": 'declare_parameter<double>("game_timeout"' in source,
        "validated motion timing": (
            'declare_parameter<double>("approach_timeout"' in source
            and 'declare_parameter<double>("orbit_timeout"' in source
            and "targetYaw_" in source
        ),
        "active head reacquisition": "active-head-search pattern" in source,
        "role-aware talk": "|role=" in source and "|age=" in source,
        "lifecycle recovery": "lifecycle_.observe" in source and "const int score" in source,
        "structured teammate arbitration": "parseTeamStatus" in source and "arbitrate" in source,
        "dynamic tactical arbitration": "decideTactics" in source and "TacticalAction" in source,
        "world ball communication": "ballDistanceEstimate" in source and "|bx=" in source,
        "defensive anchor": "defenderAnchorGain_" in source and "navigateTo" in source,
        "obstacle avoidance": "obstacleAhead" in source and "side" in source,
        "boundary hysteresis": "BoundaryGuard" in source and "reset()" in source,
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
