from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    share = get_package_share_directory("cupcup")
    model = join(share, "models", "bitbots-2026", "opencv.onnx")
    team_name = LaunchConfiguration("team_name")
    common_parameters = {
        "ball_model": model,
        "ball_min_score": 0.30,
        "left_kick_x": 0.399,
        "right_kick_x": 0.575,
        "kick_y": 0.78,
        "kick_pitch": 60.0,
        "forward_boundary_margin": 0.75,
        "defender_boundary_margin": 0.75,
        "ball_track_innovation_gate": 0.30,
        "ball_track_timeout": 0.60,
        "ball_confirm_hits": 2,
        "restart_grace": 1.50,
        "defender_home_x": 1.55,
        "defender_home_z": 0.0,
        "defender_anchor_gain": 0.45,
        "support_x": 0.40,
        "claim_stale_after": 1.20,
        "takeover_after": 2.50,
        "defender_clear_enabled": True,
        "defender_clear_radius": 0.075,
        "defender_clear_bearing": 28.0,
        "defender_clear_heading": 18.0,
        "defender_clear_stable_frames": 3,
        "defender_clear_cooldown": 4.0,
        "kick_action_pulse": 1.0,
        "game_timeout": 2.50,
        "approach_timeout": 60.0,
        "orbit_timeout": 25.0,
        "settle_seconds": 0.08,
        "align_stable_frames": 3,
    }
    return LaunchDescription([
        DeclareLaunchArgument("team_name", default_value="cupcup"),
        Node(
            package="cupcup",
            executable="cupcup",
            arguments=[PythonExpression(["'", team_name, "' + '_1'"])],
            parameters=[common_parameters],
            output="screen",
        ),
        Node(
            package="cupcup",
            executable="cupcup",
            arguments=[PythonExpression(["'", team_name, "' + '_2'"])],
            parameters=[common_parameters],
            output="screen",
        ),
    ])
