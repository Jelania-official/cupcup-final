from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("cupcup")
    model = join(share, "models", "bitbots-2026", "opencv.onnx")
    team_name = LaunchConfiguration("team_name")
    # Load the actual shared parameter file for both dynamically named nodes.
    # Only model installation and explicit launch overrides belong here.
    common_parameters = [join(share, "config", "strategy.yaml"), {
        "ball_model": model,
        "ball_verifier_model": ParameterValue(
            LaunchConfiguration("ball_verifier_model"), value_type=str),
        "ball_pattern_filter": ParameterValue(
            LaunchConfiguration("ball_pattern_filter"), value_type=bool),
        "kick_settle_frames": ParameterValue(
            LaunchConfiguration("kick_settle_frames"), value_type=int),
    }]
    return LaunchDescription([
        DeclareLaunchArgument("team_name", default_value="cupcup"),
        DeclareLaunchArgument("ball_verifier_model", default_value=""),
        DeclareLaunchArgument("ball_pattern_filter", default_value="true"),
        DeclareLaunchArgument("kick_settle_frames", default_value="16"),
        Node(
            package="cupcup",
            executable="cupcup",
            arguments=[PythonExpression(["'", team_name, "' + '_1'"])],
            parameters=common_parameters,
            output="screen",
        ),
        Node(
            package="cupcup",
            executable="cupcup",
            arguments=[PythonExpression(["'", team_name, "' + '_2'"])],
            parameters=common_parameters,
            output="screen",
        ),
    ])
