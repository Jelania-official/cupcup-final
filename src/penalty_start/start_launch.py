import os
import launch
from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import get_webots_home, controller_protocol, controller_ip_address
from ament_index_python.packages import get_package_share_directory, get_package_prefix


class ControllerLanucher(ExecuteProcess):
    def __init__(self, controller, output='screen', respawn=False, robot_name='', port='1234', **kwargs):
        webots_controller = (os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller'))

        protocol = controller_protocol()
        ip_address = controller_ip_address() if (protocol == 'tcp') else ''

        robot_name_option = [] if not robot_name else ['--robot-name=' + robot_name]
        ip_address_option = [] if not ip_address else ['--ip-address=' + ip_address]

        node_name = 'webots_controller' + (('_' + robot_name) if robot_name else '')
        parameters = [controller, robot_name]
        super().__init__(
            output=output,
            cmd=[
                webots_controller,
                *robot_name_option,
                ['--protocol=', protocol],
                *ip_address_option,
                ['--port=', port],
                *parameters,
            ],
            name=node_name,
            respawn=respawn,
            # Set WEBOTS_HOME to package directory to load correct controller library
            additional_env={'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')},
            **kwargs
        )

    def execute(self, context: LaunchContext):
        return super().execute(context)

    def _shutdown_process(self, context, *, send_sigint):
        return super()._shutdown_process(context, send_sigint=send_sigint)


def generate_launch_description():
    webots = WebotsLauncher(
        world=os.path.join(get_package_share_directory('webots'), 'models/worlds', 'penalty-robot.wbt')
    )
    webots_home = get_webots_home()
    os.environ['LD_LIBRARY_PATH'] = os.path.join(webots_home, 'lib', 'controller') + ':' + os.environ.get('LD_LIBRARY_PATH')
    ctrl_exe_path = os.path.join(get_package_prefix('controller'), 'lib/controller')
    red_1 = ControllerLanucher(os.path.join(ctrl_exe_path, 'controller'), robot_name='red_1')
    blue_1 = ControllerLanucher(os.path.join(ctrl_exe_path, 'controller'), robot_name='blue_1')
    supervisor = ControllerLanucher(os.path.join(ctrl_exe_path, 'supervisor'), robot_name='judge')
    return LaunchDescription([
        webots,
        red_1,
        blue_1,
        supervisor,
        Node(
            package='params',
            executable='params'
        ),
        Node(
            package='motion',
            executable='motion',
            arguments=['red_1'],
            output="screen"
        ),
        Node(
            package='motion',
            executable='motion',
            arguments=['blue_1'],
            output="screen"
        ),
        launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])