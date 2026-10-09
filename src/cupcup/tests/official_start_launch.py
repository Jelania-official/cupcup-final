"""
Test-only adapter for the released launcher's local Webots ABI mismatch.

Reuse the installed original launch except for its controller-library root.
An explicit CUPCUP_TRACE_JUDGE may replace only the judge executable with the
read-only step instrumentation built around the untouched released source.
"""

import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def load_released_module():
    """Load one private launch module with only the local library-root fix."""
    source = Path(get_package_share_directory("start")) / "start_launch.py"
    spec = importlib.util.spec_from_file_location("cupcup_released_start", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original_prefix = module.get_package_prefix
    module.get_package_prefix = lambda name: (
        module.get_webots_home() if name == "webots_ros2_driver"
        else original_prefix(name)
    )
    return module


def generate_launch_description():
    module = load_released_module()
    binary = os.environ.get("CUPCUP_TRACE_JUDGE")
    if binary:
        original_controller = module.ControllerLanucher

        def controller(executable, **kwargs):
            if kwargs.get("robot_name") == "judge":
                executable = binary
            return original_controller(executable, **kwargs)

        module.ControllerLanucher = controller
    return module.generate_launch_description()
