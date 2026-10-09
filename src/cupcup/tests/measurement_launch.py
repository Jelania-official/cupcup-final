"""Launch the released world with a measurement-only judge, not a match referee."""

import importlib.util
import os
from pathlib import Path


def generate_launch_description():
    spec = importlib.util.spec_from_file_location(
        "cupcup_measure_base", Path(__file__).with_name("official_start_launch.py"))
    base = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(base)
    module = base.load_released_module()
    binary = os.environ["CUPCUP_MEASUREMENT_JUDGE"]
    original_controller = module.ControllerLanucher
    original_webots = module.WebotsLauncher

    def controller(executable, **kwargs):
        if kwargs.get("robot_name") == "judge":
            executable = binary
        elif os.environ.get("CUPCUP_MEASUREMENT_CONTROLLER"):
            executable = os.environ["CUPCUP_MEASUREMENT_CONTROLLER"]
        return original_controller(executable, **kwargs)

    module.ControllerLanucher = controller
    module.WebotsLauncher = lambda **kwargs: original_webots(mode="fast", **kwargs)
    return module.generate_launch_description()
