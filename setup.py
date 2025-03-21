from pathlib import Path
from typing import List

from generate_parameter_library_py.setup_helper import generate_parameter_module
from setuptools import setup

package_name = "panda_ft_sensor_calibration"
project_source_dir = Path(__file__).parent

module_name = "pd_plus_trajectory_follower_parameters"
yaml_file = "panda_ft_sensor_calibration/pd_plus_trajectory_follower_parameters.yaml"
generate_parameter_module(module_name, yaml_file)


def get_files(dir: Path, pattern: str) -> List[str]:
    return [x.as_posix() for x in (dir).glob(pattern) if x.is_file()]


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    package_dir={},
    install_requires=["setuptools"],
    zip_safe=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            f"share/{package_name}/launch",
            get_files(project_source_dir / "launch", "*.launch.py"),
        ),
    ],
    maintainer="Krzysztof Wojciechowski",
    maintainer_email="kwojciecho@laas.fr",
    description="ROS2 agimus_controller package",
    license="BSD",
    entry_points={
        "console_scripts": [
            "pd_plus_trajectory_follower = panda_ft_sensor_calibration.pd_plus_trajectory_follower:main",
        ],
    },
)
