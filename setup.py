from pathlib import Path
from typing import List

from generate_parameter_library_py.setup_helper import generate_parameter_module
from setuptools import setup

package_name = "panda_ft_sensor_calibration"
project_source_dir = Path(__file__).parent

module_name = "pd_plus_trajectory_follower_parameters"
yaml_file = f"{package_name}/pd_plus_trajectory_follower_parameters.yaml"
generate_parameter_module(module_name, yaml_file)

module_name = "reference_pose_publisher_parameters"
yaml_file = f"{package_name}/reference_pose_publisher_parameters.yaml"
validation_module = f"{package_name}.custom_validation"
generate_parameter_module(module_name, yaml_file, validation_module)


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
        (
            "share/ament_index/resource_index/packages/resource",
            get_files(project_source_dir / "resource", "*"),
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            f"share/{package_name}/launch",
            get_files(project_source_dir / "launch", "*.launch.py"),
        ),
        (
            f"share/{package_name}/config",
            get_files(project_source_dir / "config", "*"),
        ),
    ],
    maintainer="Guilhem Saurel",
    maintainer_email="guilhem.saurel@laas.fr",
    description="Force-Torque sensor calibration procedures for Panda robot used in Agimus project",
    license="BSD",
    entry_points={
        "console_scripts": [
            "pd_plus_trajectory_follower = panda_ft_sensor_calibration.pd_plus_trajectory_follower:main",
            "reference_pose_publisher = panda_ft_sensor_calibration.reference_pose_publisher:main",
        ],
    },
)
