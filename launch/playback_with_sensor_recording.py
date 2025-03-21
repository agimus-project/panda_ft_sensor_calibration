from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_franka_launch,
    get_use_sim_time,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    PathJoinSubstitution,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = generate_include_franka_launch("franka_common_lfc.launch.py")

    pd_plus_trajectory_follower_params = PathJoinSubstitution(
        [
            FindPackageShare("panda_ft_sensor_calibration"),
            "config",
            "pd_plus_trajectory_follower_params.yaml",
        ]
    )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    pd_plus_trajectory_follower = Node(
        package="panda_ft_sensor_calibration",
        executable="pd_plus_trajectory_follower",
        parameters=[get_use_sim_time(), pd_plus_trajectory_follower_params],
        output="screen",
    )

    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[pd_plus_trajectory_follower],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        DeclareLaunchArgument(
            "move_to_initial_configuration",
            default_value="true",
            description="Move the robot to initial configuration.",
        ),
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)],
    )
