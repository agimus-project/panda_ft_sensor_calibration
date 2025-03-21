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
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    output_rosbag_path = LaunchConfiguration("output_rosbag_path")
    move_to_initial_configuration = LaunchConfiguration("move_to_initial_configuration")

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
        parameters=[
            get_use_sim_time(),
            pd_plus_trajectory_follower_params,
            {
                "recording_mode": True,
                "move_to_initial_configuration": move_to_initial_configuration,
            },
        ],
        output="screen",
    )

    record_rosbag_process = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", output_rosbag_path, "/sensor"],
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
        RegisterEventHandler(
            event_handler=OnProcessIO(
                target_action=pd_plus_trajectory_follower,
                on_stdout=lambda event: (
                    None
                    if "Initial configuration reached."
                    not in event.text.decode().strip()
                    else record_rosbag_process
                ),
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "move_to_initial_configuration",
            default_value="true",
            description="Move the robot to initial configuration.",
        ),
        DeclareLaunchArgument(
            "output_rosbag_path",
            default_value="",
            description="Path to which rosbags will be recorded.  Defaults to a timestamped folder in the current directory.",
        ),
    ]
    return LaunchDescription(
        declared_arguments
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)],
    )
