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
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    joint_trajectory_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("panda_ft_sensor_calibration"),
            "config",
            "joint_trajectory_controller_params.yaml",
        ]
    )

    joint_trajectory_controller_names = [
        "joint_trajectory_controller",
    ]

    franka_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka_common.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "arm_id": LaunchConfiguration("arm_id"),
            "aux_computer_ip": LaunchConfiguration("aux_computer_ip"),
            "aux_computer_user": LaunchConfiguration("aux_computer_user"),
            "on_aux_computer": LaunchConfiguration("on_aux_computer"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "disable_collision_safety": LaunchConfiguration("disable_collision_safety"),
            "external_controllers_params": joint_trajectory_controller_params,
            "external_controllers_names": str(joint_trajectory_controller_names),
            "use_gazebo": LaunchConfiguration("use_gazebo"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "gz_verbose": LaunchConfiguration("gz_verbose"),
            "gz_headless": LaunchConfiguration("gz_headless"),
        }.items(),
    )

    reference_pose_publisher_params = PathJoinSubstitution(
        [
            FindPackageShare("panda_ft_sensor_calibration"),
            "config",
            "reference_pose_publisher_params.yaml",
        ]
    )

    reference_pose_publisher = Node(
        package="panda_ft_sensor_calibration",
        executable="reference_pose_publisher",
        parameters=[
            get_use_sim_time(),
            reference_pose_publisher_params,
        ],
        output="screen",
    )

    record_rosbag_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-s",
            "mcap",
            "/franka/joint_states",
            "/moving_to_new_pose",
            "/pose_reached",
            "/force_torque_sensor_broadcaster/wrench",
        ],
        output="screen",
    )

    return [
        franka_robot_launch,
        reference_pose_publisher,
        RegisterEventHandler(
            event_handler=OnProcessIO(
                target_action=reference_pose_publisher,
                # log info is directed to stderr
                on_stderr=lambda event: (
                    None
                    if "All data received. Moving to first target..."
                    not in event.text.decode().strip()
                    else [record_rosbag_process]
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
            description="Path to which rosbags will be recorded. "
            + "Defaults to a timestamped folder in the current directory.",
        ),
        DeclareLaunchArgument(
            "input_rosbag_path",
            default_value="",
            description="Path from which rosbag containing desired motion will be played back.",
        ),
        DeclareLaunchArgument(
            "rosbag_replay_rate",
            default_value="1.0",
            description="Rate at which the rosbag is played.",
        ),
        DeclareLaunchArgument(
            "replay_delay",
            default_value="3.0",
            description="Delay in seconds after reaching desired configuration "
            + "to start replaying rosbag. If less than 1.0 second, will default to 1.0",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="false",
            description="Perform a 'dry run' without recording a rosbag.",
        ),
    ]
    return LaunchDescription(
        declared_arguments
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)],
    )
