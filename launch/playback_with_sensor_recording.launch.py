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
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    move_to_initial_configuration = LaunchConfiguration("move_to_initial_configuration")
    output_rosbag_path = LaunchConfiguration("output_rosbag_path")
    input_rosbag_path = LaunchConfiguration("input_rosbag_path")
    rosbag_replay_rate = LaunchConfiguration("rosbag_replay_rate")
    dry_run = LaunchConfiguration("dry_run")
    replay_delay = LaunchConfiguration("replay_delay")
    replay_delay_float = float(context.perform_substitution(replay_delay))
    # Set delay to at least one second
    replay_delay_float = max(replay_delay_float, 1.0)

    franka_robot_launch = generate_include_franka_launch("franka_common_lfc.launch.py")

    pd_plus_trajectory_follower_params = PathJoinSubstitution(
        [
            FindPackageShare("panda_ft_sensor_calibration"),
            "config",
            "pd_plus_trajectory_follower_replay_params.yaml",
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
                "recording_mode": False,
                "move_to_initial_configuration": move_to_initial_configuration,
            },
        ],
        output="screen",
    )

    record_rosbag_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            output_rosbag_path,
            "/control",
            "/sensor",
            "/robot_description",
            "/franka/joint_states",
            "/gripper/joint_states",
        ],
        output="screen",
        condition=UnlessCondition(dry_run),
    )

    playback_rosbag_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            input_rosbag_path,
            "-r",
            rosbag_replay_rate,
            "-s",
            "mcap",
            "--read-ahead-queue-size",
            "1000",
            "--remap",
            "/sensor/throttled:=/reference/sensor",
            "--wait-for-all-acked",
            "10",
        ],
        output="screen",
    )

    on_configuration_reached_actions = [
        LogInfo(msg=f"Starting to replay rosbag in {replay_delay_float:.2f} seconds."),
        TimerAction(period=replay_delay_float, actions=[playback_rosbag_process]),
        # Start recording rosbag 0.2 second before playing the trajectory
        TimerAction(period=replay_delay_float - 0.2, actions=[record_rosbag_process]),
    ]

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
                # log info is directed to stderr
                on_stderr=lambda event: (
                    None
                    if "Initial configuration reached."
                    not in event.text.decode().strip()
                    else on_configuration_reached_actions
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
