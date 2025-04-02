from typing import Union

import numpy as np
import pinocchio as pin
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.qos_overriding_options import QoSOverridingOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Automatically generated file
from panda_ft_sensor_calibration.reference_pose_publisher_parameters import (
    reference_pose_publisher,
)  # noqa: E402


class ReferencePosePublisher(Node):
    def __init__(self):
        super().__init__("reference_pose_publisher")

        try:
            self._param_listener = reference_pose_publisher.ParamListener(self)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        jtc_qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            qos_profile=jtc_qos_profile,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self._new_pose_pub = self.create_publisher(Empty, "/moving_to_new_pose", 10)

        self._pose_reached_pub = self.create_publisher(Empty, "/pose_reached", 10)

        self._joint_states_sub = self.create_subscription(
            JointState,
            "/franka/joint_states",
            self._joint_states_cb,
            qos_profile=jtc_qos_profile,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self._robot_description_sub = self.create_subscription(
            String,
            "/robot_description",
            self._robot_description_cb,
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        self._robot_model: Union[pin.Model, None] = None
        self._robot_data: Union[pin.Data, None] = None
        self._joint_state_msg: Union[JointState, None] = None

        self._pose_reached = True
        self._pose_reached_stamp = self.get_clock().now()
        self._tcp_frame_id = -1
        self._base_frame_id = -1
        self._pose_cnt = 0
        self._is_intermittent = False
        self._pose_tolerance = self._params.configuration_tolerance_pose
        self._rot_tolerance = self._params.configuration_tolerance_rot

        self._target_pose: Union[pin.SE3, None] = None

        self._dt = 1.0 / self._params.controller_frequency
        self._control_timer = self.create_timer(self._dt, self._control_timer_callback)
        self._task_timer = self.create_timer(1.0 / 10.0, self._task_timer_callback)

        self.get_logger().info("Node started.")

    def _robot_description_cb(self, msg: String) -> None:
        """Callback called when robot description was received. URDF is parsed by pinocchio
            and robot model and robot data structures are created.

        Args:
            msg (std_msgs.msg.String): Message containing robot description.
        """
        robot_model_full = pin.buildModelFromXML(msg.data)
        locked_joint_names = [
            name
            for name in robot_model_full.names
            if name not in self._params.moving_joint_names and name != "universe"
        ]

        locked_joint_ids = [
            robot_model_full.getJointId(name) for name in locked_joint_names
        ]
        self._robot_model = pin.buildReducedModel(
            robot_model_full,
            list_of_geom_models=[],
            list_of_joints_to_lock=locked_joint_ids,
            reference_configuration=np.zeros(robot_model_full.nq),
        )[0]
        self._robot_data = self._robot_model.createData()

        self._tcp_frame_id = self._robot_model.getFrameId(self._params.tcp_frame_id)
        self._base_frame_id = self._robot_model.getFrameId(self._params.base_frame_id)
        self.get_logger().info("Robot description received.")

    def _joint_states_cb(self, msg: JointState) -> None:
        """Callback called on every joint state message received.

        Args:
            msg (sensor_msgs.msg.JointState): Message containing
                joint states of the robot.
        """
        self._joint_state_msg = msg

    def _task_timer_callback(self) -> None:
        """Periodically called callback used to set new goals for the robot."""
        if self._pose_cnt >= len(self._params.poses_to_reach_names):
            self.get_logger().info(f"All poses reached.", once=True)
            return

        if self._pose_reached and (
            self.get_clock().now() - self._pose_reached_stamp
        ) > Duration(seconds=self._params.settle_time):
            pose_name = self._params.poses_to_reach_names[self._pose_cnt]
            self._target_pose = pin.SE3(
                pin.Quaternion(np.array(self._params.get_entry(pose_name).quat)),
                np.array(self._params.get_entry(pose_name).xyz),
            )
            self._is_intermittent = self._params.get_entry(pose_name).intermittent
            if self._is_intermittent:
                self._pose_tolerance = (
                    self._params.intermittent_configuration_tolerance_pose
                )
                self._rot_tolerance = (
                    self._params.intermittent_configuration_tolerance_rot
                )
            else:
                self._pose_tolerance = self._params.configuration_tolerance_pose
                self._rot_tolerance = self._params.configuration_tolerance_rot

            self.get_logger().info(f"Reaching pose '{pose_name}'...")
            self._new_pose_pub.publish(Empty())
            self._pose_cnt += 1
            self._inner_iters = 1
            self._pose_reached = False

    def _control_timer_callback(self) -> None:
        """Periodically called callback used to compute new control signal for the robot."""

        if self._robot_model is None:
            self.get_logger().info(
                "Waiting for the Robot model...", throttle_duration_sec=2.0
            )
            return
        if self._joint_state_msg is None:
            self.get_logger().info(
                "Waiting for the first joint state message...",
                throttle_duration_sec=2.0,
            )
            return

        if self._target_pose is None:
            self.get_logger().info(
                "Waiting for the first pose to be set...", throttle_duration_sec=2.0
            )
            return

        self.get_logger().info(
            "All data received. Moving to first target...", once=True
        )

        joint_map = {
            name: position
            for name, position in zip(
                self._joint_state_msg.name,
                self._joint_state_msg.position,
            )
        }
        q0 = np.array([joint_map[name] for name in self._params.moving_joint_names])
        q = np.copy(q0)

        trajectory_points = []
        for i in range(self._params.n_trajectory_points):
            for j in range(int(200 // (i + 1)) + 1):
                pin.framesForwardKinematics(self._robot_model, self._robot_data, q)

                pin.updateFramePlacement(
                    self._robot_model, self._robot_data, self._tcp_frame_id
                )

                rMf = (
                    self._robot_data.oMf[self._base_frame_id]
                    .actInv(self._robot_data.oMf[self._tcp_frame_id])
                    .actInv(self._target_pose)
                )

                err = pin.log6(rMf).vector

                # If goal reached up to a tolerance, don't move
                abs_err = np.abs(err)

                if np.all(abs_err[:3] < self._pose_tolerance) and np.all(
                    abs_err[3:] < self._rot_tolerance
                ):
                    if i == 0 and j == 0:
                        if not self._pose_reached:
                            if not self._is_intermittent:
                                self._pose_reached_stamp = self.get_clock().now()
                            self._pose_reached_pub.publish(Empty())
                            self.get_logger().info("Pose reached.")
                        self._pose_reached = True
                    # v = np.zeros_like(q)
                fJf = pin.computeFrameJacobian(
                    self._robot_model,
                    self._robot_data,
                    q,
                    self._tcp_frame_id,
                    pin.LOCAL,
                )
                rJf = pin.Jlog6(rMf)
                J = -rJf @ fJf
                v = -J.T @ (np.linalg.solve(J @ J.T + 1e-12 * np.eye(6), err))
                q = pin.integrate(self._robot_model, q, v * self._dt)

            q[0] *= 0.95

            v = (q - q0) / self._dt
            v_max = np.max(np.abs(v))
            if v_max > self._params.max_angular_velocity:
                v = v / v_max * self._params.max_angular_velocity
            q = pin.integrate(self._robot_model, q0, v * self._dt)

            trajectory_points.append(
                JointTrajectoryPoint(
                    positions=q,
                    velocities=v,
                    accelerations=np.zeros_like(q),
                    time_from_start=Duration(seconds=self._dt * (i + 1)).to_msg(),
                )
            )

        jt = JointTrajectory(
            header=Header(stamp=self.get_clock().now().to_msg()),
            joint_names=self._params.moving_joint_names,
            points=trajectory_points,
        )

        self._joint_trajectory_pub.publish(jt)


def main(args=None):
    rclpy.init(args=args)
    reference_pose_publisher = ReferencePosePublisher()

    try:
        rclpy.spin(reference_pose_publisher)
    except KeyboardInterrupt:
        pass

    reference_pose_publisher.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
