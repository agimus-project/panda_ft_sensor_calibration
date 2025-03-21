from typing import Union

import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types
import numpy as np
import pinocchio as pin
import rclpy
import rclpy.duration
import rclpy.time
from linear_feedback_controller_msgs.msg import Control, Sensor
from linear_feedback_controller_msgs_py.numpy_conversions import (
    control_numpy_to_msg,
    sensor_msg_to_numpy,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.qos_overriding_options import QoSOverridingOptions
from std_msgs.msg import String

# Automatically generated file
from panda_ft_sensor_calibration.pd_plus_trajectory_follower_parameters import (
    pd_plus_trajectory_follower,
)  # noqa: E402


class PDPlusTrajectoryFollower(Node):
    def __init__(self):
        super().__init__("pd_plus_trajectory_follower")

        try:
            self._param_listener = pd_plus_trajectory_follower.ParamListener(self)
            self._params = self._param_listener.get_params()
            self._refresh_parameters(True)
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        self._control_pub = self.create_publisher(
            Control,
            "/control",
            qos_profile=qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self._sensor_sub = self.create_subscription(
            Sensor,
            "/sensor",
            self._sensor_cb,
            qos_profile=qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        if not self._params.recording_mode:
            self._reference_sensor_sub = self.create_subscription(
                Sensor,
                "reference/sensor",
                self._reference_sensor_cb,
                qos_profile=qos_profile_sensor_data,
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

        initial_configuration = np.array(
            [
                self._params.get_entry(joint_name).q_init
                for joint_name in self._params.moving_joints
            ]
        )

        self._robot_model: Union[pin.Model, None] = None
        self._robot_data: Union[pin.Data, None] = None
        self._sensor_msg: Union[Sensor, None] = None
        self._reference_sensor_cb: Union[lfc_py_types.Sensor, None] = None
        self._initial_configuration = lfc_py_types.Sensor(
            joint_state=lfc_py_types.JointState(
                position=initial_configuration,
                velocity=np.zeros_like(initial_configuration),
                effort=np.zeros_like(initial_configuration),
            )
        )

        self._initial_configuration_reached = False

        self._timer = self.create_timer(
            1.0 / self._params.controller_frequency, self._timer_callback
        )

        self.get_logger().info("Node started.")
        if self._params.recording_mode:
            self.get_logger().warn(
                "Node configured in 'recording mode'. "
                + "Robot will not follow any published trajectory!."
            )

    def _refresh_parameters(self, skip_parameter_update: bool = False) -> None:
        """Update parameters and related data structures.

        Args:
            skip_parameter_update (bool, optional): Skip update of
                parameters and only create data structures. Defaults to False.
        """
        if self._param_listener.is_old(self._params):
            # Update parameters
            if not skip_parameter_update:
                self._param_listener.refresh_dynamic_parameters()
                self._params = self._param_listener.get_params()
                self.get_logger().info("Parameter change occurred!")

            # Parse parameters
            self._p_gains = np.array(
                [
                    self._params.get_entry(joint_name).p
                    for joint_name in self._params.moving_joints
                ]
            )
            self._d_gains = np.array(
                [
                    self._params.get_entry(joint_name).d
                    for joint_name in self._params.moving_joints
                ]
            )
            self._pd_gains = np.hstack((np.diag(self._p_gains), np.diag(self._d_gains)))

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
            if name not in self._moving_joint_names and name != "universe"
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
        self.get_logger().info("Robot description received.")

    def _sensor_cb(self, msg: Sensor) -> None:
        """Callback called on every robot state message received.

        Args:
            msg (linear_feedback_controller_msgs.msg.Sensor): Message containing
                state of the robot.
        """
        self._sensor_msg = msg

    def _reference_sensor(self, msg: Sensor) -> None:
        """Callback called on every reference robot state message received.
            Message is converted to numpy representation on every callback.

        Args:
            msg (linear_feedback_controller_msgs.msg.Sensor): Message containing
                state of the robot.
        """
        self._reference_sensor_cb = sensor_msg_to_numpy(msg)

    def _timer_callback(self) -> None:
        """Periodically called callback used to compute new control signal for the robot."""

        self._refresh_parameters(False)

        if self._robot_model is None:
            self.get_logger().info(
                "Waiting for the Robot model...", throttle_duration_sec=2.0
            )
            return
        if self._sensor_msg is None:
            self.get_logger().info(
                "Waiting for the first sensor message...", throttle_duration_sec=2.0
            )
            return

        if (
            self._reference_sensor is None
            and not self._params.recording_mode
            and self._initial_configuration_reached
        ):
            self.get_logger().info(
                "Waiting for the first reference sensor message...",
                throttle_duration_sec=2.0,
            )
            return

        sensor = sensor_msg_to_numpy(self._sensor_msg)

        if (
            self._move_to_initial_configuration
            and not self._initial_configuration_reached
        ):
            target = self._initial_configuration
            # Check if configuration was reached
            if (
                np.allclose(
                    target.joint_state.position,
                    sensor.joint_state.position,
                    self._params.initial_configuration_tolerance,
                )
                and np.sum(np.abs(sensor.joint_state.velocity))
                < self._params.initial_configuration_tolerance
            ):
                self._initial_configuration_reached = True
                self.get_logger().info("Initial configuration reached.")
                if self._params.recording_mode:
                    self.get_logger().info("Robot is now controllable by human.")
                else:
                    self.get_logger().info("Starting to follow the trajectory.")
        else:
            target = self._reference_sensor
            # In recording mode make easy to control and only slightly try to go back
            # to initial position
            if self._params.recording_mode:
                self._p_gains *= self._params.recording_mode_pd_scale
                self._d_gains *= self._params.recording_mode_pd_scale
                self._pd_gains *= self._params.recording_mode_pd_scale

        delta_q = target.joint_state.position - sensor.joint_state.position
        delta_dq = target.joint_state.velocity - sensor.joint_state.velocity
        tau_g = pin.computeGeneralizedGravity(
            self._robot_model, self._robot_data, sensor.joint_state.position
        )

        tau = (tau_g - self._p_gains * delta_q - self._d_gains * delta_dq).reshape(
            self.pin_model.nv, 1
        )

        sensor = lfc_py_types.Sensor(
            base_pose=np.zeros(7),
            base_twist=np.zeros(6),
            joint_state=lfc_py_types.JointState(
                name=self.moving_joint_names,
                position=self.sensor.joint_state.position,
                velocity=np.zeros((self._robot_model.nv, 1)),
                effort=np.zeros((self._robot_model.nv, 1)),
            ),
            contacts=[],
        )

        if not self._params.recording_mode:
            self._control_pub.publish(
                control_numpy_to_msg(
                    lfc_py_types.Control(
                        feedback_gain=self._pd_gains,
                        feedforward=tau,
                        initial_state=sensor,
                    )
                )
            )


def main(args=None):
    rclpy.init(args=args)
    pd_plus_controller = PDPlusTrajectoryFollower()

    try:
        rclpy.spin(pd_plus_controller)
    except KeyboardInterrupt:
        pass

    pd_plus_controller.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
