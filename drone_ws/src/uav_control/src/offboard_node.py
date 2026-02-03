#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand

SETPOINT_CYCLES = 200


class OffbaordNode(Node):
    def __init__(self):
        super().__init__("offboard_node")
        self.get_logger().info("Publishing OffboardControlMode messages...")

        # QoS profiles
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0,
        )
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0,
        )

        # subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "fmu/out/vehicle_status_v1",
            self._vehicle_status_callback,
            qos_profile_sub,
        )

        # publishers
        self._offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile_pub
        )
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile_pub
        )
        self._vehicle_cmd_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile_pub,
        )

        self._nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self._arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self._setpoint_counter = 0

        # timer for control loop
        self._timer_period = 0.02
        self._timer = self.create_timer(self._timer_period, self._control_loop_callback)

    def _vehicle_status_callback(self, msg: VehicleStatus):
        self._nav_state = msg.nav_state
        self._arming_state = msg.arming_state

        self.get_logger().info(
            f"Nav state: {self._nav_state}, Arming state: {self._arming_state} (Needed: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}, {VehicleStatus.ARMING_STATE_ARMED})",
            throttle_duration_sec=1.0,
        )

    def _control_loop_callback(self):
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()

        if self._setpoint_counter < SETPOINT_CYCLES:
            self._setpoint_counter += 1
            return

        if self._nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info(
                "Requesting OFFBOARD mode", throttle_duration_sec=1.0
            )
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,  # custom mode
                6.0,  # offboard mode
            )
            return

        if self._arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Requesting ARM", throttle_duration_sec=1.0)
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,  # arm
                0.0,
            )
            return

    def _publish_offboard_control_mode(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False

        self._offboard_pub.publish(offboard_msg)

    def _publish_trajectory_setpoint(self):
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        setpoint_msg.position[0] = 0.0
        setpoint_msg.position[1] = 0.0
        setpoint_msg.position[2] = -5.0
        setpoint_msg.yaw = float("nan")

        self._trajectory_pub.publish(setpoint_msg)

    def _publish_vehicle_command(self, command, param1, param2):
        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.command = command
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True

        self._vehicle_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    offboard_node = OffbaordNode()

    rclpy.spin(offboard_node)

    offboard_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
