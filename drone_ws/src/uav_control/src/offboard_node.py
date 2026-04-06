#!/usr/bin/env python3

import math
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
from uav_msgs.msg import Intent
from uav_constants import (
    SETPOINT_CYCLES,
    CONTROL_LOOP_PERIOD,
    DEFAULT_ALTITUDE,
    DEFAULT_HOLD_X,
    DEFAULT_HOLD_Y,
    DEFAULT_HOLD_Z,
)


# PX4 custom mode constants ; mirrors px4_custom_mode.h in PX4-Autopilot
# src/modules/commander/px4_custom_mode.h
# Used as param2 in VEHICLE_CMD_DO_SET_MODE (param1 must always be 1.0)
#
class PX4CustomMainMode:
    MANUAL = 1
    ALTCTL = 2
    POSCTL = 3
    AUTO = 4  # requires a sub-mode in param3
    ACRO = 5
    OFFBOARD = 6
    STABILIZED = 7
    RATTITUDE_LEGACY = 8
    SIMPLE = 9  # reserved, unused
    TERMINATION = 10
    ALTITUDE_CRUISE = 11


class PX4CustomSubModeAuto:
    READY = 1  # legacy, unused in modern PX4 - do not command
    TAKEOFF = 2
    LOITER = 3
    MISSION = 4
    RTL = 5
    LAND = 6
    # 7 is reserved (was RTGS, deleted 2020-03-05) - do not use
    FOLLOW_TARGET = 8
    PRECLAND = 9
    VTOL_TAKEOFF = 10


class OffboardNode(Node):
    def __init__(self):
        super().__init__("offboard_node")
        self.get_logger().info("Publishing OffboardControlMode messages...")

        # QoS profiles
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self._status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self._vehicle_status_callback,
            qos_sub,
        )
        self._intent_sub = self.create_subscription(
            Intent,
            "control/intent",
            self._intent_callback,
            qos_sub,
        )

        # Publishers
        self._offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos_pub,
        )
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos_pub,
        )
        self._vehicle_cmd_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_pub,
        )

        self._nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self._arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self._setpoint_counter = 0

        # Current setpoint — initialized to default hold position
        self._current_x = DEFAULT_HOLD_X
        self._current_y = DEFAULT_HOLD_Y
        self._current_z = DEFAULT_HOLD_Z
        self._current_yaw = 0.0

        # Intent watchdog timer — reverts to default if intent times out
        self._intent_watchdog_timeout = 2.0  # seconds
        self._intent_watchdog = self.create_timer(
            self._intent_watchdog_timeout, self._intent_watchdog_callback
        )
        self._intent_watchdog.cancel()  # start disabled

        # Control loop timer
        self._timer = self.create_timer(
            CONTROL_LOOP_PERIOD, self._control_loop_callback
        )

    def _vehicle_status_callback(self, msg: VehicleStatus):
        self._nav_state = msg.nav_state
        self._arming_state = msg.arming_state
        self.get_logger().info(
            f"[status] nav={self._nav_state} arming={self._arming_state}",
            throttle_duration_sec=1.0,
        )

    def _intent_callback(self, msg: Intent):
        """Receive waypoint intent from decision layer."""
        self.get_logger().info(
            f"[intent] waypoint: ({msg.target_x:.2f}, {msg.target_y:.2f}, {msg.target_z:.2f}) "
            f"yaw={math.degrees(msg.yaw):.1f}°",
            throttle_duration_sec=1.0,
        )
        # Update current setpoint from intent
        self._current_x = msg.target_x
        self._current_y = msg.target_y
        self._current_z = msg.target_z
        self._current_yaw = msg.yaw

        # Reset intent watchdog timer
        self._intent_watchdog.reset()

    def _intent_watchdog_callback(self):
        """
        Watchdog callback — reverts to default hold position if intent is not received.
        This ensures safe fallback when decision layer stops publishing.
        """
        self.get_logger().warn(
            f"[intent_watchdog] Intent timeout — reverting to default hold position "
            f"({DEFAULT_HOLD_X}, {DEFAULT_HOLD_Y}, {DEFAULT_HOLD_Z})"
        )
        self._current_x = DEFAULT_HOLD_X
        self._current_y = DEFAULT_HOLD_Y
        self._current_z = DEFAULT_HOLD_Z
        self._current_yaw = 0.0

    def _control_loop_callback(self):
        # Always stream OffboardControlMode and TrajectorySetpoint.
        # OffboardControlMode keeps PX4 happy and prevents failsafe.
        # TrajectorySetpoint must stream continuously — if it stops,
        # PX4 will trigger failsafe and land.
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()

        # Phase 1: fill setpoint buffer before sending any commands
        if self._setpoint_counter < SETPOINT_CYCLES:
            self._setpoint_counter += 1
            return

        # Phase 2: arm
        if self._arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Requesting ARM", throttle_duration_sec=1.0)
            self._publish_arm()
            return

        # Phase 3: enter and maintain offboard mode
        if self._nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info(
                "Requesting OFFBOARD mode", throttle_duration_sec=1.0
            )
            self._publish_set_offboard_mode()
            return

        # In offboard mode — log status
        self.get_logger().info("In OFFBOARD mode", throttle_duration_sec=1.0)

    def _publish_offboard_control_mode(self):
        """
        Stream OffboardControlMode at control loop rate.
        This is the 'proof of life' signal PX4 requires (>2 Hz).
        The first non-False field determines what setpoint type is active.
        """
        msg = OffboardControlMode()
        msg.timestamp = self._timestamp()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self._offboard_pub.publish(msg)

    def _publish_trajectory_setpoint(self):
        """
        Position setpoint in NED frame (metres).
        Z is negative for altitude above ground (NED convention).
        Uses current setpoint from intent or default hold position.
        """
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp()
        msg.position = [
            self._current_x,
            self._current_y,
            self._current_z,
        ]
        msg.velocity = [float("nan"), float("nan"), float("nan")]
        msg.yaw = self._current_yaw
        self._trajectory_pub.publish(msg)

    def _publish_vehicle_command(
        self,
        command: int,
        param1: float = float("nan"),
        param2: float = float("nan"),
        param3: float = float("nan"),
        param4: float = float("nan"),
        param5: float = float("nan"),
        param6: float = float("nan"),
        param7: float = float("nan"),
    ):
        cmd = VehicleCommand()
        cmd.timestamp = self._timestamp()
        cmd.command = command
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.param3 = param3
        cmd.param4 = param4
        cmd.param5 = param5
        cmd.param6 = param6
        cmd.param7 = param7
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self._vehicle_cmd_pub.publish(cmd)

    def _publish_arm(self):
        """Arm the vehicle."""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,  # 1 = arm
            param2=0.0,
        )

    def _publish_disarm(self):
        """Disarm the vehicle."""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,  # 0 = disarm
            param2=0.0,
        )

    def _publish_takeoff(self, altitude_m: float):
        """
        Command takeoff to the specified altitude (metres, relative to home).
        Source: MAVLink common.xml MAV_CMD_NAV_TAKEOFF (value 22)
          param1 = min pitch (ignored for MC)
          param4 = yaw (NaN = current heading)
          param5 = latitude (NaN = current)
          param6 = longitude (NaN = current)
          param7 = altitude in metres
        After completion PX4 transitions to NAVIGATION_STATE_AUTO_LOITER.
        """
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=float("nan"),
            param4=float("nan"),  # yaw — keep current heading
            param5=float("nan"),  # latitude — use current position
            param6=float("nan"),  # longitude — use current position
            param7=altitude_m,
        )

    def _publish_land(self):
        """
        Command landing at current position.
        Source: MAVLink common.xml MAV_CMD_NAV_LAND (value 21)
          param4 = yaw (NaN = current heading)
          param5 = latitude (NaN = current)
          param6 = longitude (NaN = current)
          param7 = altitude (0 = ground level)
        """
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            param4=float("nan"),
            param5=float("nan"),
            param6=float("nan"),
            param7=0.0,
        )

    def _publish_set_offboard_mode(self):
        """
        Switch to OFFBOARD mode via DO_SET_MODE.
        param1=1.0 sets MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.
        param2=PX4CustomMainMode.OFFBOARD (= 6).
        """
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=float(PX4CustomMainMode.OFFBOARD),
        )

    def _publish_loiter(self):
        """
        Switch to AUTO/LOITER (Hold) mode.
        The vehicle will hover at its current GPS position and altitude.
        """
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=float(PX4CustomMainMode.AUTO),
            param3=float(PX4CustomSubModeAuto.LOITER),
        )

    def _timestamp(self) -> int:
        """Return current time in microseconds (PX4 timestamp convention)."""
        return int(self.get_clock().now().nanoseconds // 1000)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
