"""Shared constants for UAV control packages."""

# Control loop constants
SETPOINT_CYCLES = 200
CONTROL_LOOP_PERIOD = 0.02  # seconds (50 Hz)

# Vehicle command constants
OFFBOARD_MODE = 6.0
ARM_COMMAND = 1.0

# Trajectory setpoint constants
DEFAULT_ALTITUDE = 5.0  # meters above ground
DEFAULT_HOLD_X = 0.0  # meters, local frame
DEFAULT_HOLD_Y = 0.0  # meters, local frame
DEFAULT_HOLD_Z = -DEFAULT_ALTITUDE  # meters, altitude

# QoS configuration
QOS_RELIABILITY_BEST_EFFORT = "BEST_EFFORT"
QOS_DURABILITY_TRANSIENT_LOCAL = "TRANSIENT_LOCAL"
QOS_DURABILITY_VOLATILE = "VOLATILE"
QOS_HISTORY_KEEP_LAST = "KEEP_LAST"
