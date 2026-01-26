# Supervisor/Watchdog Layer

The supervisor is the safety authority of the companion computer. It determines whether autonomy is allowed to act, under what constraints can autonomy act, and what actions must occur to keep the drone in a safe state. The architecture is designed around the PX4 authority model which treats offboard control as advisory, revocable and non-authoritative. 

### PX4 Authority model

The PX4 authority model refers to which source (pilot, internal autopilot, companion computer, etc.) has control over the vehicle at any given time, and under what conditions can that authority be granted, shared, or revoked. PX4 operates in discrete flight modes (Land, Return, Offboard, Mission) and each determines who provides the setpoints and how much authority the autopilot has over actuators. 

Offboard mode is the important mode for companion computers. The external system sends setpoints via MAVLink/uXRCE-DDS. PX4 delegates low-level control to these external setpoints while still running its internal rate/attitude controllers.

Even in offboard mode PX4 retains override authority for failsafes, RC override, and internal safety checks. The companion computer only has high-level setpoint authority.

### Supervisor Position in Stack

The Supervisor sits above the Control and Decision layers in terms of authority, but has a different set of responsibilities

- The decision layer decides what the system would like to do.
- The control layer decides how to safety express those decision to the flight controller.
- The supervisor decide whether either of those layers is allowed to influence the vehicle at all.

The supervisor's responsiblity is to authorize autonomy, send explicit directives to the control layer to perform safe actions, and classify faults to determine what directive to take. The supervisor does not have direct communication to the flight controller so if the supervisor needs to perform an action in the name of safety it must do so through control layer directives.

### Lease Manager

The lease manager is the supervisor's primary enforcement mechanism for the execution of autonomy in the stack. The lease manager has the following responsibilities:

- Grant leases to the control layer when certain preconditions are met.
- Periodically renew leases while conditions remain valid.
- Revoke leases. Examples of reasons to revoke a lease: safety violation, fault classification requiring loss of autonomy, loss of heartbeat from control or decision layers, or PX4 flightmode change out of offboard.

Each layer that is under the Supervisor is lease-gated. The control layer is lease-gated from actuation (commands send to PX4 based off of intent by the decision layer) and the decision layer is gated from sending intent to the control layer. Control layer will still be able to send fallback setpoints to the flight controller even if the lease is invalid, it will also send commands to maintain the handshake between the control layer and the flight controller. The lease only gates commands that was previously intent from the decision layer.

Preconditions for granting/renewing a lease, and how long is a lease and when does it get renewed is dependent on implementation.

For the example implementation of the framework the lease is implemented as follows:
**TODO**

### Health Monitoring

The supervisor continuously monitors the health of the software running in the stack, the hardware that the stack is running on, and the condition of the flight controller.

- Monitors heartbeats from the control and decision layer. Heartbeat loss is treated as a fault and will be handled in fault classification.
- Monitor the control loop in the control layer to ensure that timing is being met.
- Monitor vehicle health given information from the flight controller. This could include failsafe flags, estimator validity, change in flight mode, etc.
- Monitor hardware system health. This includes CPU load, memory ussage, or sensor uptime.

If any of these systems are not within acceptable values a fault is generated and handled by the fault classifier.

### Fault Classification

A fault is any observed condition that invalidates on ror more safety assumptions required for safe autonomous operation. Faults can originate from:

- Interal stack monitoring (e.g. heartbeat loss, control-loop timing violation, high CPU/memory usage)
- PX4 telemetry (e.g. asserted failsafe flags, estimator rejections, invalid positions).
- Other nodes (e.g., perception pipeline detects an imminent collision or geofence violation)

A warning is an observed condition that does not yet invalidate safety assumptions but is logged for information or predicitve purposes.

The supervisor is the sole authority for classifying incoming faults and deciding the response. Classification follows a severity based match-and-action model:

1. Receieve the fault report.
2. Match the fault type/code against predfined severity levels.
3. Execute the corresponding directive. A directive is a high priority command sent to the control layer.

Directives have absolute precedence over any intent from the decision layer. The control layer must honor the directive while preserving the offboard handshake.

Example severity levels and directives (customizable per implementation):

** TODO**


Directives are explicit messages that instruct the control layer to:

- Transition to a specific PX4 mode.
- Publish fallback setpoints.
- Execute predfined safe behaviors.
- Ignore decision layer intent until cleared.


### Supervisor required Topics

/fmu/out/failsafe_flags
```
timestamp: 1769017538192250
mode_req_angular_velocity: 8312190
mode_req_attitude: 8311166
mode_req_local_alt: 8257918
mode_req_local_position: 7995448
mode_req_local_position_relaxed: 262212
mode_req_global_position: 56
mode_req_global_position_relaxed: 0
mode_req_mission: 8
mode_req_offboard_signal: 16384
mode_req_home_position: 32
mode_req_wind_and_flight_time_compliance: 2621464
mode_req_prevent_arming: 3944480
mode_req_manual_control: 34119
mode_req_other: 2139095040
angular_velocity_invalid: false
attitude_invalid: false
local_altitude_invalid: false
local_position_invalid: false
local_position_invalid_relaxed: false
local_velocity_invalid: false
global_position_invalid: false
global_position_invalid_relaxed: false
auto_mission_missing: true
offboard_control_signal_lost: true
home_position_invalid: false
manual_control_signal_lost: true
gcs_connection_lost: true
battery_warning: 0
battery_low_remaining_time: false
battery_unhealthy: false
geofence_breached: false
mission_failure: false
vtol_fixed_wing_system_failure: false
wind_limit_exceeded: false
flight_time_limit_exceeded: false
position_accuracy_low: false
navigator_failure: false
fd_critical_failure: false
fd_esc_arming_failure: false
fd_imbalanced_prop: false
fd_motor_failure: false
```

/fmu/out/vehicle_status_v1
```
timestamp: 1769017639546586
armed_time: 1769017626134683
takeoff_time: 1769017627946722
arming_state: 2
latest_arming_reason: 3
latest_disarming_reason: 0
nav_state_timestamp: 1769017632590821
nav_state_user_intention: 4
nav_state: 4
executor_in_charge: 0
valid_nav_states_mask: 2147411327
can_set_nav_states_mask: 8308095
failure_detector_status: 0
hil_state: 0
vehicle_type: 1
failsafe: false
failsafe_and_user_took_over: false
failsafe_defer_state: 0
gcs_connection_lost: true
gcs_connection_lost_counter: 0
high_latency_data_link_lost: false
is_vtol: false
is_vtol_tailsitter: false
in_transition_mode: false
in_transition_to_fw: false
system_type: 2
system_id: 1
component_id: 1
safety_button_available: true
safety_off: true
power_input_valid: true
usb_connected: false
open_drone_id_system_present: false
open_drone_id_system_healthy: false
parachute_system_present: false
parachute_system_healthy: false
rc_calibration_in_progress: false
calibration_enabled: false
pre_flight_checks_pass: true
```

/fmu/out/estimator_status_flags
```
timestamp: 1769017692429033
timestamp_sample: 1769017692258029
control_status_changes: 15
cs_tilt_align: true
cs_yaw_align: true
cs_gnss_pos: true
cs_opt_flow: false
cs_mag_hdg: false
cs_mag_3d: true
cs_mag_dec: false
cs_in_air: true
cs_wind: false
cs_baro_hgt: true
cs_rng_hgt: false
cs_gps_hgt: true
cs_ev_pos: false
cs_ev_yaw: false
cs_ev_hgt: false
cs_fuse_beta: false
cs_mag_field_disturbed: false
cs_fixed_wing: false
cs_mag_fault: false
cs_fuse_aspd: false
cs_gnd_effect: false
cs_rng_stuck: false
cs_gnss_yaw: false
cs_mag_aligned_in_flight: true
cs_ev_vel: false
cs_synthetic_mag_z: false
cs_vehicle_at_rest: false
cs_gnss_yaw_fault: false
cs_rng_fault: false
cs_inertial_dead_reckoning: false
cs_wind_dead_reckoning: false
cs_rng_kin_consistent: false
cs_fake_pos: false
cs_fake_hgt: false
cs_gravity_vector: false
cs_mag: true
cs_ev_yaw_fault: false
cs_mag_heading_consistent: true
cs_aux_gpos: false
cs_rng_terrain: false
cs_opt_flow_terrain: false
cs_valid_fake_pos: false
cs_constant_pos: false
cs_baro_fault: false
cs_gnss_vel: true
cs_gnss_fault: false
cs_yaw_manual: false
cs_gnss_hgt_fault: false
fault_status_changes: 0
fs_bad_mag_x: false
fs_bad_mag_y: false
fs_bad_mag_z: false
fs_bad_hdg: false
fs_bad_mag_decl: false
fs_bad_airspeed: false
fs_bad_sideslip: false
fs_bad_optflow_x: false
fs_bad_optflow_y: false
fs_bad_acc_vertical: false
fs_bad_acc_clipping: false
innovation_fault_status_changes: 4
reject_hor_vel: false
reject_ver_vel: false
reject_hor_pos: false
reject_ver_pos: false
reject_yaw: false
reject_airspeed: false
reject_sideslip: false
reject_hagl: false
reject_optflow_x: false
reject_optflow_y: false
```

/fmu/out/vehicle_land_detected
```
timestamp: 1769017743973002
freefall: false
ground_contact: true
maybe_landed: true
landed: true
in_ground_effect: true
in_descend: true
has_low_throttle: true
vertical_movement: false
horizontal_movement: false
rotational_movement: false
close_to_ground_or_skipped_check: true
at_rest: true
```