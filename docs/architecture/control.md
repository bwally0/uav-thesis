# Control Layer

The control layer is responsible for converting decision layer intent into PX4-compatible flight commands when authorized through a valid lease. It serves as the interface between the companion computer and the flight controller, which means that all commands to the flight controller, whether they originate from decision intent or supervisor directives, pass through the control layer.

The control layer enforces safe boundaries through deterministic validation and maintains the PX4 Offboard handshake regardless of lease authorization state, ensuring the vehicle remains in a controlled and predictable state at all times.

## Table of Contents

- [Offboard Interface](#offboard-interface)
- [Lease Gate](#lease-gate)
- [Intent Validation](#intent-validation)
- [Directive Handling](#directive-handling)
- [Arbitrating Intent](#arbitrating-intent)

![Control Layer Flowchart](https://i.imgur.com/phVT4zX.png)

### Offboard Interface

The FC Offboard Interface maintains the PX4 Offboard handshake at the required minimum rate of 2Hz. PX4's Offboard mode allows an external system to send position, velocity, or attitude setpoints over a communication interface, but requires continuous handshaking to maintain control authority.

Most importantly, PX4 always retains override authority - if the external setpoint stream is lost, RC override is detected, or an internal failsafe condition is triggered, then PX4 will immediately exit Offboard mode and transition into a predefined safe state.

The Offboard Interface executes a fixed-rate control loop (typically 20-50Hz), publishing either:
- **Authorized intent commands** when a valid lease exists and intent has passed validation
- **Safe hold setpoints** when the lease is revoked/expired or no intent is available

This ensures the control loop never stops publishing to PX4, maintaining the required handshake regardless of the state of the layers above.

### Lease Gate

The control layer checks the lease state before forwarding decision layer intent to the flight controller:

- **Valid lease**: Intent proceeds to validation and then to the Offboard Interface
- **No valid lease**: Intent is blocked and safe hold setpoints are published instead

The lease exclusively gates decision layer commands. The control layer always maintains the PX4 Offboard handshake regardless of lease state.

See the [Architecture Overview](overview.md#the-lease-gate) for complete details on the lease gate mechanism.

### Intent Validation

The Intent Validator applies deterministic safety filters to all incoming commands from the decision layer, ensuring they remain within safe operational bounds before being sent to the flight controller.

Validation checks include:

- **Physical Limits**: Velocity and acceleration commands are clamped to the vehicle's physical capabilities
- **Geofence Boundaries**: Position setpoints are validated against configured geographic boundaries
- **Operational Constraints**: Flight envelope restrictions based on current vehicle state (e.g., altitude limits, restricted zones)

If a command fails validation, it can be either:
- **Clamped** to the nearest safe value (e.g., reducing excessive velocity)
- **Rejected** entirely, reverting to safe hold behavior

This validation layer provides defense-in-depth, ensuring that even if the decision layer produces unsafe intent (due to bugs, edge cases, or unforeseen scenarios), the control layer prevents it from reaching the flight controller.

**Note**: Safety directives from the supervisor bypass intent validation entirely, as they are critical safety instructions that must be executed immediately without modification.

### Directive Handling

Directives are high-priority commands issued by the Supervisor Layer's Fault Handler in response to detected faults or safety conditions. They take absolute precedence over any intent coming from the decision layer.

Directives instruct the control layer to:
- **Transition to a specific PX4 mode** (e.g., Return-to-Home, Land, Hold)
- **Publish specific fallback setpoints** (e.g., hold current position)
- **Ignore decision layer intent** until the fault is cleared

**Key Characteristics**:

- **Bypass validation**: Directives skip the Intent Validator entirely and go directly to the Offboard Interface
- **Immediate execution**: No lease check is required; directives always execute
- **Override intent**: If both a directive and decision intent exist, the directive wins

This design ensures that safety-critical actions commanded by the supervisor can always be executed immediately without any gating or filtering that might delay the response to a fault condition.

The supervisor has no direct communication with the flight controller and must enforce all safety actions through the control layer via these directives.

### Arbitrating Intent

The decision layer is responsible for producing a single intent stream via behavior tree arbitration. The control layer receives this intent as-is, so **no further arbitration is needed at the control layer**.

This design choice simplifies the control layer's responsibilities and ensures a clean interface between layers. All decision arbitration logic (choosing between competing behaviors, priority handling, etc.) is constrained within the behavior tree structure rather than distributed across multiple layers.

The control layer's job is purely to:
1. Check lease authorization
2. Validate the single intent stream for safety
3. Convert it to PX4-compatible commands
4. Maintain the offboard handshake

For more information on how intent arbitration works, see the [Decision Layer](decision.md) documentation.



