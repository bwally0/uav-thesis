# UAV Lease-Gated Autonomy Stack

This section provides a high-level overview of the **UAV Lease-Gated Autonomy Stack**, a software archiecture built on top of ROS2 for autonomous drone systems. The core design philosphy is explicit, revocable authorization and deterministic safety enforcement, seperating concerns of system health monitoring, control, and decision-making.

### The Lease Gate

The system is built around the concept of a Lease Gate. This is a time-bounded authorization barrier that sits between the autonomy logic of the companion computer (CC) and the flight controller (FC). It acts as a deterministic switch:

- **Granted**: When a valid, unexpired lease is active, authorized control commands are allowed to pass through to the FC.
- **Revoked/Expired**: The gate immediately blocks all actuation commands. The control software must cease and may only hold position or execute predefined safe behavior.

A lease is a short-lived token granted by the supervisor only when a strict set of safety preconditions are met. This ensures that autonomy is never assumed, but always permitted within a known-safe envelope.

## Layers of the Stack

The stack is logically divided into three distinct layers: *Supervisor Layer*, *Control Layer*, and the *Decision Layer*. Each with a specific responsibility and well defined interfaces between them.

### Supervisor Layer

The supervisor is the ultimate authority for autonomous operation. It coninuously monitors the entire system and decides if and under what constraints autonomy is allowed. When a fault is detected, the supervisor is responsible for enforcing the appropriate responsed according to predefiend policy.

- **Lease Manager**: Responsible for granting or revoking control leases to the control layer. Lease is granted under certain preconditions and if those preconditions fail, the lease is revoked.
- **Health Monitor**: Responsible for verifying the liveliness of the decision and control layers, monitoring the control loop, watching for FC failsafe flags, monitoring CC resources, and fault handling.
- **Fault Handler**: Executes predefined response policies when faults are detected such as mode switching, lease revocation, and emergency procedures.

If PX4 leaves OFFBOARD mode due to RC override, internal failsafe, or manual mode switch the supervisor immediately revokes any active leases. 

### Control Layer

The control layer is responsible for converting high-level intent actions into FC compatible commands when authorized through a lease.

- **Lease Gate**: Only passes actuation commands when the lease is valid.
- **Intent Arbiter**: Processes the single intent stream from the decision layer, applying priority and envelope checks.
- **Intent Validation**: Applies detemrinistic safety filters to all commands, ensuring they remain within the vehicle's physical limits, geofence boundaries, and operational constraints.
- **FC Offboard Interface**: Maintains the PX4 offboard handshake (2Hz minimum heartbeat) and executes a fixed-rate control loop (20-50Hz). Publishes safe hold setpoints when autonomy is not authorized.

### Decision Layer

The deicison layer determines what the vehicle should do based on mission objectives, sensor data, and environmental context.

- **Behavior Tree Engine**: Executes modular, composable behaviors with priority-based arbitration. Higher-priority safet behaviores can preempt mission tasks.
- TODO

The decison layer produces advisory intent only, it has no direct command authority. All intent must pass through the control layer's safety validation and lease gating.
