# Supervisor/Watchdog Layer

The supervisor is the safety authority of the companion computer. It determines whether autonomy is allowed to act, under what constraints can autonomy act, and what actions must occur to keep the drone in a safe state. The architecture is designed around the PX4 authority model which treats offboard control as advisory, revocable and non-authoritative. 

### PX4 Authority model

notes:
- supervisor works on top of PX4 safety, PX4 can independelty trigger failsafes and change modes and the supervisor will be able to react by revoking lease and maintaining a safe internal state.

### Supervisor Position in Stack

The Supervisor sits above the Control and Decision layers in terms of authority, but has a different set of responsibilities

- The decision layer decides what the system would like to do.
- The control alyer decides how to safety express those decision to the flight controller.
- The supervisor decide whether either of those layers is allowed to influence the vehicle at all.

The supervisor's responsiblity is to authorize autonomy, send explcit directives to the control layer to perform safe actions, and classify faults to determine what directive to take. The supervisor does not have direct communication to the flight controller so if the supervisor needs to perfrom an action in the name of safety it must do so through control layer directives.

### Lease Manager

The lease manager is the supervisor's primary enforcement mechanmism for the execution of autonomy in the stack. The lease manager has the following responsibilities:

- Grant leases to the control layer when certain preconditions are met.
- Periodically renew leases while conditions remain valid.
- Revoke leases. Examples of reasons to revoke a lease: safety violation, fault classification requiring loss of autonomy, loss of heartbeat from control or decision layers, or PX4 flightmode change out of offboard.

Each layer that is under the Supervisor is lease-gated. The control layer is lease-gated from actuation (commands send to PX4 based off of intent by the decision layer) and the decision layer is gated from sending intent to the control layer. Control layer will still be able to send fallback setpoints to the flight controller even if the lease is invalid, it will also send commands to maintain the handshake between the control layer and the flight controller. The lease only gates commands that was previously intent from the decision layer.

TODO what are precondixtoins for granting/renewing a lease?
TODO how long is a lease and when does it get renewed?

### Health Monitoring

The supervisor continuously monitors the health of the software running in the stack, the hardware that the stack is running on, and the condition of the flight controller.

- Monitors heartbeats from the control and decision layer. Heartbeat loss is treated as a fault and will be handled in fault classification.
- Monitor the control loop in the control layer to ensure that timing is being met.
- Monitor vehicle health given information from the flight controller. This could include failsafe flags, estimator validity, change in flight mode, etc.
- Monitor hardware system health. This includes CPU load, memory ussage, or sensor uptime.

TODO add more explicits to what is monitored: XRCE-DSS link heath, temperature, critical process and node liveness.

### Fault Classification

A fault is an observed condition that invalidates one or more safety assumptions required for autonomy. Any node can send a fault to the supervisor for handling if it determines that a safety contraint is not met. A warning is an observed condition that does not break the safety assumptions used for informational use.

The supervisor is responsible for classifying the faults it receieves and deciding the appropriate action to take based on the severity of the fault. Fault handling follows a match and action model. Match the fault to the severity level and perform an action based on it. Actions could range from lease revocation, to flight control mode switches, to execution of emergency procedures. The supervisor must used the control layer to perform an action since it does not have direct communication with the flight controller. This type of interaction is done with directives that the supervisor gives to the control layer. Directive action have absolute priority over intents send by the decision layer. The control layer must execute the appropriate commands based on the directive while maintaining the handshake with the flight controller.

TODO add example table for severity levels.
TODO add examples of directives sent to the flight controller.