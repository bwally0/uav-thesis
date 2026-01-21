# Control Layer

The control layer converts a single intent stream from the decision layer into PX4-compatible flight commands when authorized bt the lease gate. It enforces safe boundaries and maintains the offboard handshake regardless of authorization state. 

### Offboard Interface

The control layer must maintain a handshake with the flight controller in order to obtain offboard control.

### Lease Gate

### Intent Validation

### Directive Handling

Directives from the supervisor bypass intent validation and take absolute priority over decision layer intents.

### Arbitrating Intent

The decision layer is repsonsible for producing a single intent stream via behavior tree arbitration. The control layer receieves this intent as-is, so no further arbitration is needed at the control layer. Read the decison layer page for more information.



