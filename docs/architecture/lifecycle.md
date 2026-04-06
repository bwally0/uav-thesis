# Node Lifecycles

ROS2 managed (lifecycle) nodes provide a standardized state machine for controlling node startup, shutdown, and runtime reconfiguration. The UAV Lease-Gated Autonomy Stack uses lifecycle nodes to ensure controlled, predictable startup ordering and graceful shutdown.

## Table of Contents

- [Lifecycle States](#lifecycle-states)
- [State Transitions](#state-transitions)
- [Startup Sequence](#startup-sequence)
- [Shutdown Sequence](#shutdown-sequence)
- [Benefits for UAV Systems](#benefits-for-uav-systems)

## Lifecycle States

Managed nodes exist in four primary states:

1. **Unconfigured**: Node has been constructed but not yet configured
2. **Inactive**: Node is configured but not yet actively processing
3. **Active**: Node is fully operational and processing data
4. **Finalized**: Node is shutting down and cleaning up resources

## State Transitions

Transitions between states are triggered by lifecycle management commands:

- `configure`: Unconfigured → Inactive
- `activate`: Inactive → Active  
- `deactivate`: Active → Inactive
- `cleanup`: Inactive → Unconfigured
- `shutdown`: Any state → Finalized

## Startup Sequence

The autonomy stack enforces a strict startup order to ensure safety infrastructure is ready before mission logic:

1. **Supervisor Layer Nodes** (Lease Manager, Health Monitor, Fault Handler)
    - Configure and activate first
    - Must be fully operational before lower layers start

2. **Control Layer Nodes** (Offboard Interface, Intent Validator)
    - Configure and activate second
    - Establishes FC communication and lease gate

3. **Decision Layer Nodes** (Behavior Tree, Mission Logic)
    - Configure and activate last
    - Only starts when supervision and control are ready

## Shutdown Sequence

Shutdown occurs in reverse order:

1. Decision layer deactivates (stops generating intent)
2. Control layer deactivates (maintains safe hold)
3. Supervisor layer deactivates last

## Benefits for UAV Systems

Lifecycle management provides critical advantages for aerial autonomy:

- **Predictable Ordering**: Safety components active before mission logic
- **Graceful Degradation**: Controlled shutdown during faults
- **Detectible Failures**: State transition failures are explicit
- **Runtime Reconfiguration**: Nodes can be restarted without full system restart

Example launch file configuration:
```python
LifecycleNode(
    package='supervisor_pkg',
    executable='lease_manager',
    name='lease_manager',
    namespace='',
    output='screen'
)
```