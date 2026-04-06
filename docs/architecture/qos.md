# ROS2 QoS Policies

Quality of Service (QoS) policies in ROS2 define how messages between nodes are transmitted, including reliability guarantees, message history depth, and durability settings. Proper QoS configuration is critical for the UAV Lease-Gated Autonomy Stack to ensure safety-critical messages are delivered reliably.

## Table of Contents

- [QoS Overview](#qos-overview)
- [Reliable QoS](#reliable-qos)
- [Best-Effort QoS](#best-effort-qos)
- [Recommended QoS Settings](#recommended-qos-settings)

## QoS Overview

ROS2 provides configurable QoS policies that control communication characteristics between publishers and subscribers. The two primary reliability modes are:

- **Reliable**: Guarantees message delivery with acknowledgments and retransmissions
- **Best-Effort**: Sends messages without guarantees, minimizing latency

## Reliable QoS

Use reliable QoS for safety-critical topics where message loss is unacceptable:

- **Lease State**: Supervisor to control layer lease authorization
- **Safety Directives**: Supervisor commands to control layer
- **Fault Notifications**: Fault reports from any component to supervisor

## Best-Effort QoS

Use best-effort QoS for high-frequency, time-sensitive data where occasional loss is acceptable:

- **Sensor Data**: Camera images, LiDAR point clouds, IMU data
- **Vehicle State Telemetry**: Position, velocity, attitude updates
- **Offboard Handshake**: High-frequency control loop messages
- **Intent Stream**: Decision layer to control layer behavioral intent

## Recommended QoS Settings

**Reliable Configuration:**
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**Best-Effort Configuration:**
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```