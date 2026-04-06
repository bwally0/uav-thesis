# Decision Layers

## Table of Contents

- [Decision Layer Overview](#decision-layer-overview)
- [Behavior Trees](#behavior-trees)
- [Flow of Control](#flow-of-control)

## Decision Layer Overview

![Decision Layer Diagram](https://i.imgur.com/5YW5Qp1.png)

The decision layer is responsible for determining what the vehicle should do based on mission objectives, sensor data, and environmental context. It provides **intent only**, meaning it has no direct command authority over the flight controller and all output must pass through the control layer's validation and lease gating before commands can be sent to the flight controller.

The decision logic is implemented as a composable behavior tree, where individual behaviors are defined as leaf nodes and the control flow is managed through control nodes that determine how and when each behavior executes. A developer implementing their own mission logic does so by building their own behavior tree nodes that produce intent, which are then composed into the tree alongside existing safety and navigation behaviors without modifying any other parts of the stack.

Higher priority safety behaviors are placed higher in the tree and can preempt lower priority mission tasks, ensuring that mission objectives always yield to safety constraints in a predictable way. The decision layer is also responsible for producing a single intent stream, keeping arbitration logic constrained within the behavior tree rather than distributed across the different layers. This single intent stream simplifies the control layer's responsibilities and ensures that the interface between the decision and control layers remains clean and well-defined.

## Behavior Trees

![Decision Tree Diagram](https://i.imgur.com/3N7MFWP.png)

Behavior trees are a hierarchical, modular data structure for building decision-making logic in autonomous systems. Originally developed for video game AI, they have become a popular paradigm for robotics behavior, replacing finite state machines due to their advantages in composability, readability, and ability to handle priority-based decisions in a structured and predictable way.

A behavior tree is composed of nodes arranged in a tree-like structure. Leaf nodes represent actions or conditions, while control nodes define the control flow. The tree is ticked at a fixed rate, with each tick propagating from the root and returning a status of success, failure, or running from each node. This model allows for higher priority safety behaviors to come before lower priority mission tasks, making behavior trees well suited to the decision layer of an autonomy stack where mission objectives must always yield first to safety constraints.

### Tree Structure

The behavior tree is structured around a **top-level priority selector** that evaluates safety behaviors before any mission logic. This means conditions like low battery, connection loss, or geofence violations will always preempt the current mission task.

**Safety Behaviors** are evaluated first and include:
- **Low Battery**: Triggers Return-to-Home (RTH) when battery falls below safe threshold
- **Connection Lost**: Initiates landing sequence if communication with ground control is lost

**Mission Objectives** are use-case defined behaviors that execute when safety conditions are met:
- Custom waypoint following
- Position hold and tracking
- Application-specific autonomous behaviors

**Safe Hold** is the fallback behavior that executes when no other behavior is active, commanding a mode change to a safe default state.

Each safety condition is implemented as a leaf node that reads from the relevant ROS2 topic, such as vehicle status or failsafe flags, and returns a failure status when the condition is not met. Mission tasks themselves are composed of subtrees, each encapsulating the actions and conditions required to complete a mission objective. This composability allows new mission behaviors to be added without modifying the safety layer, since the priority selector structure guarantees that safety checks are always evaluated first.

The behavior tree ticks at a fixed rate, producing a single intent command per cycle that is forwarded to the control layer for validation and gating.

## Flow of Control

![Control Flow Chart](https://i.imgur.com/JT1SuIY.png)

To understand how the decision layer integrates with the rest of the system, it's important to understand the full flow of a command through the entire autonomy stack. This flow demonstrates how the three-layer architecture (Supervisor, Control, Decision) works together to ensure safe autonomous operation.

### Command Flow Through the System

The flow begins with **sensor data** and **vehicle state** being consumed by the **behavior tree** in the decision layer. The behavior tree evaluates its priority selector structure, checking safety conditions first before mission logic. Based on the current state and sensor inputs, the tree produces a **decision intent** - a high-level command representing what the vehicle should do (such as moving to a target position, changing flight mode, or executing a mission behavior).

This intent is passed as a stream to the **control layer** through the **Intent Buffer**. At this point, the control layer performs the critical lease-gating check. The **Lease Gate** examines the current lease state provided by the Supervisor Layer's **Lease Manager**. If a valid, unexpired lease exists, the intent is authorized to proceed. If the lease is revoked or expired, the intent is blocked at the gate.

When intent is authorized, it passes through the **Intent Validator**, which applies deterministic safety filters to ensure commands remain within the vehicle's physical limits, geofence boundaries, and operational constraints. After validation, the command is forwarded to the **Offboard Interface**, which maintains the PX4 Offboard handshake (at minimum 2Hz) and publishes the command to the **flight controller**.

If the lease is not valid, the control layer blocks decision layer intent and instead publishes safe fallback setpoints while maintaining the offboard handshake. This ensures the vehicle remains in a controlled and predictable state even when the supervisor has not authorized decision layer intent.

Throughout this entire journey, the **Supervisor Layer** is continuously monitoring system health, PX4 state, and node liveliness through its **Health Monitor**. The **Lease Manager** grants, renews, or revokes leases based on predefined safety conditions. When faults are detected, the **Fault Handler** can issue **Safety Directives** that bypass the lease gate entirely and take absolute precedence over decision layer intent, instructing the control layer to execute critical safety actions such as mode transitions or emergency procedures.

Together, these three layers and the lease gate mechanism form an autonomy stack where safety enforcement is a structural requirement, and where mission logic, control enforcement, and supervision are cleanly separated so that each can be extended or replaced without compromising the integrity of the others.