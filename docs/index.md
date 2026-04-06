# A Lease-Gated Architecture for Safe and Modular Autonomous Drone Systems

**Brendan Waldrop**  
Barrett, The Honors College and School of Computing and Augmented Intelligence  
Arizona State University, Spring 2026

This documentation describes the design, implementation and simulation of an open-source companion computer autonomy stack for PX4-based unmanned aerial vehicles (UAVs). The project focuses on companion-side software architecture, safety supervision, and reproducible development workflows rather than flight-controller firmware or vehicle-specific hardware.

## Abstract

This project presents an open-source companion computer stack focused on **deterministic safety** and **modular architecture** for autonomous UAV systems. The core is based around the PX4 autopilot software, with a focus on an on-board companion computer running ROS2. Together, these tools form a capable foundation for autonomous UAV development.

The stack is organized into three decoupled layers – **Supervisor**, **Control**, and **Decision** – unified by a **lease-gate mechanism** that ensures autonomous operation is always explicitly authorized and never implicitly assumed. Documentation is provided alongside the architecture, validated in simulation using Cosys-AirSim and PX4 SITL, offering a reproducible development workflow that allows researchers and developers to iterate and test without requiring physical hardware.

## Project Goals

- **Describe an open-source companion computer architecture** focused on:
    - Deterministic Safety
    - Modular Architecture
- **Help bridge the gap** between tool documentation and production
- **Provide a reproducible development workflow**

## Motivation

Modern autonomous UAV systems rely on a companion computer to perform perception, planning, and mission logic while delegating low-level controls like drone movement and stabilization to a flight controller like PX4. While PX4 provides documentation for Offboard control and communication interfaces, guidance on how to build companion-side autonomy is limited.

Most publically available resources focus on:

- Basic ROS2 setup and message passing
- Minimal examples of Offboard setpoint publishing

In practice, production-grade systems require significantly more structure: supervision, fault handling, deterministic control, and clear seperation between safety, control, and mission logic. While these patterns are well established in ground robotics in frameworks such as Nav2, there are not many established frameworks for single drone autonomy.

Many mature companion-computer autonomy stacks are proprietary, domain-specific, or tightly coupled to internal company infrastructure, limiting their usefulness as a learning resource or reusable foundations for research and development.

**This project is motivated by the need for:**

- A clear, open reference architecture for UAV companion computers
- Practical workflows for development, testing, and simulation
- A framework that can be extended to different levels of autonomy, missions, and research use-cases without rewriting core functionality
- Treating safety enforcement as a **first-class design concern**, not an afterthought

### Why is Safety Critical for Aerial Robotics?

Unlike ground robots where a safe idle state is as simple as stopping in place, a UAV must maintain active flight even during fallback conditions. A ground robot encountering a fault can decelerate and stop, allowing for recovery without much consequence. For UAVs, there's no equivalent passive safe state—an aerial vehicle must actively manage its behavior during any fault condition, whether returning home, executing a controlled descent, or alerting an operator.

This fundamental difference makes explicit and structured supervision critical for autonomous aerial systems.

## Scope
The scope of this documentation is strictly limited to the companion computer software layer and its interaction with PX4 through Offboard control interfaces. It defines the design of the architecture itself as well as development workflows and how to use the provided implementation for your use-case. Documentation was written and tested with a Windows 11 + WSL2 environment. Although an example implementation of these concepts is provided, you could implement the architecture on your own. 

## Description

The UAV Lease-Gated Autonomy Stack is an open-source reference architecture and companion computer software framework for building safe, modular, and reproducible autonomous single-drone systems on top of PX4 and ROS2. Unlike basic Offboard examples that focus only on setpoint publishing or simple ROS2 message passing, this project delivers an autonomy stack with production concerns in mind. 

The architecture takes inspiration from existing ground-robotics patterns (particularly Nav2) but adapts them to the constraints and safety requirements of aerial vehicles. The core design philosophy is simple: **autonomy must always be explicitly authorized and never implicitly assumed**.

### Core Architectural Principles

1. **Explicit Authorization**: Through the lease-gate mechanism, autonomous operation requires continuous active authorization from the supervisor
2. **Layered Separation of Concerns**: Three decoupled layers (Supervisor, Control, Decision) each with distinct responsibilities
3. **Fail-Safe by Design**: The system maintains safe behavior even when authorization is revoked
4. **Modular and Extensible**: Mission logic can be added or modified without compromising safety infrastructure

## Software Stack

![Software Stack](https://i.imgur.com/4eOCp7g.png)

The stack integrates simulation, middleware, and layed autonomy logic for both development and eventual hardware deployment.

Cosys-AirSim serves as the primary simulation environment (although Gazebo can work just as well for your implementation), providing high-fidelity rendering, physics, and sensor simulation, allowing for photo realistic testing and iteration without physical hardware. The companion computer communicates bidirectionally with the PX4 flight controller via uXRCE-DDS (the ROS2 bridge for PX4's uORB topics). This allows the autonomy stack to subscribe to vehicle state/telemetry and publish Offboard commands. Cosys-AirSim includes a ROS2 wrapper that bridges simulation sensor data into standard ROS2 topics, making it easy to feed behavior nodes.

The autonomy logic itself is structued in three decoupled layers (read more in Architecture section).

- Supervisor Layer: continuous monitors system health, PX4 failsafe flags, node liveliness, resource usage, and environmental constraints. It leases autonomy to the control layer only when strict safety preconditions are met.
- Control Layer: enforced by the lease gate, validates and filters all outgoing commands, maintains communication with the PX4 flight controller, and falls back to safe hold setpoints when the lease is revoked.
- Decision Layer: generates high-level intent via composable behavior trees. It has no direct actuation authority as all outputs pass through validation and gating.

## Table of Contents

### Architecture Documentation
- [Architecture Overview](architecture/overview.md) - High-level overview of the UAV Lease-Gated Autonomy Stack
- [Supervisor Layer](architecture/supervisor.md) - Safety authority, lease management, and fault handling
- [Control Layer](architecture/control.md) - Intent validation and PX4 interface
- [Decision Layer](architecture/decision.md) - Behavior trees and mission logic
- [ROS2 QoS Policies](architecture/qos.md) - Quality of Service configuration for reliable communication
- [Node Lifecycles](architecture/lifecycle.md) - Lifecycle management and startup sequencing

### Tutorial Documentation
- [Installation Guide](tutorials/installation.md) - Setting up the development environment
- [Configuration Guide](tutorials/configuration.md) - Configuring Cosys-AirSim and system parameters