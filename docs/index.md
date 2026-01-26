# Autonomous UAV Implementation and Simulation for Drone Development

This documentation describes the design, implementation and simulation of an open-source companion computer autonomy stack for PX4-based unmanned aerial vehicles (UAVs). The project focuses on companion-side software architecture, safety supervision, and reproducible development workflows rather than flight-controller firmware or vehicle-specific hardware.

## Motivation
Modern autonomous UAV systems rely on a companion computer to perform perception, planning, and mission logic while delegating low-level controls like drone movement and stabilization to a flight controller like PX4. While PX4 provides documentation for Offboard control and communication interfaces, guidance on how to build companion-side autonomy is limited.

Most publically available resources focus on:

- Basic ROS2 setup and message passing.
- Minimal examples of Offboard setpoint publishing.

In practice, production-grade systems require significantly more structure: supervision, fault handling, deterministic control, and clear seperation between safety, control, and mission logic. While these patterns are well established in ground robotics in frameworks such as Nav2, there are not many established frameworks for single drone autonomy.

Many mature companion-computer autonomy stacks are proprietary, domain-specific, or tightly coupled to internal company infrastructure, limiting their usefulness as a learning resource or reusable foundations for research and development.

This project is motivated by the need for:

- A clear, open reference architecture for UAV companion computers.
- Practical workflows for development, testing, and simulation.
- A framework that can be extended to different levels of autonomy, missions, and research use-cases without rewriting core functionality.

## Scope
The scope of this documentation is strictly limited to the companion computer software layer and its interaction with PX4 through Offboard control interfaces. It defines the design of the architecture itself as well as development workflows and how to use the provided implementation for your use-case. Documentation was written and tested with a Windows 11 + WSL2 environment. Although an example implementation of these concepts is provided, you could implement the architecture on your own. 

## Description

The UAV Lease-Gated Autonomy Stack is an open-source reference architecture and companion computer software framework for building safe, modular, and reproducible autonomous single-drone systems on top of PX4 and ROS2. Unlike basic Offboard examples that focus only on setpoint publishing or simple ROS2 message passing, this project delivers an autonomy stack with production in mind. The architecture takes inspiriation from existing ground-robotics patterns but adapts to the constraints of aerial vehicles. Autonomy is always explicity leased and is never implicitly assumed.

## Software Stack

![Software Stack](https://i.imgur.com/4eOCp7g.png)

The stack integrates simulation, middleware, and layed autonomy logic for both development and eventual hardware deployment.

Cosys-AirSim serves as the primary simulation environment (although Gazebo can work just as well for your implementation), providing high-fidelity rendering, physics, and sensor simulation, allowing for photo realistic testing and iteration without physical hardware. The companion computer communicates bidirectionally with the PX4 flight controller via uXRCE-DDS (the ROS2 bridge for PX4's uORB topics). This allows the autonomy stack to subscribe to vehicle state/telemetry and publish Offboard commands. Cosys-AirSim includes a ROS2 wrapper that bridges simulation sensor data into standard ROS2 topics, making it easy to feed behavior nodes.

The autonomy logic itself is structued in three decoupled layers (read more in Architecture section).

- Supervisor Layer: continuous monitors system health, PX4 failsafe flags, node liveliness, resource usage, and environmental constraints. It leases autonomy to the control layer only when strict safety preconditions are met.
- Control Layer: enforced by the lease gate, validates and filters all outgoing commands, maintains communication with the PX4 flight controller, and falls back to safe hold setpoints when the lease is revoked.
- Decision Layer: generates high-level intent via composable behavior trees. It has no direct actuation authority as all outputs pass through validation and gating.

## Table of Contents