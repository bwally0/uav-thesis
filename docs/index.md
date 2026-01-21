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
The scope of this documentation is strictly limited to the companion computer software layer and its interaction with PX4 through Offboard control interfaces.


**TODO**
other scope
- single drone, no swarm,
- documentation was written and tested with a Windows 11 + WSL2 development environment.
- provide foundational templates reusable for different scenarios.
- audience: hobbyists, researchers, developers

## Description

This project provides a basic framework for building, testing, and simulating single autonomous drones. 

## Software Stack

## Table of Contents