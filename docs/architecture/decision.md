# Decision Layers

## Table of Contents

- [Decision Layer Overview](#decision-layer-overview)
- [Behavior Trees](#behavior-trees)
- [Flow of Control](#flow-of-control)

![Decision Layer Diagram](https://i.imgur.com/5YW5Qp1.png)
![Decision Tree Diagram](https://i.imgur.com/3N7MFWP.png)

## Decision Layer Overview

The decision layer determines what the vehicle should do based on mission objectives, sensor data, and environmental context. It generates high-level intent via composable behavior trees and has no direct actuation authority - all outputs pass through validation and gating in the control layer.

## Behavior Trees

Behavior trees provide a modular, composable way to define mission logic with priority-based arbitration. Higher-priority safety behaviors can preempt lower-priority mission tasks.

## Flow of Control

![Control Flow Chart](https://i.imgur.com/JT1SuIY.png)