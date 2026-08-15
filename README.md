<p align="center">
  <img src="assets/amrita_logo.png" alt="Amrita Vishwa Vidyapeetham" width="420" />
</p>

<div align="center">

# Geometric (SO(3)) Quaternion Flight Controller
### First-Principles Autonomous Multirotor Control

![ArduPilot SITL](https://img.shields.io/badge/ArduPilot-SITL-0A66C2?style=for-the-badge)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-5C2D91?style=for-the-badge)
![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge)
![DDS](https://img.shields.io/badge/ROS%202-DDS-0E7490?style=for-the-badge)
![MATLAB ROS 2](https://img.shields.io/badge/MATLAB-ROS%202%20Toolbox-EA580C?style=for-the-badge)

</div>

---

## Team

| Name | Roll Number |
| --- | --- |
| **Ravishanmugam K** | `CB.SC.U4AIE24347` |
| **Ishwarya M** | `CB.SC.U4AIE24220` |
| **Aparna B** | `CB.SC.U4AIE24304` |
| **Cibikumar B** | `CB.SC.U4AIE24212` |
| **Akhilan S** | `CB.SC.U4AIE24362` |

**Institution:** Amrita Vishwa Vidyapeetham

---

## Abstract

This project presents a first-principles nonlinear flight-control framework for autonomous multirotors based on geometric control on $SO(3)$ and unit-quaternion attitude representation. The formulation targets coupled position/velocity tracking and geometric attitude stabilization under nonlinear rigid-body dynamics, while keeping the controller structure compatible with simulation-centric validation workflows. The integration context combines ArduPilot SITL, Gazebo, ROS 2/DDS communication, and MATLAB ROS 2 interfacing for closed-loop analysis without claiming hardware-deployment outcomes.

---

## Introduction

Quadrotors are nonlinear, strongly coupled, and underactuated systems, which makes reliable attitude and trajectory control nontrivial. Classical Euler-angle-based controllers can suffer from singularities (gimbal lock) and local-coordinate limitations during aggressive maneuvers. This project therefore adopts an $SO(3)$ geometric attitude representation with quaternion-based state information to support globally consistent attitude handling, with the immediate goal of simulation-first validation prior to broader deployment.

---

## Methodology

```mermaid
flowchart LR
    G[Gazebo] --> A[ArduPilot SITL]
    A --> E[State Estimation]
    E --> R[ROS 2 / DDS]
    R --> C[SO(3) Controller]
    C --> A
    M[MATLAB ROS 2] <--> R
```

Gazebo provides the physics and sensor simulation environment, while ArduPilot SITL acts as the flight-stack execution layer and source of estimator outputs. Estimated motion states are bridged through ROS 2/DDS, where the geometric controller computes thrust/moment commands from a quaternion-consistent state representation and feeds command signals back into SITL for closed-loop simulation.

MATLAB ROS 2 integration is positioned on the same ROS 2/DDS backbone for analysis, rapid prototyping, and controller-side validation workflows. This architecture keeps simulator, estimator, controller, and analysis tooling modular while maintaining a single message-driven control loop.

The controller primarily consumes:

- `/ap/v1/pose/filtered`
- `/ap/v1/twist/filtered`

and maps them to

$$
\mathbf{x} = [\mathbf{p},\ \mathbf{v},\ \mathbf{q},\ \boldsymbol{\Omega}]
$$

where $\mathbf{p}$ and $\mathbf{q}$ are obtained from filtered pose, and $\mathbf{v}$ and $\boldsymbol{\Omega}$ are obtained from filtered twist.

> [!NOTE]
> This repository snapshot currently contains `README.md` and `LICENSE`; the document above captures the intended simulation architecture and control formulation without reporting unsupported implementation or performance claims.
