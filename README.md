# Drones_S5_CD2_3D_Mapping_using_normal_monocular_camera_and_Visual-Inertial-Odometry-VIO

# Autonomous GPS-Denied 3D Mapping & $SO(3)$ Quaternion Flight Control

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![C++](https://img.shields.io/badge/Language-C%2B%2B17-blue.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Language-Python%203.10-yellow.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

An end-to-end autonomous, GPS-denied navigation and mapping pipeline for multirotor UAVs. This repository implements monocular Visual-Inertial Odometry (VIO) scale fusion, dynamic 3D voxel occupancy mapping, minimum-snap trajectory generation, and a from-scratch $SO(3)$ unit quaternion cascaded attitude and position controller.

---

## 📌 Overview

Commercial autopilots often rely on black-box state estimation, planar 2D grids, or Euler-angle flight controllers susceptible to gimbal lock. This project addresses these limitations by providing a fully transparent, first-principles robotics stack operating on ROS 2:

* **Monocular VIO Pipeline:** Fuses a single 2D camera feed (30 FPS) with high-frequency IMU telemetry (200 Hz) to recover metric scale ($x, y, z$) without requiring heavy RGB-D or LiDAR payloads.
* **3D Voxel Mapping:** Converts sparse point clouds into real-time 3D spatial occupancy grids (`OctoMap` / `Voxblox`) using ray-casting for obstacle detection and clearance inflation.
* **Pure $SO(3)$ Dynamics:** Bypasses commercial flight boards by computing orientation errors directly on the Special Orthogonal Group $SO(3)$ via unit quaternions, ensuring global tracking stability across all flight envelopes ($360^\circ$).

---

## 📐 Cascaded Control Architecture

The flight controller is structured as a dual-loop cascaded system. The 6-DoF VIO state vector is decoupled into translational states ($\mathbf{p}, \mathbf{v}$) for the outer loop and rotational states ($q, \boldsymbol{\omega}$) for the inner loop.

```text
[ Desired Waypoints (X,Y,Z) ] ──────┐
                                    ▼
[ VIO Pos/Vel (p, v) ] ───────► [ Outer Loop: Position ] ──► [ Desired Acceleration Vector ]
                                                                      │
[ Desired Yaw (ψd) ] ─────────────────────────────────────────────────┤
                                                                      ▼
                                                      [ Target Quaternion Extraction (qd) ]
                                                                      │
[ VIO Att/Rates (q, ω) ] ─────────────────────────────────────────────┤
                                                                      ▼
                                                           [ Inner Loop: Attitude ]
                                                                      │
                                                                      ▼
                                                      [ Raw Torques & Thrust Output ]
                                                                      │
                                                                      ▼
                                                            [ Motor Mixer / PWM ]
