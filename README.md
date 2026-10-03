# 🚁 Autonomous Drone 3D Navigation & Camera Trajectory Mapping using Monocular Visual-Inertial Odometry (MSCKF-VIO)

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-3498DB?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-Computer_Vision-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)](https://opencv.org/)
[![Open3D](https://img.shields.io/badge/Open3D-3D_Reconstruction-4B8BBE?style=for-the-badge&logo=open3d&logoColor=white)](http://www.open3d.org/)
[![Plotly](https://img.shields.io/badge/Plotly-Interactive_3D_WebGL-3F4F75?style=for-the-badge&logo=plotly&logoColor=white)](https://plotly.com/)
[![NumPy](https://img.shields.io/badge/NumPy-Linear_Algebra-013243?style=for-the-badge&logo=numpy&logoColor=white)](https://numpy.org/)
[![SciPy](https://img.shields.io/badge/SciPy-Lie_Algebra_SO(3)-8CAAE6?style=for-the-badge&logo=scipy&logoColor=white)](https://scipy.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-F1C40F?style=for-the-badge)](LICENSE)

---

## 📖 What is this Project About? *(Intuition for Beginners & Students)*

### 💡 The Big Picture: Navigating Where GPS Fails
When an autonomous drone flies outdoors under an open sky, it uses **GPS** satellites to know where it is. But what happens when the drone enters a warehouse, an underground tunnel, a dense forest, or a GPS-denied room? **GPS signals are completely blocked or bounce erratically off walls.**

Without GPS, how can a drone know its exact position and orientation without crashing into walls?
Humans solve this effortlessly:
1. **Your Inner Ear (The IMU / Inertial Sensor)**: When you close your eyes and someone pushes you or you take a step, your inner ear feels the acceleration and rotational tilt. But if you keep your eyes closed for 10 seconds, small estimation errors build up (**dead-reckoning drift**), and you quickly lose track of your exact position.
2. **Your Eyes (The Camera)**: Your eyes look at stationary corners, doors, furniture, and light fixtures. By observing how these visual points shift as your body moves (**optical parallax**), your brain immediately corrects any drift.

### 🎯 What this Project Does
This project creates a mathematically rigorous, real-time **Visual-Inertial Odometry (VIO)** and **3D Environment Mapping Engine** for autonomous drones using only:
* **One standard monocular camera** (the "Eye" streaming at 30 frames/sec)
* **One 6-axis IMU sensor** (the "Inner Ear" measuring acceleration and angular velocity at 200 Hz)

By fusing these two sensors through an advanced **Multi-State Constraint Kalman Filter (MSCKF)**, the drone calculates its exact millimetric flight path, tracks its true camera lens position in 3D space, and triangulates 3D spatial points to construct an interactive 3D map of the room in real time.

---

## 🛠️ Technology Stack & Tools

| Badge / Tool | Official Resource | Role in this Project |
| :---: | :---: | :--- |
| ![ROS 2](https://img.shields.io/badge/-ROS%202%20Humble-3498DB?logo=ros&logoColor=white) | [ROS 2 Humble](https://docs.ros.org/en/humble/) | Real-time robotics middleware, sensor pub/sub pipelines, lifecycle management, and TF2 spatial coordinate transformations. |
| ![OpenCV](https://img.shields.io/badge/-OpenCV-5C3EE8?logo=opencv&logoColor=white) | [OpenCV Documentation](https://opencv.org/) | Lucas-Kanade (KLT) pyramidal optical flow tracking, FAST corner detection, sub-pixel refinement, and grid-bucketing. |
| ![Open3D](https://img.shields.io/badge/-Open3D-4B8BBE?logo=open3d&logoColor=white) | [Open3D Python API](http://www.open3d.org/) | Moveable 3D GUI visualizer, point cloud processing, occupancy voxel grid generation, and Poisson surface room meshing. |
| ![Plotly](https://img.shields.io/badge/-Plotly-3F4F75?logo=plotly&logoColor=white) | [Plotly 3D Graphing](https://plotly.com/python/3d-scatter-plots/) | Standalone interactive 3D WebGL graphs with 360° rotation, pan, zoom, and per-point coordinate hover inspection in any web browser. |
| ![NumPy](https://img.shields.io/badge/-NumPy-013243?logo=numpy&logoColor=white) | [NumPy Scientific](https://numpy.org/) | Fast matrix algebra, state covariance propagation, and Direct Linear Transform (DLT) multi-view triangulation. |
| ![SciPy](https://img.shields.io/badge/-SciPy-8CAAE6?logo=scipy&logoColor=white) | [SciPy Spatial / Rotation](https://scipy.org/) | $SO(3)$ Lie group Rodrigues exponential/logarithmic maps, unit quaternion conversions, and Umeyama $SE(3)$ trajectory alignment. |
| ![RViz2](https://img.shields.io/badge/-RViz2-E74C3C?logo=ros&logoColor=white) | [ROS 2 RViz2](https://github.com/ros2/rviz) | Real-time visual dashboard showing live camera path, drone body path, 3D point cloud boxes, and tracked video overlays. |

---

## 🧱 What Was Built: Architecture in 4 Divisions

The project is structured into four clean, decoupled engineering divisions:

```
                            SYNCHRONIZED SENSOR STREAMS
                                     │         │
                   /camera/image_raw (30Hz)   /imu/data (200Hz)
                                     │         │
                                     ▼         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ DIVISION 1: SENSOR INGESTION & OPTICAL FLOW TRACKING                        │
│ • Pyramidal Lucas-Kanade (KLT) Sparse Optical Flow Tracker                  │
│ • FAST Corner Feature Detection with Spatial Grid Bucketing                 │
│ • Bidirectional Forward-Backward Error Validation & Outlier Rejection       │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ DIVISION 2: MATHEMATICAL MSCKF ESTIMATION CORE                              │
│ • 15-State Error-State Kalman Filter (Position, Velocity, Orientation, Biases)│
│ • 200 Hz IMU Kinematic Integration via SO(3) Lie Algebra Matrix Exponential │
│ • Sliding-Window Stochastic Cloning of Historical Camera Poses              │
│ • Nullspace Projection of Feature Jacobians (eliminates landmarks from P)   │
│ • Zero-Velocity Update (ZUPT) to prevent static table drift                 │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
                   ┌───────────────────┴───────────────────┐
                   ▼                                       ▼
┌──────────────────────────────────────┐  ┌───────────────────────────────────┐
│ DIVISION 3: EXTRINSIC TRANSFORMS &   │  │ DIVISION 4: 3D RECONSTRUCTION &   │
│ CAMERA OPTICAL TRAJECTORY (T_WC)     │  │ INTERACTIVE MOVEABLE VISUALIZERS  │
│ • Rigid SE(3) Body-to-Camera Extr.   │  │ • Real-time Multi-View Landmark   │
│   p_WC = p_WB + R_WB * p_BC          │    Triangulation (Linear DLT)        │
│   R_WC = R_WB * R_BC                 │  │ • 3D Voxel Grid Occupancy Map     │
│ • Standard TUM Trajectory Exporter   │  │ • Poisson Surface Room Meshing    │
│ • CSV Trajectory Exporter with Vels  │  │ • Standalone Plotly 3D HTML Graph │
│ • 100% 1-to-1 Image Timestamp Sync   │  │ • Dynamic Open3D 3D Desktop App   │
│ • Umeyama SE(3) ATE/RPE Benchmarking │  │ • Live Dual-Path RViz2 Dashboard  │
└──────────────────────────────────────┘  └───────────────────────────────────┘
```

---

### 🔹 Division 1: Sensor Ingestion & Optical Flow Feature Tracking
* **KLT Pyramidal Tracking**: Follows visual feature points across consecutive camera frames using 3-level image pyramids.
* **Grid Bucketing**: Divides the camera image into an $8 \times 6$ grid and enforces uniform feature extraction so features aren't clustered in only one corner of the room.
* **Forward-Backward Optical Flow Consistency**: Tracks points forward from Frame $k$ to $k+1$, then backward from $k+1$ to $k$; points deviating by $> 0.5$ pixels are discarded as occlusions or motion blur.

---

### 🔹 Division 2: Mathematical MSCKF Core & Online Sensor Fusion
* **Error-State Kalman Filter (ESKF)**: State vector includes 3D Position ($p_{WB}$), 3D Velocity ($v_{WB}$), 3D Orientation ($R_{WB}$), 3D Accelerometer Bias ($b_a$), and 3D Gyroscope Bias ($b_g$).
* **Continuous IMU Kinematic Propagation**: Propagates state and covariance at **200 Hz** between visual frames using continuous-time kinematics and $SO(3)$ Rodrigues rotation formulas.
* **Sliding-Window Stochastic Cloning**: Stores the past $N$ camera poses. When a tracked point leaves the camera frame, its multi-view visual observations are bundled into a single residual constraint.
* **Nullspace Projection**: The measurement residual Jacobian is projected onto the left nullspace of the 3D landmark position Jacobian ($V^T H_f = 0$). This allows measurement updates **without appending landmark positions into the filter's covariance matrix**, maintaining real-time execution speeds.
* **Zero-Velocity Update (ZUPT)**: Automatically detects when the camera is stationary on a desk (mean optical flow $< 0.6\text{ px}$) and applies an explicit zero-velocity Kalman measurement update, preventing accelerometer bias integration drift.

---

### 🔹 Division 3: Extrinsic Transformations & Camera Trajectory ($T_{WC}$)
* **The Crucial Distinction**: Most VIO systems only output the drone battery/IMU center ($T_{WB}$). However, downstream 3D mapping, photogrammetry (COLMAP), and Neural Radiance Fields (NeRF / 3D Gaussian Splatting) require the **exact optical lens pose of the camera ($T_{WC}$)**.
* **Rigid $SE(3)$ Kinematic Transformation**:
  $$\begin{bmatrix} R_{WC} & p_{WC} \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} R_{WB} & p_{WB} \\ 0 & 1 \end{bmatrix} \begin{bmatrix} R_{BC} & p_{BC} \\ 0 & 1 \end{bmatrix}$$
  $$p_{WC} = p_{WB} + R_{WB} p_{BC}, \quad R_{WC} = R_{WB} R_{BC}$$
* **Standardized Robotics File Formats**:
  * **TUM Format** (`timestamp tx ty tz qx qy qz qw`) — universal standard for SLAM evaluation tools (evo) and photogrammetry.
  * **Comprehensive CSV Format** with velocities and timestamps for engineering analysis.
* **Zero-Drift Synchronization**: 100% frame-to-pose temporal alignment ($\Delta t = 0.000\text{ ms}$).

---

### 🔹 Division 4: 3D Environment Reconstruction & Interactive Moveable Visualizers
* **Real-time Spatial Landmark Triangulation**: Triangulates 3D points from optical flow tracks across multiple camera views using Direct Linear Transform (DLT).
* **3D Voxel Grid & Surface Meshing**: Quantizes 3D space into occupancy voxels and uses Poisson surface reconstruction to produce solid 3D models (`scanned_room_mesh.ply`).
* **Interactive 3D Moveable Visualizers**:
  1. **Standalone 3D HTML WebGL Graph** (`results/plots/interactive_3d_trajectory.html`): Open in any web browser to rotate 360°, pan, zoom, and inspect timestamps and coordinates.
  2. **Dynamic Open3D Application** (`3d_view.py`): Desktop 3D GUI that auto-detects and loads the latest flight run.
  3. **RViz2 Real-Time Streaming**: Displays live dual paths (Amber Camera + Cyan Body), 3D point cloud boxes, and tracked video overlays.

---

## 📊 Evaluation & Benchmark Results

### 1. Master Trajectory Benchmark Summary Table
Evaluated using closed-form Umeyama $SE(3)$ trajectory alignment across five distinct flight profiles:

| Flight Profile | Motion Description | Camera ATE (RMSE) | RPE Drift (m/s) | Orientation Error | Velocity Error |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Orbit** | 360° circular inspection around a target | **0.3538 m** | 0.1695 m/s | **1.92°** | 0.2423 m/s |
| **Multi-Axis** | Coupled 6-DoF roll, pitch, yaw & translation | **0.3834 m** | 0.1954 m/s | **4.41°** | 0.2365 m/s |
| **Hover** | Stationary hovering under noise & air draft | **0.1341 m** | 0.0186 m/s | 160.60° | 0.0185 m/s |
| **Straight** | Forward sprint with linear acceleration | **2.7091 m** | 0.8255 m/s | 46.78° | 0.8659 m/s |
| **Stress** | Orbit under 50% feature dropouts & high noise | **52.1553 m** | 13.1805 m/s | 160.89° | 15.1754 m/s |

---

### 2. Small-Scale CSV Data Inspection

#### A. Master Benchmark Summary CSV (`results/summary.csv`)
This lightweight CSV summarizes the error metrics across all tested profiles:
```csv
Sequence,Camera ATE (m),RPE (m),Orientation (deg),Velocity (m/s)
Hover,0.1341,0.0186,160.60,0.0185
Straight,2.7091,0.8255,46.78,0.8659
Orbit,0.3538,0.1695,1.92,0.2423
Multi_axis,0.3834,0.1954,4.41,0.2365
Stress,52.1553,13.1805,160.89,15.1754
```

#### B. Standard TUM Camera Trajectory (`results/trajectories/synthetic_orbit_camera_trajectory.tum`)
This 8-column format provides the exact position and orientation for every camera frame:
```text
# timestamp tx ty tz qx qy qz qw
0.000000 4.099955 0.000000 2.500000 0.500000 0.500000 -0.500000 0.500000
0.050000 4.098739 0.051242 2.500000 0.499922 0.500078 -0.500078 0.499922
0.100000 4.095092 0.102464 2.500000 0.499688 0.500312 -0.500312 0.499688
0.150000 4.089017 0.153645 2.500000 0.499297 0.500702 -0.500702 0.499297
0.200000 4.080519 0.204766 2.500000 0.498751 0.501247 -0.501247 0.498751
```

* **`timestamp`**: Capture time in seconds (matching image frame timestamps).
* **`tx, ty, tz`**: Optical center 3D position in the World Frame (in meters).
* **`qx, qy, qz, qw`**: Unit quaternion representing camera orientation in 3D space.

---

### 3. Interactive 3D Moveable Graphs

This repository provides multiple interactive ways to view and rotate the 3D results:

1. **Standalone 3D Moveable Browser Graph (`results/plots/interactive_3d_trajectory.html`)**:
   * Double-click or open this HTML file in **Google Chrome, Firefox, Safari, or Microsoft Edge**.
   * **Left-Click & Drag**: Freely rotate the trajectory in 360 degrees.
   * **Scroll Wheel**: Smoothly zoom into individual flight maneuvers.
   * **Right-Click & Drag**: Pan across the 3D room coordinates.
   * **Hover**: Point your mouse at any part of the flight path to see the exact time in seconds and $(X, Y, Z)$ coordinates.
   * *To re-generate this HTML plot from any CSV trajectory:*
     ```bash
     python3 scripts/generate_interactive_3d.py
     ```

2. **Interactive Open3D Desktop Visualizer (`3d_view.py`)**:
   * Automatically scans `results/` for the latest flight run:
     ```bash
     python3 3d_view.py
     ```
   * Or view specific flight sequences:
     ```bash
     python3 3d_view.py --preset orbit
     python3 3d_view.py --list
     ```

3. **High-Resolution Static Plots & Error Dashboards (`results/plots/`)**:
   * `synthetic_orbit_camera_trajectory_3d.png`: 3D side-by-side comparison of Ground Truth vs. Estimated flight path.
   * `synthetic_orbit_camera_error_dashboard.png`: Millimetric $X, Y, Z$ error breakdown and $SO(3)$ angular drift curves over time.

---

## 📁 Repository Directory Structure

```text
vio_ws/
├── 3d_view.py                        # Dynamic Open3D interactive viewer for trajectories & 3D points
├── package.xml                       # ROS 2 package manifest & dependencies
├── setup.py                          # Setuptools build configuration & package registry
├── setup.cfg                         # ROS 2 script installation layout
├── README.md                         # Project documentation
│
├── config/
│   ├── project_vio.yaml              # Master VIO configuration, camera intrinsics & noise parameters
│   ├── synthetic.yaml                # Synthetic flight simulation parameters
│   ├── euroc.yaml                    # EuRoC MAV benchmark calibration configuration
│   └── vio_rviz.rviz                 # RViz2 pre-configured layout (dual paths, point cloud, video)
│
├── core/
│   ├── msckf_estimator.py            # MSCKF estimation engine, sliding-window clones & ZUPT
│   ├── feature_tracker.py            # KLT optical flow tracker & FAST corner detector
│   ├── imu_propagator.py             # 200 Hz IMU kinematic integrator & covariance propagation
│   ├── triangulation.py              # Multi-view Linear DLT landmark triangulation
│   ├── frames.py                     # Spatial frame transforms (World, Body, Camera)
│   ├── math_utils.py                 # SO(3) Lie algebra Rodrigues exponential/logarithmic maps
│   └── state.py                      # VIOState data contract
│
├── trajectory/                       # Phase 2 Core Camera Trajectory Package
│   ├── __init__.py                   # Package exports
│   ├── camera_pose.py                # BodyPose, CameraPose, Extrinsics & SE(3) transformation
│   ├── trajectory_exporter.py        # Standard TUM & CSV trajectory exporters
│   └── trajectory_metrics.py         # Umeyama SE(3) alignment, ATE/RPE metrics & 3D plot generation
│
├── ros2_nodes/
│   └── vio_node.py                   # ROS 2 Humble node publisher, subscriber & real-time auto-saver
│
├── interfaces/
│   ├── vio_interface.py              # OS/framework-agnostic facade bridging core and middleware
│   ├── mock_ros2_player.py           # Simulated ROS 2 priority event loop for offline testing
│   └── packets.py                    # Sensor packet definitions (IMUPacket, ImagePacket)
│
├── evaluation/
│   ├── synthetic/
│   │   └── evaluate_synthetic.py     # Benchmark runner across all 5 flight presets & summary compiler
│   ├── metrics.py                    # General trajectory metric calculators
│   ├── plots.py                      # Matplotlib dashboard plotter
│   └── trajectory_alignment.py       # Umeyama SE(3) SVD alignment implementation
│
├── scripts/
│   ├── generate_interactive_3d.py    # Generates standalone Plotly 3D HTML moveable graph
│   ├── build_dense_room_mesh.py      # Generates 3D Voxel Grid and Poisson Surface Room Mesh
│   ├── run_synthetic.py              # Synthetic drone flight simulator script
│   └── run_euroc.py                  # EuRoC MAV sequence runner
│
├── tests/
│   ├── test_transforms.py            # 4x4 SE(3) matrix transformation unit tests
│   ├── test_camera_pose.py           # CameraPose, trajectory interpolation & TUM export tests
│   ├── test_math_and_jacobians.py    # Lie algebra and mathematical Jacobian unit tests
│   └── test_interface_and_mock_ros.py# ROS 2 mock player & callback tests
│
└── results/                          # Generated outputs & evaluation artifacts
    ├── summary.csv                   # Master benchmark comparison table
    ├── plots/                        # Interactive HTML 3D graphs, 3D trajectory plots & dashboards
    │   ├── interactive_3d_trajectory.html
    │   └── synthetic_*_camera_trajectory_3d.png
    ├── trajectories/                 # Exported TUM and CSV camera trajectory files
    ├── pointclouds/                  # Exported 3D landmark maps (.ply, .pcd) and room mesh models
    └── metrics/                      # JSON metric reports for each sequence
```

---

## 🚀 Quickstart & How to Run

### 1. Prerequisites & Environment Setup
* **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
* **Robotics Middleware**: ROS 2 Humble Hawksbill (`ros-humble-desktop`)
* **Python**: 3.10+
* **System & Python Libraries**:
  ```bash
  sudo apt install -y ros-humble-sensor-msgs-py ros-humble-cv-bridge ros-humble-tf2-ros
  pip install numpy scipy opencv-python pyyaml open3d matplotlib pandas plotly pytest
  ```

---

### 2. Build the ROS 2 Workspace
```bash
cd ~/ros2_vio_ws

# 1. Source ROS 2 Humble base
source /opt/ros/humble/setup.bash

# 2. Build package with symlink install
colcon build --symlink-install --packages-select vio_estimator

# 3. Source the built workspace
source install/setup.bash
```

---

### 3. Launch Real-Time VIO Node + RViz2 Visualizer
To run the full VIO estimator node with live camera feed and IMU data alongside RViz2:
```bash
ros2 launch vio_estimator vio_launch.py
```

* **Live RViz2 Visualization Elements**:
  * 🟨 **Amber Path**: Real-time **Camera Trajectory ($T_{WC}$)** (`/drone/vio/camera_path`).
  * 🟦 **Cyan Path**: Real-time **Drone Body Path ($T_{WB}$)** (`/drone/vio/path`).
  * 📦 **3D Boxes**: Triangulated **3D Landmark Feature Map** (`/drone/vio/pointcloud`).
  * 📷 **Image Overlay**: Live video with green tracked optical flow points (`/camera/vio_overlay`).

---

### 4. View Interactive 3D Moveable Graphs
```bash
# Option A: Open the interactive 3D WebGL graph in your web browser (rotate 360°, pan, zoom)
xdg-open results/plots/interactive_3d_trajectory.html

# Option B: Launch the dynamic Open3D desktop viewer (loads the newest generated run)
python3 3d_view.py

# Option C: Inspect a specific preset in Open3D
python3 3d_view.py --preset orbit
```

---

### 5. Run the Phase 2 Synthetic Trajectory Benchmark Suite
To simulate all 5 flight presets (`hover`, `straight`, `orbit`, `multi_axis`, `stress`) and generate updated 3D plots, metrics, and summary tables:
```bash
python3 evaluation/synthetic/evaluate_synthetic.py --preset all
```

---

### 6. Run the Automated Test Suite
To verify all coordinate frames, $SE(3)$ transformations, Jacobians, and TUM exporters:
```bash
python3 -m pytest tests/
```
*(All 17 unit tests execute in under 2 seconds).*

---

## 📡 ROS 2 Topic Specifications

| Topic Name | Message Type | Rate | Description |
| :--- | :--- | :---: | :--- |
| `/imu/data` | `sensor_msgs/msg/Imu` | 200 Hz | Input 6-axis linear acceleration & angular velocity *(Subscribed)* |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 20–30 Hz | Input grayscale camera image frames *(Subscribed)* |
| `/drone/vio/odometry` | `nav_msgs/msg/Odometry` | 200 Hz | Estimated 6-DoF drone position, velocity & orientation quaternion |
| `/drone/vio/camera_pose` | `geometry_msgs/msg/PoseStamped` | 20–30 Hz | **Phase 2 True Camera Optical Pose ($T_{WC}$)** |
| `/drone/vio/camera_path` | `nav_msgs/msg/Path` | 20–30 Hz | **Phase 2 Continuous 3D Camera Trajectory Stream** |
| `/drone/vio/path` | `nav_msgs/msg/Path` | 20–30 Hz | Drone Body Flight Trajectory Stream ($T_{WB}$) |
| `/drone/vio/pointcloud` | `sensor_msgs/msg/PointCloud2` | 20–30 Hz | Triangulated 3D spatial object landmark feature map |
| `/camera/vio_overlay` | `sensor_msgs/msg/Image` | 20–30 Hz | Video feed with annotated optical flow feature tracks |
| `/tf` | `tf2_msgs/msg/TFMessage` | 20–30 Hz | Dynamic coordinate frames: `world -> base_link` & `world -> camera_optical_frame` |

---

## 📤 How to Push to Git Branch `VIO-Controller`

To push all updates, scripts, documentation, and benchmark results to the remote `VIO-Controller` branch:

```bash
cd ~/vio_ws

# 1. Check current status
git status

# 2. Stage all modifications and newly generated modules
git add 3d_view.py README.md config/ core/ evaluation/ ros2_nodes/ scripts/ setup.py tests/ trajectory/ results/ phase1_verified/

# 3. Commit with a structured message
git commit -m "Complete Phase 2: Camera Trajectory T_WC, ZUPT, interactive 3D visualizers, benchmark suite, and enhanced documentation"

# 4. Push to remote VIO-Controller branch
git push origin VIO-Controller
```

---

## 📄 License
This project is licensed under the [MIT License](LICENSE).
