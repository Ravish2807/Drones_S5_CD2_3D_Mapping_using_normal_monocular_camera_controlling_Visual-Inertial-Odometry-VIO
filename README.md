# 🚁 ROS 2 Visual-Inertial Odometry (VIO) for Autonomous Drone 3D Navigation

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-brightgreen.svg)](https://www.python.org/downloads/)

A production-grade, standalone **Multi-State Constraint Kalman Filter (MSCKF) Visual-Inertial Odometry (VIO)** package for ROS 2 Humble. Fuses monocular camera frames and 6-axis IMU streams to estimate real-time 6-DoF drone pose, velocity, dynamic TF transforms, 3D object feature landmarks, and trajectory streams.

---

## 🌟 Key Features

* **Real-time 6-DoF State Estimation**: 200 Hz position, velocity, orientation quaternion, and online IMU bias estimation.
* **Dynamic TF Broadcasting**: Continuously publishes dynamic coordinate frame transformations (`world -> base_link`).
* **3D Landmark Feature Map**: Triangulates and streams 3D spatial object feature landmarks via `sensor_msgs/msg/PointCloud2`.
* **Annotated Camera Overlay**: Publishes camera video feed annotated with live optical flow feature tracks (`sensor_msgs/msg/Image`).
* **1-Command ROS 2 Launch**: Pre-configured launch script bringing up the VIO node alongside RViz2 pre-loaded with point cloud, trajectory path, and video overlay panels.
* **Interactive 3D Visualizer**: Included Open3D script (`3d_view.py`) for offline/interactive inspection of flight trajectory lines and 3D object feature landmarks.

---

## 📐 System Architecture

```
                    SYNCHRONIZED SENSOR STREAM
                     │                      │
             /camera/image_raw         /imu/data
                     │                      │
                     ▼                      ▼
             ┌───────────────┐      ┌───────────────┐
             │ KLT Feature   │      │ IMU           │
             │ Tracking      │      │ Propagation   │
             └───────┬───────┘      └───────┬───────┘
                     │                      │
                     └──────────┬───────────┘
                                │
                                ▼
                     ┌─────────────────────┐
                     │ MSCKF State Update  │
                     │ (Nullspace Proj.)   │
                     └──────────┬──────────┘
                                │
                                ▼
                      ROS 2 PUBLISHER NODE
                     /drone/vio/odometry
                     /drone/vio/path
                     /drone/vio/pointcloud
                     /camera/vio_overlay
                     /tf (world -> base_link)
```

---

## 📁 Repository Structure

```
ros2_vio_ws/
├── package.xml                       # ROS 2 package manifest & dependencies
├── setup.py                          # Setuptools build & launch installation script
├── setup.cfg                         # ROS 2 script configuration
├── README.md                         # Project documentation
├── 3d_view.py                        # Standalone Open3D tracking & trajectory viewer
├── launch/
│   └── vio_launch.py                 # ROS 2 launch file (VIO Node + RViz2)
├── config/
│   ├── project_vio.yaml              # VIO algorithm, camera intrinsics & IMU noise config
│   └── vio_rviz.rviz                 # RViz2 pre-configured display layout
├── ros2_nodes/
│   └── vio_node.py                   # ROS 2 Humble node publisher/subscriber
├── interfaces/
│   └── vio_interface.py              # Adapter interface wrapping estimator core
├── core/
│   ├── msckf_estimator.py            # MSCKF estimation engine & landmark map
│   └── feature_tracker.py            # KLT optical flow tracker & RGB sampling
├── scripts/
│   ├── run_synthetic.py              # Synthetic drone flight simulator & evaluator
│   └── build_dense_room_mesh.py      # Open3D Voxel Grid & Surface Mesh generator
└── results/                          # Output trajectories, point clouds & metric JSONs
```

---

## 🛠️ Prerequisites & Dependencies

* **OS**: Linux (Ubuntu 22.04 LTS recommended)
* **ROS 2**: Humble Hawksbill (`ros-humble-desktop`)
* **Python**: 3.10+
* **Dependencies**:
  ```bash
  sudo apt install ros-humble-sensor-msgs-py ros-humble-cv-bridge ros-humble-tf2-ros
  pip install numpy scipy opencv-python pyyaml open3d matplotlib pandas
  ```

---

## 🚀 Quickstart & Usage

### 1. Build the ROS 2 Workspace

```bash
cd ~/ros2_vio_ws

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build package
colcon build --packages-select vio_estimator

# Source workspace setup
source install/setup.bash
```

---

### 2. Launch VIO Node + RViz2 Visualizer

Run the unified launch file to start the VIO estimator node and open RViz2 pre-configured for 3D visualization:

```bash
ros2 launch vio_estimator vio_launch.py
```

---

### 3. Run Standalone Node

If you want to run the VIO node without RViz2:

```bash
ros2 run vio_estimator vio_node
```

---

### 4. Interactive 3D Feature & Trajectory Visualizer

To visualize the drone's 3D flight trajectory curve and tracked object feature landmarks in an interactive 3D window:

```bash
python 3d_view.py
```

---

## 📡 ROS 2 Topic Specifications

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/imu/data` | `sensor_msgs/msg/Imu` | Input 6-axis IMU stream (Subscribed) |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Input camera image feed (Subscribed) |
| `/drone/vio/odometry` | `nav_msgs/msg/Odometry` | 6-DoF Position, Velocity & Orientation at 200 Hz |
| `/drone/vio/path` | `nav_msgs/msg/Path` | Continuous 3D drone trajectory stream |
| `/drone/vio/pointcloud` | `sensor_msgs/msg/PointCloud2` | 3D triangulated object feature landmarks |
| `/camera/vio_overlay` | `sensor_msgs/msg/Image` | Live video feed with visual feature tracks |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic `world -> base_link` transform |

---

## 📊 Benchmarking & Simulation

Run synthetic drone trajectory benchmarks (Orbit, Multi-Axis, Stress test) to evaluate Absolute Trajectory Error (ATE) and Relative Pose Error (RPE):

```bash
# Run orbital scanning trajectory benchmark
python scripts/run_synthetic.py --preset orbit

# Run multi-axis maneuver benchmark
python scripts/run_synthetic.py --preset multi_axis
```

---

## 📄 License

This repository is released under the [MIT License](LICENSE).
