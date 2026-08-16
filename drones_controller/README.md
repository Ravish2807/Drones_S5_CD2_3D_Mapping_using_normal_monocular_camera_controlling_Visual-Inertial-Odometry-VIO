#  Drones Controller

ROS 2 controller package for integrating the **SO(3) / Quaternion-based control algorithm** with the **ArduPilot + Gazebo** simulation environment.

##  File Descriptions & Use Cases

| File Path            | Description                                                                                  |
| -------------------- | -------------------------------------------------------------------------------------------- |
| `config/`            | ⚙️ Configuration and controller parameter files.                                             |
| `drones_controller/` | 🤖 Main Python source files containing the controller implementation and control algorithms. |
| `resource/`          | 📦 ROS 2 package resource used for package discovery and installation.                       |
| `package.xml`        | 📋 ROS 2 package metadata and required dependencies.                                         |
| `setup.py`           | 🔧 Python package setup and ROS 2 executable entry-point configuration.                      |
| `setup.cfg`          | ⚙️ Package installation and setup configuration.                                             |
| `README.md`          | 📖 Documentation for the controller package, files, and ArduPilot integration.               |

---

##  ArduPilot Published Topics

**ArduPilot SITL acts as the main vehicle-state provider for the controller.**
After enabling the ROS 2/DDS interface, ArduPilot publishes sensor, position, velocity, attitude and timing information that can be consumed by `drones_controller`.

| Topic Published by ArduPilot     | Use Case for Drone Controller                                                                                  |
| -------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `/ap/imu/experimental/data`      | 🧭 High-rate IMU data — used for angular velocity and acceleration required for attitude/control calculations. |
| `/ap/pose/filtered`              | 📍 Filtered local position + orientation — used for vehicle pose and attitude estimation.                      |
| `/ap/twist/filtered`             | 🚀 Filtered local velocity — used for velocity feedback and trajectory/position control.                       |
| `/ap/geopose/filtered`           | 🌍 Global position + orientation — useful for global navigation and position reference.                        |
| `/ap/navsat/navsat0`             | 🛰️ GPS position — used for global localization and navigation.                                                |
| `/ap/gps_global_origin/filtered` | 📌 Navigation origin — provides the reference for the local/global coordinate relationship.                    |
| `/ap/battery`                    | 🔋 Battery state — used for monitoring and safety-related controller decisions.                                |
| `/ap/airspeed`                   | 💨 Airspeed estimate — useful for vehicle-state monitoring and applicable control modes.                       |
| `/ap/tf_static`                  | 🔗 Static frame transformations — used to understand sensor/vehicle frame relationships.                       |
| `/ap/time`                       | ⏱️ ArduPilot time — useful for timestamping and synchronization.                                               |
| `/ap/clock`                      | 🕐 ROS clock — useful for simulation time synchronization.                                                     |

###  Main Topics Used by This Controller

For the **SO(3) / Quaternion controller**, the most important inputs are:

```text
/ap/imu/experimental/data
        │
        ├── Angular Velocity
        └── Linear Acceleration
                 │
                 ▼
          Controller State

/ap/pose/filtered
        │
        ├── Position
        └── Orientation
                 │
                 ▼
        Quaternion / SO(3)

/ap/twist/filtered
        │
        └── Linear Velocity
                 │
                 ▼
        Velocity Feedback
```

These inputs allow the controller to follow the general flow:

```text
🛩️ ArduPilot
      │
      │ State / Sensor Topics
      ▼
🤖 drones_controller
      │
      ├── Quaternion
      ├── SO(3) Rotation Matrix
      ├── Attitude Error
      ├── Velocity Error
      └── Control Computation
      │
      │ Control Command
      ▼
🛩️ ArduPilot
      │
      ▼
🌎 Gazebo
```

> **ArduPilot's role:** ArduPilot SITL provides the simulated vehicle state through ROS 2/DDS and receives the resulting control commands. Gazebo provides the simulated physics and environment, while `drones_controller` performs the project's custom SO(3)/Quaternion control computation.

---

##  Project Tree Structure

```text
drones_controller/
│
├── 📁 config/
│   └── Controller configuration / parameters
│
├── 📁 drones_controller/
│   └── Main controller Python source files
│
├── 📁 resource/
│   └── drones_controller
│
├── 📄 package.xml
│   └── ROS 2 package metadata & dependencies
│
├── 📄 setup.py
│   └── Python package & ROS 2 executable configuration
│
├── 📄 setup.cfg
│   └── Package installation configuration
│
└── 📄 README.md
    └── Controller documentation
```

---

##  Build

```bash
cd ~/drones_ws
colcon build --packages-select drones_controller
source install/setup.bash
```

##  Run

Check the available controller executables:

```bash
ros2 pkg executables drones_controller
```

Then run the required controller node:

```bash
ros2 run drones_controller <executable_name>
```

##  Check ArduPilot Topics

```bash
ros2 topic list
```

Inspect a topic:

```bash
ros2 topic echo /ap/pose/filtered
```

Check publishers/subscribers:

```bash
ros2 topic info /ap/pose/filtered
```

Check publishing frequency:

```bash
ros2 topic hz /ap/imu/experimental/data
```

> **Note:** The exact topics exposed depend on the ArduPilot DDS configuration/build. Use `ros2 topic list` after starting ArduPilot SITL to see the topics available in the current simulation.
