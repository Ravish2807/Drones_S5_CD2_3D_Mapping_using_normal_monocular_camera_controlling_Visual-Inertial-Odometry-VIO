# 🚁 Drones Controller

ROS 2 controller package for integrating the **SO(3) / Quaternion-based control algorithm** with the **ArduPilot + Gazebo** simulation environment.

## 📂 File Descriptions & Use Cases

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

## 🌳 Project Tree Structure

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

## 🔗 ArduPilot Integration

The controller operates as part of the following simulation pipeline:

```text
🌎 Gazebo
    │
    │ Sensor / State Data
    ▼
🛩️ ArduPilot SITL
    │
    │ ROS 2 / DDS
    ▼
🤖 drones_controller
    │
    │ SO(3) / Quaternion
    │ Control Computation
    ▼
🛩️ ArduPilot
    │
    ▼
🌎 Gazebo
```

### 🚀 Build

```bash
cd ~/drones_ws
colcon build --packages-select drones_controller
source install/setup.bash
```

### ▶️ Run

Check the available controller executables:

```bash
ros2 pkg executables drones_controller
```

Then run the required controller node:

```bash
ros2 run drones_controller <executable_name>
```

### 📡 Check ROS 2 Communication

```bash
ros2 topic list
ros2 topic echo /<topic_name>
ros2 topic info /<topic_name>
```

> **Note:** The controller should be started after the ArduPilot SITL + Gazebo simulation and required ROS 2/DDS communication are running.
