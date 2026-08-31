# ROS 2 Interface Specification (Ubuntu 22.04 + ROS 2 Humble)

This document formalizes the interface boundary between the frozen VIO mathematical core and the ROS 2 ecosystem (Gazebo simulations, physical drone companion computers, or hardware sensor bags).

---

## 1. Input Subscriptions

| Stream | Default Topic Name | ROS 2 Message Type | Target Rate | Units / Frame ID |
| :--- | :--- | :--- | :---: | :--- |
| **IMU** | `/imu/data` | `sensor_msgs/msg/Imu` | 200 Hz | Linear accel: $\text{m/s}^2$, Angular vel: $\text{rad/s}$<br>`frame_id: "imu_link"` |
| **Camera** | `/camera/image_raw` | `sensor_msgs/msg/Image` | 20–30 Hz | 8-bit mono/BGR image matrix<br>`frame_id: "camera_optical_link"` |

---

## 2. Output Publications

| Output | Default Topic Name | ROS 2 Message Type | Target Rate | Description |
| :--- | :--- | :--- | :---: | :--- |
| **VIO Odometry** | `/drone/vio/odometry` | `nav_msgs/msg/Odometry` | 20–30 Hz | Full 6-DoF state estimate: position $[x,y,z]$, orientation $[w,x,y,z]$, velocity $[vx,vy,vz]$, and covariance. |
| **VIO Pose** | `/drone/vio/pose` | `geometry_msgs/msg/PoseStamped` | 20–30 Hz | Position and orientation in world frame for path planners / trajectory followers. |
| **TF2 Transform** | `/tf` | `tf2_msgs/msg/TFMessage` | 20–30 Hz | Dynamic coordinate frame transform: `world -> base_link`. |

---

## 3. Standard Trajectory Export Format (`vio_estimate.csv`)

Whether running from Windows synthetic playback, EuRoC offline sequences, Gazebo, or real drone logs, all outputs are recorded in the exact same schema:

```csv
timestamp,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bax,bay,baz,bgx,bgy,bgz
```

Where:
- `timestamp`: Float timestamp in seconds ($t$).
- `px, py, pz`: 3D position in the VIO World reference frame ($m$).
- `vx, vy, vz`: 3D linear velocity in the VIO World reference frame ($m/s$).
- `qw, qx, qy, qz`: Unit quaternion representing Body orientation relative to World ($R_{WB}$, Hamilton convention).
- `bax, bay, baz`: Estimated accelerometer bias vector in Body frame ($m/s^2$).
- `bgx, bgy, bgz`: Estimated gyroscope bias vector in Body frame ($rad/s$).

---

## 4. Node Execution Architecture

```
                      ROS 2 Executor Loop (rclpy.spin)
                                    │
    ┌───────────────────────────────┴───────────────────────────────┐
    ▼                                                               ▼
/imu/data                                                  /camera/image_raw
    │                                                               │
IMUPacket(t, a, w)                                         ImagePacket(t, img)
    │                                                               │
    ▼                                                               ▼
VIOInterface.push_imu()                                    VIOInterface.push_camera_frame()
    │                                                               │
    └───────────────────────► [ VIO CORE ] ◄────────────────────────┘
                                    │
                                VIOState
                                    │
                ┌───────────────────┴───────────────────┐
                ▼                                       ▼
        /drone/vio/odometry                         TF2: world -> base_link
```
