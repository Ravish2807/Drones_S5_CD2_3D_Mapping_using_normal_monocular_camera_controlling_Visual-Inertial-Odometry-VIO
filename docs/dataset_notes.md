# Dataset Notes & Structure

---

## 1. EuRoC MAV Benchmark Dataset

The European Robotics Challenges (EuRoC) MAV dataset is the gold standard benchmark for visual-inertial state estimation.

### Structure of an extracted EuRoC sequence:
```
data/
└── MH_01_easy/
    └── mav0/
        ├── cam0/
        │   ├── data/                 # PNG images (timestamp.png in nanoseconds)
        │   ├── data.csv              # #timestamp [ns], filename
        │   └── sensor.yaml           # Intrinsics, distortion, T_BS
        ├── cam1/
        │   ├── data/                 # Synchronized right camera PNGs
        │   ├── data.csv
        │   └── sensor.yaml
        ├── imu0/
        │   ├── data.csv              # #timestamp [ns], w_RS_x, w_RS_y, w_RS_z, a_RS_x, a_RS_y, a_RS_z
        │   └── sensor.yaml           # Noise densities & random walks
        └── state_groundtruth_estimate0/
            ├── data.csv              # #timestamp [ns], p_RS_R_x, ..., q_RS_w, ..., v_RS_R_x, ..., b_w_RS_S_x, ..., b_a_RS_S_x
            └── sensor.yaml
```

### Dataset Access:
Download official sequences directly from the ETH ASL EuRoC page:
https://projects.asl.ethz.ch/datasets/euroc-mav/

*Recommended sequence progression:*
1. `V1_01_easy` / `MH_01_easy`: Clean trajectories, good lighting, smooth motion.
2. `V1_02_medium` / `MH_02_easy`: Moderate dynamic maneuvers.
3. `V1_03_difficult` / `MH_04_difficult`: Fast rotational motions, motion blur, aggressive dynamic transitions.

---

## 2. Project Synthetic Drone Simulator

To provide complete control over trajectory profiles and error injection without downloading multi-gigabyte archives, the repository includes a built-in synthetic visual-inertial generator.

### Generated Sequences:
1. **`hover`**: Near-stationary drone hover with small drift and sensor noise. Tests zero-velocity / slow-motion stability and scale maintenance.
2. **`straight`**: Constant forward translation with gentle acceleration. Tests linear velocity integration and monotonic feature flow.
3. **`orbit`**: 360° circular trajectory inspecting an object/building at the center. Simulates inspection and photogrammetric scanning flight paths.
4. **`multi_axis`**: Simultaneous 3-DoF translation and 3-DoF pitch/roll/yaw rotation. Tests full coupled nonlinear observability and bias estimation.
5. **`stress`**: Orbit trajectory under injected feature dropouts, elevated IMU noise, and temporal lag.
