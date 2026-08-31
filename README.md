# Drone Visual-Inertial Odometry (VIO) Pipeline - Phase 1

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)

This repository implements **Phase 1** of the autonomous drone 3D scanning project: a standalone, reproducible Visual-Inertial Odometry (VIO) state estimator and evaluation suite that fuses camera imagery and 6-axis IMU measurements before integration with ROS 2, Gazebo, ArduPilot, and downstream dense reconstruction.

---

## 1. System Architecture

```
                    SYNCHRONIZED SENSOR STREAM
                     |                      |
                 CAMERA I(t)            IMU a(t), w(t)
                     |                      |
                     v                      v
             +---------------+      +---------------+
             | Feature       |      | IMU           |
             | Tracking      |      | Propagation   |
             +-------+-------+      +-------+-------+
                     |                      |
                     +----------+-----------+
                                |
                                v
                     +---------------------+
                     | MSCKF State Update  |
                     | (Nullspace Proj.)   |
                     +----------+----------+
                                |
                                v
                     Estimated Drone State:
                     x = [p, v, R, b_a, b_g]
                                |
                                v
                     +---------------------+
                     | SE(3) Umeyama Eval  |
                     | & Quality Dashboard |
                     +---------------------+
```

---

## 2. Directory Structure

```
VIO-CV/
├── README.md                     # Project overview and reproduction guide
├── LICENSE                       # MIT License
├── THIRD_PARTY_NOTICES.md        # OpenVINS, EuRoC, DROID-SLAM attributions
├── docs/
│   ├── calibration.md            # Sensor intrinsics, extrinsics, and noise models
│   ├── coordinate_frames.md      # World (W), Body (B), Camera (C) definitions
│   ├── dataset_notes.md          # EuRoC and Synthetic dataset descriptions
│   └── experiment_protocol.md    # Metrics (ATE, RPE) and pass/fail criteria
├── config/
│   ├── euroc.yaml                # EuRoC MAV configuration
│   ├── synthetic.yaml            # Synthetic drone flight simulator config
│   └── project_vio.yaml          # Master frozen VIO configuration
├── scripts/
│   ├── run_synthetic.py          # Synthetic benchmark runner
│   ├── run_synthetic.sh          # Batch synthetic test script
│   ├── run_euroc.py              # EuRoC dataset runner
│   ├── run_euroc.sh              # Batch EuRoC test script
│   └── evaluate.py               # Multi-run evaluation aggregator
├── src/
│   ├── dataset_io/               # Data loaders (EuRoC) & Synthetic generator
│   ├── estimator_wrapper/        # MSCKF estimator, IMU propagator, feature tracker
│   ├── evaluation/               # Umeyama SE(3) alignment & ATE/RPE metrics
│   └── visualization/            # 3D trajectory, error, velocity, and dashboard plots
├── results/
│   ├── trajectories/             # Exported timestamped trajectory CSVs
│   ├── metrics/                  # JSON metric summaries
│   └── figures/                  # 3D plots, error profiles, dashboards
└── data/                         # Local dataset directory (not committed)
```

---

## 3. Quickstart & Reproducibility

### Prerequisites
* Python 3.10+
* `numpy`, `scipy`, `opencv-python`, `matplotlib`, `pyyaml`, `pandas`

### Run Drone Synthetic Benchmark
```bash
# 1. Run 360-degree orbital scanning flight
python scripts/run_synthetic.py --preset orbit

# 2. Run multi-axis coupled dynamic maneuver
python scripts/run_synthetic.py --preset multi_axis

# 3. Run stress test with feature dropouts
python scripts/run_synthetic.py --preset stress --dropout

# 4. Generate multi-run benchmark summary table
python scripts/evaluate.py
```

### Run on EuRoC MAV Benchmark Dataset
1. Download a sequence (e.g. `MH_01_easy`) from the [ETH EuRoC MAV page](https://projects.asl.ethz.ch/datasets/euroc-mav/) into `data/MH_01_easy`.
2. Execute the benchmark:
```bash
python scripts/run_euroc.py --sequence_path data/MH_01_easy
```

---

## 4. Evaluation Metrics

* **ATE (Absolute Trajectory Error)**: Global root-mean-square position error aligned via $SE(3)$ Umeyama SVD.
* **RPE (Relative Pose Error)**: Local translation/rotation drift over step intervals.
* **Velocity RMSE**: Estimation quality of the 3D dynamic velocity state.
* **Bias Stability**: Online estimation and tracking of accelerometer ($b_a$) and gyroscope ($b_g$) drift.
