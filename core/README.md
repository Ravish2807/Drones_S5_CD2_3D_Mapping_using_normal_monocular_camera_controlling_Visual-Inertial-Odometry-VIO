# ⚙️ Core Module — Mathematical MSCKF VIO Engine

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-3498DB?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Status](https://img.shields.io/badge/Phase_1-Verified_Frozen-success)]()

The `core/` directory contains the foundational mathematics, state estimation filters, and visual tracking pipelines implementing the **Multi-State Constraint Kalman Filter (MSCKF)** for monocular visual-inertial odometry.

---

## 🏛️ Architecture & Component Mapping

| File | Primary Responsibility | Key Equations / Algorithms |
| :--- | :--- | :--- |
| [`msckf_estimator.py`](msckf_estimator.py) | Master MSCKF filter & state manager | Sliding window cloning, Nullspace projection ($V^T H_f = 0$), ZUPT |
| [`feature_tracker.py`](feature_tracker.py) | Lucas-Kanade (KLT) optical flow tracking | Pyramidal optical flow, bidirectional error ($<0.5\text{px}$), grid bucketing |
| [`imu_propagator.py`](imu_propagator.py) | 200 Hz IMU kinematic state propagation | Continuous-time state transition $\Phi(t_k + \Delta t, t_k)$, $SO(3)$ matrix exponential |
| [`triangulation.py`](triangulation.py) | 3D feature position estimation | Multi-view Linear Direct Linear Transform (DLT) via SVD |
| [`math_utils.py`](math_utils.py) | Lie algebra & rotation conversions | Rodrigues exponential $\exp(\lfloor \theta \times \rfloor)$, unit quaternions |
| [`frames.py`](frames.py) | Spatial frame definitions | Coordinate transforms between World ($W$), Body ($B$), and Camera ($C$) |
| [`state.py`](state.py) | Data contracts & types | `VIOState` structure, covariance slicing |

---

## 📐 Mathematical Pipeline

```mermaid
flowchart TD
    A["Raw IMU Data (200 Hz)"] --> B["imu_propagator.py<br/>Kinematic Integration"]
    B --> C["Error-State Propagation<br/>P = Phi * P * Phi^T + Q"]
    D["Camera Frame (30 Hz)"] --> E["feature_tracker.py<br/>KLT Optical Flow"]
    E --> F{"Is Camera Static?<br/>mean flow < 0.6 px"}
    F -- "Yes" --> G["msckf_estimator.py<br/>Zero-Velocity Update (ZUPT)"]
    F -- "No" --> H["msckf_estimator.py<br/>Stochastic State Cloning"]
    H --> I["triangulation.py<br/>Linear DLT Multi-View Triangulation"]
    I --> J["Nullspace Projection<br/>Residual r_o = V^T * r"]
    J --> K["Kalman State Correction<br/>dx = K * r_o, P = (I - K*H)*P"]
```

---

## 🔬 Core Innovations

### 1. Sliding Window Stochastic Cloning
Instead of inserting 3D landmark points permanently into the state vector (which causes $O(N^3)$ computational scaling like traditional EKF-SLAM), MSCKF clones historical camera poses at past image capture times:
$$\mathbf{X} = \begin{bmatrix} \mathbf{x}_B^T & \mathbf{x}_{C_1}^T & \dots & \mathbf{x}_{C_N}^T \end{bmatrix}^T$$

### 2. Nullspace Residual Projection
When a tracked feature drops out, its multi-view residual is linearized:
$$r_j = H_{x,j} \tilde{\mathbf{X}} + H_{f,j} \tilde{\mathbf{p}}_f + n_j$$
By projecting onto the left nullspace of landmark Jacobian $H_f$ ($V^T H_f = 0$):
$$r_{o,j} = V^T r_j = V^T H_{x,j} \tilde{\mathbf{X}} + V^T n_j$$
The 3D point $\tilde{\mathbf{p}}_f$ cancels out completely, allowing state updates without expanding the covariance matrix.

### 3. Zero-Velocity Update (ZUPT)
When the drone rests stationary on a desk:
* Zero optical parallax prevents feature triangulation.
* Unconstrained double-integration of IMU accelerometer bias causes quadratic position drift.
* `msckf_estimator.py` detects stationary states (`mean_optical_flow < 0.6 px`) and injects a pseudo-measurement $v_{WB} = \mathbf{0}$, locking dead-reckoning drift and calibrating accelerometer bias in real time.
