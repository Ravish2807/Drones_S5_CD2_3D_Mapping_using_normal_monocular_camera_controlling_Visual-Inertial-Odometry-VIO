# Experiment Protocol & Evaluation Metrics

---

## 1. Mathematical State Vector

The active state vector estimated by the visual-inertial estimator at time $t_k$ is:
$$x_k = \begin{bmatrix} p_{WB} \\ v_{WB} \\ q_{WB} \\ b_a \\ b_g \end{bmatrix} \in \mathbb{R}^{16}$$
with error state $\delta x \in \mathbb{R}^{15}$:
$$\delta x_k = \begin{bmatrix} \delta p \\ \delta v \\ \delta \theta \\ \delta b_a \\ \delta b_g \end{bmatrix}$$
where $\delta \theta \in \mathbb{R}^3$ represents the orientation error in $SO(3)$:
$$R_{WB} = \hat{R}_{WB} \text{Exp}(\delta \theta)$$

When visual features are tracked across multiple frames, the state vector is augmented with cloned camera poses (MSCKF formulation):
$$x_{MSCKF} = \begin{bmatrix} x_k^T & q_{C_1}^T & p_{C_1}^T & \dots & q_{C_N}^T & p_{C_N}^T \end{bmatrix}^T$$

---

## 2. Evaluation Metrics

### 2.1 Absolute Trajectory Error (ATE)
Measures the global consistency of the trajectory.
1. Align the estimated trajectory $T_{est}(t)$ and ground truth $T_{gt}(t)$ via an optimal $SE(3)$ transformation $S$ computed via Umeyama SVD:
   $$T_{\Delta}(t) = T_{gt}(t)^{-1} S T_{est}(t)$$
2. Position ATE RMSE:
   $$\text{RMSE}_{ATE} = \sqrt{\frac{1}{N} \sum_{i=1}^N \| p_{gt}(t_i) - (R_S p_{est}(t_i) + p_S) \|^2}$$

### 2.2 Relative Pose Error (RPE)
Measures the local drift over a fixed time interval $\Delta t$ or distance interval $\Delta d$:
$$E_i(\Delta t) = (T_{gt}(t_i)^{-1} T_{gt}(t_i + \Delta t))^{-1} (T_{est}(t_i)^{-1} T_{est}(t_i + \Delta t))$$
$$\text{RPE}_{trans} = \sqrt{\frac{1}{M} \sum_{i=1}^M \| \text{trans}(E_i) \|^2}$$
$$\text{RPE}_{rot} = \sqrt{\frac{1}{M} \sum_{i=1}^M \| \text{angle}(E_i) \|^2}$$

### 2.3 Velocity and Bias Stability
* **Velocity RMSE**: Assesses accuracy of the estimated dynamic state against ground truth velocities.
* **Bias Drift**: Tracks convergence and bounds of estimated accelerometer ($b_a$) and gyroscope ($b_g$) biases.

---

## 3. Pass/Fail Criteria for Phase 1 Validation

| Milestone Test | Sequence | Acceptance Criteria |
| :--- | :--- | :--- |
| **VIO-1 (Synthetic Baseline)** | `synthetic: orbit` | Continuous trajectory, $\text{RMSE}_{ATE} < 0.15$ m, no divergence. |
| **VIO-2 (Synthetic Coupled)** | `synthetic: multi_axis` | Full 6-DoF excitation, stable bias estimation, $\text{RMSE}_{ATE} < 0.20$ m. |
| **VIO-3 (Stress Test)** | `synthetic: stress` | Survives feature dropout and noise, graceful recovery. |
| **VIO-4 (EuRoC MAV)** | `EuRoC: MH_01_easy` | Stable initialization, tracking continuity $> 95\%$, $\text{RMSE}_{ATE} < 0.25$ m. |
