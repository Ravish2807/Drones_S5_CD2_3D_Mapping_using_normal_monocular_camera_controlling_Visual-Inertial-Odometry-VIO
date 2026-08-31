# Sensor Calibration and Noise Parameters Guide

Accurate sensor calibration is fundamental to visual-inertial state estimation. Small errors in intrinsics, extrinsics, or time alignment can cause state divergence.

---

## 1. Camera Intrinsics Model

The pinhole camera intrinsic matrix $K$:
$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$
where:
* $f_x, f_y$: Focal lengths along image axes in pixels.
* $c_x, c_y$: Principal point coordinates (optical center) in pixels.
* Distortion vector $D = [k_1, k_2, p_1, p_2]$ (radial $k_i$ and tangential $p_i$ coefficients).

For real sensors, calibration is performed using a high-precision AprilTag or checkerboard grid across multiple viewing angles and distances.

---

## 2. IMU Stochastic Noise Model (Allan Variance)

The continuous-time IMU measurement model is:
$$a_m(t) = R_{WB}^T(t)(a_W(t) - g_W) + b_a(t) + n_a(t)$$
$$\omega_m(t) = \omega(t) + b_g(t) + n_g(t)$$

where:
* **White Noise (Continuous Density)**:
  * Accelerometer noise density: $\sigma_{a} \text{ [m/s}^2/\sqrt{\text{Hz}}\text{]}$ or $\sigma_{a\_discrete} = \frac{\sigma_a}{\sqrt{\Delta t}}$
  * Gyroscope noise density: $\sigma_{g} \text{ [rad/s}/\sqrt{\text{Hz}}\text{]}$ or $\sigma_{g\_discrete} = \frac{\sigma_g}{\sqrt{\Delta t}}$
* **Bias Random Walk**:
  * Accelerometer random walk: $\sigma_{ba} \text{ [m/s}^3/\sqrt{\text{Hz}}\text{]}$
  * Gyroscope random walk: $\sigma_{bg} \text{ [rad/s}^2/\sqrt{\text{Hz}}\text{]}$
  $$\dot{b}_a(t) = n_{ba}(t), \quad \dot{b}_g(t) = n_{bg}(t)$$

### Standard EuRoC MAV IMU Parameters:
* Gyroscope continuous noise ($\sigma_g$): `1.6968e-04` $\text{rad/s}/\sqrt{\text{Hz}}$
* Gyroscope random walk ($\sigma_{bg}$): `1.9393e-05` $\text{rad/s}^2/\sqrt{\text{Hz}}$
* Accelerometer continuous noise ($\sigma_a$): `2.0000e-03` $\text{m/s}^2/\sqrt{\text{Hz}}$
* Accelerometer random walk ($\sigma_{ba}$): `3.0000e-03` $\text{m/s}^3/\sqrt{\text{Hz}}$

---

## 3. Spatial Extrinsics ($T_{BC}$)

Spatial extrinsics define the rigid 6-DoF transformation from the camera optical center to the IMU coordinate frame:
* $R_{BC} \in SO(3)$: Rotation matrix aligning the camera axes with the body/IMU axes.
* $p_{BC} \in \mathbb{R}^3$: Lever arm vector from IMU center of mass to the camera optical center.

---

## 4. Temporal Calibration ($t_{offset}$)

Visual and inertial measurements must be expressed in a synchronized time frame:
$$t_{camera} = t_{imu} + t_{offset}$$
In the EuRoC benchmark dataset, timestamps are synchronized hardware-side. In our synthetic test generator, temporal offsets can be artificially injected to test estimator robustness.
