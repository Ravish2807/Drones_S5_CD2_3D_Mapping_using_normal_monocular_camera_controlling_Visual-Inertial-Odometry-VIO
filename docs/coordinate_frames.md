# Coordinate Frames and Transformations

To avoid ambiguity when integrating with cameras, IMUs, and downstream ROS 2 / Gazebo / ArduPilot modules, all spatial coordinate systems and transforms are frozen as follows.

---

## 1. Frame Definitions

| Frame Symbol | Name | Description | Standard Orientation |
| :--- | :--- | :--- | :--- |
| **`W`** | **World Frame (VIO)** | Gravity-aligned fixed inertial reference frame initialized at the starting drone state. | $+Z$ points opposite to gravity ($g_W = [0, 0, -9.81]^T$ m/s$^2$), $+X$ arbitrary forward, $+Y$ completes right-handed system (ENU-like). |
| **`B`** | **Body / IMU Frame** | Origin fixed at the IMU accelerometer sensing center. | $+X$ forward along drone fuselage, $+Y$ left, $+Z$ upward. |
| **`C`** | **Camera Frame** | Origin fixed at the optical center of the camera. | $+Z$ forward along optical axis (depth), $+X$ right in image plane, $+Y$ downward in image plane (standard OpenCV pinhole convention). |

---

## 2. Transformation Representation

A 3D spatial transformation from frame $A$ to frame $B$ is denoted as $T_{BA} \in SE(3)$:
$$T_{BA} = \begin{bmatrix} R_{BA} & p_B^{A\_origin} \\ 0_{1\times 3} & 1 \end{bmatrix}$$

A 3D point represented in frame $A$, $P_A$, is transformed into frame $B$ via:
$$P_B = R_{BA} P_A + p_{BA}$$

### Key Transforms
* **$T_{WB}(t)$**: The estimated 6-DoF pose of the IMU/body in the World frame at timestamp $t$.
  * $p_{WB} \in \mathbb{R}^3$: Position of the drone body in world coordinates.
  * $R_{WB} \in SO(3)$: Orientation of the drone body relative to the world.
* **$T_{BC}$**: Static camera-to-body extrinsic calibration transform.
  * $R_{BC} \in SO(3)$: Rotation matrix mapping camera vectors to body vectors.
  * $p_{BC} \in \mathbb{R}^3$: Position of the camera optical center expressed in body coordinates.
* **$T_{WC}(t)$**: The pose of the camera in the world frame:
  $$T_{WC}(t) = T_{WB}(t) \cdot T_{BC}$$
  $$R_{WC} = R_{WB} R_{BC}, \quad p_{WC} = R_{WB} p_{BC} + p_{WB}$$

---

## 3. Rotation Parameterization

* **SO(3) Matrix**: $R \in \mathbb{R}^{3\times 3}, \quad R^T R = I, \quad \det(R) = +1$.
* **Unit Quaternion**: $q = [q_w, q_x, q_y, q_z]^T$ with $\|q\| = 1$ (Hamilton convention).
* **Lie Algebra $\mathfrak{so}(3)$**: For an incremental rotation vector $\delta\theta \in \mathbb{R}^3$:
  $$\text{Exp}(\delta\theta) = \exp(\lfloor \delta\theta \times \rfloor) = I + \frac{\sin(\|\delta\theta\|)}{\|\delta\theta\|} \lfloor \delta\theta \times \rfloor + \frac{1 - \cos(\|\delta\theta\|)}{\|\delta\theta\|^2} \lfloor \delta\theta \times \rfloor^2$$
  where $\lfloor \cdot \times \rfloor$ is the skew-symmetric operator:
  $$\lfloor v \times \rfloor = \begin{bmatrix} 0 & -v_z & v_y \\ v_z & 0 & -v_x \\ -v_y & v_x & 0 \end{bmatrix}$$

---

## 4. Camera Projection Model (Pinhole + Radial-Tangential)

For a 3D landmark $P_W = [X_W, Y_W, Z_W]^T$:
1. Transform to camera coordinates:
   $$P_C = [X_C, Y_C, Z_C]^T = R_{CW} (P_W - p_{WC}) = R_{BC}^T (R_{WB}^T (P_W - p_{WB}) - p_{BC})$$
2. Normalized coordinates:
   $$x_n = \frac{X_C}{Z_C}, \quad y_n = \frac{Y_C}{Z_C}, \quad r^2 = x_n^2 + y_n^2$$
3. Radial distortion (RadTan / Brown-Conrady):
   $$x_d = x_n (1 + k_1 r^2 + k_2 r^4) + 2 p_1 x_n y_n + p_2 (r^2 + 2 x_n^2)$$
   $$y_d = y_n (1 + k_1 r^2 + k_2 r^4) + p_1 (r^2 + 2 y_n^2) + 2 p_2 x_n y_n$$
4. Pixel projection:
   $$u = f_x x_d + c_x, \quad v = f_y y_d + c_y$$
