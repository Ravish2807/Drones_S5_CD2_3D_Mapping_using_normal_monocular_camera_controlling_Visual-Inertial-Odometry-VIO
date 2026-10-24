<div align="center">

<img src="https://github.com/user-attachments/assets/060f7774-a73f-4132-9413-36887ed09cfa" alt="Amrita Vishwa Vidyapeetham" width="320">

# 3D Environment Mapping & Monocular Visual-Inertial Odometry

### Geometric Control for Autonomous Multirotor UAVs

**Group CD2 · Amrita Vishwa Vidyapeetham, Coimbatore**

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Python-3.x-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![ArduPilot SITL](https://img.shields.io/badge/ArduPilot-SITL-E76F00?style=for-the-badge)](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-264653?style=for-the-badge)](https://gazebosim.org/)
[![MATLAB](https://img.shields.io/badge/Tools-MATLAB-0076A8?style=for-the-badge)](https://www.mathworks.com/products/ros.html)
[![License](https://img.shields.io/badge/License-Apache--2.0-2A9D8F?style=for-the-badge)](LICENSE)

**Explore the project:** [Controller](#-controller-at-a-glance) · [Architecture](#-system-architecture) · [Quick start](#-quick-start) · [Results](#-results--visual-evidence) · [Team](#-team)

</div>


## 👥 Team

| Name | Roll number |
| --- | --- |
| Ravishanmugam K | `CB.SC.U4AIE24347` |
| Ishwarya Murugesan | `CB.SC.U4AIE24220` |
| Aparna Bharani | `CB.SC.U4AIE24304` |
| Cibikumar B | `CB.SC.U4AIE24212` |
| Akhilan S | `CB.SC.U4AIE24362` |

---

> [!IMPORTANT]
> **Implementation scope:** The current committed ROS 2 implementation focuses on SO(3) geometric control and ArduPilot topic integration. Monocular VIO and full 3D reconstruction are project goals/research context; this README does not present them as completed ROS 2 modules.

## ✨ Overview

Drones operating indoors or in GPS-denied environments need reliable state estimation, position control, and a useful representation of the surrounding space. This project explores that broader challenge while implementing a modular multirotor controller built around **rotation matrices and geometric attitude control on SO(3)**.

The repository combines a ROS 2 Python package, an ArduPilot SITL/Gazebo interface, MATLAB live scripts for staged modelling and experimentation, and saved result images.

### What is in this repository?

| Area | Current contents |
| --- | --- |
| **Geometric control** | Position/velocity error calculation, desired-force and attitude construction, SO(3) attitude errors, and moment-control calculations |
| **ROS 2 integration** | Controller node that consumes filtered pose/twist state and publishes velocity commands plus diagnostics |
| **Simulation workflow** | ArduPilot SITL and Gazebo integration context |
| **MATLAB experiments** | Staged live scripts covering waypoints, trajectories, synthetic sensors, state estimation, controller logic, dynamics, and results |
| **Mapping & VIO** | Project objective and research direction; dedicated end-to-end perception/mapping nodes are not currently included in the main ROS 2 package |

## 🚁 Controller at a glance

The controller avoids Euler angles as its internal attitude representation. It uses rotation matrices to represent orientation and computes geometric tracking errors to form control quantities.

- **Position and velocity feedback** to track a desired state.
- **SO(3) attitude handling** using desired rotation matrices and geometric attitude error.
- **Configurable gains and setpoints** stored in `drones_controller/config/controller.yaml`.
- **ROS 2 diagnostics** for position error, velocity error, desired force/moment, attitude error, and desired attitude.
- **Command gating**: `enable_command` defaults to `false` in the supplied configuration.

## 🧭 System architecture

The animated architecture diagram is maintained as a standalone SVG so it can be previewed and edited independently of this README. Animated nodes, connectors, and traveling dots illustrate the flow from simulation and ROS 2 state topics through the SO(3) controller to command output and diagnostics.

![Animated drone control system architecture](docs/system_architecture.svg)

**Control loop, simplified**

1. Read the filtered pose and twist state from ROS 2 topics.
2. Compare measured position and velocity with the desired state.
3. Compute desired force and construct a target attitude.
4. Calculate geometric attitude and angular-velocity errors.
5. Publish diagnostics and a bounded velocity command through the configured interface.

> [!NOTE]
> The controller's force and moment calculations are geometric-control quantities. The currently configured command interface publishes a `geometry_msgs/TwistStamped` velocity command; do not interpret the diagnostics as direct motor-level actuator outputs.

## 🛠️ Technology stack

| Technology | Role |
| --- | --- |
| [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) | Nodes, messages, topics, and runtime integration |
| [ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) | Software-in-the-loop flight stack and vehicle interface |
| [Gazebo](https://gazebosim.org/) | Simulation environment |
| Python + NumPy | Controller implementation and numerical linear algebra |
| MATLAB Live Scripts | Staged modelling and controller experiments |

## 🚀 Quick start

These steps build the `drones_controller` package inside an existing ROS 2 workspace. They do **not** install or launch ArduPilot or Gazebo for you.

### Prerequisites

- A working ROS 2 installation compatible with the workspace (the project uses ROS 2 Humble).
- Python dependencies declared by the package, including NumPy.
- A configured ArduPilot SITL/DDS and simulation environment if you intend to connect to the vehicle topics.
- `colcon` installed in your ROS 2 environment.

### 1. Put the package in your workspace

The package directory should be inside your workspace's `src/` directory. For example:

```bash
mkdir -p ~/drones_ws/src
cd ~/drones_ws/src
git clone https://github.com/Ravish2807/Drones_S5_CD2_3D_Mapping_using_normal_monocular_camera_controlling_Visual-Inertial-Odometry-VIO.git
```

If you already cloned this repository, use that local checkout instead of cloning it again.

### 2. Build the controller

```bash
cd ~/drones_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drones_controller
source install/setup.bash
```

### 3. Inspect available executables

```bash
ros2 pkg executables drones_controller
```

The package declares these console entry points:

- `controller_node` — the geometric controller node.
- `teleop` — keyboard velocity teleoperation.

### 4. Check topics before running

Start your configured simulation and DDS bridge, then inspect the topics actually available in your environment:

```bash
ros2 topic list
ros2 topic info /ap/v1/pose/filtered
ros2 topic info /ap/v1/twist/filtered
```

The controller defaults to these topic names, which can be changed via ROS parameters:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `pose_topic` | `/ap/v1/pose/filtered` | Filtered pose input |
| `twist_topic` | `/ap/v1/twist/filtered` | Filtered velocity input |
| `cmd_vel_topic` | `/ap/v1/cmd_vel` | Velocity command output |
| `control_rate` | `25.0` | Controller timer frequency in Hz |
| `desired_x`, `desired_y`, `desired_z` | `0.0`, `0.0`, `2.0` | Desired position |
| `desired_yaw` | `0.0` | Desired heading |
| `max_velocity` | `1.0` | Velocity command limit configured by the node |
| `state_timeout` | `0.5` | Maximum state age used by the controller |
| `enable_command` | `false` | Command publishing gate |

> [!WARNING]
> **Safety first:** Keep command publishing disabled while checking the topics and controller setup. Before enabling commands, verify the simulation, coordinate frames, topic remappings, setpoints, and emergency-stop procedure. Test in simulation before considering any physical vehicle.

## ⚙️ Configuration

Controller parameters live in [`drones_controller/config/controller.yaml`](drones_controller/config/controller.yaml). The file includes:

- Vehicle mass and gravity.
- Position/velocity gains (`kp`, `kv`).
- Attitude/angular-rate gains (`kR`, `kOmega`).
- Diagonal inertia values.
- Desired position and yaw.
- Command rate, velocity limit, state timeout, and command enable flag.

Treat the supplied values as example configuration, not as validated gains for every aircraft. Confirm units, frame conventions, and model assumptions for your vehicle before flight.

## 🧪 MATLAB toy-model workflow

The `Toy Model/` directory contains a staged set of MATLAB Live Scripts:

| Step | Script | Focus |
| --- | --- | --- |
| 1 | `step1_waypoints.mlx` | Waypoints |
| 2 | `step2_trajectory.mlx` | Trajectory generation |
| 3 | `step3_synthetic_sensors.mlx` | Synthetic sensing |
| 4 | `step4_state_estimation.mlx` | State estimation |
| 5 | `step5_so3_controller.mlx` | SO(3) controller |
| 6 | `step6_drone_dynamics.mlx` | Drone dynamics |
| 7 | `step7_final_results.mlx` | Final experiment/results |

Open the scripts in MATLAB to inspect the assumptions and execute the stages supported by your installed toolboxes.


## 🖼️ Results & Visual Evidence

Selected simulation and controller results are shown below.

<table>
  <tr>
    <th align="center" width="50%">Result 2</th>
    <th align="center" width="50%">Result 3</th>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="https://github.com/user-attachments/assets/d122ae33-f167-4c13-a4c2-f45f272cd94b" alt="SO(3) controller result image 2" width="480" />
    </td>
    <td align="center" width="50%">
      <img src="https://github.com/user-attachments/assets/3f261eaa-6f23-4e33-86c8-9594887bc254" alt="SO(3) controller result image 3" width="480" />
    </td>
  </tr>
</table>

### 🎥 SO(3) Target-Point Flight Demonstration

<video src="https://github.com/user-attachments/assets/9868e487-e9f7-4664-9d9b-47bc433d0293" controls muted loop playsinline width="100%">
  Your browser does not support embedded video. [Watch the SO(3) target-point flight demonstration](https://github.com/user-attachments/assets/9868e487-e9f7-4664-9d9b-47bc433d0293).
</video>

**Video:** [SO3_TARGET(pt)_FLYING.webm](https://github.com/user-attachments/assets/9868e487-e9f7-4664-9d9b-47bc433d0293)


## 📐 Key state and control quantities

| Quantity | Meaning |
| --- | --- |
| Position, `p = (x, y, z)` | Vehicle position in the configured local frame |
| Linear velocity, `v` | Translational velocity feedback |
| Orientation | Rotation-matrix representation used for geometric attitude control |
| Rotation matrix, `R ∈ SO(3)` | Geometric attitude representation |
| Desired force, `F_d` | Translational-control output |
| Desired moment, `M_d` | Geometric attitude-control quantity |
| Position/velocity/attitude errors | Published controller diagnostics |

## 📐 Model Predictive Control (MPC): Mathematical Formulation

### 1. Purpose and control-loop overview

Model Predictive Control (MPC) predicts the drone's future states over a finite horizon, selects a sequence of control inputs that minimizes a cost function, applies only the first input, and repeats the optimization using the latest state estimate. This repeated optimization is called **receding-horizon control**.

The formulation below uses a discrete-time linear prediction model identified from recorded state/input transitions. It is a local approximation intended for operation around the chosen hover condition; its accuracy must be validated against the drone model and data used in the experiment.

The pipeline is:

1. Collect consecutive state and input samples.
2. Identify the discrete-time matrices \(A\) and \(B\) using Dynamic Mode Decomposition with control (DMDc).
3. Define the prediction horizon, objective weights, and state/input constraints.
4. Stack the predictions and convert the optimization into a quadratic program (QP).
5. Solve the QP and apply only the first optimized input.
6. Convert the deviation command to absolute thrust/torque, then to rotor commands if a compatible actuator-allocation model is available.
7. Update the state estimate and repeat at the next sample.

### 2. State-space prediction model

The discrete-time model is

$$
\boxed{x_{k+1}=Ax_k+Bu_k}
$$

where \(k\) is the sample index and the sample interval used in this formulation is

$$
\Delta t=0.04\ \mathrm{s},\qquad f_s=\frac{1}{\Delta t}=25\ \mathrm{Hz}.
$$

The 12-dimensional state vector follows the variable ordering used in the formulation slides:

$$
x_k=
\begin{bmatrix}
p_x&p_y&p_z&v_x&v_y&v_z&\phi&\theta&\psi&
\omega_x&\omega_y&\omega_z
\end{bmatrix}^{T}
\in\mathbb{R}^{12}.
$$

Here, \(p_x,p_y,p_z\) are position components; \(v_x,v_y,v_z\) are linear-velocity components; \(\phi,\theta,\psi\) are the attitude coordinates used by the model; and \(\omega_x,\omega_y,\omega_z\) are angular-velocity components. The coordinate frame, angle convention, and units must be consistent throughout data collection, identification, estimation, and control.

The input is defined relative to hover thrust:

$$
u_k=
\begin{bmatrix}
\delta T_k&\tau_{x,k}&\tau_{y,k}&\tau_{z,k}
\end{bmatrix}^{T}
\in\mathbb{R}^{4},
$$

where \(\delta T_k=T_k-mg\) is the thrust deviation from hover, and \(\tau_x,\tau_y,\tau_z\) are the roll, pitch, and yaw torque commands.

The matrix dimensions are

$$
A\in\mathbb{R}^{12\times12},\qquad
B\in\mathbb{R}^{12\times4}.
$$

Thus, the state update is dimensionally consistent:

$$
(12\times12)(12\times1)+(12\times4)(4\times1)
=12\times1.
$$

### 3. Identifying \(A\) and \(B\) with DMDc

DMDc estimates a linear state-transition model from recorded samples. Let \(M\) be the number of consecutive state transitions. Form the data matrices

$$
X=\begin{bmatrix}x_0&x_1&\cdots&x_{M-1}\end{bmatrix}
\in\mathbb{R}^{12\times M},
$$

$$
X_+=\begin{bmatrix}x_1&x_2&\cdots&x_M\end{bmatrix}
\in\mathbb{R}^{12\times M},
$$

$$
U=\begin{bmatrix}u_0&u_1&\cdots&u_{M-1}\end{bmatrix}
\in\mathbb{R}^{4\times M}.
$$

Stack the state and input data and define the combined model matrix:

$$
\Omega=\begin{bmatrix}X\\U\end{bmatrix}
\in\mathbb{R}^{16\times M},\qquad
G=\begin{bmatrix}A&B\end{bmatrix}
\in\mathbb{R}^{12\times16}.
$$

The data approximately satisfy

$$
X_+\approx G\Omega.
$$

Estimate \(G\) by least squares:

$$
\min_G\|X_+-G\Omega\|_F^2.
$$

A Moore–Penrose pseudoinverse solution is

$$
\boxed{G=X_+\Omega^\dagger}
$$

and the matrices are extracted by columns:

$$
\boxed{A=G_{(:,1:12)},\qquad B=G_{(:,13:16)}}.
$$

The product dimensions are \((12\times M)(M\times16)=12\times16\). Reliable identification requires sufficiently informative data and adequate rank of \(\Omega\); the fitted model should also be checked on held-out transitions rather than judged only by its training error.

### 4. Finite-horizon MPC objective

At each control sample, MPC predicts \(N\) steps into the future. With \(N=25\) and \(\Delta t=0.04\) s, the prediction horizon is

$$
T_H=N\Delta t=25(0.04)=1.0\ \mathrm{s}.
$$

Starting from the current state estimate \(\hat{x}_k\), the predicted states satisfy

$$
x_0=\hat{x}_k,\qquad x_{i+1}=Ax_i+Bu_i.
$$

The finite-horizon objective is

$$
\boxed{
J=\sum_{i=0}^{N-1}
\left(x_i^TQx_i+u_i^TRu_i\right)+x_N^TP_fx_N
}
$$

where:

- \(Q\in\mathbb{R}^{12\times12}\), \(Q\succeq0\), weights state deviations.
- \(R\in\mathbb{R}^{4\times4}\), \(R\succ0\), penalizes control effort.
- \(P_f\in\mathbb{R}^{12\times12}\), \(P_f\succeq0\), weights the terminal state.

The optimization penalizes both departure from the desired equilibrium and excessive control effort. The numerical entries of \(Q\), \(R\), and \(P_f\) must be tuned and validated for the selected vehicle, units, and operating envelope.

The constrained optimization problem is

$$
\begin{aligned}
\min_{u_0,\ldots,u_{N-1}}\quad&
\sum_{i=0}^{N-1}(x_i^TQx_i+u_i^TRu_i)+x_N^TP_fx_N\\
\text{subject to}\quad&
x_{i+1}=Ax_i+Bu_i,\quad x_0=\hat{x}_k,\\
&u_{\min}\leq u_i\leq u_{\max},\\
&-x_{\max}\leq Cx_i\leq x_{\max}.
\end{aligned}
$$

Here \(C\) selects the state components that are constrained. For example, if velocity and roll/pitch are constrained, \(C\in\mathbb{R}^{5\times12}\) selects \(v_x,v_y,v_z,\phi,\theta\), and \(x_{\max}\in\mathbb{R}^{5}\). The chosen rows and limits must match the state ordering above.

### 5. Stack the predictions

To express the horizon as one optimization vector, stack the future inputs:

$$
\mathcal{U}=
\begin{bmatrix}u_0\\u_1\\\vdots\\u_{N-1}\end{bmatrix}
\in\mathbb{R}^{4N}.
$$

For \(N=25\), \(\mathcal{U}\in\mathbb{R}^{100}\). Stack the predicted states:

$$
\mathcal{X}=
\begin{bmatrix}x_1\\x_2\\\vdots\\x_N\end{bmatrix}
\in\mathbb{R}^{12N}.
$$

For \(N=25\), \(\mathcal{X}\in\mathbb{R}^{300}\). Repeated substitution of the state equation gives

$$
\boxed{\mathcal{X}=S_x\hat{x}_k+S_u\mathcal{U}}
$$

with

$$
S_x=\begin{bmatrix}A\\A^2\\\vdots\\A^N\end{bmatrix}
\in\mathbb{R}^{12N\times12}
$$

and

$$
S_u=
\begin{bmatrix}
B&0&\cdots&0\\
AB&B&\cdots&0\\
A^2B&AB&\cdots&0\\
\vdots&\vdots&\ddots&\vdots\\
A^{N-1}B&A^{N-2}B&\cdots&B
\end{bmatrix}
\in\mathbb{R}^{12N\times4N}.
$$

For the stated horizon, \(S_x\) is \(300\times12\), \(S_u\) is \(300\times100\), and both terms in the prediction equation are \(300\times1\).

### 6. Convert the objective to a quadratic program

Define the block-diagonal weight matrices

$$
\bar{Q}=\operatorname{diag}(\underbrace{Q,\ldots,Q}_{N-1\text{ blocks}},P_f)
\in\mathbb{R}^{12N\times12N},
$$

$$
\bar{R}=\operatorname{diag}(\underbrace{R,\ldots,R}_{N\text{ blocks}})
\in\mathbb{R}^{4N\times4N}.
$$

The terminal block in \(\bar{Q}\) weights \(x_N\); the other blocks weight \(x_1,\ldots,x_{N-1}\). The cost term for the fixed current state \(x_0=\hat{x}_k\) is constant during the optimization and may be omitted without changing the optimizer.

Substituting the stacked prediction into the cost and collecting terms gives

$$
J=\frac{1}{2}\mathcal{U}^TH\mathcal{U}+f^T\mathcal{U}+\text{constant},
$$

where

$$
\boxed{H=2(S_u^T\bar{Q}S_u+\bar{R})},\qquad
\boxed{f=2S_u^T\bar{Q}S_x\hat{x}_k}.
$$

For \(N=25\), \(H\in\mathbb{R}^{100\times100}\) and \(f\in\mathbb{R}^{100}\). The factors of two correspond to the convention that the QP objective is written with \(\frac12\mathcal{U}^TH\mathcal{U}\).

Input bounds can be stacked into linear inequalities. State bounds can also be written as linear inequalities by defining \(\bar{C}=I_N\otimes C\) and substituting \(\mathcal{X}=S_x\hat{x}_k+S_u\mathcal{U}\). The resulting standard QP is

$$
\boxed{
\begin{aligned}
\min_{\mathcal{U}}\quad&
\frac{1}{2}\mathcal{U}^TH\mathcal{U}+f^T\mathcal{U}\\
\text{subject to}\quad&
G_{\mathrm{qp}}\mathcal{U}\leq h_{\mathrm{qp}},\\
&A_{\mathrm{eq}}\mathcal{U}=b_{\mathrm{eq}}.
\end{aligned}
}
$$

The equality constraints are optional and depend on the implementation. The inequality matrix and bound vector must be built consistently from the selected input and state limits. The QP is feasible only when all imposed constraints can be satisfied simultaneously.

### 7. Terminal cost from the discrete algebraic Riccati equation

A common choice for the terminal penalty is the positive-semidefinite solution \(P\) of the discrete algebraic Riccati equation (DARE):

$$
P=A^TPA-A^TPB(R+B^TPB)^{-1}B^TPA+Q.
$$

The corresponding discrete-time LQR feedback gain is

$$
\boxed{K=(R+B^TPB)^{-1}B^TPA}
$$

and the unconstrained feedback law is \(u_k=-Kx_k\). With the dimensions defined above, \(K\in\mathbb{R}^{4\times12}\). The MPC terminal penalty can then be set to \(P_f=P\), provided the DARE is solvable for the identified model and selected weights. Stability claims must be checked using the actual identified matrices and chosen weights.

### 8. Receding-horizon control loop

After solving the QP, the optimizer returns the sequence

$$
\mathcal{U}^{*}=
\begin{bmatrix}
u_0^*&u_1^*&\cdots&u_{N-1}^*
\end{bmatrix}^{T}.
$$

Only the first input is applied:

$$
\boxed{u_{\mathrm{applied},k}=u_0^*}.
$$

At the next sample, the controller obtains a new state estimate, rebuilds the state-dependent objective and constraint bounds, solves the QP again, and applies the new first input. This feedback process accounts for newly observed deviations and avoids executing an entire open-loop plan without correction.

### 9. Convert the deviation input to actuator commands

Because the MPC input uses thrust deviation from hover, first recover absolute thrust and torque:

$$
u_{\mathrm{abs},k}=
\begin{bmatrix}T_k\\\tau_{x,k}\\\tau_{y,k}\\\tau_{z,k}\end{bmatrix}
=
\begin{bmatrix}mg\\0\\0\\0\end{bmatrix}+u_k,
\qquad T_k=mg+\delta T_k.
$$

If the model includes a valid four-rotor allocation matrix \(\Gamma\), define

$$
u_{\mathrm{abs},k}=\Gamma f_{\mathrm{rotor}},\qquad
f_{\mathrm{rotor}}=
\begin{bmatrix}f_1&f_2&f_3&f_4\end{bmatrix}^{T}.
$$

When \(\Gamma\in\mathbb{R}^{4\times4}\) is invertible,

$$
f_{\mathrm{rotor}}=\Gamma^{-1}u_{\mathrm{abs},k}.
$$

For a quadratic rotor-thrust model \(f_i=k_f\Omega_i^2\), the corresponding rotor speed is

$$
\Omega_i=\sqrt{\frac{f_i}{k_f}}.
$$

This conversion is valid only when the allocation matrix, rotor ordering, coefficient units, and actuator limits match the actual vehicle. Negative requested rotor thrusts and motor saturation must be handled explicitly; a conventional multirotor cannot produce negative rotor thrust. The equations here describe the model formulation and should not be interpreted as evidence that the current ROS 2 node publishes motor-speed commands.

### 10. State-estimator feedback

MPC depends on a current state estimate. In a linear state-estimation model, the prediction step may be written as

$$
\hat{x}_{k+1|k}=A\hat{x}_{k|k}+Bu_{\mathrm{applied},k},
$$

$$
\Sigma_{k+1|k}=A\Sigma_{k|k}A^T+Q_e,
$$

where \(\Sigma\in\mathbb{R}^{12\times12}\) is the estimation-error covariance and \(Q_e\in\mathbb{R}^{12\times12}\) is the process/model-error covariance.

For a nonlinear measurement \(z_k=h(x_k)+v_k\), linearize the measurement function about the predicted estimate:

$$
H_k=\left.\frac{\partial h}{\partial x}\right|_{\hat{x}_{k|k-1}}.
$$

The innovation and Kalman gain are

$$
r_k=z_k-h(\hat{x}_{k|k-1}),
$$

$$
L_k=\Sigma_{k|k-1}H_k^T
\left(H_k\Sigma_{k|k-1}H_k^T+R_n\right)^{-1},
$$

followed by the state correction

$$
\boxed{\hat{x}_{k|k}=\hat{x}_{k|k-1}+L_kr_k}.
$$

Here \(R_n\) is measurement-noise covariance and is distinct from the MPC control-effort matrix \(R\). The dimensions of \(H_k\), \(r_k\), and \(R_n\) depend on the measurement vector. A production EKF may use a different, larger error-state representation; this simplified equation is appropriate only if its state definition and measurement model are consistent with the MPC state.

### 11. Assumptions and implementation checks

Before using this formulation for control, verify the following:

- The state ordering, coordinate frames, angle convention, and units are identical in the data, \(A,B\), estimator, constraints, and controller.
- The identified model predicts held-out state transitions accurately enough near the intended operating point.
- The input vector is consistently defined in **deviation coordinates** until the conversion to absolute thrust/torque.
- \(Q\), \(R\), \(P_f\), state bounds, and input bounds are configured for the vehicle rather than copied blindly from an example.
- The QP remains feasible and can be solved within the available control-cycle time.
- Actuator allocation includes rotor ordering, saturation, nonnegative thrust, and the vehicle's actual thrust/torque coefficients.
- The estimator and controller agree on state dimensions and ordering.
- The MPC formulation is validated in simulation before any physical flight.

This section is a mathematical formulation and implementation guide. The repository's actual runtime behavior is determined by the committed code and configuration, as described in the implementation-scope note earlier in this README.


## 📁 Repository layout

```text
.
├── Base Paper.pdf
├── Introduction to Drones.pptx
├── LICENSE
├── README.md
├── Results/
│   ├── image1.png
│   ├── image2.png
│   ├── image3.png
│   ├── image4.png
│   └── ... additional result captures
├── Toy Model/
│   └── step1_waypoints.mlx ... step7_final_results.mlx
└── drones_controller/
    ├── README.md
    ├── package.xml
    ├── setup.py
    ├── config/
    │   └── controller.yaml
    └── drones_controller/
        ├── controller_node.py
        ├── so3_controller.py
        ├── state_adapter.py
        ├── teleop.py
        └── trajectory_generator.py
```

For package-specific details, see [`drones_controller/README.md`](drones_controller/README.md).

## 📚 References

1. **Geometric quadrotor control:** Lee, T., Leok, M., and McClamroch, N. H., *Geometric Tracking Control of a Quadrotor UAV on SE(3)* — [arXiv:1003.2005](https://arxiv.org/abs/1003.2005).
2. **Visual-inertial odometry:** [VINS-Mono paper](https://arxiv.org/abs/1708.03852).
3. **Monocular SLAM reference:** [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).
4. **ArduPilot SITL documentation:** [Developer documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html).
5. **ROS 2 Humble documentation:** [Official docs](https://docs.ros.org/en/humble/index.html).
6. **Gazebo documentation:** [Official docs](https://gazebosim.org/docs).
7. **MATLAB ROS Toolbox:** [MathWorks documentation](https://www.mathworks.com/help/ros/).
8. **Base paper:** [`Base Paper.pdf`](Base%20Paper.pdf).

---

<div align="center">

**Amrita Vishwa Vidyapeetham · Group CD2**  
*Geometric Control · ROS 2 · Autonomous UAV Research*

</div>
