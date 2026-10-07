# toy_model3 — MATLAB live scripts for both equation flows

Six live scripts in the style of the 22AIE448 course notes, plus one helper file.
Open MATLAB (R2024a, no toolboxes needed) with this folder as the current folder.

| Live script | Blocks |
|---|---|
| `M1_1_Dynamics_Linearization.mlx` | Model 1: dynamics, Taylor / Jacobians, error dynamics |
| `M1_2_Error_State_EKF.mlx` | Model 1: error-state EKF (VIO) |
| `M1_3_SO3_Control_Closed_Loop.mlx` | Model 1: position, force/attitude, SO(3) attitude, mixer, closed loop |
| `M2_1_Flight_Data_DMDc.mlx` | Model 2: flight data, DMDc (creates `dmdc_model.mat`) |
| `M2_2_LQR_Bellman_Riccati_DARE.mlx` | Model 2: Bellman, LQR, Riccati, DARE |
| `M2_3_MPC_QP_Apply_to_Drone.mlx` | Model 2: MPC, QP, receding horizon, nonlinear flight |

`DroneLib.m` holds the shared functions. `pdf/` has every live script already run and exported.
Run the Model 2 files in order (M2_1 → M2_2 → M2_3).

The full explanation of every equation is in `docs/CD2_Drone_Documentation.pdf`.
