# Model 2 — data-driven model (DMDc), optimal control (LQR / MPC) and state estimation (EKF)

Run everything with `run_all_model2`, or run the steps in order. Everything is discrete-time at Δt = 0.04 s (25 Hz). State `x_k = [p − p_ref; v; Log(R); ω]` ∈ ℝ¹² and input `u_k = [T − m g₀; τ]` ∈ ℝ⁴ are deviations from hover.

| Step | Equations (docs) | Inputs | Outputs |
|---|---|---|---|
| `step1_flight_data` | M2.1 | nonlinear drone, stabilising pilot, random excitation | 5 flights × 625 samples of x_k ∈ ℝ¹², u_k ∈ ℝ⁴ |
| `step2_dmdc` | M2.2–M2.4 | X, X⁺ ∈ ℝ¹²ˣ⁽ᴺ⁻¹⁾, U ∈ ℝ⁴ˣ⁽ᴺ⁻¹⁾ | A ∈ ℝ¹²ˣ¹², B ∈ ℝ¹²ˣ⁴ (compared with physics, validated on unseen flight) |
| `step3_bellman_cost` | M2.5–M2.7 | A, B; Bryson limits | Q ∈ ℝ¹²ˣ¹², R ∈ ℝ⁴ˣ⁴; Bellman minimiser checked by brute force |
| `step4_lqr_gain` | M2.8–M2.9 | A, B, R, P | K ∈ ℝ⁴ˣ¹², u_k = −K x_k |
| `step5_riccati_recursion` | M2.10 | A, B, Q, R, P₀ = Q | P_k → P_∞ (convergence plot) |
| `step6_dare` | M2.11 | A, B, Q, R | P (direct solution), final K; cost check x₀ᵀPx₀ |
| `step7_mpc_problem` | M2.12 | A, B, Q, R, P_f = P, N = 25, input / speed / tilt limits | MPC problem struct |
| `step8_qp_formulation` | M2.13–M2.15 | MPC problem, x₀ | S_x, S_u, H ∈ ℝ¹⁰⁰ˣ¹⁰⁰, f, G ∈ ℝ⁴⁵⁰ˣ¹⁰⁰, h; U* (MPC = LQR check) |
| `step9_mpc_control_input` | M2.16 | x_k at every sample | U* = [u₀*, …, u_{N−1}*], applied u₀*; MPC vs. LQR on learned model |
| `step10_apply_to_drone` | M2.17–M2.18 | u₀* or −K x_k, u_trim | [T; τ] → rotor thrusts → Ω ∈ ℝ⁴; nonlinear flight comparison (controllers given the true state) |
| `step11_state_estimation_ekf` | M2.19–M2.24 | applied u_k, camera pixels of known floor markers, gyro, learned A, B | estimate x̂_k ∈ ℝ¹², covariance Σ_k ∈ ℝ¹²ˣ¹²; nonlinear flight with MPC / LQR fed by x̂_k |

`lib/` contains `dmdc`, `bellman_backup`, `lqr_gain`, `riccati_recursion`, `dare_solve`, `mpc_build`, `mpc_solve` and `qp_solve`, the EKF pieces `camera_model`, `camera_measure` (pinhole model and its Jacobian), `so3_right_jacobian`, `ekf2_predict` and `ekf2_update`, plus copies of the physics helpers from Model 1, so this folder runs on its own.

## Why step 11 (the EKF) exists

Up to step 10 the controllers are handed the **true** state plus a little noise. A real drone never sees its true state. The camera gives pixels, the gyroscope gives body rates, and nothing measures the velocity.

Step 11 adds block 11, which closes Model 2's loop:

apply to drone (10) → drone moves → camera + gyro → **EKF (11)** → x̂ₖ → MPC (9) / LQR

The filter works in three parts:

- **Prediction** uses the *learned* DMDc model, so no mass or inertia is needed.
- **Process noise** `Q_e` is learned from the DMDc residuals on the held-out flight.
- **Correction** uses the pinhole-camera equation `pixel = f·q₁/q₃ + c`. That equation is nonlinear, which is why the filter is *extended*: it needs the Jacobian of `camera_measure.m`.

Results: about 3 mm position error and 3 cm/s velocity error, even though velocity is never measured. 98.7 % of errors stay inside the predicted ±3σ band, and the drone reaches the setpoint flying only on the estimate.
