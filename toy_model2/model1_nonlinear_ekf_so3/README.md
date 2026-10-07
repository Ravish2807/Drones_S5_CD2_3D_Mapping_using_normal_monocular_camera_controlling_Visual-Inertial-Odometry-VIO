# Model 1 — nonlinear model-based estimation and geometric SO(3) control

Run everything with `run_all_model1`, or run the steps in order. State `x = (p, v, R, ω)` is stored as 18 numbers `[p; v; vec(R); ω]`. Error state `δx = [δp; δv; δθ; δω]` ∈ ℝ¹². Input `u = [T; τ]` ∈ ℝ⁴.

| Step | Equations (docs) | Inputs | Outputs |
|---|---|---|---|
| `step1_nonlinear_dynamics` | M1.1–M1.4 | x (p,v ∈ ℝ³ m, m/s; R ∈ SO(3); ω ∈ ℝ³ rad/s), u = [T (N); τ (N m)] | ẋ = f(x,u); 4 s open-loop flight; `P`, `x_hover`, `u_hover` |
| `step2_taylor_jacobian` | M1.5–M1.6 | operating point (x̂, û) | A ∈ ℝ¹²ˣ¹², B ∈ ℝ¹²ˣ⁴ (checked numerically); Taylor accuracy table |
| `step3_local_error_dynamics` | M1.7–M1.9 | A, B, Δt = 0.01 s, δx(0), δu | F = I + AΔt; linear vs. true error propagation |
| `step4_error_state_ekf` | M1.10–M1.14 | u_k, z_k (camera/VIO pose 25 Hz, gyro 100 Hz), x̂₀, P₀, Q, R_n | x̂_k = (p̂, v̂, R̂, ω̂), P_k ∈ ℝ¹²ˣ¹²; RMSE and 3σ consistency |
| `step5_position_control` | M1.15–M1.16 | p̂, v̂; p_d, v_d, p̈_d; K_p, K_v | e_p, e_v ∈ ℝ³; a_d ∈ ℝ³ (m/s²); ω_n, ζ per axis |
| `step6_force_attitude` | M1.17–M1.19 | a_d, ψ_d, m, g | F_d ∈ ℝ³ (N), b₃d, R_d ∈ SO(3) |
| `step7_attitude_control` | M1.20–M1.23 | R̂, ω̂, R_d, F_d, ω_d, ω̇_d; K_R, K_ω | e_R, e_ω ∈ ℝ³; τ ∈ ℝ³ (N m); T (N); Lyapunov check and sign test |
| `step8_actuators_feedback` | M1.24–M1.25 | u = [T; τ], Γ, k_f | motor speeds Ω ∈ ℝ⁴ (rad/s); full closed-loop helix flight |

`lib/` holds one function per block (`quad_dynamics`, `quad_jacobians`, `ekf_predict`, `ekf_update`, `position_control`, `force_attitude`, `attitude_control`, `motor_allocation`) plus SO(3) helpers (`hat`, `vee`, `so3_exp`, `so3_log`, `state_boxplus`, `state_boxminus`).

Step 7 corrects a sign error in the original flow image. With `e_ω = ω_d − ω̂` the drone tumbles within 0.07 s. With `e_ω = ω̂ − R̂ᵀR_d ω_d` (the form used by `so3_controller.py`) it recovers from a 60° tilt.
