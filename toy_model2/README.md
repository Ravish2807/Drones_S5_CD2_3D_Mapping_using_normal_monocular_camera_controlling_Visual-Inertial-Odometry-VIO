# toy_model2 — runnable versions of the two equation flows

Each block of the two flow diagrams (`docs/mathematical_models/figures/flow_model1.png`, `flow_model2.png`) is one MATLAB script. Every script:

- prints a banner with the block's equations;
- prints every **INPUT** and **OUTPUT** with its name, size, unit, meaning and value;
- **Part A** reproduces the hand-worked toy example from `docs/mathematical_models/Mathematical_Models.pdf`, so the numbers can be checked against the document;
- **Part B** passes real data on to the next step, saving it to `results/*.mat` and figures to `figures/`.

No toolbox is needed (tested on MATLAB R2024a). The repository ships its own QP solver (`qp_solve.m`, interior point) and DARE solver (`dare_solve.m`).

| Folder | Flow | Run |
|---|---|---|
| [`model1_nonlinear_ekf_so3/`](model1_nonlinear_ekf_so3/) | Image 1: nonlinear dynamics → linearization → error-state EKF → SO(3) control → motors | `run_all_model1` |
| [`model2_dmdc_lqr_mpc/`](model2_dmdc_lqr_mpc/) | Image 2: flight data → DMDc → Bellman / LQR / Riccati / DARE and MPC / QP → drone | `run_all_model2` |

Steps can also be run one at a time, in order, because each step loads what the previous one saved. `results_log.txt` in each folder holds the complete printed output of the last full run.

Parameters (mass 1.5 kg, inertia, all gains, 2 m hover setpoint, 25 Hz, `max_velocity` 1 m/s) come from `drones_controller/config/controller.yaml`. The rotor constants are those of the ArduPilot Gazebo Iris.
