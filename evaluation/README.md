# 📊 Evaluation & Benchmarking Suite

[![Benchmarks](https://img.shields.io/badge/Benchmarks-5_Presets_Evaluated-success?style=flat-square)]()
[![Umeyama](https://img.shields.io/badge/Alignment-Umeyama_SE(3)-blue?style=flat-square)]()

The `evaluation/` directory contains tools, benchmark scripts, and automated evaluation pipelines to validate VIO accuracy against Ground Truth trajectories across realistic flight maneuvers.

---

## 🏃 Running the Benchmarks

To evaluate all five flight presets in a single automated command:
```bash
python3 evaluation/synthetic/evaluate_synthetic.py --preset all
```

Or run an individual profile:
```bash
python3 evaluation/synthetic/evaluate_synthetic.py --preset orbit
```

---

## 📋 Flight Presets & Summary Results

All error metrics are computed after closed-form Umeyama $SE(3)$ spatial alignment ($R, t$):

| Preset | Maneuver Characteristics | Camera ATE (RMSE) | RPE Drift (m/s) | Geodesic Angle Error |
| :--- | :--- | :---: | :---: | :---: |
| **Orbit** | 360° circular inspection | **0.3538 m** | 0.1695 m/s | **1.92°** |
| **Multi-Axis** | Coupled 6-DoF roll, pitch, yaw | **0.3834 m** | 0.1954 m/s | **4.41°** |
| **Hover** | Pure stationary hovering | **0.1341 m** | 0.0186 m/s | 160.60° |
| **Straight** | High-speed forward sprint | **2.7091 m** | 0.8255 m/s | 46.78° |
| **Stress** | 50% feature dropout & sensor noise | **52.1553 m** | 13.1805 m/s | 160.89° |

Machine-readable outputs are automatically exported to [`results/summary.csv`](../results/summary.csv).
