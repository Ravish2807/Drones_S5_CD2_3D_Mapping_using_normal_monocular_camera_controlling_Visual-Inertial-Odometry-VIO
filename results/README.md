# 📁 Results, Visualizations & Data Artifacts

This directory contains all exported camera trajectories, interactive 3D visualizations, surface models, and benchmark evaluation outputs.

---

## 🎨 Interactive 3D Moveable Visualizers

### 1. Standalone 3D WebGL Graph (Browser)
Open [`plots/interactive_3d_trajectory.html`](plots/interactive_3d_trajectory.html) in your browser:
* **Left-Click + Drag**: 360° orbital rotation.
* **Scroll Wheel**: Smooth zoom.
* **Right-Click + Drag**: Pan.
* **Mouse Hover**: Real-time inspection of $(X, Y, Z)$ coordinates and timestamps.

### 2. Open3D Desktop Visualizer
Run the dynamic viewer from the workspace root:
```bash
python3 3d_view.py
```

---

## 📂 Subdirectory Breakdown

| Directory / File | Description | Standard Formats |
| :--- | :--- | :--- |
| [`summary.csv`](summary.csv) | Master error comparison table across all 5 flight presets | CSV |
| [`plots/`](plots/) | 3D rotating GIF animations, interactive HTML WebGL, and static matplotlib dashboards | `.gif`, `.html`, `.png` |
| [`trajectories/`](trajectories/) | Millimetric camera flight trajectories with exact timestamps | `.tum`, `.csv` |
| [`pointclouds/`](pointclouds/) | Triangulated 3D spatial landmarks, voxel grid maps, and Poisson surface meshes | `.ply`, `.pcd` |
| [`metrics/`](metrics/) | Comprehensive JSON performance summaries for automated CI/CD parsing | `.json` |
