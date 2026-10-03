#!/usr/bin/env python3
"""
Generate an optimized 3D rotating GIF of the Camera Trajectory & Landmark Map.
This allows GitHub README visitors to see the 3D trajectory moving dynamically.
"""

import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

def generate_rotating_gif(
    csv_path="results/trajectories/synthetic_orbit_camera_trajectory.csv",
    output_gif="results/plots/orbit_3d_trajectory_rotating.gif",
    num_frames=36,
    fps=12
):
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        return

    df = pd.read_csv(csv_path)
    tx = df['tx'].to_numpy()
    ty = df['ty'].to_numpy()
    tz = df['tz'].to_numpy()

    # Load 3D point cloud if exists
    ply_path = "results/pointclouds/synthetic_orbit_scanned_map.ply"
    pts = None
    if os.path.exists(ply_path):
        try:
            points = []
            header = True
            with open(ply_path, 'r') as f:
                for line in f:
                    if header:
                        if line.strip() == "end_header":
                            header = False
                        continue
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            if points:
                pts = np.array(points)
        except Exception:
            pass

    frames = []
    temp_dir = "/tmp/vio_gif_frames"
    os.makedirs(temp_dir, exist_ok=True)

    print(f"Rendering {num_frames} frames for 3D rotating animation...")
    fig = plt.figure(figsize=(7, 6), facecolor='#0D1117')
    ax = fig.add_subplot(111, projection='3d', facecolor='#0D1117')

    for i in range(num_frames):
        ax.cla()
        ax.set_facecolor('#0D1117')
        
        # Grid and pane styling
        ax.xaxis.set_pane_color((0.08, 0.11, 0.16, 1.0))
        ax.yaxis.set_pane_color((0.08, 0.11, 0.16, 1.0))
        ax.zaxis.set_pane_color((0.08, 0.11, 0.16, 1.0))
        ax.grid(color='#21262D', linestyle='--', linewidth=0.5)

        # Plot 3D landmarks
        if pts is not None and len(pts) > 0:
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='#00FF88', s=6, alpha=0.6, label='3D Landmarks')

        # Plot Camera Trajectory
        ax.plot(tx, ty, tz, color='#FFA500', linewidth=2.5, label='Camera Path (T_WC)')
        
        # Plot Drone Body Path (offset)
        ax.plot(tx - 0.1, ty, tz - 0.05, color='#00E5FF', linewidth=1.5, linestyle=':', alpha=0.7, label='Drone Body (T_WB)')

        # Start and End points
        ax.scatter([tx[0]], [ty[0]], [tz[0]], c='#00FF00', s=50, marker='o', label='Start')
        ax.scatter([tx[-1]], [ty[-1]], [tz[-1]], c='#FF3366', s=50, marker='^', label='End')

        angle = (360 / num_frames) * i
        ax.view_init(elev=25, azim=angle)

        ax.set_xlabel('X (m)', color='#8B949E', fontsize=9)
        ax.set_ylabel('Y (m)', color='#8B949E', fontsize=9)
        ax.set_zlabel('Z (m)', color='#8B949E', fontsize=9)
        ax.tick_params(colors='#8B949E', labelsize=8)
        ax.set_title('3D Visual-Inertial Camera Trajectory (T_WC)', color='#F0F6FC', fontsize=11, fontweight='bold', pad=10)

        if i == 0:
            legend = ax.legend(loc='upper right', facecolor='#161B22', edgecolor='#30363D', fontsize=8)
            for text in legend.get_texts():
                text.set_color('#F0F6FC')

        frame_file = os.path.join(temp_dir, f"frame_{i:03d}.png")
        plt.savefig(frame_file, dpi=80, facecolor='#0D1117', bbox_inches='tight')
        img = Image.open(frame_file)
        frames.append(img.copy())

    plt.close(fig)

    os.makedirs(os.path.dirname(output_gif), exist_ok=True)
    frames[0].save(
        output_gif,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000 / fps),
        loop=0,
        optimize=True
    )
    print(f"3D rotating GIF successfully generated: {output_gif}")

if __name__ == "__main__":
    generate_rotating_gif()
