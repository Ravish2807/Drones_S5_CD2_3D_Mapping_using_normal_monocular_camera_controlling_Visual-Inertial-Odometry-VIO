#!/usr/bin/env python3
"""
Interactive 3D Moveable Trajectory & Point Cloud Visualizer (HTML / WebGL).
Generates a standalone, interactive 3D HTML plot powered by Plotly.
Users can orbit 360 degrees, pan, zoom, and inspect timestamps & 3D poses.
"""

import os
import sys
import numpy as np
import pandas as pd
import plotly.graph_objects as go

def load_ply_points(ply_path):
    """Simple parser for ASCII or basic PLY point clouds."""
    if not os.path.exists(ply_path):
        return None
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(ply_path)
        pts = np.asarray(pcd.points)
        if len(pts) > 0:
            return pts
    except Exception:
        pass
    
    # Fallback basic PLY parser
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
        return np.array(points) if points else None
    except Exception as e:
        print(f"[Warning] Failed to read PLY: {e}")
        return None

def generate_interactive_3d_plot(
    csv_path="results/trajectories/synthetic_orbit_camera_trajectory.csv",
    ply_path="results/pointclouds/synthetic_orbit_scanned_map.ply",
    output_html="results/plots/interactive_3d_trajectory.html"
):
    print(f"Loading trajectory: {csv_path}")
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        return

    df = pd.read_csv(csv_path)
    times = df['timestamp'].to_numpy()
    tx = df['tx'].to_numpy()
    ty = df['ty'].to_numpy()
    tz = df['tz'].to_numpy()

    # Calculate Body path assuming standard extrinsics if available, or approximate offset
    # In VIO, T_WC = T_WB * T_BC -> p_WB = p_WC - R_WB * p_BC
    # For visual contrast, we plot the camera path and start/end markers
    fig = go.Figure()

    # 1. 3D Landmark Point Cloud
    pts = load_ply_points(ply_path)
    if pts is not None and len(pts) > 0:
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0],
            y=pts[:, 1],
            z=pts[:, 2],
            mode='markers',
            name='3D Object Landmarks',
            marker=dict(
                size=3,
                color='#00FF88',
                opacity=0.7,
                symbol='circle'
            ),
            hoverinfo='text',
            text=[f"Landmark #{i}<br>X: {p[0]:.2f}m<br>Y: {p[1]:.2f}m<br>Z: {p[2]:.2f}m" for i, p in enumerate(pts)]
        ))

    # 2. Camera Optical Trajectory (T_WC) - Golden Amber
    hover_texts = [
        f"Time: {t:.2f}s<br>X: {x:.3f}m<br>Y: {y:.3f}m<br>Z: {z:.3f}m"
        for t, x, y, z in zip(times, tx, ty, tz)
    ]
    
    fig.add_trace(go.Scatter3d(
        x=tx,
        y=ty,
        z=tz,
        mode='lines',
        name='Camera Trajectory (T_WC)',
        line=dict(
            color='#FFB300',
            width=6
        ),
        hoverinfo='text',
        text=hover_texts
    ))

    # 3. Start Marker (Green Sphere)
    fig.add_trace(go.Scatter3d(
        x=[tx[0]],
        y=[ty[0]],
        z=[tz[0]],
        mode='markers+text',
        name='Flight Start',
        text=['START'],
        textposition='top center',
        marker=dict(
            size=8,
            color='#00E676',
            symbol='diamond'
        )
    ))

    # 4. End Marker (Red Sphere)
    fig.add_trace(go.Scatter3d(
        x=[tx[-1]],
        y=[ty[-1]],
        z=[tz[-1]],
        mode='markers+text',
        name='Flight End',
        text=['END'],
        textposition='top center',
        marker=dict(
            size=8,
            color='#FF1744',
            symbol='square'
        )
    ))

    # Sleek dark-mode aesthetic
    fig.update_layout(
        title=dict(
            text="<b>🚁 Autonomous Drone 3D Camera Trajectory & Landmark Map</b><br><sup>Interactive 3D Visualizer — Monocular MSCKF-VIO (Drag to Rotate, Scroll to Zoom)</sup>",
            font=dict(family="Arial, sans-serif", size=18, color="#FFFFFF"),
            x=0.05,
            y=0.95
        ),
        scene=dict(
            xaxis=dict(title='X (meters)', backgroundcolor="#111625", gridcolor="#2A3550", showbackground=True, zerolinecolor="#4A5568", color="#A0AEC0"),
            yaxis=dict(title='Y (meters)', backgroundcolor="#111625", gridcolor="#2A3550", showbackground=True, zerolinecolor="#4A5568", color="#A0AEC0"),
            zaxis=dict(title='Z (meters)', backgroundcolor="#111625", gridcolor="#2A3550", showbackground=True, zerolinecolor="#4A5568", color="#A0AEC0"),
            aspectmode='data'
        ),
        paper_bgcolor="#0A0E17",
        plot_bgcolor="#0A0E17",
        legend=dict(
            x=0.02,
            y=0.90,
            bgcolor="rgba(17, 22, 37, 0.85)",
            bordercolor="#2A3550",
            borderwidth=1,
            font=dict(color="#FFFFFF", size=12)
        ),
        margin=dict(l=0, r=0, b=0, t=60)
    )

    os.makedirs(os.path.dirname(output_html), exist_ok=True)
    fig.write_html(output_html)
    print(f"Successfully generated interactive 3D plot: {output_html}")

if __name__ == "__main__":
    generate_interactive_3d_plot()
