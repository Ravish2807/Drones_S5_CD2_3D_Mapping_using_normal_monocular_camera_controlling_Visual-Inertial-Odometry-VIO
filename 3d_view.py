#!/usr/bin/env python3
"""
Dynamic 3D Visualizer for VIO Camera Trajectories & 3D Environment Reconstruction.
Supports:
  - Phase 2: Dual Trajectories (Amber Camera T_WC & Cyan Body T_WB)
  - Phase 3: Dense Point Clouds, 3D Surface Meshes, Keyframe Camera Frustums,
             World Coordinate Frame, and Target Object Centroid Frame.
"""

import os
import sys
import glob
import time
import csv
import json
import argparse
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


def find_latest_file(search_dirs, patterns, exclude_patterns=None):
    """
    Dynamically scans directories for matching patterns and returns
    the most recently modified file along with its modification timestamp.
    """
    candidates = []
    exclude_patterns = exclude_patterns or []

    for s_dir in search_dirs:
        abs_s_dir = os.path.expanduser(s_dir)
        if not os.path.exists(abs_s_dir):
            continue
        for pat in patterns:
            matched = glob.glob(os.path.join(abs_s_dir, pat))
            for m in matched:
                if any(ex in os.path.basename(m) for ex in exclude_patterns):
                    continue
                if os.path.isfile(m) and os.path.getsize(m) > 50:
                    candidates.append(m)

    if not candidates:
        return None, 0.0

    candidates.sort(key=os.path.getmtime, reverse=True)
    newest = candidates[0]
    return newest, os.path.getmtime(newest)


def load_trajectory_poses(filepath):
    """
    Loads full 6-DoF trajectory poses (timestamp, position, orientation quaternion).
    Returns list of dicts: {"timestamp": t, "p": [x,y,z], "q": [qx,qy,qz,qw], "R": 3x3, "T": 4x4}.
    """
    if not filepath or not os.path.exists(filepath):
        return []

    poses = []
    # 1. Try CSV format
    try:
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            header = next(reader, None)
            for row in reader:
                if len(row) >= 8:
                    try:
                        t = float(row[0])
                        p = np.array([float(row[1]), float(row[2]), float(row[3])], dtype=np.float64)
                        q = np.array([float(row[4]), float(row[5]), float(row[6]), float(row[7])], dtype=np.float64)
                        R = Rotation.from_quat(q).as_matrix()
                        T = np.eye(4)
                        T[0:3, 0:3] = R
                        T[0:3, 3] = p
                        poses.append({"timestamp": t, "p": p, "q": q, "R": R, "T": T})
                    except ValueError:
                        continue
    except Exception:
        pass

    # 2. Try TUM format
    if len(poses) < 2:
        poses = []
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    parts = line.split()
                    if len(parts) >= 8:
                        try:
                            t = float(parts[0])
                            p = np.array([float(parts[1]), float(parts[2]), float(parts[3])], dtype=np.float64)
                            q = np.array([float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])], dtype=np.float64)
                            R = Rotation.from_quat(q).as_matrix()
                            T = np.eye(4)
                            T[0:3, 0:3] = R
                            T[0:3, 3] = p
                            poses.append({"timestamp": t, "p": p, "q": q, "R": R, "T": T})
                        except ValueError:
                            continue
        except Exception:
            pass

    return poses


def create_line_set(pts: np.ndarray, color_rgb: list) -> o3d.geometry.LineSet:
    """Constructs an Open3D LineSet from sequential 3D points."""
    lines = [[i, i + 1] for i in range(len(pts) - 1)]
    colors = [color_rgb for _ in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(pts)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def create_camera_frustum_lineset(
    T_WC: np.ndarray,
    scale: float = 0.22,
    color_rgb: Optional[List[float]] = None
) -> o3d.geometry.LineSet:
    """Creates a 3D wireframe camera frustum."""
    color = color_rgb or [1.0, 0.72, 0.0]
    fx, fy, cx, cy = 400.0, 400.0, 320.0, 240.0
    w, h = 640, 480
    s = float(scale)

    x0 = -cx * s / fx
    x1 = (w - cx) * s / fx
    y0 = -cy * s / fy
    y1 = (h - cy) * s / fy

    pts_C = np.array([
        [0.0, 0.0, 0.0],
        [x0, y0, s],
        [x1, y0, s],
        [x1, y1, s],
        [x0, y1, s],
        [0.0, y0 * 1.3, s]
    ], dtype=np.float64)

    R_WC = T_WC[0:3, 0:3]
    p_WC = T_WC[0:3, 3]
    pts_W = (R_WC @ pts_C.T).T + p_WC

    lines = [
        [0, 1], [0, 2], [0, 3], [0, 4],
        [1, 2], [2, 3], [3, 4], [4, 1],
        [1, 5], [5, 2]
    ]
    colors = [color for _ in range(len(lines))]

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts_W)
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector(colors)
    return ls


def main():
    parser = argparse.ArgumentParser(description="Phase 3 Dynamic 3D Viewer: Camera Trajectory, Frustums & Dense 3D Map")
    parser.add_argument("--traj", type=str, default=None, help="Explicit path to trajectory CSV/TUM file")
    parser.add_argument("--pcd", type=str, default=None, help="Explicit path to point cloud PLY/PCD file")
    parser.add_argument("--mesh", type=str, default=None, help="Explicit path to 3D surface mesh PLY/OBJ file")
    parser.add_argument("--preset", type=str, default=None, choices=["orbit", "hover", "straight", "multi_axis", "stress", "realtime"],
                        help="Filter by specific flight preset")
    parser.add_argument("--frustums", action="store_true", default=True, help="Render 3D camera frustums along trajectory")
    parser.add_argument("--no-frustums", dest="frustums", action="store_false", help="Disable camera frustum rendering")
    parser.add_argument("--list", action="store_true", help="List all available files with modification dates and exit")
    args = parser.parse_args()

    raw_traj_dirs = [
        os.path.abspath("results/trajectories"),
        os.path.expanduser("~/vio_ws/results/trajectories"),
    ]
    raw_pcd_dirs = [
        os.path.abspath("results/pointclouds"),
        os.path.expanduser("~/vio_ws/results/pointclouds"),
    ]
    traj_dirs = list(dict.fromkeys([os.path.realpath(d) for d in raw_traj_dirs if os.path.exists(d)]))
    pcd_dirs = list(dict.fromkeys([os.path.realpath(d) for d in raw_pcd_dirs if os.path.exists(d)]))

    if args.list:
        print("=" * 80)
        print("  AVAILABLE TRAJECTORY & RECONSTRUCTION FILES:")
        print("=" * 80)
        for d in traj_dirs + pcd_dirs:
            for f in sorted(glob.glob(os.path.join(d, "*.*")), key=os.path.getmtime, reverse=True):
                mtime = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(f)))
                size_kb = os.path.getsize(f) / 1024.0
                print(f"  [{mtime}] ({size_kb:7.1f} KB) -> {f}")
        return

    # 1. Locate Latest Camera Trajectory File (T_WC)
    if args.traj:
        cam_traj_path = args.traj
        cam_mtime = os.path.getmtime(cam_traj_path) if os.path.exists(cam_traj_path) else 0.0
    elif args.preset:
        pattern = f"*{args.preset}*camera_trajectory.csv" if args.preset != "realtime" else "camera_trajectory.csv"
        cam_traj_path, cam_mtime = find_latest_file(traj_dirs, [pattern])
    else:
        cam_traj_path, cam_mtime = find_latest_file(
            traj_dirs,
            ["*camera_trajectory.csv", "camera_trajectory.csv", "*camera*.tum"],
            exclude_patterns=["alignment"]
        )
        if not cam_traj_path:
            cam_traj_path, cam_mtime = find_latest_file(traj_dirs, ["*.csv"], exclude_patterns=["alignment"])

    # 2. Locate Latest Point Cloud File (Prioritize Phase 3 dense_reconstruction.ply)
    if args.pcd:
        pcd_path = args.pcd
        pcd_mtime = os.path.getmtime(pcd_path) if os.path.exists(pcd_path) else 0.0
    elif args.preset:
        pattern = f"*{args.preset}*dense*.ply"
        pcd_path, pcd_mtime = find_latest_file(pcd_dirs, [pattern, f"*{args.preset}*.ply"])
    else:
        pcd_path, pcd_mtime = find_latest_file(
            pcd_dirs,
            ["dense_reconstruction.ply", "*dense*.ply", "scanned_room_mesh.ply", "*.ply", "*.pcd"]
        )

    # 3. Locate Latest 3D Surface Mesh
    mesh_path = args.mesh
    if not mesh_path:
        mesh_path, _ = find_latest_file(
            pcd_dirs,
            ["dense_reconstruction_mesh.ply", "scanned_room_mesh.ply", "*mesh*.ply"]
        )

    print("=" * 80)
    print("  PHASE 3 VIO-GUIDED 3D RECONSTRUCTION & TRAJECTORY VIEWER")
    print("=" * 80)

    geometries = []

    # A. World Origin Coordinate Frame (Red=X, Green=Y, Blue=Z)
    world_axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
    geometries.append(world_axis)

    # B. Load Trajectory Poses
    cam_poses = load_trajectory_poses(cam_traj_path)
    if cam_poses:
        mtime_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(cam_mtime))
        print(f"  [CAMERA TRAJECTORY] Loaded: {os.path.basename(cam_traj_path)}")
        print(f"                      -> Timestamp: {mtime_str}")
        print(f"                      -> Poses:     {len(cam_poses)} camera optical poses")

        pts_array = np.array([p["p"] for p in cam_poses], dtype=np.float64)
        cam_line = create_line_set(pts_array, [1.0, 0.72, 0.0]) # Amber
        geometries.append(cam_line)

        # C. Render Camera Frustums along trajectory
        if args.frustums and len(cam_poses) > 0:
            stride = max(1, len(cam_poses) // 18) # ~18 frustums evenly spaced
            print(f"  [CAMERA FRUSTUMS]   Rendering wireframe frustums (every {stride} poses)...")
            for idx in range(0, len(cam_poses), stride):
                frustum = create_camera_frustum_lineset(cam_poses[idx]["T"], scale=0.22, color_rgb=[1.0, 0.75, 0.0])
                geometries.append(frustum)
            # Add latest pose frustum in bright green
            latest_frustum = create_camera_frustum_lineset(cam_poses[-1]["T"], scale=0.28, color_rgb=[0.0, 1.0, 0.3])
            geometries.append(latest_frustum)
    else:
        print(f"  [CAMERA TRAJECTORY] Could not load: {cam_traj_path}")

    # D. Load Dense 3D Point Cloud
    pcd_loaded = None
    if pcd_path and os.path.exists(pcd_path):
        mtime_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(pcd_mtime))
        pcd_loaded = o3d.io.read_point_cloud(pcd_path)
        print(f"  [3D POINT CLOUD]    Loaded: {os.path.basename(pcd_path)}")
        print(f"                      -> Timestamp: {mtime_str}")
        print(f"                      -> 3D Points: {len(pcd_loaded.points)} points")

        if len(pcd_loaded.points) > 0:
            if not pcd_loaded.has_colors():
                pcd_loaded.paint_uniform_color([0.2, 0.85, 0.4]) # Emerald Green
            geometries.append(pcd_loaded)

            # E. Target Object Centroid Coordinate Frame & Bounding Box
            pts_np = np.asarray(pcd_loaded.points)
            centroid = np.mean(pts_np, axis=0)
            extent = np.ptp(pts_np, axis=0)

            print(f"  [TARGET OBJECT]     Centroid: [{centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f}] m")
            print(f"                      Extent:   [{extent[0]:.2f} x {extent[1]:.2f} x {extent[2]:.2f}] m")

            # Object Coordinate Frame (Red=X, Green=Y, Blue=Z at object center)
            obj_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.45, origin=centroid)
            geometries.append(obj_frame)

            # Object Bounding Box (Magenta)
            bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=centroid - extent * 0.52,
                max_bound=centroid + extent * 0.52
            )
            bbox.color = [0.9, 0.1, 0.8]
            geometries.append(bbox)

    # F. Optional: 3D Surface Mesh
    if mesh_path and os.path.exists(mesh_path):
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        if len(mesh.vertices) > 0:
            mesh.compute_vertex_normals()
            print(f"  [3D SURFACE MESH]   Loaded: {os.path.basename(mesh_path)} ({len(mesh.triangles)} triangles)")
            # Wireframe or translucent mesh
            geometries.append(mesh)

    print("-" * 80)
    print("  3D VIEWER LEGEND:")
    print("   * 🟨 Amber Line          = Camera Optical Trajectory (T_WC)")
    print("   * 📐 Amber Wireframes    = Keyframe Camera Frustums (Field-of-View)")
    print("   * 🟢 Green Frustum       = Current / Final Camera Pose")
    print("   * ☁️ Colored Point Cloud  = Dense Reconstructed Object Surface")
    print("   * 🎯 Center Axis & Box   = Target Object Centroid & 3D Bounding Box")
    print("   * 🔴🟢🔵 Origin Axes     = World Coordinate Frame [0, 0, 0] (RGB = XYZ)")
    print("=" * 80)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Phase 3 VIO-Guided 3D Reconstruction Viewer", width=1280, height=720)
    for geom in geometries:
        vis.add_geometry(geom)

    render_option = vis.get_render_option()
    render_option.point_size = 5.0
    render_option.line_width = 2.5
    render_option.background_color = np.array([0.04, 0.04, 0.07]) # Dark modern background

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
