import os
import sys
import glob
import time
import csv
import argparse
import numpy as np
import open3d as o3d


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
                # Exclude unwanted patterns (e.g. alignment logs)
                if any(ex in os.path.basename(m) for ex in exclude_patterns):
                    continue
                if os.path.isfile(m) and os.path.getsize(m) > 50:
                    candidates.append(m)

    if not candidates:
        return None, 0.0

    # Sort candidates by last modification time (newest first)
    candidates.sort(key=os.path.getmtime, reverse=True)
    newest = candidates[0]
    return newest, os.path.getmtime(newest)


def load_trajectory(filepath):
    """
    Robust trajectory loader supporting both:
    - Standard Phase 2 CSV: timestamp, tx, ty, tz, qx, qy, qz, qw
    - Phase 1 VIO CSV: timestamp, px, py, pz, vx, vy, vz, ...
    - Standard TUM text format: timestamp tx ty tz qx qy qz qw
    """
    if not filepath or not os.path.exists(filepath):
        return None

    positions = []
    # 1. Try CSV format
    try:
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            header = next(reader, None)
            for row in reader:
                if len(row) >= 4:
                    try:
                        px, py, pz = float(row[1]), float(row[2]), float(row[3])
                        positions.append([px, py, pz])
                    except ValueError:
                        continue
    except Exception:
        pass

    # 2. Try TUM text format if CSV yielded fewer than 2 poses
    if len(positions) < 2:
        positions = []
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    parts = line.split()
                    if len(parts) >= 4:
                        try:
                            px, py, pz = float(parts[1]), float(parts[2]), float(parts[3])
                            positions.append([px, py, pz])
                        except ValueError:
                            continue
        except Exception:
            pass

    if len(positions) < 2:
        return None
    return np.array(positions, dtype=np.float64)


def create_line_set(pts: np.ndarray, color_rgb: list) -> o3d.geometry.LineSet:
    """Constructs an Open3D LineSet from sequential 3D points."""
    lines = [[i, i + 1] for i in range(len(pts) - 1)]
    colors = [color_rgb for _ in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(pts)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def list_available_files(traj_dirs, pcd_dirs):
    """Prints all discovered trajectory and point cloud files ordered by date."""
    print("=" * 80)
    print("  AVAILABLE TRAJECTORY FILES (ORDERED NEWEST TO OLDEST):")
    print("=" * 80)
    all_trajs = []
    for d in traj_dirs:
        abs_d = os.path.expanduser(d)
        if os.path.exists(abs_d):
            for f in glob.glob(os.path.join(abs_d, "*.csv")) + glob.glob(os.path.join(abs_d, "*.tum")):
                if "alignment" not in f:
                    all_trajs.append(f)

    all_trajs.sort(key=os.path.getmtime, reverse=True)
    for f in all_trajs:
        mtime = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(f)))
        size_kb = os.path.getsize(f) / 1024.0
        print(f"  [{mtime}] ({size_kb:6.1f} KB) -> {f}")

    print("\n" + "=" * 80)
    print("  AVAILABLE POINT CLOUD FILES (ORDERED NEWEST TO OLDEST):")
    print("=" * 80)
    all_pcds = []
    for d in pcd_dirs:
        abs_d = os.path.expanduser(d)
        if os.path.exists(abs_d):
            for f in glob.glob(os.path.join(abs_d, "*.ply")) + glob.glob(os.path.join(abs_d, "*.pcd")):
                all_pcds.append(f)

    all_pcds.sort(key=os.path.getmtime, reverse=True)
    for f in all_pcds:
        mtime = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(f)))
        size_kb = os.path.getsize(f) / 1024.0
        print(f"  [{mtime}] ({size_kb:6.1f} KB) -> {f}")
    print("=" * 80)


def main():
    parser = argparse.ArgumentParser(description="Dynamic 3D Visualizer for Phase 2 Camera & Drone Trajectories")
    parser.add_argument("--traj", type=str, default=None, help="Explicit path to trajectory CSV/TUM file")
    parser.add_argument("--pcd", type=str, default=None, help="Explicit path to point cloud PLY/PCD file")
    parser.add_argument("--preset", type=str, default=None, choices=["orbit", "hover", "straight", "multi_axis", "stress", "realtime"],
                        help="Filter by specific preset name")
    parser.add_argument("--list", action="store_true", help="List all available files with modification dates and exit")
    args = parser.parse_args()

    # Search paths (supports current workspace and ros2_vio_ws, deduplicated)
    raw_traj_dirs = [
        os.path.abspath("results/trajectories"),
        os.path.expanduser("~/vio_ws/results/trajectories"),
        os.path.expanduser("~/ros2_vio_ws/results/trajectories"),
    ]
    raw_pcd_dirs = [
        os.path.abspath("results/pointclouds"),
        os.path.expanduser("~/vio_ws/results/pointclouds"),
        os.path.expanduser("~/ros2_vio_ws/results/pointclouds"),
    ]
    traj_dirs = list(dict.fromkeys([os.path.realpath(d) for d in raw_traj_dirs if os.path.exists(d)]))
    pcd_dirs = list(dict.fromkeys([os.path.realpath(d) for d in raw_pcd_dirs if os.path.exists(d)]))

    if args.list:
        list_available_files(traj_dirs, pcd_dirs)
        return

    # 1. Dynamically locate the Latest Camera Trajectory File (Phase 2: T_WC)
    if args.traj:
        cam_traj_path = args.traj
        cam_mtime = os.path.getmtime(cam_traj_path) if os.path.exists(cam_traj_path) else 0.0
    elif args.preset:
        pattern = f"*{args.preset}*camera_trajectory.csv" if args.preset != "realtime" else "camera_trajectory.csv"
        cam_traj_path, cam_mtime = find_latest_file(traj_dirs, [pattern])
    else:
        # Auto-pick the newest camera trajectory (prioritizing Phase 2 camera files)
        cam_traj_path, cam_mtime = find_latest_file(
            traj_dirs,
            ["*camera_trajectory.csv", "camera_trajectory.csv", "*camera*.tum"],
            exclude_patterns=["alignment"]
        )
        if not cam_traj_path:
            # Fallback to any latest trajectory CSV
            cam_traj_path, cam_mtime = find_latest_file(traj_dirs, ["*.csv"], exclude_patterns=["alignment"])

    # 2. Dynamically locate the Latest Point Cloud File
    if args.pcd:
        pcd_path = args.pcd
        pcd_mtime = os.path.getmtime(pcd_path) if os.path.exists(pcd_path) else 0.0
    elif args.preset:
        pattern = f"*{args.preset}*scanned_map.ply" if args.preset != "realtime" else "scanned_map.ply"
        pcd_path, pcd_mtime = find_latest_file(pcd_dirs, [pattern, f"*{args.preset}*.ply"])
    else:
        # Auto-pick the newest point cloud
        pcd_path, pcd_mtime = find_latest_file(
            pcd_dirs,
            ["scanned_map.ply", "*scanned_map.ply", "*.ply", "*.pcd"]
        )

    # 3. Dynamically locate the Latest Drone Body Trajectory (Phase 1: T_WB)
    body_traj_path, body_mtime = find_latest_file(
        traj_dirs,
        ["vio_trajectory.csv", "*estimated_trajectory.csv"],
        exclude_patterns=["camera", "alignment"]
    )

    print("=" * 80)
    print("  PHASE 2 DYNAMIC 3D VIO TRAJECTORY & MAPPING VIEWER")
    print("=" * 80)

    geometries = []

    # A. World Origin Coordinate Frame (Red=X, Green=Y, Blue=Z)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
    geometries.append(axis)

    # B. Load and Render Tracked 3D Landmark Point Cloud
    if pcd_path and os.path.exists(pcd_path):
        mtime_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(pcd_mtime))
        pcd = o3d.io.read_point_cloud(pcd_path)
        print(f"  [POINT CLOUD] Loaded: {os.path.basename(pcd_path)}")
        print(f"                -> Timestamp: {mtime_str}")
        print(f"                -> 3D Points: {len(pcd.points)} landmarks")
        if len(pcd.points) > 0:
            # If point cloud lacks color, assign a warm gradient
            if not pcd.has_colors():
                pcd.paint_uniform_color([0.2, 0.9, 0.4]) # Fresh green
            geometries.append(pcd)
    else:
        print("  [POINT CLOUD] No 3D point cloud file found.")

    # C. Load and Render Camera Trajectory (Phase 2: T_WC) in Amber/Gold
    cam_pts = load_trajectory(cam_traj_path)
    if cam_pts is not None:
        mtime_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(cam_mtime))
        print(f"  [CAMERA TRAJECTORY] Loaded: {os.path.basename(cam_traj_path)}")
        print(f"                      -> Timestamp: {mtime_str}")
        print(f"                      -> Poses:     {len(cam_pts)} camera optical poses")
        cam_line = create_line_set(cam_pts, [1.0, 0.72, 0.0]) # Amber/Gold
        geometries.append(cam_line)
    else:
        print(f"  [CAMERA TRAJECTORY] Could not load: {cam_traj_path}")

    # D. Load and Render Body Trajectory (Phase 1: T_WB) in Cyan
    body_pts = load_trajectory(body_traj_path)
    if body_pts is not None and body_traj_path != cam_traj_path:
        mtime_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(body_mtime))
        print(f"  [BODY TRAJECTORY]   Loaded: {os.path.basename(body_traj_path)}")
        print(f"                      -> Timestamp: {mtime_str}")
        print(f"                      -> Poses:     {len(body_pts)} body IMU poses")
        body_line = create_line_set(body_pts, [0.0, 0.8, 1.0]) # Bright Cyan
        geometries.append(body_line)

    if len(geometries) == 1:
        print("\n[ERROR] No valid trajectory or point cloud could be loaded.")
        print("Run 'python3 3d_view.py --list' to inspect available files.")
        return

    print("-" * 80)
    print("  3D VIEWER LEGEND:")
    print("   * 🟨 Amber / Gold Line = Phase 2 Camera Optical Trajectory (T_WC)")
    print("   * 🟦 Cyan Line        = Drone Body / IMU Trajectory (T_WB)")
    print("   * 🟢 Colored Dots     = Triangulated 3D Object Landmarks")
    print("   * 🔴🟢🔵 RGB Axes      = World Origin [0, 0, 0] (Red=X, Green=Y, Blue=Z)")
    print("=" * 80)

    # Launch Interactive Open3D Window
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Phase 2 Dynamic Viewer: Camera Trajectory & 3D Map", width=1280, height=720)
    for geom in geometries:
        vis.add_geometry(geom)

    render_option = vis.get_render_option()
    render_option.point_size = 7.0       # Bold, clearly visible landmark points
    render_option.line_width = 3.5       # Bold trajectory lines
    render_option.background_color = np.array([0.04, 0.04, 0.07]) # Sleek dark background

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
