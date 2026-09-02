import os
import csv
import open3d as o3d
import numpy as np

def load_trajectory(csv_path):
    """Loads 3D position points from VIO trajectory CSV."""
    if not os.path.exists(csv_path):
        return None
    
    positions = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) >= 4:
                try:
                    px, py, pz = float(row[1]), float(row[2]), float(row[3])
                    positions.append([px, py, pz])
                except ValueError:
                    continue
    if len(positions) < 2:
        return None
    return np.array(positions)

def main():
    pc_dir = "results/pointclouds"
    traj_dir = "results/trajectories"
    
    # 1. Candidate 3D Landmark Point Cloud Files
    pcd_path = os.path.join(pc_dir, "scanned_map.ply")
    if not os.path.exists(pcd_path):
        pcd_path = os.path.join(pc_dir, "synthetic_orbit_scanned_map.ply")
        
    # 2. Candidate Trajectory Files
    traj_path = os.path.join(traj_dir, "vio_trajectory.csv")
    if not os.path.exists(traj_path):
        traj_path = os.path.join(traj_dir, "synthetic_orbit_estimated_trajectory.csv")

    geometries = []

    # A. Render 3D World Coordinate Axes (Red=X, Green=Y, Blue=Z)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    geometries.append(axis)

    # B. Render Tracked 3D Object Feature Points
    if os.path.exists(pcd_path):
        print(f"Loading Tracked 3D Landmark Points from: {pcd_path}")
        pcd = o3d.io.read_point_cloud(pcd_path)
        print(f"Loaded {len(pcd.points)} tracked 3D object features.")
        
        # Increase point visibility
        geometries.append(pcd)
    else:
        print(f"No 3D landmark file found at {pcd_path}")

    # C. Render Drone Flight Trajectory (3D Line Strip)
    traj_pts = load_trajectory(traj_path)
    if traj_pts is not None:
        print(f"Loading Drone Flight Trajectory ({len(traj_pts)} poses) from: {traj_path}")
        lines = [[i, i + 1] for i in range(len(traj_pts) - 1)]
        colors = [[0, 0.8, 1.0] for _ in range(len(lines))] # Bright Cyan Line

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(traj_pts)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        geometries.append(line_set)

    if len(geometries) == 1:
        print("No feature tracks or trajectory found! Run the VIO estimator node first.")
        return

    print("=" * 60)
    print("VIO TRACKING & DRONE ODOMETRY VISUALIZER")
    print(" - Bright Cyan Line = Drone 3D Flight Path")
    print(" - Color Dots       = Tracked 3D Object Landmarks")
    print(" - RGB Axes         = World Origin (Red=X, Green=Y, Blue=Z)")
    print("=" * 60)
    
    # Launch Visualizer with larger point size
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="VIO Drone Object Tracking & Trajectory", width=1280, height=720)
    for geom in geometries:
        vis.add_geometry(geom)
    
    render_option = vis.get_render_option()
    render_option.point_size = 6.0  # Make feature points clearly visible
    render_option.line_width = 3.0  # Make trajectory line thick and clear
    render_option.background_color = np.array([0.05, 0.05, 0.08]) # Modern dark background
    
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    main()
