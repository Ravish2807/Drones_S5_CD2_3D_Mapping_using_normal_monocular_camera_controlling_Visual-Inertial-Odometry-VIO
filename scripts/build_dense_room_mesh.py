import os
import open3d as o3d
import numpy as np

def main():
    ply_path = "results/pointclouds/scanned_map.ply"
    if not os.path.exists(ply_path):
        ply_path = "results/pointclouds/synthetic_orbit_scanned_map.ply"

    if not os.path.exists(ply_path):
        print(f"Error: Point cloud file not found at {ply_path}. Please run VIO simulation or node first.")
        return

    print(f"Loading VIO 3D Point Cloud from: {ply_path}")
    pcd = o3d.io.read_point_cloud(ply_path)
    print(f"Point cloud loaded with {len(pcd.points)} points.")

    # 1. Estimate Normals for 3D Mesh Reconstruction
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(k=15)

    # 2. Generate 3D Voxel Grid Map for Drone Obstacle Avoidance
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.15)
    voxel_path = "results/pointclouds/scanned_room_voxels.ply"
    os.makedirs("results/pointclouds", exist_ok=True)
    o3d.io.write_voxel_grid(voxel_path, voxel_grid)
    print(f"Saved 3D Voxel Grid Map (Obstacle Occupancy) to: {voxel_path}")

    # 3. Poisson Surface Reconstruction to create 3D Room Mesh
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)
    vertices_to_remove = densities < np.quantile(densities, 0.05)
    mesh.remove_vertices_by_mask(vertices_to_remove)

    mesh_path = "results/pointclouds/scanned_room_mesh.ply"
    o3d.io.write_triangle_mesh(mesh_path, mesh)
    print(f"Saved 3D Surface Mesh of Environment to: {mesh_path}")

    # 4. Display 3D Environment in Interactive Window
    print("Opening 3D Interactive Window (Point Cloud + 3D Mesh)...")
    o3d.visualization.draw_geometries([pcd, mesh], window_name="3D Room & Obstacle Reconstruction")

if __name__ == "__main__":
    main()
