from ardulog import run_ardulog, run_home
from config import INPUTS_PATH, OUTPUTS_PATH

import numpy as np
import open3d as o3d
import os


def add_home(_vis):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    sphere.paint_uniform_color([0, 0, 1])
    sphere.translate([0, 0, 0])
    _vis.add_geometry(sphere)

def add_cloud(_vis, _idx):
    pcd = o3d.io.read_point_cloud(os.path.join(f'{INPUTS_PATH}', f'rtabmap_cloud_{_idx}.ply'))
    _vis.add_geometry(pcd)

def add_landing_cloud(_vis, _idx):
    pcd = o3d.io.read_point_cloud(os.path.join(f'{OUTPUTS_PATH}', f'landing_cloud_{_idx}.ply'))

    _vis.add_geometry(pcd)

    for i in range(len(pcd.points)):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
        sphere.paint_uniform_color(pcd.colors[i])
        sphere.translate(pcd.points[i])
        _vis.add_geometry(sphere)

def save_landing_cloud(_log_file, _idx):
    coords_s, coords_f = run_ardulog(_log_file)

    # print('coords_s:')
    # print(coords_s)
    # print('coords_f:')
    # print(coords_f)

    local_coords_s, local_coords_f = run_home(
        os.path.join(INPUTS_PATH, f'home_{_idx}.csv'),
        coords_s,
        coords_f
    )

    # print('local_coords_s:')
    # print(local_coords_s)
    # print('local_coords_f:')
    # print(local_coords_f)

    # Create separate point clouds
    success_pcd = o3d.geometry.PointCloud()
    success_pcd.points = o3d.utility.Vector3dVector(local_coords_s)
    
    failure_pcd = o3d.geometry.PointCloud()
    failure_pcd.points = o3d.utility.Vector3dVector(local_coords_f)

    # Assign colors
    success_pcd.paint_uniform_color([0, 1, 0])  # Green for success
    failure_pcd.paint_uniform_color([1, 0, 0])  # Red for failure

    # Combine into a single point cloud
    combined_pcd = success_pcd + failure_pcd

    # Save as PLY file
    o3d.io.write_point_cloud(os.path.join(f'{OUTPUTS_PATH}', f'landing_cloud_{_idx}.ply'), combined_pcd, write_ascii=False)

def viz(_idx):
    save_landing_cloud(os.path.join(f'{INPUTS_PATH}', 'log_0_2025-5-27-13-19-50.bin'), _idx)

    # Start visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    add_home(vis)
    add_cloud(vis, _idx)
    add_landing_cloud(vis, _idx)

    # Run visualization
    vis.run()
    vis.destroy_window()


def main():
    viz(2)

if __name__=="__main__":
    main()
