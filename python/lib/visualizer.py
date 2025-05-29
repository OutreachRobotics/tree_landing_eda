from ardulog import run_ardulog, run_home
from config import INPUTS_PATH

import numpy as np
import open3d as o3d
import os

def add_home(_vis):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    sphere.paint_uniform_color([0, 0, 1])
    sphere.translate([0, 0, 0])
    _vis.add_geometry(sphere)

def add_cloud(_vis):
    pcd = o3d.io.read_point_cloud(os.path.join(f'{INPUTS_PATH}', 'rtabmap_cloud_2.ply'))
    _vis.add_geometry(pcd)

def add_landing_cloud(_log_file, _idx, _vis):
    coords_s, coords_f = run_ardulog(_log_file)

    print('coords_s:')
    print(coords_s)
    print('coords_f:')
    print(coords_f)

    local_coords_s, local_coords_f = run_home(
        os.path.join(f'{INPUTS_PATH}', f'home_{_idx}.csv'),
        coords_s,
        coords_f
    )

    print('local_coords_s:')
    print(local_coords_s)
    print('local_coords_f:')
    print(local_coords_f)

    # Create highlight point cloud
    success_landing_pcd = o3d.geometry.PointCloud()
    success_landing_pcd.points = o3d.utility.Vector3dVector(local_coords_s)

    failed_landing_pcd = o3d.geometry.PointCloud()
    failed_landing_pcd.points = o3d.utility.Vector3dVector(local_coords_f)

    success_colors = np.tile([0, 1, 0], (len(success_landing_pcd.points), 1))  # Green for success
    failure_colors = np.tile([1, 0, 0], (len(failed_landing_pcd.points ), 1))  # Red for failure
    
    success_landing_pcd.colors = o3d.utility.Vector3dVector(success_colors)
    failed_landing_pcd.colors = o3d.utility.Vector3dVector(failure_colors)

    _vis.add_geometry(success_landing_pcd)
    _vis.add_geometry(failed_landing_pcd)

    for point in success_landing_pcd.points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
        sphere.paint_uniform_color([0, 1, 0])
        sphere.translate(point)
        _vis.add_geometry(sphere)

    for point in failed_landing_pcd.points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
        sphere.paint_uniform_color([1, 0, 0])
        sphere.translate(point)
        _vis.add_geometry(sphere)

def viz(_idx):
    # Start visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    add_home(vis)
    add_cloud(vis)
    add_landing_cloud(os.path.join(f'{INPUTS_PATH}', 'log_0_2025-5-27-13-19-50.bin'), _idx, vis)

    # Run visualization
    vis.run()
    vis.destroy_window()


def main():
    viz(2)

if __name__=="__main__":
    main()
