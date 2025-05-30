from ardulog import run_ardulog, run_home
from geo_proj import compute_geo_ref_rgb, compute_geo_ref_cloud

import config
import open3d as o3d
import os

def add_home(_vis):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    sphere.paint_uniform_color([0, 0, 1])
    sphere.translate([0, 0, 0])
    _vis.add_geometry(sphere)

def add_cloud(_vis, _idx):
    pcd = o3d.io.read_point_cloud(os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY))
    _vis.add_geometry(pcd)

def add_landing_cloud(_vis, _idx):
    pcd = o3d.io.read_point_cloud(os.path.join(config.INPUTS_PATH, str(_idx), config.LANDINGS_CLOUD_PLY))

    _vis.add_geometry(pcd)

    for i in range(len(pcd.points)):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
        sphere.paint_uniform_color(pcd.colors[i])
        sphere.translate(pcd.points[i])
        _vis.add_geometry(sphere)

def get_local_coords(_log_file, _idx):
    coords_s, coords_f = run_ardulog(_log_file)

    # print('coords_s:')
    # print(coords_s)
    # print('coords_f:')
    # print(coords_f)

    local_coords_s, local_coords_f = run_home(
        os.path.join(config.INPUTS_PATH, str(_idx), config.HOME_CSV),
        coords_s,
        coords_f
    )

    # print('local_coords_s:')
    # print(local_coords_s)
    # print('local_coords_f:')
    # print(local_coords_f)

    return local_coords_s, local_coords_f

def save_landing_cloud(_log_file, _idx):
    local_coords_s, local_coords_f = get_local_coords(_log_file, _idx)

    # local_coords_s = [[x - 24.0, y + 26.0, z + 3.0] for x, y, z in local_coords_s]
    # local_coords_f = [[x - 24.0, y + 26.0, z + 3.0] for x, y, z in local_coords_f]

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
    o3d.io.write_point_cloud(os.path.join(config.INPUTS_PATH, str(_idx), config.LANDINGS_CLOUD_PLY), combined_pcd, write_ascii=False)

def viz(_idx):
    os.makedirs(os.path.join(config.OUTPUTS_PATH, str(_idx)), exist_ok=True)
    save_landing_cloud(os.path.join(config.LOGS_PATH, 'log_0_2025-5-27-13-19-50.bin'), _idx)
    compute_geo_ref_rgb(
        os.path.join(config.INPUTS_PATH, str(_idx), config.IMAGE_RGB_PNG),
        os.path.join(config.INPUTS_PATH, str(_idx), config.IMAGE_RGB_POSE_CSV),
        os.path.join(config.OUTPUTS_PATH, str(_idx), config.IMAGE_RGB_GEO_REF_TIF)
    )
    compute_geo_ref_cloud(
        os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY),
        os.path.join(config.INPUTS_PATH, str(_idx), config.HOME_CSV),
        os.path.join(config.OUTPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_GEO_REF_LAS)
    )
    compute_geo_ref_cloud(
        os.path.join(config.INPUTS_PATH, str(_idx), config.LANDINGS_CLOUD_PLY),
        os.path.join(config.INPUTS_PATH, str(_idx), config.HOME_CSV),
        os.path.join(config.OUTPUTS_PATH, str(_idx), config.LANDINGS_CLOUD_GEO_REF_LAS)
    )

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
    viz(3)

if __name__=="__main__":
    main()
