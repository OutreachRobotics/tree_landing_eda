from ardulog import save_landing_cloud
from geo_proj import compute_geo_ref_rgb, compute_geo_ref_cloud

import config
import open3d as o3d
import os

def add_origin(_vis):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    sphere.paint_uniform_color([0, 0, 1])
    sphere.translate([0, 0, 0])
    _vis.add_geometry(sphere)

def add_cloud(_vis, _idx):
    pcd = o3d.io.read_point_cloud(os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY))
    _vis.add_geometry(pcd)

def add_bbox(_vis, _idx):
    pcd = o3d.io.read_point_cloud(os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY))
    bbox = pcd.get_minimal_oriented_bounding_box()
    bbox.color = (1, 0, 0)
    print(bbox.extent[2])
    _vis.add_geometry(bbox)

def add_landing_cloud(_vis, _idx):
    pcd = o3d.io.read_point_cloud(os.path.join(config.INPUTS_PATH, str(_idx), config.LANDINGS_CLOUD_PLY))

    _vis.add_geometry(pcd)

    for i in range(len(pcd.points)):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.8)
        sphere.paint_uniform_color(pcd.colors[i])
        sphere.translate(pcd.points[i])
        _vis.add_geometry(sphere)

def viz_logs(_idx):
    os.makedirs(os.path.join(config.OUTPUTS_PATH, str(_idx)), exist_ok=True)
    save_landing_cloud(_idx, False)

    compute_geo_ref_cloud(
        os.path.join(config.INPUTS_PATH, str(_idx), config.LANDINGS_CLOUD_PLY),
        os.path.join(config.OUTPUTS_PATH, str(_idx), config.LANDINGS_CLOUD_GEO_REF_LAS),
        os.path.join(config.INPUTS_PATH, str(_idx), config.ORIGIN_LOG_CSV)
    )

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    add_landing_cloud(vis, _idx)

    vis.run()
    vis.destroy_window()

def viz(_idx, _show_all: bool = True):
    if _show_all:
        os.makedirs(os.path.join(config.OUTPUTS_PATH, str(_idx)), exist_ok=True)
        save_landing_cloud(_idx, True)

        rgb_image_path = os.path.join(config.INPUTS_PATH, str(_idx), config.IMAGE_RGB_PNG)
        pose_csv_path = os.path.join(config.INPUTS_PATH, str(_idx), config.IMAGE_RGB_POSE_CSV)
        
        if os.path.exists(rgb_image_path) and os.path.exists(pose_csv_path):
            compute_geo_ref_rgb(
                rgb_image_path,
                os.path.join(config.OUTPUTS_PATH, str(_idx), config.IMAGE_RGB_GEO_REF_TIF),
                pose_csv_path
            )
        else:
            print(f"Skipping geo-referencing for index {_idx} because input files are missing.")

        compute_geo_ref_cloud(
            os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY),
            os.path.join(config.OUTPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_GEO_REF_LAS),
            os.path.join(config.INPUTS_PATH, str(_idx), config.ORIGIN_CSV)
        )
        compute_geo_ref_cloud(
            os.path.join(config.INPUTS_PATH, str(_idx), config.LANDINGS_CLOUD_PLY),
            os.path.join(config.OUTPUTS_PATH, str(_idx), config.LANDINGS_CLOUD_GEO_REF_LAS),
            os.path.join(config.INPUTS_PATH, str(_idx), config.ORIGIN_CSV)
        )

    # Start visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    add_cloud(vis, _idx)

    if _show_all:
        # add_origin(vis)
        # add_bbox(vis, _idx)
        add_landing_cloud(vis, _idx)

    # Run visualization
    vis.run()
    vis.destroy_window()


def main():
    viz(0, True)
    # viz_logs(100)

if __name__=="__main__":
    main()
