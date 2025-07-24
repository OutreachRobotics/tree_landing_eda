from ardupilot_log_reader.reader import Ardupilot
from geo_proj import get_local_coord, get_home, get_origin

import bisect
import config
import numpy as np
import os
import open3d as o3d
import pandas as pd

def get_logs(_idx):
    prefix = f'log_'
    folder_path = os.path.join(config.INPUTS_PATH, str(_idx))
    log_files = []
    # List all files in the directory
    for filename in os.listdir(folder_path):
        # Check if the file starts with the prefix and ends with '.bin'
        if filename.startswith(prefix) and filename.endswith('.bin'):
            # Join the folder path with the filename to get the full path
            log_files.append(os.path.join(folder_path, filename))
    return log_files

def extract_rising_edges(_timestamps, _signal, _threshold):
    rising_edges_timestamps = []
    is_rised = False
    for i in range(0, len(_signal)):
        if _signal[i] > _threshold:
            if not is_rised:
                rising_edges_timestamps.append(float(_timestamps[i]))
            is_rised = True
        else:
            is_rised = False

    print("Rising edges timestamps")
    print(str(rising_edges_timestamps))

    return rising_edges_timestamps

def find_closest_timestamps_idx(_input_timestamps, _timestamps_list):
    closest_indices = []
    for timestamp in _input_timestamps:
        # Find the position where the timestamp would be inserted to keep the list sorted
        pos = bisect.bisect_left(_timestamps_list, timestamp)
        
        # Check if the timestamp is exactly matched or find the closest one
        if pos == 0:
            closest_indices.append(0)
        elif pos == len(_timestamps_list):
            closest_indices.append(len(_timestamps_list) - 1)
        else:
            # Compare the difference with the previous and next timestamp
            prev_diff = abs(_timestamps_list[pos - 1] - timestamp)
            next_diff = abs(_timestamps_list[pos] - timestamp)
            if prev_diff <= next_diff:
                closest_indices.append(pos - 1)
            else:
                closest_indices.append(pos)
    
    return closest_indices

def run_ardulog(_idx):
    mockup_logs = get_logs(_idx)
    type_request = ['RCIN', 'POS']

    coords_s_list = []
    coords_f_list = []
    for mockup_log in mockup_logs:
        parsed_log = Ardupilot.parse(mockup_log, types=type_request, zero_time_base=True)
        dfs_mockup = parsed_log.dfs

        ### Extract landing timestamps ###
        THRESHOLD = 1200
        landing_timestamps_s = extract_rising_edges(dfs_mockup['RCIN']['timestamp'], dfs_mockup['RCIN']['C5'], THRESHOLD)
        landing_timestamps_f = extract_rising_edges(dfs_mockup['RCIN']['timestamp'], dfs_mockup['RCIN']['C6'], THRESHOLD)

        POS_idx_list_s = find_closest_timestamps_idx(landing_timestamps_s, dfs_mockup['POS']['timestamp'].to_list())
        POS_idx_list_f = find_closest_timestamps_idx(landing_timestamps_f, dfs_mockup['POS']['timestamp'].to_list())

        ### Extract landing positions ###
        latitudes_s = dfs_mockup['POS']['Lat'][POS_idx_list_s].to_list()
        longitudes_s = dfs_mockup['POS']['Lng'][POS_idx_list_s].to_list()
        alt_s = dfs_mockup['POS']['Alt'][POS_idx_list_s].to_list()

        latitudes_f = dfs_mockup['POS']['Lat'][POS_idx_list_f].to_list()
        longitudes_f = dfs_mockup['POS']['Lng'][POS_idx_list_f].to_list()
        alt_f = dfs_mockup['POS']['Alt'][POS_idx_list_f].to_list()

        coords_s_list.append(np.array([latitudes_s, longitudes_s, alt_s]).T)
        coords_f_list.append(np.array([latitudes_f, longitudes_f, alt_f]).T)

    coords_s = np.vstack(coords_s_list)
    coords_f = np.vstack(coords_f_list)

    return coords_s, coords_f

def run_home(_filepath_home, _coords_s, _coords_f):
    coord_home = get_home(_filepath_home)

    local_coords_s = np.array([get_local_coord(coord_home, coord) for coord in _coords_s])
    local_coords_f = np.array([get_local_coord(coord_home, coord) for coord in _coords_f])

    return local_coords_s, local_coords_f

def run_origin(_filepath_origin, _coords_s, _coords_f):
    coord_origin = get_origin(_filepath_origin)

    local_coords_s = np.array([get_local_coord(coord_origin, coord) for coord in _coords_s])
    local_coords_f = np.array([get_local_coord(coord_origin, coord) for coord in _coords_f])

    return local_coords_s, local_coords_f

def get_projected_alt(_coords, _points, _radius=0.8):
    coords = np.array(_coords)
    points = np.array(_points)
    highest_z = np.full(len(_coords), np.nan)  # Default to NaN if no points found
    
    for i, coord in enumerate(_coords):
        # Compute squared distances (faster than Euclidean)
        dx = points[:, 0] - coord[0]
        dy = points[:, 1] - coord[1]
        dist_sq = dx**2 + dy**2  # Ignore Z for radius check
        
        # Filter points within XY radius
        mask = dist_sq <= _radius**2
        candidates_z = points[mask, 2]  # Only extract Z-values
        
        if len(candidates_z) > 0:
            highest_z[i] = np.max(candidates_z)  # Store max Z
    
    # Update Z-coordinates (skip if NaN)
    coords[:, 2] = np.where(np.isnan(highest_z), coords[:, 2], highest_z)
    
    return np.array(coords.tolist())

def run_project_alt(_filepath_rtabmap, _local_coords_s, _local_coords_f):
    pcd = o3d.io.read_point_cloud(_filepath_rtabmap)
    points = np.asarray(pcd.points)

    local_coords_matched_s = get_projected_alt(_local_coords_s, points)
    local_coords_matched_f = get_projected_alt(_local_coords_f, points)

    return local_coords_matched_s, local_coords_matched_f

def get_bbox_filtered(_bbox, _coords, _distance_threshold):
    grown_extent = _bbox.extent + [_distance_threshold, _distance_threshold, 999] # Ignore Z
    grown_bbox = o3d.geometry.OrientedBoundingBox(_bbox.center, _bbox.R, grown_extent)
    points_to_check = o3d.utility.Vector3dVector(_coords)
    indices_inside = grown_bbox.get_point_indices_within_bounding_box(points_to_check)
    filtered_coords = np.asarray(_coords)[indices_inside]

    return filtered_coords

def run_bbox_filter(_filepath_rtabmap, _local_coords_s, _local_coords_f, _distance_threshold):
    pcd = o3d.io.read_point_cloud(_filepath_rtabmap)
    obb = pcd.get_minimal_oriented_bounding_box()

    local_coords_filtered_s = get_bbox_filtered(obb, _local_coords_s, _distance_threshold)
    local_coords_filtered_f = get_bbox_filtered(obb, _local_coords_f, _distance_threshold)
    
    return local_coords_filtered_s, local_coords_filtered_f

def run_local_ardulog(_idx):
    coords_s, coords_f = run_ardulog(_idx)

    # print('coords_s:')
    # print(coords_s)
    # print('coords_f:')
    # print(coords_f)

    local_coords_s, local_coords_f = run_origin(
        os.path.join(config.INPUTS_PATH, str(_idx), config.ORIGIN_CSV),
        coords_s,
        coords_f
    )

    DRONE_RADIUS = 1.5
    local_coords_filtered_s, local_coords_filtered_f = run_bbox_filter(
        os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY),
        local_coords_s,
        local_coords_f,
        DRONE_RADIUS
    )

    return local_coords_filtered_s, local_coords_filtered_f

def save_landing_cloud(_idx):
    local_coords_s, local_coords_f = run_local_ardulog(_idx)

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

    return local_coords_s, local_coords_f

def add_ardulog(_df, _idx):
    df = _df.copy()
    coords_s, coords_f = save_landing_cloud(_idx)

    success_values = np.concatenate([np.array([True] * len(coords_s)), np.array([False] * len(coords_f))])
    combined_local_coords = coords_s + coords_f

    df = df.loc[df.index.repeat(len(coords_s) + len(coords_f))].reset_index(drop=True)
    df.insert(0, 'success', success_values)
    df.insert(1, 'landing_x', np.array(combined_local_coords).T[0])
    df.insert(2, 'landing_y', np.array(combined_local_coords).T[1])
    df.insert(3, 'landing_z', np.array(combined_local_coords).T[2])

    return df
