from ardupilot_log_reader.reader import Ardupilot
from geo_proj import get_local_coord, get_home, get_origin
from scipy.spatial.distance import cdist

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

def extract_landings(_dfs, _timestamps):
    landing_idx_list = find_closest_timestamps_idx(_timestamps, _dfs['POS']['timestamp'].to_list())

    ### Extract landing positions ###
    latitudes = _dfs['POS']['Lat'][landing_idx_list].to_list()
    longitudes = _dfs['POS']['Lng'][landing_idx_list].to_list()
    alt = _dfs['POS']['Alt'][landing_idx_list].to_list()
    roll = _dfs['ATT']['Roll'][landing_idx_list].to_list()
    pitch = _dfs['ATT']['Pitch'][landing_idx_list].to_list()

    return np.array([latitudes, longitudes, alt, roll, pitch]).T

def run_ardulog(_idx):
    mockup_logs = get_logs(_idx)
    type_request = ['RCIN', 'POS', 'ATT']

    landings_s_list = []
    landings_f_list = []
    for mockup_log in mockup_logs:
        parsed_log = Ardupilot.parse(mockup_log, types=type_request, zero_time_base=True)
        dfs_mockup = parsed_log.dfs

        ### Extract landing timestamps ###
        THRESHOLD = 1200
        landing_timestamps_s = extract_rising_edges(dfs_mockup['RCIN']['timestamp'], dfs_mockup['RCIN']['C5'], THRESHOLD)
        landing_timestamps_f = extract_rising_edges(dfs_mockup['RCIN']['timestamp'], dfs_mockup['RCIN']['C6'], THRESHOLD)

        landings_s_list.append(extract_landings(dfs_mockup, landing_timestamps_s))
        landings_f_list.append(extract_landings(dfs_mockup, landing_timestamps_f))

    landings_s = np.vstack(landings_s_list)
    landings_f = np.vstack(landings_f_list)

    return landings_s, landings_f

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

def get_bbox_filtered(_bbox, _landings, _distance_threshold):
    grown_extent = _bbox.extent + [_distance_threshold, _distance_threshold, 999] # Ignore Z
    grown_bbox = o3d.geometry.OrientedBoundingBox(_bbox.center, _bbox.R, grown_extent)
    points_to_check = o3d.utility.Vector3dVector(_landings[:, :3])
    indices_inside = grown_bbox.get_point_indices_within_bounding_box(points_to_check)
    filtered_landings = np.asarray(_landings)[indices_inside]

    return filtered_landings

def run_bbox_filter(_filepath_rtabmap, _landings_s, _landings_f, _distance_threshold):
    pcd = o3d.io.read_point_cloud(_filepath_rtabmap)
    obb = pcd.get_minimal_oriented_bounding_box()

    local_coords_filtered_s = get_bbox_filtered(obb, _landings_s, _distance_threshold)
    local_coords_filtered_f = get_bbox_filtered(obb, _landings_f, _distance_threshold)
    
    return local_coords_filtered_s, local_coords_filtered_f

def run_angle_filter(_landings_s, _landings_f, _max_angle):
    condition = np.any(np.abs(_landings_s[:, 3:]) > _max_angle, axis=1)
    bad_indices = np.where(condition)[0]
    bad_landings = _landings_s[bad_indices]

    landings_filtered_s = np.delete(_landings_s, bad_indices, axis=0)
    landings_filtered_f = np.vstack((_landings_f, bad_landings))

    return landings_filtered_s, landings_filtered_f

def overlap_filter(_landings, _min_dist):
    if _landings.shape[0] < 2:
        return _landings

    # 1. Get coordinates and calculate all pairwise distances
    coords = _landings[:, :3]
    dist_matrix = cdist(coords, coords)

    # 2. Create a boolean matrix of overlaps, ignoring the diagonal (k=1)
    # This isolates pairs where point j is close to point i, and j > i.
    upper_triangle_overlaps = np.triu(dist_matrix < _min_dist, k=1)

    # 3. Find any column that contains an overlap.
    # A 'True' in column j means point j is too close to an earlier point i.
    # We check along axis=0 (down the columns).
    to_remove_mask = np.any(upper_triangle_overlaps, axis=0)

    # 4. Invert the mask to get the points to keep and filter the array
    to_keep_mask = ~to_remove_mask
    return _landings[to_keep_mask]

def run_overlap_filter(_landings_s, _landings_f, _min_dist):
    filtered_landings_s = overlap_filter(_landings_s, _min_dist)
    filtered_landings_f = overlap_filter(_landings_f, _min_dist)

    return filtered_landings_s, filtered_landings_f

def run_filtered_ardulog(_idx):
    landings_s, landings_f = run_ardulog(_idx)

    local_coords_s, local_coords_f = run_origin(
        os.path.join(config.INPUTS_PATH, str(_idx), config.ORIGIN_CSV),
        landings_s[:, :3],
        landings_f[:, :3]
    )

    local_landings_s = np.concatenate((local_coords_s, landings_s[:, 3:]), axis=1)
    local_landings_f = np.concatenate((local_coords_f, landings_f[:, 3:]), axis=1)

    DRONE_RADIUS = 1.5
    local_landings_bbox_s, local_landings_bbox_f = run_bbox_filter(
        os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY),
        local_landings_s,
        local_landings_f,
        DRONE_RADIUS
    )

    MAX_ANGLE = 45.0
    local_landings_angle_s, local_landings_angle_f = run_angle_filter(
        local_landings_bbox_s,
        local_landings_bbox_f,
        MAX_ANGLE
    )

    # OVERLAP_DIST = 0.3
    local_landings_overlap_s, local_landings_overlap_f = run_overlap_filter(
        local_landings_angle_s,
        local_landings_angle_f,
        DRONE_RADIUS
    )

    return local_landings_overlap_s, local_landings_overlap_f


def save_landing_cloud(_idx):
    landings_s, landings_f = run_filtered_ardulog(_idx)

    local_coords_s = landings_s[:, :3]
    local_coords_f = landings_f[:, :3]

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
