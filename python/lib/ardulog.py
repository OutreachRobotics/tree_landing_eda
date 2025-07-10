from ardupilot_log_reader.reader import Ardupilot
from geo_proj import get_local_coord, get_home, get_origin

import bisect
import config
import numpy as np
import os
import open3d as o3d

def get_log(_idx):
    prefix = f'log_'
    folder_path = os.path.join(config.INPUTS_PATH, str(_idx))
    # List all files in the directory
    for filename in os.listdir(folder_path):
        # Check if the file starts with the prefix and ends with '.bin'
        if filename.startswith(prefix) and filename.endswith('.bin'):
            # Join the folder path with the filename to get the full path
            return os.path.join(folder_path, filename)
    return None  # Return None if no matching file is found

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
    filepath_mockup = get_log(_idx)
    type_request = ['RCIN', 'POS']
    output = Ardupilot.parse(filepath_mockup, types=type_request, zero_time_base=True)
    dfs_mockup = output.dfs

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

    coords_s = np.array([latitudes_s, longitudes_s, alt_s]).T
    coords_f = np.array([latitudes_f, longitudes_f, alt_f]).T

    return coords_s, coords_f

def run_home(_filepath_home, _coords_s, _coords_f):
    coord_home = get_home(_filepath_home)

    local_coords_s = []
    for coord_s in _coords_s:
        local_coords_s.append(get_local_coord(coord_home, coord_s))

    local_coords_f = []
    for coord_f in _coords_f:
        local_coords_f.append(get_local_coord(coord_home, coord_f))

    return local_coords_s, local_coords_f

def run_origin(_filepath_origin, _coords_s, _coords_f):
    coord_origin = get_origin(_filepath_origin)

    local_coords_s = []
    for coord_s in _coords_s:
        local_coords_s.append(get_local_coord(coord_origin, coord_s))

    local_coords_f = []
    for coord_f in _coords_f:
        local_coords_f.append(get_local_coord(coord_origin, coord_f))

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
    
    return coords.tolist()

def run_project_alt(_filepath_rtabmap, _local_coords_s, _local_coords_f):
    pcd = o3d.io.read_point_cloud(_filepath_rtabmap)
    points = np.asarray(pcd.points)

    local_coords_matched_s = get_projected_alt(_local_coords_s, points)
    local_coords_matched_f = get_projected_alt(_local_coords_f, points)

    return local_coords_matched_s, local_coords_matched_f

def add_ardulog(_df, _idx):
    df = _df.copy()
    coords_s, coords_f = run_ardulog(_idx)

    # local_coords_s, local_coords_f = run_home(
    #     os.path.join(config.INPUTS_PATH, str(_idx), config.HOME_CSV),
    #     coords_s,
    #     coords_f
    # )
    local_coords_s, local_coords_f = run_origin(
        os.path.join(config.INPUTS_PATH, str(_idx), config.ORIGIN_CSV),
        coords_s,
        coords_f
    )

    # local_coords_matched_s, local_coords_matched_f = run_project_alt(
    #     os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY),
    #     local_coords_s,
    #     local_coords_f
    # )
    local_coords_matched_s = local_coords_s
    local_coords_matched_f = local_coords_f

    success_values = np.concatenate([np.array([True] * len(local_coords_matched_s)), np.array([False] * len(local_coords_matched_f))])
    combined_local_coords = local_coords_matched_s + local_coords_matched_f

    df = df.loc[df.index.repeat(len(local_coords_matched_s) + len(local_coords_matched_f))].reset_index(drop=True)
    df.insert(0, 'success', success_values)
    df.insert(1, 'landing_x', np.array(combined_local_coords).T[0])
    df.insert(2, 'landing_y', np.array(combined_local_coords).T[1])

    return df
