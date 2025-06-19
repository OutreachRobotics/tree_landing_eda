from ardupilot_log_reader.reader import Ardupilot
from geo_proj import get_local_coord, get_home

import bisect
import config
import numpy as np
import os
import pandas as pd

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
    type_request = ['RCIN', 'IMU', 'POS', 'BARO', 'MODE', 'MAG', 'XKF1', 'ORGN']
    output = Ardupilot.parse(filepath_mockup, types=type_request, zero_time_base=True)
    dfs_mockup = output.dfs

    ### Extract landing positions ###
    THRESHOLD = 1200
    landing_timestamps_s = extract_rising_edges(dfs_mockup['RCIN']['timestamp'], dfs_mockup['RCIN']['C5'], THRESHOLD)
    landing_timestamps_f = extract_rising_edges(dfs_mockup['RCIN']['timestamp'], dfs_mockup['RCIN']['C6'], THRESHOLD)

    POS_idx_list_s = find_closest_timestamps_idx(landing_timestamps_s, dfs_mockup['POS']['timestamp'].to_list())
    POS_idx_list_f = find_closest_timestamps_idx(landing_timestamps_f, dfs_mockup['POS']['timestamp'].to_list())

    latitudes_s = dfs_mockup['POS']['Lat'][POS_idx_list_s].to_list()
    longitudes_s = dfs_mockup['POS']['Lng'][POS_idx_list_s].to_list()

    latitudes_f = dfs_mockup['POS']['Lat'][POS_idx_list_f].to_list()
    longitudes_f = dfs_mockup['POS']['Lng'][POS_idx_list_f].to_list()

    XKF1_idx_list_s = find_closest_timestamps_idx(landing_timestamps_s, dfs_mockup['XKF1']['timestamp'].to_list())
    XKF1_idx_list_f = find_closest_timestamps_idx(landing_timestamps_f, dfs_mockup['XKF1']['timestamp'].to_list())

    relalt_s = dfs_mockup['XKF1']['PD'][XKF1_idx_list_s].to_list()
    relalt_f = dfs_mockup['XKF1']['PD'][XKF1_idx_list_f].to_list()

    for i in range(len(relalt_s)):
        # Flip Z
        relalt_s[i] = -relalt_s[i]

    for i in range(len(relalt_f)):
        # Flip Z
        relalt_f[i] = -relalt_f[i]

    coords_s = np.array([latitudes_s, longitudes_s, relalt_s]).T
    coords_f = np.array([latitudes_f, longitudes_f, relalt_f]).T

    return coords_s, coords_f

def run_home(_filepath_home, _coords_s, _coords_f):
    coord_home = get_home(_filepath_home)

    # print('coord_home:')
    # print(coord_home)

    local_coords_s = []
    for coord_s in _coords_s:
        local_coords_s.append(get_local_coord(coord_home, coord_s))

    local_coords_f = []
    for coord_f in _coords_f:
        local_coords_f.append(get_local_coord(coord_home, coord_f))

    return local_coords_s, local_coords_f

def add_ardulog(_df, _idx):
    df = _df.copy()
    coords_s, coords_f = run_ardulog(_idx)

    local_coords_s, local_coords_f = run_home(
        os.path.join(config.INPUTS_PATH, str(_idx), config.HOME_CSV),
        coords_s,
        coords_f
    )

    success_values = np.concatenate([np.array([True] * len(local_coords_s)), np.array([False] * len(local_coords_f))])
    combined_local_coords = local_coords_s + local_coords_f

    df = df.loc[df.index.repeat(len(local_coords_s) + len(local_coords_f))].reset_index(drop=True)
    df.insert(0, 'success', success_values)
    df.insert(1, 'landing_x', np.array(combined_local_coords).T[0])
    df.insert(2, 'landing_y', np.array(combined_local_coords).T[1])

    return df
