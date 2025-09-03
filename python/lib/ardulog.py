from ardupilot_log_reader.reader import Ardupilot
from geo_proj import get_local_coord, get_home, get_origin
from scipy.spatial import KDTree
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

    if not log_files:
        print('Missing log files')
    
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

def extract_landings(_dfs, _timestamps, _is_success: bool):
    landing_idx_list = find_closest_timestamps_idx(_timestamps, _dfs['POS']['timestamp'].to_list())

    ### Extract landing positions ###
    latitudes = _dfs['POS']['Lat'][landing_idx_list].to_list()
    longitudes = _dfs['POS']['Lng'][landing_idx_list].to_list()
    alt = _dfs['POS']['Alt'][landing_idx_list].to_list()
    roll = _dfs['ATT']['Roll'][landing_idx_list].to_list()
    pitch = _dfs['ATT']['Pitch'][landing_idx_list].to_list()

    success_flags = np.full(len(latitudes), _is_success)

    return np.array([latitudes, longitudes, alt, roll, pitch, success_flags]).T

def run_ardulog(_idx):
    origin_path = os.path.join(config.INPUTS_PATH, str(_idx), config.ORIGIN_CSV)

    landings_list = []
    mockup_logs = get_logs(_idx)
    type_request = ['RCIN', 'POS', 'ATT']
    should_compensate = True

    if not os.path.exists(origin_path):
        print(f"WARN: {origin_path} from mavros was not found, saving mockup_logs[0]'s origin as {config.ORIGIN_LOG_CSV} instead.")
        origin_path = os.path.join(config.INPUTS_PATH, str(_idx), config.ORIGIN_LOG_CSV)
        parsed_log = Ardupilot.parse(mockup_logs[0], types=['ORGN'], zero_time_base=True)
        log_origin = [parsed_log.dfs['ORGN']['Lat'], parsed_log.dfs['ORGN']['Lng'], parsed_log.dfs['ORGN']['Alt']]
        save_origin(origin_path, log_origin)
        should_compensate = False

    for mockup_log in mockup_logs:
        parsed_log = Ardupilot.parse(mockup_log, types=type_request, zero_time_base=True)
        dfs_mockup = parsed_log.dfs

        ### Extract landing timestamps ###
        THRESHOLD = 1200
        landing_timestamps_s = extract_rising_edges(dfs_mockup['RCIN']['timestamp'], dfs_mockup['RCIN']['C5'], THRESHOLD)
        landing_timestamps_f = extract_rising_edges(dfs_mockup['RCIN']['timestamp'], dfs_mockup['RCIN']['C6'], THRESHOLD)

        landings_list.append(extract_landings(dfs_mockup, landing_timestamps_s, True))
        landings_list.append(extract_landings(dfs_mockup, landing_timestamps_f, False))

    local_landings_list = run_origin(
        origin_path,
        np.vstack(landings_list),
        should_compensate
    )

    return local_landings_list # Column titles: local_coord_x, local_coord_y, local_coord_z, latitudes, longitudes, alt, roll, pitch, success_flags

def run_home(_filepath_home, _coords_s, _coords_f):
    coord_home = get_home(_filepath_home)

    local_coords_s = np.array([get_local_coord(coord_home, coord) for coord in _coords_s])
    local_coords_f = np.array([get_local_coord(coord_home, coord) for coord in _coords_f])

    return local_coords_s, local_coords_f

def save_origin(_filepath_origin, _origin):
    origin_dict = {
        'latitude': _origin[0],
        'longitude': _origin[1],
        'altitude': _origin[2],
    }

    origin_df = pd.DataFrame(origin_dict)

    output_dir = os.path.dirname(_filepath_origin)
    os.makedirs(output_dir, exist_ok=True)
    origin_df.to_csv(_filepath_origin, index=False)
    
    print(f"Origin from log data successfully saved to {_filepath_origin}.")


def run_origin(_filepath_origin, _landings, _should_compensate: bool = True):
    coord_origin = get_origin(_filepath_origin, _should_compensate)
    local_coords = np.array([get_local_coord(coord_origin, coord) for coord in _landings[:, :3]])
    local_landings = np.concatenate((local_coords, _landings), axis=1)

    return local_landings

def run_project_alt(_pcd, _landings, _radius = 0.1):
    points = np.asarray(_pcd.points)
    highest_z = np.full(len(_landings), np.nan)  # Default to NaN if no points found
    diff_z = np.full(len(_landings), 0.0)  # Default to NaN if no points found
    
    for i, landing in enumerate(_landings):
        # Compute squared distances (faster than Euclidean)
        dx = points[:, 0] - landing[0]
        dy = points[:, 1] - landing[1]
        dist_sq = dx**2 + dy**2  # Ignore Z for radius check
        
        # Filter points within XY radius
        mask = dist_sq <= _radius**2
        candidates_z = points[mask, 2]  # Only extract Z-values
        
        if len(candidates_z) > 0:
            highest_z[i] = np.max(candidates_z)  # Store max Z
            diff_z[i] = landing[2] - highest_z[i]
    
    # Update Z-coordinates (skip if NaN)
    projected_landings = _landings.copy()
    projected_landings[:, 2] = np.where(np.isnan(highest_z), _landings[:, 2], highest_z)
    projected_landings[:, 5] = _landings[:, 5] + diff_z

    print()
    print(f"run_project_alt modified {(~np.isnan(highest_z)).sum()} landings out of {len(_landings)} landings")
    
    return projected_landings

def run_bbox_filter(_pcd, _landings, _distance_threshold):
    pcd_almost_2d = o3d.geometry.PointCloud()
    points_almost_2d = np.asarray(_pcd.points).copy()
    noise = np.random.uniform(-0.0001, 0.0001, size=(len(_pcd.points),))
    points_almost_2d[:, 2] = noise  # Set all Z values to about 0
    pcd_almost_2d.points = o3d.utility.Vector3dVector(points_almost_2d)

    bbox_2d = pcd_almost_2d.get_minimal_oriented_bounding_box()
    grown_extent = bbox_2d.extent + [_distance_threshold, _distance_threshold, 999.0] # Ignore Z
    grown_bbox = o3d.geometry.OrientedBoundingBox(bbox_2d.center, bbox_2d.R, grown_extent)

    points_to_check = o3d.utility.Vector3dVector(_landings[:, :3])
    indices_inside = grown_bbox.get_point_indices_within_bounding_box(points_to_check)
    filtered_landings = np.asarray(_landings)[indices_inside]

    print()
    print(f"run_bbox_filter removed {len(_landings) - len(filtered_landings)} landings out of {len(_landings)} landings")
    
    return filtered_landings

def run_missing_points_filter(_pcd, _landings, _radius, _min_points):
    # Filters success points from neighboring trees (not in current point cloud)
    points = np.asarray(_pcd.points)
    filtered_landings = _landings.copy()

    kdtree = KDTree(points)

    keep_mask = np.ones(len(_landings), dtype=bool)

    for i, landing in enumerate(_landings):
        is_success = landing[-1] == 1

        # Only check successful landings; we always keep failed ones
        if is_success:
            # Find the number of neighbors within the radius for the current landing point
            # kdtree.query_ball_point is extremely fast for this.
            neighbors_indices = kdtree.query_ball_point(landing[:3], r=_radius)
            num_neighbors = len(neighbors_indices)

            print(f"num_neighbors: {num_neighbors}")

            # If a successful landing has too few neighbors, mark it for removal
            if num_neighbors < _min_points:
                keep_mask[i] = False # Mark this row to be removed

    # After the loop, create the new array using the boolean mask in a single step
    filtered_landings = _landings[keep_mask]

    print()
    print(f"run_missing_points_filter removed {len(_landings) - len(filtered_landings)} landings out of {len(_landings)} landings")

    return filtered_landings

def run_angle_filter(_landings, _max_angle):
    print()
    print('Angles: ')
    print(_landings[:, -3:-1])

    landings_filtered = _landings.copy()
    success_mask = landings_filtered[:, -1] == True
    angles_mask = np.any(np.abs(landings_filtered[:, -3:-1]) > _max_angle, axis=1)
    rows_to_flip_mask = success_mask & angles_mask
    landings_filtered[rows_to_flip_mask, -1] = False

    print()
    print(f"run_angle_filter modified {rows_to_flip_mask.sum()} landings out of {len(_landings)} landings")

    return landings_filtered

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

def run_overlap_filter(_landings, _min_dist):
    success_mask = _landings[:, -1] == True
    filtered_landings_s = overlap_filter(_landings[success_mask], _min_dist)
    filtered_landings_f = overlap_filter(_landings[~success_mask], _min_dist)
    filtered_landings = np.concatenate((filtered_landings_s, filtered_landings_f), axis=0)

    print()
    print(f"run_overlap_filter removed {len(_landings[success_mask]) - len(filtered_landings_s)} successful landings out of {len(_landings[success_mask])} successful landings")
    print(f"run_overlap_filter removed {len(_landings[~success_mask]) - len(filtered_landings_f)} failed landings out of {len(_landings[~success_mask])} failed landings")

    return filtered_landings

def run_filtered_ardulog(_idx, _should_filter: bool=True):
    landings = run_ardulog(_idx)

    print('Unfiltered landings: ')
    print(landings)

    landings_output = landings

    if _should_filter:
        DRONE_RADIUS = 1.5
        pcd = o3d.io.read_point_cloud(os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY))

        landings_bbox = run_bbox_filter(
            pcd,
            landings,
            0.0
        )

        PROJECTION_RADIUS = 0.3
        landings_proj = run_project_alt(
            pcd,
            landings_bbox,
            PROJECTION_RADIUS
        )

        MIN_POINTS = 300
        landings_empty = run_missing_points_filter(
            pcd,
            landings_proj,
            DRONE_RADIUS,
            MIN_POINTS
        )

        MAX_ANGLE = 45.0
        landings_angle = run_angle_filter(
            landings_empty,
            MAX_ANGLE
        )

        # OVERLAP_DIST = 0.3
        landings_overlap = run_overlap_filter(
            landings_angle,
            DRONE_RADIUS/3.0
        )

        if landings_overlap.size != 0:
            landings_output = landings_overlap

        print('Filtered landings')
        print(landings_output)

    return landings_output


def save_landing_cloud(_idx, _should_filter: bool=True):
    landings = run_filtered_ardulog(_idx, _should_filter)

    # Create separate point clouds
    local_coords = landings[:, :3]
    success_mask = landings[:, -1] == True

    success_pcd = o3d.geometry.PointCloud()
    success_pcd.points = o3d.utility.Vector3dVector(local_coords[success_mask])
    
    failure_pcd = o3d.geometry.PointCloud()
    failure_pcd.points = o3d.utility.Vector3dVector(local_coords[~success_mask])

    # Assign colors
    success_pcd.paint_uniform_color([0, 1, 0])  # Green for success
    failure_pcd.paint_uniform_color([1, 0, 0])  # Red for failure

    # Combine into a single point cloud
    combined_pcd = success_pcd + failure_pcd

    # Save as PLY file
    o3d.io.write_point_cloud(os.path.join(config.INPUTS_PATH, str(_idx), config.LANDINGS_CLOUD_PLY), combined_pcd, write_ascii=False)

    return landings

def add_ardulog(_df, _idx):
    landings = save_landing_cloud(_idx)

    df_ard = pd.DataFrame({
        'success': landings.T[-1],
        'landing_x': landings.T[0],
        'landing_y': landings.T[1],
        'landing_z': landings.T[2],
        'landing_lat': landings.T[3],
        'landing_long': landings.T[4],
        'landing_alt': landings.T[5],
        'landing_roll': landings.T[6],
        'landing_pitch': landings.T[7]
    })

    return pd.concat([_df, df_ard], axis=1)

def main():
    idx = 17
    df = pd.DataFrame()

    df = add_ardulog(df, idx)
    df.to_csv(os.path.join(config.OUTPUTS_PATH, str(idx), config.OUTPUT_CSV), index=False)
    print(df)

if __name__=="__main__":
    main()
