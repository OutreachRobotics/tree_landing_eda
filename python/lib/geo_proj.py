from affine import Affine
from matplotlib import image

import config
import math
import numpy as np
import open3d as o3d
import os
import pandas as pd
import pdal
import rasterio
import pyproj


class DronePose:
    def __init__(self, _lat, _long, _rel_alt, _yaw):
        self.lat = _lat    # Latitude
        self.long = _long  # Longitude
        self.rel_alt = _rel_alt  # Altitude/height
        # Transforming from deg to rad and
        # Shifting 0 degrees from heading north to east
        self.yaw = math.radians(_yaw) - math.radians(90)

def get_home(_filepath_home):
    home_csv = pd.read_csv(_filepath_home)
    coord_home = home_csv[['latitude', 'longitude', 'altitude']].iloc[0].tolist()

    # Compensates mavros home ellipsoid to geoid
    # https://github.com/mavlink/mavros/blob/7ee83a833676af38a0cacc6c12733746f84cacca/mavros/src/plugins/home_position.cpp#L165
    undulation = get_geoid_undulation(coord_home[0], coord_home[1])
    coord_home[2] += undulation

    # print('coord_home:')
    # print(coord_home)

    return coord_home

def get_origin(_filepath_origin):
    origin_csv = pd.read_csv(_filepath_origin)
    coord_origin = origin_csv[['latitude', 'longitude', 'altitude']].iloc[0].tolist()

    print('coord_origin:')
    print(coord_origin)

    return coord_origin

def get_global_pose(_filepath_pose):
    global_pose_csv = pd.read_csv(_filepath_pose)
    global_pose = global_pose_csv[['latitude', 'longitude', 'rel_alt', 'yaw']].iloc[0].tolist()
    return global_pose

def get_meters_per_degree_at_coord(_coord_lat, _coord_long):
    # Define the WGS84 ellipsoid
    geod = pyproj.Geod(ellps="WGS84")

    # Calculate meters per degree of longitude (change in longitude, keep latitude constant)
    _, _, meters_per_degree_lon = geod.inv(_coord_long, _coord_lat, _coord_long + 1, _coord_lat)

    # Calculate meters per degree of latitude (change in latitude, keep longitude constant)
    _, _, meters_per_degree_lat = geod.inv(_coord_long, _coord_lat, _coord_long, _coord_lat + 1)

    return meters_per_degree_lat, meters_per_degree_lon

def get_geoid_undulation(latitude: float, longitude: float) -> float:
    """
    Calculates the geoid undulation (N) at a specific location.
    This is the height of the geoid relative to the WGS84 ellipsoid.
    """
    # # DEBUGGING:
    # print(pyproj.datadir.get_data_dir())
    # tg = pyproj.transformer.TransformerGroup(4326, 5713)
    # print(tg)
    # # print(tg.transformers[0].description)
    # # print(tg.unavailable_operations[0].name)
    # # print(tg.unavailable_operations[0].grids[0].url)

    # https://epsg.io/
    # 3855 EGM2008 height
    # 4326 WGS84 2D (Ellipsoid)
    # 4979 WGS84 3D (Ellipsoid)
    # 5713 AMSL Canada (height) (Geoid)
    # 5714 AMSL World (height) (Geoid)
    # 5773 EGM96 (height) (Geoid) (Mavros) # https://github.com/mavlink/mavros/blob/7ee83a833676af38a0cacc6c12733746f84cacca/mavros/include/mavros/mavros_uas.hpp#L164
    # 9705 WGS84 + AMSL World (Geoid)
    t = pyproj.Transformer.from_crs("epsg:4326", "epsg:5773", always_xy=True)
    _, _, geoid_undulation = t.transform(longitude, latitude, 0.0)

    print(f'Undulation: {geoid_undulation}')
    
    return geoid_undulation

def compute_transform(_lat, _long, _scale_lat, _scale_long, _yaw, _debug=False):
    trans_x = _long
    trans_y = _lat
    scale_x = _scale_long
    scale_y = _scale_lat
    # Create the individual matrices
    translation_matrix = Affine.translation(trans_x, trans_y)
    scaling_matrix = Affine.scale(scale_x, scale_y)
    rotation_matrix = Affine.rotation(math.degrees(_yaw))
    transform = translation_matrix * scaling_matrix * rotation_matrix

    if _debug:
        # Print the individual matrices
        print("\nTranslation Matrix:")
        print(f"|{translation_matrix.a:.8f}, {translation_matrix.b:.8f}, {translation_matrix.c:.8f}|")
        print(f"|{translation_matrix.d:.8f}, {translation_matrix.e:.8f}, {translation_matrix.f:.8f}|")
        print(f"|{translation_matrix.g:.8f}, {translation_matrix.h:.8f}, {translation_matrix.i:.8f}|")

        print("\nRotation Matrix:")
        print(f"|{rotation_matrix.a:.8f}, {rotation_matrix.b:.8f}, {rotation_matrix.c:.8f}|")
        print(f"|{rotation_matrix.d:.8f}, {rotation_matrix.e:.8f}, {rotation_matrix.f:.8f}|")
        print(f"|{rotation_matrix.g:.8f}, {rotation_matrix.h:.8f}, {rotation_matrix.i:.8f}|")

        print("\nScaling Matrix:")
        print(f"|{scaling_matrix.a:.8f}, {scaling_matrix.b:.8f}, {scaling_matrix.c:.8f}|")
        print(f"|{scaling_matrix.d:.8f}, {scaling_matrix.e:.8f}, {scaling_matrix.f:.8f}|")
        print(f"|{scaling_matrix.g:.8f}, {scaling_matrix.h:.8f}, {scaling_matrix.i:.8f}|")

        print(f"\nImage transform:")
        print(f"|{transform.a:.8f}, {transform.b:.8f}, {transform.c:.8f}|")
        print(f"|{transform.d:.8f}, {transform.e:.8f}, {transform.f:.8f}|")
        print(f"|{transform.g:.8f}, {transform.h:.8f}, {transform.i:.8f}|")

    return transform

def get_local_coord(_origin, _coord):
    meters_per_degree_lat, meters_per_degree_lon = get_meters_per_degree_at_coord(_origin[0], _origin[1])
    transform = compute_transform(
        _origin[0],
        _origin[1],
        1/meters_per_degree_lat,
        1/meters_per_degree_lon,
        0
    )

    transform_inv = ~transform

    # print(f"\nTransform:")
    # print(f"|{transform.a}, {transform.b}, {transform.c}|")
    # print(f"|{transform.d}, {transform.e}, {transform.f}|")
    # print(f"|{transform.g}, {transform.h}, {transform.i}|")

    # print(f"\nTransform_inv:")
    # print(f"|{transform_inv.a}, {transform_inv.b}, {transform_inv.c}|")
    # print(f"|{transform_inv.d}, {transform_inv.e}, {transform_inv.f}|")
    # print(f"|{transform_inv.g}, {transform_inv.h}, {transform_inv.i}|")

    local_x, local_y = transform_inv * (_coord[1], _coord[0])

    local_z = _coord[2] - _origin[2]

    return np.array([local_x, local_y, local_z])


### RGB GEO REF ###

def compute_pixel_size(_fov, _full_res, _height):
    length = 2*_height*math.tan(math.radians(_fov/2.0))
    return float(length / float(_full_res))


def compute_img_size(_fov, _full_res, _partial_res, _height):
    pixel_size = compute_pixel_size(_fov, _full_res, _height)
    return pixel_size * _partial_res

def rotate_corners(_corners, _yaw):
    """
    Rotate the corners around the center using the given yaw angle.

    Args:
        _corners (list of tuples): Local coordinates of the corners relative to the center.
        _yaw (float): Yaw angle in radians.

    Returns:
        list of tuples: Rotated corners in local coordinates.
    """
    corners_rotated = [
        (
            x * math.cos(_yaw) - y * math.sin(_yaw),  # Rotated x
            x * math.sin(_yaw) + y * math.cos(_yaw),  # Rotated y
        )
        for x, y in _corners
    ]
    return corners_rotated

def get_corners(_width, _height):
    return [
        (-_width / 2.0, _height / 2.0),  # Top-left
        (_width / 2.0, _height / 2.0),   # Top-right
        (-_width / 2.0, -_height / 2.0), # Bottom-left
        (_width / 2.0, -_height / 2.0),  # Bottom-right
    ]

def compute_bbox(_corners):
    max_x = max(corner[0] for corner in _corners)
    min_x = min(corner[0] for corner in _corners)
    max_y = max(corner[1] for corner in _corners)
    min_y = min(corner[1] for corner in _corners)

    x_size = max_x - min_x
    y_size = max_y - min_y

    return x_size, y_size

def compute_geo_ref(_drone_pose, _png_img):
    image_height_pixels = _png_img.shape[0]
    image_width_pixels = _png_img.shape[1]
    image_width_meters = compute_img_size(config.CAM_FOV_H, config.CAM_RES_H, image_width_pixels, _drone_pose.rel_alt)
    image_height_meters = compute_img_size(config.CAM_FOV_V, config.CAM_RES_V, image_height_pixels, _drone_pose.rel_alt)

    meters_per_degree_lat, meters_per_degree_lon = get_meters_per_degree_at_coord(_drone_pose.lat, _drone_pose.long)

    print(f"\n\nImage Width (meters): {image_width_meters:.2f}")
    print(f"Meters per Degree Longitude: {meters_per_degree_lon:.2f}")
    print(f"Image Height (meters): {image_height_meters:.2f}")
    print(f"Meters per Degree Latitude: {meters_per_degree_lat:.2f}")
    print(f"Meters per degree of longitude at latitude {_drone_pose.lat}: {meters_per_degree_lon}")
    print(f"Meters per degree of latitude at latitude {_drone_pose.lat}: {meters_per_degree_lat}\n\n")

    # Convert the rotated local coordinates to geographic coordinates
    corners_meters = get_corners(image_width_meters, image_height_meters)
    corners_meters_rotated = rotate_corners(corners_meters, _drone_pose.yaw)
    corners_geo = [
        (_drone_pose.long + (x / meters_per_degree_lon), _drone_pose.lat + (y / meters_per_degree_lat))
        for x, y in corners_meters_rotated
    ]
    img_long, img_lat = compute_bbox(corners_geo)
    return img_long, img_lat, corners_geo

def compute_scale(_png_img, _yaw, _img_lat, _img_long, _corners_geo):
    image_height_pixels = _png_img.shape[0]
    image_width_pixels = _png_img.shape[1]

    corners_pixels = get_corners(image_width_pixels, image_height_pixels)
    corners_pixels_rotated = rotate_corners(corners_pixels, _yaw)
    img_pix_x, img_pix_y = compute_bbox(corners_pixels_rotated)

    pixel_size_long = _img_long/img_pix_x
    pixel_size_lat = _img_lat/img_pix_y

    print("Geo corners: " + str(_corners_geo))
    print(f"img_long: {_img_long:.8f}")
    print(f"img_lat: {_img_lat:.8f}")
    print(f"img_pix_x: {img_pix_x:.8f}")
    print(f"img_pix_y: {img_pix_y:.8f}")
    print(f"Image Width (pixels): {image_width_pixels:.8f}")
    print(f"Image Height (pixels): {image_height_pixels:.8f}")
    print(f"Top-Left Longitude: {_corners_geo[0][0]:.8f}")
    print(f"Top-Left Latitude: {_corners_geo[0][1]:.8f}")
    print(f"YAW: {_yaw:.8f}")
    print(f"Pixel Size (Long, degrees): {pixel_size_long:.8f}")
    print(f"Pixel Size (Lat, degrees): {pixel_size_lat:.8f}")

    return pixel_size_lat, pixel_size_long

def save_raster(_output_img_file, _png_img, _transform):
    # Define the CRS (e.g., WGS84)
    crs = 'EPSG:4326'

    # Save the geo-referenced image as a GeoTIFF
    with rasterio.open(
        _output_img_file,
        'w',
        driver='GTiff',
        height=_png_img.shape[0],
        width=_png_img.shape[1],
        count=3,  # Number of bands (3 for RGB)
        dtype=_png_img.dtype,
        crs=crs,
        transform=_transform,
    ) as dst:
        # Write the RGB bands to the GeoTIFF
        for band in range(3):
            dst.write(_png_img[:, :, band], band + 1)

def compute_geo_ref_rgb(_input_img_file: str, _input_img_pose_file: str, _output_img_file: str):
    global_pose = get_global_pose(_input_img_pose_file)
    drone_pose = DronePose(global_pose[0], global_pose[1], global_pose[2], global_pose[3])

    # Load the PNG image as a numpy array
    png_image = image.imread(_input_img_file)

    img_long, img_lat, corners_geo = compute_geo_ref(drone_pose, png_image)

    # Find the top-left corner in geographic coordinates
    top_left_lon = corners_geo[0][0]
    top_left_lat = corners_geo[0][1]

    pixel_size_lat, pixel_size_long = compute_scale(
        png_image,
        drone_pose.yaw,
        img_lat,
        img_long,
        corners_geo
    )

    transform = compute_transform(
        top_left_lat,
        top_left_lon,
        -pixel_size_lat, # Invert y axis
        pixel_size_long,
        -drone_pose.yaw # Invert rotation because of inverted y axis
    )
    save_raster(_output_img_file, png_image, transform)


### CLOUD GEO REF ###

def compute_cloud_size(_point_cloud):
    # Get the points as a NumPy array
    points = np.asarray(_point_cloud.points)

    # Compute the minimum and maximum values for X and Y
    min_x, min_y = np.min(points[:, 0]), np.min(points[:, 1])
    max_x, max_y = np.max(points[:, 0]), np.max(points[:, 1])

    # Compute the dimensions (width and height)
    x_dimension = max_x - min_x
    y_dimension = max_y - min_y

    return x_dimension, y_dimension

def affine2pdal(_affine):
    # Convert the affine matrix to a 4x4 matrix for PDAL
    affine_np = np.array(_affine).reshape(3, 3)
    matrix_3d = np.eye(4)  # Create a 4x4 identity matrix
    matrix_3d[:2, :2] = affine_np[:2, :2]  # Copy rotation and scaling
    matrix_3d[:2, 3] = affine_np[:2, 2]    # Copy translation

    print("\nTransformation Matrix:")
    for row in matrix_3d:
        print(f"|{row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}, {row[3]:.8f}|")

    # Flatten the matrix to a space-separated string for PDAL
    return " ".join(map(str, matrix_3d.flatten()))

def compute_pdal(_transform, _input_file, _output_file):
    # Flatten the matrix to a space-separated string for PDAL
    pdal_matrix = affine2pdal(_transform)

    pipeline_json = """
    [
        {
            "type": "readers.ply",
            "filename": "{INPUT}"
        },
        {
            "type": "filters.transformation",
            "matrix": "{MATRIX}"
        },
        {
            "type": "writers.las",
            "a_srs": "EPSG:4326",
            "filename": "{OUTPUT}",
            "scale_x":"0.0000001",
            "scale_y":"0.0000001",
            "scale_z":"0.0000001"
        }
    ]
    """

    # Replace the placeholder with the actual matrix
    pipeline_json = pipeline_json.replace("{INPUT}", _input_file)
    pipeline_json = pipeline_json.replace("{OUTPUT}", _output_file)
    pipeline_json = pipeline_json.replace("{MATRIX}", pdal_matrix)

    # Create the PDAL pipeline
    pipeline = pdal.Pipeline(pipeline_json)

    # Execute the pipeline
    pipeline.execute()

def compute_geo_ref_cloud(_input_file: str, _origin_file: str, _output_file: str):
    # coord_home = get_home(_home_file)
    # home_lat = coord_home[0]
    # home_long = coord_home[1]

    coord_origin = get_origin(_origin_file)
    origin_lat = coord_origin[0]
    origin_long = coord_origin[1]

    meters_per_degree_lat, meters_per_degree_lon = get_meters_per_degree_at_coord(origin_lat, origin_long)

    # Load the point cloud from the PLY file
    point_cloud = o3d.io.read_point_cloud(_input_file)
    x_dimension, y_dimension = compute_cloud_size(point_cloud)

    # Print the results
    print(f"X Dimension (Width): {x_dimension:.8f} units")
    print(f"Y Dimension (Height): {y_dimension:.8f} units")
    print(f"meters_per_degree_lon: {meters_per_degree_lon:.10f}")
    print(f"meters_per_degree_lat: {meters_per_degree_lat:.10f}")
    print(f"1/meters_per_degree_lon: {1/meters_per_degree_lon:.10f}")
    print(f"1/meters_per_degree_lat: {1/meters_per_degree_lat:.10f}")

    transform = compute_transform(
        origin_lat,
        origin_long,
        1/meters_per_degree_lat,
        1/meters_per_degree_lon,
        0 # Local frame is usually already aligned with global frame
    )

    compute_pdal(transform, _input_file, _output_file)


def main():
    id = 5
    compute_geo_ref_rgb(
        os.path.join(config.INPUTS_PATH, str(id), config.IMAGE_RGB_PNG),
        os.path.join(config.INPUTS_PATH, str(id), config.IMAGE_RGB_POSE_CSV),
        os.path.join(config.OUTPUTS_PATH, str(id), config.IMAGE_RGB_GEO_REF_TIF)
    )
    compute_geo_ref_cloud(
        os.path.join(config.INPUTS_PATH, str(id), config.RTABMAP_CLOUD_PLY),
        # os.path.join(config.INPUTS_PATH, str(id), config.HOME_CSV),
        os.path.join(config.INPUTS_PATH, str(id), config.ORIGIN_CSV),
        os.path.join(config.OUTPUTS_PATH, str(id), config.RTABMAP_CLOUD_GEO_REF_LAS)
    )

if __name__=="__main__":
    main()
    