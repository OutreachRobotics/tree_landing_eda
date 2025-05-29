import os

WS_PATH = os.path.join(os.path.expanduser('~'), 'tree_landing_eda')

INPUTS_PATH = os.path.join(WS_PATH, 'data', 'inputs')
OUTPUTS_PATH = os.path.join(WS_PATH, 'data', 'outputs')

HOME = 'home.csv'
IMAGE_RGB = 'img_input.png'
IMAGE_RGB_GEO_REF = 'img_input_geo_ref.tif'
RTABMAP_CLOUD = 'rtabmap_cloud.ply'
RTABMAP_CLOUD_GEO_REF = 'rtabmap_cloud.las'
LANDINGS_CLOUD = 'landings_cloud.ply'
LANDINGS_CLOUD_GEO_REF = 'landings_cloud.las'
PCL_CSV = 'output_pcl.csv'
BOXES_CSV = 'boxes.csv'
TZKINV = 'TZKinv.txt'
OUTPUT_CSV = 'output.csv'

CAM_FOV_H = 87
CAM_FOV_V = 58
