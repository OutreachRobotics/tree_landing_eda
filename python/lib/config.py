import os

WS_PATH = os.path.join(os.path.expanduser('~'), 'tree_landing_eda')

INPUTS_PATH = os.path.join(WS_PATH, 'data', 'inputs')
OUTPUTS_PATH = os.path.join(WS_PATH, 'data', 'outputs')

HOME_CSV = 'home.csv'
ORIGIN_CSV = 'origin.csv'
SPECIE_CSV = 'specie.csv'
ORIGIN_LOG_CSV = 'origin_log.csv'
IMAGE_RGB_PNG = os.path.join('img_input', 'img_input.png')
IMAGE_RGB_POSE_CSV = os.path.join('img_input', 'img_input_global_pose.csv')
IMAGE_RGB_GEO_REF_TIF = 'img_input_geo_ref.tif'
RTABMAP_CLOUD_PLY = 'rtabmap_cloud.ply'
RTABMAP_CLOUD_GEO_REF_LAS = 'rtabmap_cloud.las'
LANDINGS_CLOUD_PLY = 'landings_cloud.ply'
LANDINGS_CLOUD_GEO_REF_LAS = 'landings_cloud.las'
PCL_CSV = 'output_pcl.csv'
PCL_PATTERN_CSV = 'output_pcl*.csv'
PCL_SCALE_PATTERN_CSV = 'output_pcl_scale_*.csv'
BOXES_CSV = 'boxes.csv'
TZKINV_TXT = 'TZKinv.txt'
OUTPUT_CSV = 'output.csv'
DECISIONTREE_SVG = 'decisiontree.svg'
DECISIONTREE_AGGREGATE_SVG = 'decisiontree_aggregate.svg'
MULTICOL_MATRIX_PLOT = 'multicolinearity_matrix'
MULTICOL_MATRIX_PLOT_PNG = f'{MULTICOL_MATRIX_PLOT}.png'
PAIR_PLOT = 'pair_plot'
PAIR_PLOT_PNG = f'{PAIR_PLOT}.png'

IGNORE_LIST = [
    'idx',
    'landing_x','landing_y','landing_z',
    'landing_lat','landing_long','landing_alt',
    'landing_roll','landing_pitch'
]

CAM_FOV_H = 87
CAM_FOV_V = 58

CAM_RES_H = 1280
CAM_RES_V = 720
