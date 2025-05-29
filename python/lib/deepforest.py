import config
import numpy as np
import os
import pandas as pd

def pixel_to_map(_u, _v, _idx):
    pix = np.array([_u, _v, 1, 1])
    TZKinv = np.loadtxt(
        os.path.join(config.INPUTS_PATH, str(_idx), config.TZKINV),
        dtype=np.float64,  # Force 64-bit precision
        delimiter=None,     # Auto-detect whitespace
        ndmin=2            # Ensure 2D even if file has 1 row
    )

    map = (TZKinv@pix)[:3]

    # print('Map:')
    # print(map)

    return map

def compute_target(_boxes, _idx):
    max_area_idx = _boxes["area"].idxmax()
    center_x = int(_boxes.loc[max_area_idx, "x"])
    center_y = int(_boxes.loc[max_area_idx, "y"])

    max_corner = pixel_to_map(_boxes.loc[max_area_idx, "xmax"], _boxes.loc[max_area_idx, "ymax"], _idx)
    min_corner = pixel_to_map(_boxes.loc[max_area_idx, "xmin"], _boxes.loc[max_area_idx, "ymin"], _idx)

    center = pixel_to_map(center_x, center_y, _idx)
    width = max_corner[0] - min_corner[0]
    height = max_corner[1] - min_corner[1]
    smallest_side = min(width, height)
    diagonal = (width**2 + height**2) ** 0.5
    area = width*height

    print('Center pix:')
    print(str(center_x) + ', ' + str(center_y))
    print('Center meters:')
    print(str(center))

    data = {
        'center_x': [center[0]],
        'center_y': [center[1]],
        'center_z': [center[2]],
        'smallest_side': [smallest_side],
        'diagonal': [diagonal],
        'area': [area]
    }

    return pd.DataFrame(data)

def add_deepforest(_df, _idx):
    boxes_csv = pd.read_csv(os.path.join(config.INPUTS_PATH, str(_idx), config.BOXES_CSV))
    df_deepforest = compute_target(boxes_csv, _idx)
    return pd.concat([_df, df_deepforest])
