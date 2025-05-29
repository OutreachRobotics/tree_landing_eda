from config import INPUTS_PATH, OUTPUTS_PATH, WS_PATH

import subprocess
import os
import pandas as pd


def run_pcl(_args):
    # Path to your compiled executable
    executable_path = os.path.join(f'{WS_PATH}', 'build', 'pcl')

    # Run the executable with arguments
    try:
        print("Running:", [executable_path] + _args)
        result = subprocess.run(
            [executable_path] + _args,
            check=True,
            text=True,
            capture_output=True
        )
        print("Program output:")
        print(result.stdout)  # Print the standard output of the program
    except subprocess.CalledProcessError as e:
        print("Error running the program:")
        print(e.stderr)  # Print the standard error of the program

def add_pcl(_df, _idx):
    pcl_data = []
    for i in range(len(_df)):
        landing_x = str(12.0) + str(i)
        landing_y = str(16.0) + str(i)
        args = [
            os.path.join(INPUTS_PATH, str(_idx), 'rtabmap_cloud.ply'),
            os.path.join(OUTPUTS_PATH, str(_idx), 'output_pcl.csv'),
            landing_x, # str(_df.at[i, 'landing_x'])
            landing_y, # str(_df.at[i, 'landing_y'])
            str(_df.at[i, 'center_x']),
            str(_df.at[i, 'center_y'])
        ]
        run_pcl(args)
        pcl_csv = pd.read_csv(os.path.join(OUTPUTS_PATH, str(_idx), 'output_pcl.csv'))
        pcl_data.append(pcl_csv.iloc[0].to_dict())

    df_pcl = pd.DataFrame(pcl_data)
    return pd.concat([_df, df_pcl], axis=1)