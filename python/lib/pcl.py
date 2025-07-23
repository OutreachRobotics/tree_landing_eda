import config
import subprocess
import os
import pandas as pd


def run_pcl(_args):
    # Path to your compiled executable
    executable_path = os.path.join(config.WS_PATH, 'build', 'pcl')

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

def add_pcl(_df, _idx, _should_view):
    pcl_data = []
    for i in range(len(_df)):
        args = [
            os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY),
            os.path.join(config.OUTPUTS_PATH, str(_idx), config.PCL_CSV),
            str(_df.at[i, 'landing_x']),
            str(_df.at[i, 'landing_y']),
            str(_df.at[i, 'landing_z']),
            str(_should_view).lower()
        ]
        run_pcl(args)
        pcl_csv = pd.read_csv(os.path.join(config.OUTPUTS_PATH, str(_idx), config.PCL_CSV))
        pcl_data.append(pcl_csv.iloc[0].to_dict())

    df_pcl = pd.DataFrame(pcl_data)
    return pd.concat([_df, df_pcl], axis=1)
