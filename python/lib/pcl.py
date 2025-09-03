import config
import subprocess
import os
import glob
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
        if i == 0 and _should_view:
            args = [
                os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY),
                os.path.join(config.OUTPUTS_PATH, str(_idx), config.PCL_CSV),
                str(_df.at[i, 'landing_x']),
                str(_df.at[i, 'landing_y']),
                str(_df.at[i, 'landing_z']),
                str(True).lower()
            ]
        else:
            args = [
                os.path.join(config.INPUTS_PATH, str(_idx), config.RTABMAP_CLOUD_PLY),
                os.path.join(config.OUTPUTS_PATH, str(_idx), config.PCL_CSV),
                str(_df.at[i, 'landing_x']),
                str(_df.at[i, 'landing_y']),
                str(_df.at[i, 'landing_z']),
                str(False).lower()
            ]
        run_pcl(args)

        search_path = os.path.join(config.OUTPUTS_PATH, str(_idx), config.PCL_PATTERN_CSV)
        csv_files = glob.glob(search_path)

        df_list = []
        for file_path in csv_files:
            try:
                filename = os.path.basename(file_path)

                if 'fail' in filename:
                    continue

                if '_scale_' in filename:
                    scale_part = filename.split('_scale_')[-1] # This gives "1.00.csv"
                    scale_factor = scale_part.replace('.csv', '') # This gives "1.00"

                    if float(scale_factor) == 0.0:
                        print(f"Skipping zero-scale file: {filename}")
                        continue # Skip this file

                    df = pd.read_csv(file_path)
                    
                    df.columns = [f"{col}_scale_{scale_factor}" for col in df.columns]
                    
                    df_list.append(df)

                else:
                    print(f"Processing non-scaled file: {filename}")
                    df = pd.read_csv(file_path)
                    df_list.append(df) # Append without renaming columns

                
            except Exception as e:
                print(f"Could not process file {file_path}. Error: {e}")

        pcl_data.append(pd.concat(df_list, axis=1).iloc[0].to_dict())

        # pcl_csv = pd.read_csv(os.path.join(config.OUTPUTS_PATH, str(_idx), config.PCL_CSV))
        # pcl_data.append(pcl_csv.iloc[0].to_dict())

    df_pcl = pd.DataFrame(pcl_data)

    return pd.concat([_df, df_pcl], axis=1)

def main():
    # Pacman 17 18 20 23 25
    # Divided trees 4 23 24 26
    # Fused trees 14 18
    idx = 17 # 6 13 15
    should_viz = True

    df = pd.read_csv(os.path.join(config.OUTPUTS_PATH, str(idx), config.OUTPUT_CSV))
    df = add_pcl(df, idx, should_viz)
    df.to_csv(os.path.join(config.OUTPUTS_PATH, str(idx), config.OUTPUT_CSV), index=False)
    print(df)

if __name__=="__main__":
    main()
