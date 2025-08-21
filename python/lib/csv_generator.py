from ardulog import add_ardulog
from deepforest import add_deepforest
from pcl import add_pcl
from visualizer import viz

import config
import glob
import os
import pandas as pd

def add_dataset_idx(_df, _idx):
    _df.insert(0, 'idx', _idx)

    return _df

def add_specie(_df, _idx):
    filepath_specie = os.path.join(config.INPUTS_PATH, str(_idx), config.SPECIE_CSV)

    try:
        specie_csv = pd.read_csv(filepath_specie)
        specie = specie_csv['specie'].iloc[0]
        _df['specie'] = specie
        
    except FileNotFoundError:
        print(f"WARNING: Specie file not found for index {_idx} at {filepath_specie}. Assigning a default value.")
        _df['specie'] = 'unknown'

    return _df

def combine_csv():
    path_pattern = os.path.join(config.OUTPUTS_PATH, '*', config.OUTPUT_CSV)
    all_files = glob.glob(path_pattern)

    if not all_files:
        print("No CSV files found to combine.")
        return

    df_list = [pd.read_csv(f) for f in all_files]
    combined_df = pd.concat(df_list, ignore_index=True)
    combined_df = combined_df.sort_values(by='idx').reset_index(drop=True)

    output_path = os.path.join(config.OUTPUTS_PATH, config.OUTPUT_CSV)

    combined_df.to_csv(output_path, index=False)

def generate_csv(_idx_list: list[int]=[], _should_view: bool=False):
    for i in _idx_list:
        os.makedirs(os.path.join(config.OUTPUTS_PATH, str(i)), exist_ok=True)

        df = pd.DataFrame()
        # df = add_deepforest(df, i)
        df = add_ardulog(df, i)
        df = add_pcl(df, i, _should_view)
        df = add_specie(df, i)
        df = add_dataset_idx(df, i)

        df.to_csv(os.path.join(config.OUTPUTS_PATH, str(i), config.OUTPUT_CSV), index=False)
        print(df)

        if _should_view:
            viz(i)

def main():
    idx_list = list(range(3))
    should_viz = False

    pd.set_option('display.max_rows', None)  # Show all rows
    pd.set_option('display.max_columns', None)  # Show all columns

    generate_csv(idx_list, should_viz)


if __name__=="__main__":
    main()
