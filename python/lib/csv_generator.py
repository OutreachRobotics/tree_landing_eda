from ardulog import add_ardulog
from deepforest import add_deepforest
from pcl import add_pcl
from visualizer import viz

import config
import os
import pandas as pd


def generate_csv(_species: list[str]=[], _should_view: bool=False):
    for i in range(0,len(_species)):
        df = pd.DataFrame()
        df = add_deepforest(df, i)
        df = add_ardulog(df, i)
        df = add_pcl(df, i, _should_view)
        df['specie'] = _species[i]

        df.to_csv(os.path.join(config.OUTPUTS_PATH, str(i), config.OUTPUT_CSV), index=False)
        print(df)

def generate_csv(_idx, _specie, _should_view: bool=False):
    df = pd.DataFrame()
    df = add_deepforest(df, _idx)
    df = add_ardulog(df, _idx)
    df = add_pcl(df, _idx, _should_view)
    df['specie'] = _specie

    df.to_csv(os.path.join(config.OUTPUTS_PATH, str(_idx), config.OUTPUT_CSV), index=False)
    print(df)


def main():
    idx = 15
    specie = 'birch'
    should_viz = True

    pd.set_option('display.max_rows', None)  # Show all rows
    pd.set_option('display.max_columns', None)  # Show all columns

    # generate_csv(['birch', 'maple'], should_viz)
    generate_csv(idx, specie, should_viz)
    if should_viz:
        viz(idx)


if __name__=="__main__":
    main()
