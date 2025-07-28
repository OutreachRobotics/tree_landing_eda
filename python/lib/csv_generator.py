from ardulog import add_ardulog
from deepforest import add_deepforest
from pcl import add_pcl
from visualizer import viz

import config
import os
import pandas as pd


def generate_csv(_init_idx: int=0, _species: list[str]=[], _should_view: bool=False):
    for i in range(_init_idx, _init_idx + len(_species)):
        df = pd.DataFrame()
        # df = add_deepforest(df, i)
        df = add_ardulog(df, i)
        df = add_pcl(df, i, _should_view)
        df['specie'] = _species[i - _init_idx]

        df.to_csv(os.path.join(config.OUTPUTS_PATH, str(i), config.OUTPUT_CSV), index=False)
        print(df)

        if _should_view:
            viz(i)

def main():
    idx = 16
    species = ['birch', 'birch', 'birch', 'birch', 'conifer']
    should_viz = True

    pd.set_option('display.max_rows', None)  # Show all rows
    pd.set_option('display.max_columns', None)  # Show all columns

    generate_csv(idx, species, should_viz)


if __name__=="__main__":
    main()
