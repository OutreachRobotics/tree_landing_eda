from ardulog import add_ardulog
from deepforest import add_deepforest
from pcl import add_pcl

import config
import os
import pandas as pd

def generate_csv(_species: list[str]=[]):
    for i in range(0,len(_species)):
        df = pd.DataFrame()
        df = add_deepforest(df, i)
        df = add_ardulog(df, i)
        df = add_pcl(df, i)
        df['specie'] = _species[i]

        df.to_csv(os.path.join(config.OUTPUTS_PATH, str(i), config.OUTPUT_CSV), index=False)
        print(df)

def generate_csv(_idx, _specie):
    df = pd.DataFrame()
    df = add_deepforest(df, _idx)
    df = add_ardulog(df, _idx)
    df = add_pcl(df, _idx)
    df['specie'] = _specie

    df.to_csv(os.path.join(config.OUTPUTS_PATH, str(_idx), config.OUTPUT_CSV), index=False)
    print(df)


def main():
    pd.set_option('display.max_rows', None)  # Show all rows
    pd.set_option('display.max_columns', None)  # Show all columns

    # generate_csv(['birch', 'maple'])
    generate_csv(1, 'birch')


if __name__=="__main__":
    main()
