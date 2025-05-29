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


def main():
    generate_csv(['birch', 'maple'])


if __name__=="__main__":
    main()
