#!/opt/conda/envs/docker/bin/python3

from pathlib import Path

import os
import sys

project_root = Path(__file__).resolve().parent.parent.parent

lib_path = project_root / "python" / "lib"
sys.path.insert(0, str(lib_path))

print(f"Project root: {project_root}")
print(f"Current sys.path: {sys.path}")

from csv_generator import generate_csv

import config


def main():
   idx = 15
   specie = 'birch'

   init_idx = 12
   species = ['maple', 'chestnut', 'birch']

   should_view = True

   for i in range(len(species)):
      os.makedirs(os.path.join(config.OUTPUTS_PATH, str(i)), exist_ok=True)

   # generate_csv(init_idx, species, should_view)
   generate_csv(idx, specie, should_view)


if __name__=="__main__":
    main()
