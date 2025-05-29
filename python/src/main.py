#!/opt/conda/envs/docker/bin/python3

from pathlib import Path

import os
import sys

project_root = Path(__file__).resolve().parent.parent.parent

lib_path = project_root / "python" / "lib"
sys.path.insert(0, str(lib_path))

print(f"Project root: {project_root}")
print(f"Current sys.path: {sys.path}")

from config import OUTPUTS_PATH
from csv_generator import generate_csv
from visualizer import viz


def main():
   species = ['maple', 'chestnut', 'birch']
   for i in range(len(species)):
      os.makedirs(os.path.join(OUTPUTS_PATH, str(i)), exist_ok=True)
   # generate_csv(species)
   # viz(2)


if __name__=="__main__":
    main()
