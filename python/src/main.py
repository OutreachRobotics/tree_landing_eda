#!/opt/conda/envs/docker/bin/python3

from pathlib import Path

import os
import sys

project_root = Path(__file__).resolve().parent.parent.parent

lib_path = project_root / "python" / "lib"
sys.path.insert(0, str(lib_path))

print(f"Project root: {project_root}")
print(f"Current sys.path: {sys.path}")

from csv_generator import generate_csv, combine_csv
from decision_tree import decision_tree

import config


def main():
   init_idx = 16
   species = ['birch', 'birch', 'birch', 'conifer']
   should_view = False

   generate_csv(init_idx, species, should_view)
   combine_csv()
   decision_tree()

if __name__=="__main__":
    main()
