#!/opt/conda/envs/docker/bin/python3

import sys
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent.parent

lib_path = project_root / "python" / "lib"
sys.path.insert(0, str(lib_path))

print(f"Project root: {project_root}")
print(f"Current sys.path: {sys.path}")

from csv_generator import generate_csv


def main():
   generate_csv(['maple', 'chestnut', 'birch'])


if __name__=="__main__":
    main()
