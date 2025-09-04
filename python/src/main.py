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
from decision_tree import generate_decision_trees
from stats_visualizer import generate_stats_viz

import config


def main():
   idx_list = list(range(27))
   should_view = False
   ignore_list_tree = ['Distance_Top','Distance_Tree_Center_2D','Distance_Tree_Center_3D',
                       'Distance_Tree_Highest_Point_2D','Distance_Tree_Highest_Point_3D']
   ignore_list_viz = [
        'Min_Curvature','Mean_Curvature','Gaussian_Curvature',
        'Distance_Top','Distance_Tree_Center_2D','Ratio_Tree_Center_2D',
        'Distance_Tree_Highest_Point_2D','Ratio_Tree_Highest_Point_2D',
        'Distance_Tree_Center_3D','Distance_Tree_Highest_Point_3D',
        'Tree_Minor_Diameter'
    ]

   generate_csv(idx_list, should_view)
   combine_csv()
   generate_decision_trees(ignore_list_tree)
   generate_decision_trees(ignore_list_tree, 'sugar_maple')
        
   generate_stats_viz(ignore_list_viz)

if __name__=="__main__":
    main()
