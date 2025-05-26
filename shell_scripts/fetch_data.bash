#!/bin/bash

scp -r latte@10.42.0.120:~/ros_ws/src/tree_landing/data/outputs/* ~/tree_landing_eda/data/inputs
# ssh latte@10.42.0.120 "rm -r ~/ros_ws/src/tree_landing/data/outputs/*"
