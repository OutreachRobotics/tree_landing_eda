#!/opt/conda/envs/docker/bin/python3

import csv_generator as csvg


def main():
   csvg.generate_csv(['maple', 'chestnut'])


if __name__=="__main__":
    main()
