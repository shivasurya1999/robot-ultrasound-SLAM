#!/usr/bin/env python3


import numpy as np


def load_traj(input_filepath):
    try:
        with open(input_filepath) as f:
            data = [line.strip().split() for line in f.readlines()]
        timestamps = [float(line[0]) for line in data]
        translations = np.array([line[1:4] for line in data], dtype=float)  # Directly create a 2D array
        rotations = np.array([line[4:8] for line in data], dtype=float)  # Directly create a 2D array
    except FileNotFoundError as e:
        print(f"Error: {e}")
        timestamps, translations, rotations = [], np.array([]), np.array([])
    
    return timestamps, translations, rotations


def compute_direc_diff(ground_truth_positions, estimated_positions):
    # Compute differences as a NumPy array
    differences = estimated_positions - ground_truth_positions

    # Separate the differences by axis
    x_diff, y_diff, z_diff = differences.T  # Transpose and unpack for clarity

    # Analyze the directionality of errors using NumPy for mean calculation
    print("Mean error in x:", np.mean(x_diff))
    print("Mean error in y:", np.mean(y_diff))
    print("Mean error in z:", np.mean(z_diff))


if __name__ == '__main__':
    input_filepath1 = '/home/ssuryalolla/fusionlab_files/data/bag10/synced_groundtruth.txt'  # Adjust this path
    input_filepath2 = '/home/ssuryalolla/fusionlab_files/data/bag10/synced_fusionbag10_data.txt'  # Adjust this path

    timestamps1, translations1, rotations1 = load_traj(input_filepath1) #load gt data 
    timestamps2, translations2, rotations2 = load_traj(input_filepath2) #load output data 

    compute_direc_diff(translations1,translations2)