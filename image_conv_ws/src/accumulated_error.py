#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def read_data(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            timestamp = float(parts[0])
            position = np.array(parts[1:4], dtype=float)
            orientation = np.array(parts[4:], dtype=float)
            data.append((timestamp, position, orientation))
    return data


def align_first_pose(gt_pose, out_pose):
    # Calculate the translation offset
    translation_offset = gt_pose[0] - out_pose[0]
    # Calculate the rotation difference using quaternions
    gt_rot = R.from_quat(gt_pose[1])
    out_rot = R.from_quat(out_pose[1])
    rotation_difference = gt_rot * out_rot.inv()
    return translation_offset, rotation_difference

def apply_alignment_to_trajectory(data, translation_offset, rotation_difference):
    aligned_data = []
    for timestamp, position, orientation in data:
        # Apply translation offset
        new_position = position + translation_offset
        # Apply rotation difference
        new_orientation = (rotation_difference * R.from_quat(orientation)).as_quat()
        aligned_data.append((timestamp, new_position, new_orientation))
    return aligned_data

def align_data(gt_data, out_data):
    aligned_data = []
    for out_timestamp, out_position, out_orientation in out_data:
        closest_gt = min(gt_data, key=lambda x: abs(x[0] - out_timestamp))
        aligned_data.append((closest_gt[1], closest_gt[2], out_position, out_orientation))
    return aligned_data

def calculate_errors(aligned_data):
    translational_errors = []
    rotational_errors = []
    for gt_pos, gt_orient, out_pos, out_orient in aligned_data:
        translational_errors.append(np.linalg.norm(gt_pos - out_pos))
        # Convert quaternions to rotation matrices for angle calculation
        gt_rot_mat = R.from_quat(gt_orient).as_matrix()
        out_rot_mat = R.from_quat(out_orient).as_matrix()
        # Calculate angle difference in radians
        rot_error_mat = np.matmul(gt_rot_mat, np.linalg.inv(out_rot_mat))
        angle_diff = np.arccos((np.trace(rot_error_mat) - 1) / 2)
        angle_diff_deg = np.degrees(angle_diff)
        rotational_errors.append(np.abs(angle_diff_deg))
    return np.array(translational_errors), np.array(rotational_errors)

def accumulate_values(values):
    return np.cumsum(values)

def main():
    gt_path = '/home/ssuryalolla/fusionlab_files/data/bag5/groundtruth.txt'  # Update with your actual groundtruth file path
    out_path = '/home/ssuryalolla/fusionlab_files/data/bag5/fusionbag5_data.klg.freiburg'  # Update with your actual output file path
    
    gt_data = read_data(gt_path)
    out_data = read_data(out_path)


    # Align the first poses and get the transformation
    translation_offset, rotation_difference = align_first_pose(gt_data[0][1:], out_data[0][1:])
    
    # Apply the alignment transformation to the entire output trajectory
    aligned_out_data = apply_alignment_to_trajectory(out_data, translation_offset, rotation_difference)
    
    aligned_data = align_data(gt_data, aligned_out_data)
    translational_errors, rotational_errors = calculate_errors(aligned_data)

    # print("Translation errors:")
    # print(translational_errors)
    # print("Rotation errors:")
    # print(rotational_errors)
    
    accumulated_trans_errors = accumulate_values(translational_errors)
    accumulated_rot_errors = accumulate_values(rotational_errors)
    
    # Replace the old distance calculation line with this
    distances = [np.linalg.norm(aligned_data[i+1][0] - aligned_data[i][0]) for i in range(len(aligned_data)-1)]
    accumulated_distances = accumulate_values([0] + distances)  # Start with 0

    
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.plot(accumulated_distances, accumulated_trans_errors, label='Translational Error')
    plt.xlabel('Distance Traveled (m)')
    plt.ylabel('Accumulated Translational Error (m)')
    plt.title('Accumulated Translational Error vs. Distance Traveled')
    plt.legend()
    
    plt.subplot(1, 2, 2)
    plt.plot(accumulated_distances, accumulated_rot_errors, label='Rotational Error', color='red')
    plt.xlabel('Distance Traveled (m)')
    plt.ylabel('Accumulated Rotational Error (deg)')
    plt.title('Accumulated Rotational Error vs. Distance Traveled')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
