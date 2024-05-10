#!/usr/bin/env python3


import numpy as np 



def load_traj(input_filepath1):
    try:
        with open(input_filepath1) as f1:
            data = [line.strip().split() for line in f1.readlines()]
        timestamps1 = [float(line[0]) for line in data]
        translations1 = [np.array(line[1:4], dtype=float) for line in data]  # Assuming x, y, z are columns 1, 2, 3
        rotations1 = [np.array(line[4:8], dtype=float) for line in data]  # Assuming qx, qy, qz, qw are in columns 4, 5, 6, 7   
                
    except FileNotFoundError as e:
        print(f"Error: {e}")
    
    return timestamps1, translations1, rotations1


def quaternion_to_rot_matrix(q):
    """Convert a quaternion into a rotation matrix."""
    q = np.array(q, dtype=np.float64)
    n = np.dot(q, q)
    if n < np.finfo(q.dtype).eps:
        return np.identity(3)
    q *= np.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3], q[1, 2]-q[3, 0], q[1, 3]+q[2, 0]],
        [q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3], q[2, 3]-q[1, 0]],
        [q[1, 3]-q[2, 0], q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]]
    ])


def rotation_matrix_to_euler_angles(R):
    """
    Convert a rotation matrix into Euler angles (roll, pitch, yaw) using the
    XYZ convention. Returns angles in degrees.
    """
    if R[2, 0] < 1:
        if R[2, 0] > -1:
            pitch = np.arcsin(R[2, 0])
            roll = np.arctan2(-R[2, 1], R[2, 2])
            yaw = np.arctan2(-R[1, 0], R[0, 0])
        else:
            pitch = -np.pi/2
            roll = -np.arctan2(R[1, 2], R[1, 1])
            yaw = 0
    else:
        pitch = np.pi/2
        roll = np.arctan2(R[1, 2], R[1, 1])
        yaw = 0

    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)


def calculate_transformation(trans1, rot1, trans2, rot2):
    """Calculate the rotation matrix and translation vector from frame 2 to frame 1."""
    rot_matrix1 = quaternion_to_rot_matrix(rot1)
    rot_matrix2 = quaternion_to_rot_matrix(rot2)
    
    # Calculate relative rotation
    relative_rot_matrix = np.dot(rot_matrix1, rot_matrix2.T)
    # relative_rot_matrix = np.array([[4.33653353e-03, 9.98151713e-01, 6.06164322e-02],
    #                         [9.96466773e-01, 7.71046981e-04, -8.39843793e-02],
    #                         [-8.38758902e-02, 6.07664616e-02, -9.94621673e-01]])
    
    # Calculate relative translation
    relative_translation = trans1 - np.dot(relative_rot_matrix, trans2)
    
    return relative_rot_matrix, relative_translation


if __name__ == '__main__':
    input_filepath1 = '/home/ssuryalolla/fusionlab_files/data/bag6/synced_groundtruth.txt'  
    input_filepath2 = '/home/ssuryalolla/fusionlab_files/data/bag6/synced_fusionbag6_data.txt'  

    timestamps1, translations1, rotations1 = load_traj(input_filepath1)
    timestamps2, translations2, rotations2 = load_traj(input_filepath2)

    relative_rot_matrix, relative_translation = calculate_transformation(translations1[0], rotations1[0], translations2[0], rotations2[0])

    print("relative_rot_matrix",relative_rot_matrix)

    roll_corr_init, pitch_corr_init, yaw_corr_init = rotation_matrix_to_euler_angles(relative_rot_matrix)
    tx_corr_init, ty_corr_init, tz_corr_init = relative_translation

    print(f"tx_init: {tx_corr_init}, ty_init: {ty_corr_init}, tz_init: {tz_corr_init}")
    print(f"roll_init: {roll_corr_init}, pitch_init: {pitch_corr_init}, yaw_init: {yaw_corr_init}")
