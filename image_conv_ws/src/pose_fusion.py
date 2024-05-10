#!/usr/bin/env python3

"""
AUTHOR: SHIVA SURYA LOLLA
DATE: 12TH APRIL 2024

INPUT:  camera 1 trajectory output file
        camera 2 trajectory output file
        
OUTPUT: to_fuse.g2o file containing the vertices and edges in the format desired 
"""

#!/usr/bin/env python3


import numpy as np 
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares


def matching_time_indices(stamps_1, stamps_2,
                          max_diff: float = 0.01):
    """
    Searches for the best matching timestamps of two lists of timestamps
    and returns the list indices of the best matches.
    :param stamps_1: first list of timestamps
    :param stamps_2: second list of timestamps 
    :param max_diff: max. allowed absolute time difference
    :return: 2 lists of the matching timestamp indices (stamps_1, stamps_2)
    """
    snd_longer = len(stamps_2) > len(stamps_1)


    if snd_longer:
        # Convert lists to NumPy arrays for efficient operations
        ts_1 = np.array(stamps_1)
        ts_2 = np.array(stamps_2)
    else:
        ts_1 = np.array(stamps_2)
        ts_2 = np.array(stamps_1)

    matching_indices_1 = []
    matching_indices_2 = []

    for index_1, t_1 in enumerate(ts_1):
        diffs = np.abs(ts_2 - t_1)
        index_2 = np.argmin(diffs)
        if diffs[index_2] <= max_diff:
            matching_indices_1.append(index_1)
            matching_indices_2.append(index_2)

    return matching_indices_1, matching_indices_2


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


def reduce_to_ids(indices,timestamps,translations,rotations):

    red_timestamps = []
    red_translations = []
    red_rotations = []

    for index in indices:
        red_timestamps.append(timestamps[index])
        red_translations.append(translations[index])
        red_rotations.append(rotations[index])
    
    return red_timestamps, red_translations, red_rotations



def save_traj(timestamps,translations,rotations,output_filepath):

    with open(output_filepath, 'w') as output_file:
        for i in range(len(timestamps)):
            timestamp = timestamps[i]
            translation = translations[i]
            rotation = rotations[i]

            # Write to output file
            output_file.write(f"{timestamp} {' '.join(map(str, translation))} {' '.join(map(str, rotation))}\n")


# def calculate_relative_transform(trans1, rot1, trans2, rot2):

#     # Convert quaternions to rotation matrices
#     r1 = R.from_quat(rot1)
#     r2 = R.from_quat(rot2)

#     # Calculate relative rotation
#     relative_rotation = r2 * r1.inv()
#     relative_translation = trans2 - r1.apply(trans1)

#     # Convert the relative rotation back to quaternion
#     relative_quaternion = relative_rotation.as_quat()

#     return relative_translation, relative_quaternion

def calculate_relative_transform(trans1, rot1, trans2, rot2):
    # Convert quaternions to rotation matrices
    r1 = R.from_quat(rot1)
    r2 = R.from_quat(rot2)

    # Calculate relative rotation
    # R_rel = R1^(-1) * R2
    relative_rotation = r1.inv() * r2

    # Calculate relative translation
    # T_rel = R1^(-1) * (T2 - T1)
    relative_translation = r1.inv().apply(trans2 - trans1)

    # Convert the relative rotation back to quaternion
    relative_quaternion = relative_rotation.as_quat()

    return relative_translation, relative_quaternion


def save_to_g2o(vertices, edges, output_filepath):
    with open(output_filepath, 'w') as file:
        # Write vertices
        for i, (timestamp, translation, rotation) in enumerate(vertices):
            tx, ty, tz = translation
            qx, qy, qz, qw = rotation
            file.write(f"VERTEX_SE3:QUAT {i} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")

        # Write edges
        for (i, j), (translation, rotation) in edges.items():
            tx, ty, tz = translation
            qx, qy, qz, qw = rotation
            file.write(f"EDGE_SE3:QUAT {i} {j} {tx} {ty} {tz} {qx} {qy} {qz} {qw} ")
            # file.write("350 0 0 0 0 0 350 0 0 0 0 350 0 0 0 720 0 0 720 0 720\n")
            file.write("50 0 0 0 0 0 50 0 0 0 0 50 0 0 0 100 0 0 100 0 100\n")




if __name__ == '__main__':
    input_filepath1 = '/home/ssuryalolla/fusionlab_files/data/pose_fusion/fusionbag13_data_cam1.klg.freiburg'  
    input_filepath2 = '/home/ssuryalolla/fusionlab_files/data/pose_fusion/fusionbag13_data_cam2.klg.freiburg'  
    output_filepath1 = '/home/ssuryalolla/fusionlab_files/data/pose_fusion/synced_fusionbag13_data_cam1.txt'  
    output_filepath2 = '/home/ssuryalolla/fusionlab_files/data/pose_fusion/synced_fusionbag13_data_cam2.txt'  

    timestamps1, translations1, rotations1 = load_traj(input_filepath1)
    timestamps2, translations2, rotations2 = load_traj(input_filepath2)

    snd_longer = len(timestamps2) > len(timestamps1)

    matching_indices_short, matching_indices_long = matching_time_indices(timestamps1,timestamps2)

    if snd_longer:
        red_timestamps1, red_translations1, red_rotations1 = reduce_to_ids(matching_indices_short,timestamps1,translations1,rotations1)
        red_timestamps2, red_translations2, red_rotations2 = reduce_to_ids(matching_indices_long,timestamps2,translations2,rotations2)
    else:
        red_timestamps1, red_translations1, red_rotations1 = reduce_to_ids(matching_indices_long,timestamps1,translations1,rotations1)
        red_timestamps2, red_translations2, red_rotations2 = reduce_to_ids(matching_indices_short,timestamps2,translations2,rotations2)


    save_traj(red_timestamps1,red_translations1,red_rotations1,output_filepath1)
    save_traj(red_timestamps2,red_translations2,red_rotations2,output_filepath2)

    # #####HARD CODE EXTRINSICS###### (not reqd in current code)
    # #Rotation from camera 2 to camera 1 
    # rot21 = np.array([[0.99809, 0.030300, -0.053754],
    #           [-0.025897, 0.996393, 0.080798],
    #           [0.056009, -0.079252, 0.995279]])
    
    # #translation from camera 2 to camera 1
    # t21 = np.array([[-148.514328],
    #           [ -24.632517],
    #           [  -4.469864]])
    
    vertices = list(zip(red_timestamps1, red_translations1, red_rotations1))
    edges = {}

    # Assuming that each cam2 pose provides the transformation for edges
    for i in range(len(vertices) - 1):
        trans1, rot1 = red_translations2[i], red_rotations2[i]
        trans2, rot2 = red_translations2[i + 1], red_rotations2[i + 1]
        relative_translation, relative_quaternion = calculate_relative_transform(trans1, rot1, trans2, rot2)
        edges[(i, i + 1)] = (relative_translation, relative_quaternion)

    # Save to .g2o file
    output_g2o_file = '/home/ssuryalolla/fusionlab_files/data/pose_fusion/to_fuse_new6.g2o'
    save_to_g2o(vertices, edges, output_g2o_file)


    


