#!/usr/bin/env python3


import numpy as np 
from scipy.spatial.transform import Rotation as R


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


# def align_trajectories(ground_truth_positions, estimated_positions):
#     """
#     Aligns estimated_positions to ground_truth_positions using the Umeyama algorithm.
#     Returns the aligned positions.
#     """
#     # Center the points around the origin
#     gt_centroid = np.mean(ground_truth_positions, axis=0)
#     est_centroid = np.mean(estimated_positions, axis=0)
#     gt_centered = ground_truth_positions - gt_centroid
#     est_centered = estimated_positions - est_centroid
    
#     # Compute the rotation that best aligns the estimated positions to the ground truth positions
#     # rotation, _ = R.align_vectors(est_centered, gt_centered)

#     #rotation from est_centered to gt_centered

#     rotation = np.array([
#     [-0.23417632,  0.97193833,  0.02230105],
#     [ 0.96844018,  0.2352246,  -0.08241967],
#     [-0.08535259,  0.0022965,  -0.99634816]
#     ])

#     transl = np.array([ 0.52378054,-0.08605232,0.24893821]) #translation from est_centered to gt_centered

#     scale = 0.497 #scale from est_centered to gt_centered


#     # Apply transformations
#     # Step 1: Scale the centered estimated positions
#     # est_scaled = est_centered * scale
#     est_scaled = est_centered 
    
#     # Step 2: Rotate the scaled estimated positions
#     est_rotated = np.dot(est_scaled, rotation.T)  # Note the transpose of the rotation matrix for correct multiplication
    
#     # Step 3: Translate the rotated and scaled estimated positions
#     aligned_est_positions = est_rotated + gt_centroid
    
#     return aligned_est_positions


def align_trajectories(estimated_positions):
    """
    Aligns estimated_positions to ground_truth_positions using a specified rotation and translation.
    Ignores scaling and centroid alignment for simplicity.
    Returns the aligned estimated positions.
    """

    # # Provided rotation and translation matrices
    # rotation = np.array([
    # [0.18514434, 0.982403, 0.02461541],
    # [0.97543672, -0.18675829, 0.11680987],
    # [0.11935149, 0.00238409, -0.9928492]
    # ])

    # translation = np.array([0.60169998, -0.33358247, 0.46644306]) #translation from est_centered to gt_centered

    #bag7 removed rot, transl and scale 
    # rotation = np.array([
    # [0.290927, 0.95584489, -0.04149724],
    # [0.95645733, -0.29162976, -0.01189375],
    # [-0.02347041, -0.03623013, -0.99906782]
    # ])

    # translation = np.array([0.47088084, -0.1420116, 0.24256002])

    # scale = 0.50879 

    # rotation = np.array([
    # [-0.06822319, 0.99753727, 0.0162785],
    # [0.98910499, 0.06976193, -0.12963254],
    # [-0.13044891, 0.0072572, -0.99142847]
    # ])

    # translation = np.array([0.59082631, -0.2470768, 0.44865837])
    # scale = 1


    # # #bag7 original rot, transl and scale 
    rotation = np.array([
    [0.28380704, 0.95860632, -0.02296722],
    [0.9584805, -0.28429955, -0.02211096],
    [-0.02772527, -0.01573839, -0.99949168]
    ])

    translation = np.array([0.47068661, -0.14101306, 0.24417201])

    scale = 0.49686

    # print(estimated_positions.shape)
    
    # Apply rotation
    rotated_est_positions = scale * np.dot(estimated_positions,rotation.T)  # Transpose rotation matrix for correct multiplication
    
    # Apply translation
    aligned_est_positions = rotated_est_positions + translation
    
    return aligned_est_positions

# def compute_direc_diff(ground_truth_positions, estimated_positions):

#     # Ensure the inputs are NumPy arrays
#     ground_truth_positions = np.array(ground_truth_positions)
#     estimated_positions = np.array(estimated_positions)

#     # Compute differences as a NumPy array
#     differences = estimated_positions - ground_truth_positions

#     # Separate the differences by axis
#     x_diff, y_diff, z_diff = differences.T  # Transpose and unpack for clarity

#     # Analyze the directionality of errors using NumPy for mean calculation
#     print("Mean error in x:", np.mean(x_diff))
#     print("Mean error in y:", np.mean(y_diff))
#     print("Mean error in z:", np.mean(z_diff))


# Modify your compute_direc_diff function to call align_trajectories
def compute_direc_diff(ground_truth_positions, estimated_positions):

    # Ensure the inputs are NumPy arrays
    ground_truth_positions = np.array(ground_truth_positions)
    estimated_positions = np.array(estimated_positions)

    # Align the estimated positions to the ground truth positions
    aligned_est_positions = align_trajectories(estimated_positions)
    
    # Compute differences as a NumPy array
    differences = aligned_est_positions - ground_truth_positions
    
    # Separate the differences by axis
    x_diff, y_diff, z_diff = differences.T  # Transpose and unpack for clarity
    
    
    # Analyze the directionality of errors using NumPy for mean calculation
    print("Mean error in x:", np.mean(x_diff))
    print("Mean error in y:", np.mean(y_diff))
    print("Mean error in z:", np.mean(z_diff))


    # Calculate the Euclidean norm of the differences to get the overall error
    overall_error = np.linalg.norm(differences, axis=1)

    print("Mean overall error for bag 6:", np.mean(overall_error))
    print("Standard deviation of overall error:", np.std(overall_error))



if __name__ == '__main__':
    input_filepath1 = '/home/ssuryalolla/fusionlab_files/data/bag12/groundtruth_new.txt'  # Adjust this path
    input_filepath2 = '/home/ssuryalolla/fusionlab_files/data/bag12/fusionbag12_data.klg.freiburg'  # Adjust this path
    output_filepath1 = '/home/ssuryalolla/fusionlab_files/data/bag12/synced_groundtruth.txt'  # Adjust this path
    output_filepath2 = '/home/ssuryalolla/fusionlab_files/data/bag12/synced_fusionbag12_data.txt'  # Adjust this path

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

    # compute_direc_diff(red_translations1,red_translations2)
