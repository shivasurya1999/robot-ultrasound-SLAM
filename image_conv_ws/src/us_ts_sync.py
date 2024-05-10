#!/usr/bin/env python3


import numpy as np 
from scipy.spatial.transform import Rotation as R
import shutil
import os


# def matching_time_indices(stamps_1, stamps_2,
#                           max_diff: float = 0.01):
#     """
#     Searches for the best matching timestamps of two lists of timestamps
#     and returns the list indices of the best matches.
#     :param stamps_1: first list of timestamps
#     :param stamps_2: second list of timestamps 
#     :param max_diff: max. allowed absolute time difference
#     :return: 2 lists of the matching timestamp indices (stamps_1, stamps_2)
#     """
#     snd_longer = len(stamps_2) > len(stamps_1)


#     if snd_longer:
#         # Convert lists to NumPy arrays for efficient operations
#         ts_1 = np.array(stamps_1)
#         ts_2 = np.array(stamps_2)
#     else:
#         ts_1 = np.array(stamps_2)
#         ts_2 = np.array(stamps_1)

#     matching_indices_1 = []
#     matching_indices_2 = []

#     for index_1, t_1 in enumerate(ts_1):
#         diffs = np.abs(ts_2 - t_1)
#         index_2 = np.argmin(diffs)
#         if diffs[index_2] <= max_diff:
#             matching_indices_1.append(index_1)
#             matching_indices_2.append(index_2)

#     return matching_indices_1, matching_indices_2

def matching_time_indices(stamps_1, stamps_2, max_diff: float = 0.05):
    ts_1 = np.array(stamps_1)
    ts_2 = np.array(stamps_2)

    matching_indices_1 = []
    matching_indices_2 = []

    used_indices = set()  # To keep track of indices in ts_2 that are already matched

    for index_1, t_1 in enumerate(ts_1):
        diffs = np.abs(ts_2 - t_1)
        index_2 = np.argmin(diffs)
        if diffs[index_2] <= max_diff and index_2 not in used_indices:
            matching_indices_1.append(index_1)
            matching_indices_2.append(index_2)
            used_indices.add(index_2)  # Mark this index as used

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



def load_us_ts(us_ts_filepath):
    try:
        with open(us_ts_filepath) as f1:
            data = [line.strip().split() for line in f1.readlines()]
        us_timestamps = [float(line[0]) for line in data]
        us_img_path = [line[1] for line in data]
                
    except FileNotFoundError as e:
        print(f"Error: {e}")
    
    return us_timestamps, us_img_path



def reduce_to_ids(indices,timestamps,translations,rotations):

    red_timestamps = []
    red_translations = []
    red_rotations = []

    for index in indices:
        red_timestamps.append(timestamps[index])
        red_translations.append(translations[index])
        red_rotations.append(rotations[index])
    
    return red_timestamps, red_translations, red_rotations


def reduce_us_ids(indices,timestamps,img_paths):

    red_timestamps = []
    red_paths = []

    for index in indices:
        red_timestamps.append(timestamps[index])
        red_paths.append(img_paths[index])
    
    return red_timestamps, red_paths


def save_traj(timestamps,translations,rotations,output_filepath):

    with open(output_filepath, 'w') as output_file:
        for i in range(len(timestamps)):
            timestamp = timestamps[i]
            translation = translations[i]
            rotation = rotations[i]

            print("lines-1 saved= ",i)

            # Write to output file
            output_file.write(f"{timestamp} {' '.join(map(str, translation))} {' '.join(map(str, rotation))}\n")


def save_us_paths(timestamps,us_img_paths,output_filepath):
    with open(output_filepath, 'w') as output_file:
        for i in range(len(timestamps)):
            timestamp = timestamps[i]
            us_img_path = us_img_paths[i]

            # Write to output file
            output_file.write(f"{timestamp } {us_img_path}\n")


def copy_selected_images(image_paths, source_dir, dest_dir):
    """
    Copies specified images from a source directory to a destination directory.
    :param image_paths: List of full paths to the images that need to be copied.
    :param source_dir: Directory where the original images are stored.
    :param dest_dir: Destination directory to copy the images to.
    """
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)  # Create the destination directory if it does not exist

    for img_path in image_paths:
        full_source_path = os.path.join(source_dir, img_path)  # Adjust if necessary
        full_dest_path = os.path.join(dest_dir, os.path.basename(img_path))
        shutil.copy(full_source_path, full_dest_path)  # Copy the file
        # print(f"Copied {full_source_path} to {full_dest_path}")  # Optional: for verification


if __name__ == '__main__':
    traj_filepath = '/home/ssuryalolla/fusionlab_files/data/bag15/groundtruth_new.txt'  # Adjust this path
    us_filepath = '/home/ssuryalolla/fusionlab_files/data/bag15/us'  # Adjust this path
    us_timestamp_path = '/home/ssuryalolla/fusionlab_files/data/bag15/us.txt'  # Adjust this path
    output_filepath1 = '/home/ssuryalolla/fusionlab_files/data/bag15/synced_gt.txt'  # Adjust this path
    output_filepath2 = '/home/ssuryalolla/fusionlab_files/data/bag15/synced_us_with_gt.txt'  # Adjust this path

    timestamps1, translations1, rotations1 = load_traj(traj_filepath)
    timestamps2, us_img_paths = load_us_ts(us_timestamp_path)

    snd_longer = len(timestamps2) > len(timestamps1)

    matching_indices_short, matching_indices_long = matching_time_indices(timestamps1,timestamps2)

    if snd_longer:
        red_timestamps1, red_translations1, red_rotations1 = reduce_to_ids(matching_indices_short,timestamps1,translations1,rotations1)
        red_timestamps2, red_us_img_paths = reduce_us_ids(matching_indices_long,timestamps2,us_img_paths)
    else:
        red_timestamps1, red_translations1, red_rotations1 = reduce_to_ids(matching_indices_long,timestamps1,translations1,rotations1)
        red_timestamps2, red_us_img_paths = reduce_us_ids(matching_indices_short,timestamps2,us_img_paths)

    
    print(len(red_timestamps1),len(red_timestamps2))


    save_traj(red_timestamps1,red_translations1,red_rotations1,output_filepath1)
    save_us_paths(red_timestamps2,red_us_img_paths,output_filepath2)

    # Copy images after everything is processed and saved
    output_us_image_dir = '/home/ssuryalolla/fusionlab_files/data/bag15/us_imsync_withgt'  # Adjust this path

    copy_selected_images(red_us_img_paths, us_filepath, output_us_image_dir)