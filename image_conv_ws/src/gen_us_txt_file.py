#!/usr/bin/env python3
import os


"""LATEST CODE"""
def generate_txt_file(image_folder, output_file, timestamp_file):
    # Read timestamps from the timestamp file
    with open(timestamp_file, 'r') as ts_file:
        timestamps = ts_file.readlines()

    # List all image files in the folder
    image_files = [f for f in os.listdir(image_folder) if os.path.isfile(os.path.join(image_folder, f))]
    
    # Sort the files by filename (assuming this matches the order of timestamps)
    image_files.sort()
    
    # Open the output file for writing
    with open(output_file, 'w') as file:
        # Iterate over sorted image files and write the lines to the output file
        for i, filename in enumerate(image_files):
            # Assuming there's one timestamp per line in the same order as the sorted filenames
            timestamp = timestamps[i].strip()  # Remove newline characters and any surrounding whitespace
            line = f"{timestamp} {os.path.join(image_folder, filename)}\n"
            file.write(line)
            
# rgb_folder_path = '/home/ssuryalolla/fusionlab_files/data/bag1/rgb'
# depth_folder_path = '/home/ssuryalolla/fusionlab_files/data/bag1/depth'  # Relative path from the script to the depth folder
# im_ts_folder_path = '/home/ssuryalolla/fusionlab_files/data/bag1/img_timestamps.txt'
# rgb_output_file = '/home/ssuryalolla/fusionlab_files/data/bag1/rgb.txt'  # Path where the rgb.txt file will be saved
# depth_output_file = '/home/ssuryalolla/fusionlab_files/data/bag1/depth.txt'  # Path where the depth.txt file will be saved

            
us_folder_path = '/home/ssuryalolla/fusionlab_files/data/bag15/us'
us_im_ts_folder_path = '/home/ssuryalolla/fusionlab_files/data/bag15/us_img_timestamps.txt'
us_output_file = '/home/ssuryalolla/fusionlab_files/data/bag15/us.txt'  # Path where the rgb.txt file will be saved

# Generate the txt files
generate_txt_file(us_folder_path, us_output_file, us_im_ts_folder_path)