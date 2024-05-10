#!/usr/bin/env python3
import os

"""OLD CODE"""
# def generate_txt_file(image_folder, output_file, im_ts_folder_path):
#     # List all image files in the folder
#     image_files = [f for f in os.listdir(image_folder) if os.path.isfile(os.path.join(image_folder, f))]
    
#     # Sort the files by filename
#     image_files.sort()
    
#     # Open the output file for writing
#     with open(output_file, 'w') as file:
#         # Iterate over sorted image files and write the lines to the output file
#         for i, filename in enumerate(image_files):
#             # Use the index as the sequence number
#             line = f"{i} .{image_folder}/{filename}\n"
#             file.write(line)

# Paths setup (adjust these paths according to your setup)
# rgb_folder_path = '/bagfiles/rgb'  # Relative path from the script to the rgb folder
# depth_folder_path = '/bagfiles/depth'  # Relative path from the script to the depth folder
# rgb_output_file = '/bagfiles/rgb.txt'  # Path where the rgb.txt file will be saved
# depth_output_file = '/bagfiles/depth.txt'  # Path where the depth.txt file will be saved
# rgb_folder_path = '/home/ssuryalolla/bagfiles/rgb'
# depth_folder_path = '/home/ssuryalolla/bagfiles/depth'  # Relative path from the script to the depth folder
# rgb_output_file = '/home/ssuryalolla/bagfiles/rgb.txt'  # Path where the rgb.txt file will be saved
# depth_output_file = '/home/ssuryalolla/bagfiles/depth.txt'  # Path where the depth.txt file will be saved


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
            

rgb_folder_path = '/home/ssuryalolla/fusionlab_files/data/bag15/rgb'
depth_folder_path = '/home/ssuryalolla/fusionlab_files/data/bag15/depth'  # Relative path from the script to the depth folder
im_ts_folder_path = '/home/ssuryalolla/fusionlab_files/data/bag15/img_timestamps.txt'
rgb_output_file = '/home/ssuryalolla/fusionlab_files/data/bag15/rgb.txt'  # Path where the rgb.txt file will be saved
depth_output_file = '/home/ssuryalolla/fusionlab_files/data/bag15/depth.txt'  # Path where the depth.txt file will be saved

# Generate the txt files
generate_txt_file(rgb_folder_path, rgb_output_file, im_ts_folder_path)
generate_txt_file(depth_folder_path, depth_output_file, im_ts_folder_path)
