#!/usr/bin/env python3

import os
from PIL import Image

# Paths to the folders
source_folder_path = '/home/ssuryalolla/fusionlab_files/ext_calib_imgs/img1'
destination_folder_path = '/home/ssuryalolla/fusionlab_files/ext_calib_imgs/rectified_img'

# Create the destination folder if it doesn't exist
if not os.path.exists(destination_folder_path):
    os.makedirs(destination_folder_path)

# Process each image in the source folder
for image_name in os.listdir(source_folder_path):
    if image_name.lower().endswith(('.png', '.jpg', '.jpeg')):
        # Construct full file path
        image_path = os.path.join(source_folder_path, image_name)
        # Load the image
        original_image = Image.open(image_path)
        # Rotate 180 degrees
        final_image = original_image.rotate(180)
        
        # # Determine whether to flip horizontally or vertically
        # if rotated_image.width < rotated_image.height:
        #     # If the image is taller than it is wide, flip horizontally
        #     final_image = rotated_image.transpose(Image.FLIP_LEFT_RIGHT)
        # else:
        #     # If the image is wider than it is tall, flip vertically
        #     final_image = rotated_image.transpose(Image.FLIP_TOP_BOTTOM)

        # Save the processed image to the destination folder
        final_image.save(os.path.join(destination_folder_path, image_name))
