#!/usr/bin/env python3


from PIL import Image
import os

# Define the directory containing images
image_dir = '/home/ssuryalolla/fusionlab_files/data/bag15/us_imsync_withgt'
# Define the output directory for cropped images
output_dir = '/home/ssuryalolla/fusionlab_files/data/bag15/us_gt_cropped_images'

# Create the output directory if it does not exist
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Define the size of the crop
crop_width = 400
crop_height = 400
# Define the starting point for the crop
start_x = 1150  # Starting X coordinate for the crop
start_y = 250  # Starting Y coordinate for the crop

# Process each image in the directory
for filename in os.listdir(image_dir):
    if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')):
        full_path = os.path.join(image_dir, filename)
        with Image.open(full_path) as img:
            # Calculate the crop coordinates
            left = start_x
            top = start_y
            right = start_x + crop_width
            bottom = start_y + crop_height

            # Check if the crop area is within the image dimensions
            if right <= img.width and bottom <= img.height:
                # Perform the crop
                cropped_img = img.crop((left, top, right, bottom))
                # Define the path for the cropped image
                cropped_path = os.path.join(output_dir, filename)
                # Save the cropped image to the new directory
                cropped_img.save(cropped_path)
            else:
                print(f"Skipping {filename}, crop area is out of image bounds.")

print("Cropping completed.")
