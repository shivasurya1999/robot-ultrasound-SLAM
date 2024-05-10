#!/usr/bin/env python3

import numpy as np
import cv2
import glob
import os


# Camera parameters to undistort and rectify images
cv_file = cv2.FileStorage()
cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()


# # Open both cameras
# cap_right = cv2.VideoCapture(2, cv2.CAP_DSHOW)                    
# cap_left =  cv2.VideoCapture(0, cv2.CAP_DSHOW)

cal_path = '/home/ssuryalolla/fusionlab_files/ext_calib_imgs'
imagesLeft = sorted(glob.glob(os.path.join(cal_path, 'img2', '*.png')) + glob.glob(os.path.join(cal_path, 'img2', '*.PNG')))
imagesRight = sorted(glob.glob(os.path.join(cal_path, 'img1', '*.png')) + glob.glob(os.path.join(cal_path, 'img1', '*.PNG')))


for imgLeft, imgRight in zip(imagesLeft, imagesRight):

    frame_right = cv2.imread(imgRight)
    frame_left = cv2.imread(imgLeft)

    # Undistort and rectify images
    frame_right = cv2.remap(frame_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    frame_left = cv2.remap(frame_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
                     
    # Show the frames
    cv2.imshow("frame right", frame_right) 
    cv2.imshow("frame left", frame_left)


    # Hit "q" to close the window
    if cv2.waitKey(5000 ) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
