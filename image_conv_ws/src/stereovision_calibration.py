#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import glob
import os


# Function to concatenate images horizontally and draw horizontal lines
def combine_images_with_lines(imgL, imgR, line_color=(0, 0, 255), line_step=25):
    # Combine images horizontally
    combined_img = np.hstack((imgL, imgR))
    
    # Draw horizontal red lines across the combined image
    for y in range(0, combined_img.shape[0], line_step):
        cv.line(combined_img, (0, y), (combined_img.shape[1], y), line_color, 1)
        
    return combined_img

################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (7,5)
frameSize = (1280,720)


# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 29
objp = objp * size_of_chessboard_squares_mm

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.

cal_path = '/home/ssuryalolla/fusionlab_files/ext_calib_imgs/charuco_checker_imgs'
imagesLeft = sorted(glob.glob(os.path.join(cal_path, 'checker_img2', '*.png')) + glob.glob(os.path.join(cal_path, 'checker_img2', '*.PNG')))
imagesRight = sorted(glob.glob(os.path.join(cal_path, 'checker_img1', '*.png')) + glob.glob(os.path.join(cal_path, 'checker_img1', '*.PNG')))


for imgLeft, imgRight in zip(imagesLeft, imagesRight):

    imgL = cv.imread(imgLeft)
    imgR = cv.imread(imgRight)
    grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    # cv.imshow('img left',grayL)
    # cv.waitKey(500)

    # Find the chess board corners
    retL, cornersL = cv.findChessboardCorners(grayL, chessboardSize, None)
    retR, cornersR = cv.findChessboardCorners(grayR, chessboardSize, None)


    # If found, add object points, image points (after refining them)
    if retL and retR == True:

        objpoints.append(objp)

        cornersL = cv.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)

        cornersR = cv.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        # cornersR = np.flipud(cornersR)
        imgpointsR.append(cornersR)

        # Draw and display the corners
        cv.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
        cv.imshow('img left', imgL)
        cv.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)
        cv.imshow('img right', imgR)
        cv.waitKey(100)


cv.destroyAllWindows()




############## CALIBRATION #######################################################

retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(objpoints, imgpointsL, frameSize, None, None)
heightL, widthL, channelsL = imgL.shape
newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))

retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(objpoints, imgpointsR, frameSize, None, None)
heightR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))



########## Stereo Vision Calibration #############################################

flags = 0
flags |= cv.CALIB_FIX_INTRINSIC
# Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
# Hence intrinsic parameters are the same 

criteria_stereo= (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
retStereo, cameraMatrixL, distL, cameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(objpoints, imgpointsL, imgpointsR, cameraMatrixL, distL, cameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)

print('R',rot)
print('T',trans)


######################CALIBRATION QUALITY CHECK
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2L, _ = cv.projectPoints(objpoints[i], rvecsL[i], tvecsL[i], cameraMatrixL, distL)
    errorL = cv.norm(imgpointsL[i], imgpoints2L, cv.NORM_L2) / len(imgpoints2L)
    
    imgpoints2R, _ = cv.projectPoints(objpoints[i], rvecsR[i], tvecsR[i], cameraMatrixR, distR)
    errorR = cv.norm(imgpointsR[i], imgpoints2R, cv.NORM_L2) / len(imgpoints2R)
    
    mean_error += (errorL + errorR) / 2.0

print(f"Total error: {mean_error / len(objpoints)}")

# Saving the parameters
cv_file = cv.FileStorage('calib_params.xml', cv.FILE_STORAGE_WRITE)
cv_file.write('cameraMatrixL', cameraMatrixL)
cv_file.write('distL', distL)
cv_file.write('cameraMatrixR', cameraMatrixR)
cv_file.write('distR', distR)
cv_file.write('R', rot)
cv_file.write('T', trans)
cv_file.release()

# Loading the saved parameters
cv_file = cv.FileStorage('calib_params.xml', cv.FILE_STORAGE_READ)
cameraMatrixL = cv_file.getNode('cameraMatrixL').mat()
distL = cv_file.getNode('distL').mat()
cameraMatrixR = cv_file.getNode('cameraMatrixR').mat()
distR = cv_file.getNode('distR').mat()
rot = cv_file.getNode('R').mat()
trans = cv_file.getNode('T').mat()
cv_file.release()

# Stereo Rectification (continued from your code)
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv.stereoRectify(cameraMatrixL, distL, cameraMatrixR, distR, grayL.shape[::-1], rot, trans)

leftMapX, leftMapY = cv.initUndistortRectifyMap(cameraMatrixL, distL, R1, P1, grayL.shape[::-1], cv.CV_32FC1)
rightMapX, rightMapY = cv.initUndistortRectifyMap(cameraMatrixR, distR, R2, P2, grayR.shape[::-1], cv.CV_32FC1)

# Example of how to use the rectification and disparity maps
for imgLeft, imgRight in zip(imagesLeft, imagesRight):
    imgL = cv.imread(imgLeft, 0)
    imgR = cv.imread(imgRight, 0)
    
    # Rectify the images
    imgL_rectified = cv.remap(imgL, leftMapX, leftMapY, cv.INTER_LANCZOS4, cv.BORDER_CONSTANT, 0)
    imgR_rectified = cv.remap(imgR, rightMapX, rightMapY, cv.INTER_LANCZOS4, cv.BORDER_CONSTANT, 0)
    
    # # Display the rectified images
    # cv.imshow('Left Rectified', imgL_rectified)
    # cv.imshow('Right Rectified', imgR_rectified)
    # cv.waitKey(50)

    # Combine the images and draw lines
    combined_with_lines = combine_images_with_lines(imgL_rectified, imgR_rectified)

    # Display the combined image
    cv.imshow('Rectified Images with Horizontal Lines', combined_with_lines)
    cv.waitKey(10000)


    cv.destroyAllWindows()

    # Disparity map calculation (optional)
    stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
    disparity = stereo.compute(imgL_rectified, imgR_rectified)
    norm_disparity = cv.normalize(disparity, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)

    cv.imshow('Disparity Map', norm_disparity)
    cv.waitKey(10000)
    cv.destroyAllWindows()


# ########## Stereo Rectification #################################################

# rectifyScale= 1
# rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv.stereoRectify(cameraMatrixL, distL, cameraMatrixR, distR, grayL.shape[::-1], rot, trans, rectifyScale,(0,0))

# stereoMapL = cv.initUndistortRectifyMap(cameraMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv.CV_16SC2)
# stereoMapR = cv.initUndistortRectifyMap(cameraMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv.CV_16SC2)

# print("Saving parameters!")
# cv_file = cv.FileStorage('stereoMap.xml', cv.FILE_STORAGE_WRITE)

# cv_file.write('stereoMapL_x',stereoMapL[0])
# cv_file.write('stereoMapL_y',stereoMapL[1])
# cv_file.write('stereoMapR_x',stereoMapR[0])
# cv_file.write('stereoMapR_y',stereoMapR[1])

# cv_file.release()


