#!/usr/bin/env python3

import os
import numpy as np
import cv2
import glob
import argparse


class StereoCalibration(object):
    def __init__(self, filepath):
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_cal = (cv2.TERM_CRITERIA_EPS +
                             cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((7*5, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints_l = []  # 2d points in image plane.
        self.imgpoints_r = []  # 2d points in image plane.

        self.cal_path = filepath
        self.read_images(self.cal_path)

    def read_images(self, cal_path):

        images_right = glob.glob(os.path.join(cal_path, 'img1', '*.png')) + glob.glob(os.path.join(cal_path, 'img1', '*.PNG'))
        images_left = glob.glob(os.path.join(cal_path, 'img2', '*.png')) + glob.glob(os.path.join(cal_path, 'img2', '*.PNG'))

        images_left.sort()
        images_right.sort()

        for i, fname in enumerate(images_right):
            img_l = cv2.imread(images_left[i])
            img_r = cv2.imread(images_right[i])

            gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)


            # Find the chess board corners
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, (7, 5), None)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, (7, 5), None)

            # Reverse the order of the points
            # corners_r = np.flipud(corners_r)

            # print(corners_r)

            # If found, add object points, image points (after refining them)
            self.objpoints.append(self.objp)

            if ret_l is True:
                rt = cv2.cornerSubPix(gray_l, corners_l, (11, 11),
                                      (-1, -1), self.criteria)
                self.imgpoints_l.append(corners_l)

                # Draw and display the corners
                ret_l = cv2.drawChessboardCorners(img_l, (7, 5),
                                                  corners_l, ret_l)
                cv2.imshow(images_left[i], img_l)
                cv2.waitKey(300)

            if ret_r is True:
                rt = cv2.cornerSubPix(gray_r, corners_r, (11, 11),
                                      (-1, -1), self.criteria)
                corners_r = np.flipud(corners_r)
                self.imgpoints_r.append(corners_r)

                # Draw and display the corners
                ret_r = cv2.drawChessboardCorners(img_r, (7, 5),
                                                  corners_r, ret_r)
                cv2.imshow(images_right[i], img_r)
                cv2.waitKey(300)
            img_shape = gray_l.shape[::-1]


        rt, self.M1, self.d1, self.r1, self.t1 = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_l, img_shape, None, None)
        rt, self.M2, self.d2, self.r2, self.t2 = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_r, img_shape, None, None)
        

        self.camera_model = self.stereo_calibrate(img_shape)

    def stereo_calibrate(self, dims):
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_ASPECT_RATIO
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
        # flags |= cv2.CALIB_RATIONAL_MODEL
        # flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_K3
        # flags |= cv2.CALIB_FIX_K4
        # flags |= cv2.CALIB_FIX_K5

        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                                cv2.TERM_CRITERIA_EPS, 100, 1e-5)

        ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_l,
            self.imgpoints_r, self.M1, self.d1, self.M2,
            self.d2, dims,
            criteria=stereocalib_criteria, flags=flags)

        print('Intrinsic_mtx_1', M1)
        print('dist_1', d1)
        print('Intrinsic_mtx_2', M2)
        print('dist_2', d2)
        print('R', R)
        print('T', T)
        print('E', E)
        print('F', F)

        # for i in range(len(self.r1)):
        #     print("--- pose[", i+1, "] ---")
        #     self.ext1, _ = cv2.Rodrigues(self.r1[i])
        #     self.ext2, _ = cv2.Rodrigues(self.r2[i])
        #     print('Ext1', self.ext1)
        #     print('Ext2', self.ext2)

        print('')

        camera_model = dict([('M1', M1), ('M2', M2), ('dist1', d1),
                            ('dist2', d2), ('rvecs1', self.r1),
                            ('rvecs2', self.r2), ('R', R), ('T', T),
                            ('E', E), ('F', F)])

        cv2.destroyAllWindows()
        return camera_model
    

    # def test_estimate(self):
    #     # Convert rotation vectors to rotation matrices
    #     R1, _ = cv2.Rodrigues(self.r1[1])
    #     R2, _ = cv2.Rodrigues(self.r2[1])

    #     # Compute the relative rotation
    #     R_relative = R2 @ R1.T

    #     # Compute the relative translation
    #     T_relative = self.t2[1] - (R_relative @ self.t1[1])

    #     # Optionally, convert R_relative back to a rotation vector if needed
    #     r_relative, _ = cv2.Rodrigues(R_relative)

    #     print("rot rel to right cam (test):",R_relative)
    #     print("transl rel to right cam (test)",T_relative)
    

    def compute_reprojection_error(self, objpoints, imgpoints, rvecs, tvecs, cameraMatrix, distCoeffs):
        total_error = 0
        total_points = 0
        for i in range(len(objpoints)):
            imgp2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs)
            error = cv2.norm(imgpoints[i], imgp2, cv2.NORM_L2) / len(imgp2)
            total_points += len(objpoints[i])
            total_error += error
        mean_error = total_error / len(objpoints)
        return mean_error, total_points
    
    def validate_extrinsics(self):
        total_error = 0
        total_points = 0
        
        for i in range(len(self.objpoints)):
            # Convert object points from checkerboard coordinate system to the left camera coordinate system
            rvec_left_to_world, _ = cv2.Rodrigues(self.r1[i])
            tvec_left_to_world = self.t1[i]
            obj_points_in_left_cam = [np.dot(rvec_left_to_world, point.reshape(3, 1)) + tvec_left_to_world for point in self.objpoints[i]]
            obj_points_in_left_cam = np.array(obj_points_in_left_cam).squeeze()

            # Apply R and T to transform these points into the right camera's coordinate system
            obj_points_in_right_cam = [np.dot(self.camera_model['R'], point.reshape(3, 1)) + self.camera_model['T'] for point in obj_points_in_left_cam]
        
            obj_points_in_right_cam = np.array(obj_points_in_right_cam).squeeze()

            # Project these 3D points onto the right camera's image plane
            imgpoints_r_proj, _ = cv2.projectPoints(obj_points_in_right_cam, np.zeros(3), np.zeros(3), self.M2, self.d2)

            # Ensure both arrays are of the same type (convert to 32-bit float)
            imgpoints_r = np.array(self.imgpoints_r[i], dtype=np.float32)
            imgpoints_r_proj = np.array(imgpoints_r_proj, dtype=np.float32)

            # Compute the reprojection error
            error_r = cv2.norm(imgpoints_r, imgpoints_r_proj, cv2.NORM_L2) / len(imgpoints_r_proj)
            
            total_error += error_r
            total_points += len(self.objpoints[i])

        mean_error = total_error / len(self.objpoints)
        return mean_error

    def evaluate_calibration(self):
        reproj_error_left, total_points_left = self.compute_reprojection_error(
            self.objpoints, self.imgpoints_l, self.r1, self.t1, self.M1, self.d1)
        reproj_error_right, total_points_right = self.compute_reprojection_error(
            self.objpoints, self.imgpoints_r, self.r2, self.t2, self.M2, self.d2)

        print(f"Left Camera Re-projection Error: {reproj_error_left}")
        print(f"Right Camera Re-projection Error: {reproj_error_right}")

        extrinsic_error = self.validate_extrinsics()

        print(f"Extrinsic estimates error: {extrinsic_error}")

        # print(self.test_estimate())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filepath', help='String Filepath')
    args = parser.parse_args()
    cal_data = StereoCalibration(args.filepath)
    cal_data.evaluate_calibration()
