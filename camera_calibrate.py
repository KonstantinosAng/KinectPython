"""
Author: Konstantinos Angelopoulos
Date: 04/02/2020
All rights reserved.
Feel free to use and modify and if you like it give it a star.

Source Code:
    https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
 
 Code to calibrate camera
 Path to find the images for calibration
 Example: path = 'calibrate/*.jpg'
"""


def calibrate(path):
    # Import libraries only for this function to minimize memory usage
    import cv2
    import numpy as np
    import glob
    import json
    import random
    import os

    # Load Images
    images = glob.glob(path)

    # Path to store undistorted image
    calibpath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'calibrate/Color/calibresult.jpg')
    configpath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'calibrate/Color/config.json')
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    size = (7, 5)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    points = np.zeros((np.prod([size]), 3), np.float32)
    points[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find chess corners
        cal_ret, corners = cv2.findChessboardCorners(gray, size, None)
        # If found, add object points, image points ( after refining them)
        if cal_ret:
            objpoints.append(points)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            print('[CALIBRATE CAMERA]: {}'.format(fname))
            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, size, corners2, cal_ret)
            # res_img = cv2.resize(img, (960, 540))
            cv2.imshow('Chess Corners', img)
            cv2.waitKey(1000)

    cv2.destroyAllWindows()

    # Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Undistort images
    img = cv2.imread(images[random.randint(0, len(images))])
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # Crop Image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite(calibpath, dst)
    undistorted = cv2.imread(calibpath)
    # res_undistorted = cv2.resize(undistorted, (960, 540))
    cv2.imshow(' Undistorted ', undistorted)
    cv2.waitKey(4000)
    cv2.destroyAllWindows()

    # Re-projection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    print("[CALIBRATE CAMERA]: total error: {}".format(mean_error / len(objpoints)))  # Must be as low as possible, less than 0

    # Create .json file with calibration settings to use as input at fabric_dimensions
    # It should contain the camera matrix, distortion, rectification, projection
    with open(configpath, 'w', encoding='utf-8') as json_file:
        configs = {"Calibration Size": size, "Old Camera Matrix": mtx.tolist(), "Optimal Camera Matrix": newcameramtx.tolist(), "Distortion Parameters": dist.tolist(),
                   "Crop Coordinates": roi, "Total Projection Error": mean_error/len(objpoints)}
        json.dump(configs, json_file, separators=(',', ':'), sort_keys=True, indent=4)

    return cal_ret
