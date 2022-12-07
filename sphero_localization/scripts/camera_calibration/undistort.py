"""
This script can be used to validate the camera calibration results. It reads the
saved calibration images from files, applies the calibrated camera model, and
saves the new undistored images to disk.
"""
import numpy as np
import cv2 as cv
import glob
import json
from datetime import datetime


with open('camera.json', 'r') as json_file:
    camera_data = json.load(json_file)
dist = np.array(camera_data["dist"])
mtx = np.array(camera_data["mtx"])

images = glob.glob('pictures/raw/calibrate*.png')
print(len(images), "images found")

assert len(images) > 0

frame = cv.imread(images[0])
h, w = frame.shape[:2]

newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (h, w), 0, (h, w))
x, y, w1, h1 = roi
yh1 = y + h1
xw1 = x + w1

for fname in images:
    img = cv.imread(fname)

    dst = cv.undistort(img, mtx, dist, None, newcameramtx)
    # dst = dst[y:yh1, x:xw1]

    cv.imshow('img', dst)
    cv.imwrite(f"pictures/fixed/remapped_{fname.partition('_')[2]}", dst)
    # cv.waitKey(1000)