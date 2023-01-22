"""
This script takes the saved images containing a calibration pattern, 
calculates the optimal camera parameters, and saves them to a json file.
"""
import numpy as np
import cv2 as cv
import glob
import json
import argparse
import os 
from pathlib import Path

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', help='name of output json, eg. camera.json', default='camera.json')
    args = parser.parse_args()

    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    rows = 5 - 1
    columns = 7 - 1
    square_size_meters = 40 * 0.001

    objp = np.zeros((rows*columns,3), np.float32)
    objp[:,:2] = np.mgrid[0:columns,0:rows].T.reshape(-1,2) * square_size_meters

    objpoints = []
    imgpoints = []

    images = glob.glob(f"{os.path.dirname(os.path.realpath(__file__))}/pictures/raw/calibrate*.png")
    print(len(images), "images found")

    ## Make necessary folder structure.
    save_path = Path(__file__).parent / 'pictures/detected'
    save_path.mkdir(parents=True, exist_ok=True)

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        chessboard_flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv.findChessboardCorners(gray, (columns,rows), chessboard_flags)

        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)

            cv.drawChessboardCorners(img, (columns,rows), corners2, ret)
            cv.imshow('img', img)
            fname = fname.split('/')[-1]
            cv.imwrite(f"{os.path.dirname(os.path.realpath(__file__))}/pictures/detected/corners_{fname.partition('_')[2]}", img)
            cv.waitKey(1500)

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print(ret)

    camera = {}

    class NumpyEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            return json.JSONEncoder.default(self, obj)

    for variable in ['ret', 'mtx', 'dist', 'rvecs', 'tvecs']:
        camera[variable] = eval(variable)

    with open(f"{os.path.dirname(os.path.realpath(__file__))}/../../config/{args.output}", 'w') as f:
        json.dump(camera, f, indent=4, cls=NumpyEncoder)

    cv.destroyAllWindows()