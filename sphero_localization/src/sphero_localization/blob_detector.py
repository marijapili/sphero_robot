#!/usr/bin/env python3
from collections import deque
from imutils.video import VideoStream, WebcamVideoStream
import numpy as np
import argparse
import cv2
import imutils
import time


class SpheroBlobDetector(object):
    def __init__(self, params: cv2.SimpleBlobDetector_Params=None, hsv_mask=None):
        # Set default params.
        if params is None:
            params = cv2.SimpleBlobDetector_Params()
            # Change thresholds
            params.minThreshold = 200
            params.maxThreshold = 255
            # Filter by Area.
            params.filterByArea = True
            params.minArea = 300
            # Filter by Circularity
            params.filterByCircularity = True
            params.minCircularity = 0.2
            # Filter by Convexity
            params.filterByConvexity = False
            params.minConvexity = 0.5
            # Filter by Inertia
            params.filterByInertia = False
            params.minInertiaRatio = 0.5
            
        if hsv_mask is None:
            self.hsv_mask = (
                (0, 0, 220),
                (255, 255, 255)
            )
            
        # Create a detector.
        self.detector = cv2.SimpleBlobDetector_create(params)
        
        # Visualization options.
        self.label_kwargs = {
            'fontFace': cv2.FONT_HERSHEY_SIMPLEX,
            'fontScale': 0.75,
            'color': (0, 0, 255),
            'thickness': 2,
        }
        
    def detect(self, frame):
        """Process the frame and find blobs matching given criteria."""
        blurred = cv2.GaussianBlur(frame, (3, 3), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_mask[0], self.hsv_mask[1])
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)  # Erode and dilate are used to remove excess small blobs.
        mask = cv2.bitwise_not(mask)

        # Detect blobs.
        keypoints = self.detector.detect(mask)
        
        # Draw detected blobs as red circles and store positions.
        for kpt in keypoints:
            cnt = (int(kpt.pt[0]), int(kpt.pt[1]))
            cv2.circle(frame, cnt, int(kpt.size / 2), (0, 255, 255), 2)
            frame = cv2.putText(frame, f'({cnt[0]}, {cnt[1]})', (cnt[0] + 20, cnt[1]), **self.label_kwargs)
            
        # TODO: When adding new detectors, synchronize the interface for getting keypoint data.
        return keypoints, frame, mask
    

def main():
    detector = SpheroBlobDetector()
    
    cams = [
            '/dev/video4',
            '/dev/video2',
    ]
    calibrations = [
        '/home/marko/WS/sphero_ws/src/sphero/sphero_localization/config/camera_north.json',
        '/home/marko/WS/sphero_ws/src/sphero/sphero_localization/config/camera_south.json'
    ]
    stitch_param = '/home/marko/WS/sphero_ws/src/sphero/sphero_localization/config/stitch_params.json'
    
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    cv2.namedWindow("proc", cv2.WINDOW_NORMAL)
    
    fs = FrameServer(cams, calibrations, stitch_param)

    while True:
        # grab the current frame
        _, frame = fs.grab()

        pts, frame, mask = detector.detect(frame)
        
        cv2.imshow("frame", frame)
        cv2.imshow("proc", mask)
        cv2.resizeWindow('frame', fs.DOUBLE_WINDOW)
        cv2.resizeWindow('proc', fs.DOUBLE_WINDOW)
    
        key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

    # if we are not using a video file, stop the camera video stream
    fs.stop()
    # otherwise, release the camera
    # close all windows
    cv2.destroyAllWindows()


if __name__ == '__main__':
    from sphero_localization.duo_c270 import FrameServer
    main()