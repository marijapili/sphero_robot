#!/usr/bin/env python3
from collections import deque
from imutils.video import VideoStream, WebcamVideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import yaml

from sphero_localization.duo_c270 import FrameServer
from sphero_localization.sort import Sort
from sphero_localization.blob_detector import SpheroBlobDetector


def np_to_points(np_array):
    x1, y1, x2, y2, ID = np_array
    # TODO: Named tuple
    center = ((x1 + x2) / 2, (y1 + y2) / 2)
    size = (abs(x1 - x2) + abs(y1 - y2)) / 2  # average of width and height
    size /= 2  # radius
    return center, size, ID

class WebcamTrackerTest(object):
    def __init__(self) -> None:
        # Load config parameters.
        config_path = '/home/marko/WS/sphero_ws/src/sphero/sphero_localization/config/config.yaml'
        with open(config_path, 'r') as config_file:
            config = yaml.safe_load(config_file)
        cams = config['cameras']
        calibrations = [f"{config_path.rpartition('/')[0]}/{cal}" for cal in config['calibrations']]
        self.show_stream = config['show_stream']
        
        # Initialize camera, detector, and SORT tracker.
        if self.show_stream:
            cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
        self.fs = FrameServer(cams, calibrations)
        self.detector = SpheroBlobDetector()
        self.tracker = Sort(max_age=1, min_hits=1, iou_threshold=0.15)
        
    def run(self):
        while True:
            # Grab the current frame.
            _, frame = self.fs.grab()
            
            # Detect blobs.
            blobs, frame, mask = self.detector.detect(frame)
            
            # Prepare detections for tracking.
            detections = np.ndarray((len(blobs), 5))
            for i, blob in enumerate(blobs):
                score = 1
                bbox_size = blob.size / 2
                x1 = blob.pt[0] - bbox_size
                x2 = blob.pt[0] + bbox_size
                y1 = blob.pt[1] - bbox_size
                y2 = blob.pt[1] + bbox_size
                detections[i] = np.array([x1, y1, x2, y2, score])
            
            # Track blobs using SORT. Remove 'frame' from function call to disable drawing.
            tracked, frame = self.tracker.update(detections, frame)
            
            for obj in tracked:
                center, size, id = np_to_points(obj)
                image_cnt = int(center[0]), int(center[1])
                cv2.circle(frame, image_cnt, 5, (0, 0, 255), -1)
                frame = cv2.putText(frame, f'ID={int(id)}', (image_cnt[0] + 20, image_cnt[1] + 30), **self.detector.label_kwargs)
                    
            # Show the frames
            if self.show_stream:
                cv2.imshow('Camera', frame)
                cv2.imshow('Mask', mask)
                cv2.resizeWindow('Camera', *self.fs.window_size)
                cv2.resizeWindow('Mask', *self.fs.window_size)
                
            key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                break
            
    def shutdown(self):
        self.fs.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        node = WebcamTrackerTest()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
