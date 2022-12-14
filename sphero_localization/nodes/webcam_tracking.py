#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import yaml
import cv2
import imutils
import numpy as np
import rospy
from imutils.video import VideoStream
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from sphero_localization.duo_c270 import FrameServer
from sphero_localization.sort import Sort
from sphero_localization.blob_detector import SpheroBlobDetector


def np_to_points(np_array):
    x1, y1, x2, y2, id = np_array
    # TODO: Named tuple
    center = ((x1 + x2) / 2, (y1 + y2) / 2)
    size = (abs(x1 - x2) + abs(y1 - y2)) / 2  # average of width and height
    size /= 2  # radius
    return center, size, int(id - 1)


class WebcamTracker(object):
    def __init__(self):
        self.num_robots = rospy.get_param('/num_of_robots', 1)
        self.robot_name = rospy.get_param('/robot_name', 'sphero')
        
        # Create publishers for positions.
        self.pos_pubs = [rospy.Publisher(f'/{self.robot_name}_{i}/position', Point, queue_size=1)
                         for i in range(self.num_robots)]
        
        # Create publishers for sending color commands.
        self.color_pubs = [rospy.Publisher(f'/{self.robot_name}_{i}/set_color', ColorRGBA, queue_size=1) 
                           for i in range(self.num_robots)]
        
        rospy.sleep(0.5)

        # Initially turn off LEDs on all robots.
        for pub in self.color_pubs:
            pub.publish(0.0, 0.0, 0.0, 0.0)

        # Load config parameters.
        config_path = rospy.get_param('~config_path')
        with open(config_path, 'r') as config_file:
            config = yaml.safe_load(config_file)
        cams = config['cameras']
        calibrations = [f"{config_path.rpartition('/')[0]}/{cal}" for cal in config['calibrations']]
        self.show_stream = config['show_stream']
        
        # Initialize camera, detector, and SORT tracker.
        if self.show_stream:
            cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
        else:
            cv2.namedWindow("CV2 window", cv2.WINDOW_NORMAL)
        self.fs = FrameServer(cams, calibrations)
        self.detector = SpheroBlobDetector()
        self.tracker = Sort(max_age=1, min_hits=1, iou_threshold=0.15)  # TODO: this tracker is not good
        
        self.freq = rospy.get_param('/data_stream_freq')
        
    def run(self):
        r = rospy.Rate(self.freq)
        led_initialized = [False] * self.num_robots
        led_countdown = [2 * self.freq] * self.num_robots
        
        ok = True
        
        while not rospy.is_shutdown():
            # Turn on LEDs one by one.
            if not all(led_initialized):
                for i in range(self.num_robots):
                    if led_countdown[i] > 0:
                        led_countdown[i] -= 1
                        break
                    else:
                        self.color_pubs[i].publish(0.0, 1.0, 0.0, 1.0)
                        led_initialized[i] = True
            
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
                cnt, size, id = np_to_points(obj)
                if id >= self.num_robots:
                    rospy.logerr("Oh no! Tracking algorithm returned ID larger than the number of robots. "
                                 "Impossible to assign positions correctly.\n"
                                 "Focus CV2 window and press Q to restart.")
                    ok = False
                    
                # Draw detections on the camera feed.
                image_cnt = int(cnt[0]), int(cnt[1])
                cv2.circle(frame, image_cnt, 5, (0, 0, 255), -1)
                frame = cv2.putText(frame, f'ID={int(id)}', (image_cnt[0] + 20, image_cnt[1] + 30), **self.detector.label_kwargs)
                
                # Publish real world coordinates.
                if ok:
                    cnt_real = self.fs.transform(cnt)
                    self.pos_pubs[id].publish(Point(cnt_real[0], cnt_real[1], 0))
            
            # Show the frames
            if self.show_stream:
                cv2.imshow('Camera', frame)
                cv2.imshow('Mask', mask)
                cv2.resizeWindow('Camera', *self.fs.window_size)
                cv2.resizeWindow('Mask', *self.fs.window_size)
            
            key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                return
            
            r.sleep()
            
    def shutdown(self):
        self.fs.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("webcam_tracker")

    try:
        node = WebcamTracker()
        rospy.on_shutdown(node.shutdown)
        node.run()
        node.shutdown()
    except rospy.ROSInterruptException:
        pass
