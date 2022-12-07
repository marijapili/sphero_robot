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

from sphero_localization.duo_c270 import FrameServer
from sphero_localization.sort_wrapper import SORTwrapper
from sphero_localization.sphero_blob_detector import SpheroBlobDetector


class WebcamTracker(object):
    def __init__(self):
        # Create publishers for sending color commands.
        self.num_robots = rospy.get_param('/num_of_robots', 1)
        self.color_pubs = [rospy.Publisher(f'/sphero_{i}/set_color', ColorRGBA, queue_size=1) 
                           for i in range(self.num_robots)]

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
        self.fs = FrameServer(cams, calibrations)
        self.detector = SpheroBlobDetector()
        self.tracker = SORTwrapper()
        
        self.freq = rospy.get_param('/data_stream_freq')
        
    def run(self):
        # TODO: what if we can't maintain this rate?
        r = rospy.Rate(self.freq)
        led_initialized = [False] * self.num_robots
        led_countdown = [2 * self.freq] * self.num_robots
        
        while not rospy.is_shutdown():
            # Turn on LEDs one by one.
            if not all(led_initialized):
                for i in range(self.num_robots):
                    if led_countdown[i] > 0:
                        led_countdown[i] -= 1
                        break
                    else:
                        self.color_pubs[i].publish(1.0, 1.0, 1.0, 1.0)
                        led_initialized[i] = True
            
            # Grab the current frame.
            _, frame = self.fs.grab()
            
            # Detect blobs.
            blobs, frame, mask = self.detector.detect(frame)
            real_blobs = []
            for blob in blobs:
                center = self.fs.transform(blob.pt)
                bbox_size = self.fs.scale(blob.size / 2)
                real_blobs.append((center, bbox_size))
            
            # Track blobs using SORT.
            self.tracker.update_cb(real_blobs)
            
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
    except rospy.ROSInterruptException:
        pass
