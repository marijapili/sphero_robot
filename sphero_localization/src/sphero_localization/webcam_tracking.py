#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time

import cv2
import imutils
import numpy as np
import rospy
from imutils.video import VideoStream
from sort_wrapper import SORTwrapper
from std_msgs.msg import ColorRGBA


def index_to_pos(row, col):
    """Return real position coordinates for list (map) indices."""
    x_real = 0.0 + col * 0.004
    y_real = 0.0 + row * 0.004
    return (x_real, y_real)


class WebcamTracker(object):
    def __init__(self):
        # Create publishers for sending color commands.
        num_robots = rospy.get_param('/num_of_robots', 1)
        color_pubs = [rospy.Publisher(f'/sphero_{i}/set_color', ColorRGBA, queue_size=1) 
                           for i in range(num_robots)]

        # Initially turn off LEDs on all robots.
        for pub in color_pubs:
            pub.publish(0.0, 0.0, 0.0, 0.0)

        # Initialize SORT tracker.
        sort = SORTwrapper()

        # Stream params
        vs = VideoStream(src='/dev/video0')
        vs.stream.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        vs.stream.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        vs.start()
        
        greenLower = (0, 0, 214)
        greenUpper = (76, 18, 255)
        rospy.sleep(2.0)
        
        freq = rospy.get_param('/data_stream_freq')
        r = rospy.Rate(freq)
        led_initialized = [False] * num_robots
        led_countdown = [2 * freq] * num_robots
        
        while not rospy.is_shutdown():
            # Turn on LEDs one by one.
            if not all(led_initialized):
                for i in range(num_robots):
                    if led_countdown[i] > 0:
                        led_countdown[i] -= 1
                        break
                    else:
                        color_pubs[i].publish(1.0, 1.0, 1.0, 1.0)
                        led_initialized[i] = True
            
            # Grab the current frame. Handles VideoCapture and VideoStream.
            frame = vs.read()
            if frame is None:
                break
            
            # frame = cv2.rotate(frame, cv2.ROTATE_180)

            # Resize the frame, blur it, and convert it to the HSV color space.
            # frame = imutils.resize(frame, width=600)  # TODO: why resize?
            blurred = cv2.GaussianBlur(frame, (3, 3), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            # Construct a mask for the color "green", then perform a series of
            # dilations and erosions to remove any small blobs left in the mask.
            mask = cv2.inRange(hsv, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Find contours in the mask and initialize the current (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            centers = []
            if cnts:
                # Find the largest contour in the mask, then use it to compute 
                # the minimum enclosing circle and centroid.
                for c in cnts:
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center = ((int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
                    if radius > 10: # and radius < 12:
                        # Draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)
                        centers.append(center)

            center_meters = []
            for center in centers:
                center_meters.append(index_to_pos(center[0], center[1]))
            sort.update_cb(center_meters)

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                break
            
            r.sleep()

        vs.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("webcam_tracker")

    try:
        node = WebcamTracker()
    except rospy.ROSInterruptException:
        pass
