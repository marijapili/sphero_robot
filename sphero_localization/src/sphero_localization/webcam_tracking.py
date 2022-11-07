#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rospy
import numpy as np
from sort_wrapper import SORTwrapper
from imutils.video import VideoStream
from std_msgs.msg import ColorRGBA
import imutils
import time

def index_to_pos(row, col):
    """Return real position coordinates for list (map) indices."""
    x_real = 0.0 + col * 0.004
    y_real = 0.0 + row * 0.004
    return (x_real, y_real)


class WebcamTracker(object):
    def __init__(self):
        # Create publishers for sending color commands
        num_of_robots = 3
        pub_keys = ['/sphero_{}/'.format(i) for i in range(num_of_robots)]

        self.pubs = dict.fromkeys(pub_keys)
        for key in self.pubs.keys():
            self.pubs[key] = rospy.Publisher(key + 'set_color', ColorRGBA, queue_size=1)
        rospy.sleep(1.0)

        for key in self.pubs.keys():
            self.pubs[key].publish(0.0, 0.0, 0.0, 0.0)
        rospy.sleep(1.0)

        sort = SORTwrapper()

        # Stream params
        vs = VideoStream(src=2).start()
        greenLower = (0, 0, 214)
        greenUpper = (76, 18, 255)
        
        time.sleep(2.0)
        r = rospy.Rate(100)
        
        c1 = [key for key in pub_keys]
        print(c1)
        c2 = [100 for i in range(num_of_robots)]
        c3 = [1 for i in range(num_of_robots)]

        while not rospy.is_shutdown():
            # Capture the video frame by frame
            # grab the current frame
            center_meters = []
            if not all(v == 0 for v in c3):
                for i in range(len(c2)):
                    if c2[i] > 0:
                        c2[i] -= 1
                        break
                    else:
                        self.pubs[c1[i]].publish(1.0, 1.0, 1.0, 1.0)
                        c3[i] = 0

            frame = vs.read()
            # handle the frame from VideoCapture or VideoStream
            frame =  cv2.rotate(frame, cv2.ROTATE_180)
            # if we are viewing a video and we did not grab a frame,
            # then we have reached the end of the video
            if frame is None:
                break
            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame, width=600)
            blurred = cv2.GaussianBlur(frame, (3, 3), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            # construct a mask for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            mask = cv2.inRange(hsv, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

                # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            centers = []
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #c = max(cnts, key=cv2.contourArea)
                for i in range(len(cnts)):

                    c = cnts[i]

                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center = ((int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
                    # only proceed if the radius meets a minimum size
                    if radius > 2 and radius < 12:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frame, (int(x), int(y)), int(radius),
                            (0, 255, 255), 2)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)
                        centers.append(center)
            # update the points queue

            for center in centers:
                center_meters.append(index_to_pos(center[0], center[1]))
            #if len(centers) == num_of_robots:
            sort.update_cb(center_meters)

            # loop over the set of tracked points
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
