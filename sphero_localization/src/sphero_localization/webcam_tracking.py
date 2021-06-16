#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy


class WebcamTracker(object):
    def __init__(self):
        self.stream = cv2.VideoCapture(0)  # The number is the index of the camera, try 0, 1, 2, ..

        while not rospy.is_shutdown():
            # Capture the video frame by frame
            ret, frame = self.stream.read()

            # Display the resulting frame
            cv2.imshow('frame', frame)

            # the 'q' button is set as the quitting button
            # you may use any desired button of your choice
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.stream.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("webcam_tracker")

    try:
        node = WebcamTracker()
    except rospy.ROSInterruptException:
        pass
