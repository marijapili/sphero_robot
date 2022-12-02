#!/usr/bin/env python3
from collections import deque
from imutils.video import VideoStream, WebcamVideoStream
import numpy as np
import argparse
import cv2
import imutils
import time





vs = VideoStream(src='/dev/video2')
vs.stream.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
vs.stream.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
vs.start()

# construct the argument parse and parse the arguments
# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (0, 0, 214)
greenUpper = (76, 18, 255)
# if a video path was not supplied, grab the reference
# to the webcam
# allow the camera or video file to warm up
time.sleep(2.0)

# keep looping
while True:
	# grab the current frame
	frame = vs.read()
	# handle the frame from VideoCapture or VideoStream
	frame =  frame
	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if frame is None:
		break
	# resize the frame, blur it, and convert it to the HSV
	# color space
	# frame = imutils.resize(frame, width=600)
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
			if radius > 2:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(frame, (int(x), int(y)), int(radius),
					(0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)
	# update the points queue

		# loop over the set of tracked points
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
# if we are not using a video file, stop the camera video stream
vs.stop()
# otherwise, release the camera
# close all windows
cv2.destroyAllWindows()
