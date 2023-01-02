#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This script will help you find the best HSV or RGB color filter parameters
for your application. By moving the sliders, you can see the effects of changed
parameters on the filter mask in real time.

USAGE: You need to specify a filter and "only one" image source

(python3) filter_param_finder --filter RGB --image /path/to/image.png
or
(python3) filter_param_finder --filter HSV --webcam

"""
import cv2
import argparse
from sphero_localization.duo_c270 import FrameServer


def callback(value):
    pass


def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", 0)

    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255

        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)


def get_arguments():
    ap = argparse.ArgumentParser()
    ap.add_argument('devices', type=str, nargs='+',
                    help="Camera (video) path. /dev/video*")
    ap.add_argument('-f', '--filter', required=True,
                    help='Range filter. RGB or HSV')
    ap.add_argument('-p', '--preview', required=False,
                    help='Show a preview of the image after applying the mask',
                    action='store_true')
    args = vars(ap.parse_args())

    if not args['filter'].upper() in ['RGB', 'HSV']:
        ap.error("Please specify a correct filter.")

    return args


def get_trackbar_values(range_filter):
    values = []

    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


def main():
    args = get_arguments()
    range_filter = args['filter'].upper()

    if args['preview']:
        cv2.namedWindow("Preview", cv2.WINDOW_NORMAL)
    else:
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Thresh", cv2.WINDOW_NORMAL)

    fs = FrameServer(args['devices'])

    setup_trackbars(range_filter)

    while True:
        individual, joined = fs.grab()

        if len(individual) == 1:
            image = individual[0]
            window = fs.SINGLE_WINDOW
        else:
            image = joined
            window = fs.DOUBLE_WINDOW

        if range_filter == 'RGB':
            frame_to_thresh = image.copy()
        else:
            frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

        if args['preview']:
            preview = cv2.bitwise_and(image, image, mask=thresh)
            cv2.imshow("Preview", preview)
            cv2.resizeWindow('Preview', *window)
        else:
            cv2.imshow("Original", image)
            cv2.imshow("Thresh", thresh)
            cv2.resizeWindow('Original', *window)
            cv2.resizeWindow('Thresh', *window)

        if cv2.waitKey(1) & 0xFF is ord('q'):
            break


if __name__ == '__main__':
    main()
