"""
This script will take the picture from a given camera stream and save it to file
every 5 seconds. It also produces an audible beep each time a new picture is
taken so you know when to reposition the calibration pattern for the next
picture.
"""

import time
import argparse
from datetime import datetime
from pathlib import Path

import beepy
import cv2
from imutils.video import VideoStream

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('camera', help='path to camera, /dev/video*', default='/dev/video0')
    args = parser.parse_args()

    ## Specify device path and resolution
    vs = VideoStream(src=args.camera)
    vs.stream.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    vs.stream.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    vs.start()

    ## Make necessary folder structure.
    save_path = Path(__file__).parent / 'pictures/raw'
    save_path.mkdir(parents=True, exist_ok=True)

    tic = time.time()
    while(True):
        frame = vs.read()

        cv2.imshow('frame',frame)
        
        if time.time() - tic > 5:
            cv2.imwrite(f"{save_path}/calibrate_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png", frame)
            beepy.beep(sound=1)
            tic = time.time()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        

    vs.stop()
    cv2.destroyAllWindows()