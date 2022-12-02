# import the opencv library
import cv2
import sys

def main(id):
    # define a video capture object
    vid = cv2.VideoCapture(f'/dev/video{id}')
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
 
    cv2.namedWindow("main", cv2.WINDOW_NORMAL)

    while(True):
        
        # Capture the video frame
        # by frame
        ret, frame = vid.read()

        # Display the resulting frame
        cv2.imshow('main', frame)
        cv2.resizeWindow('main', 960, 720)
        
        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()


if __name__ == '__main__':
    id = sys.argv[1]
    
    main(id)
