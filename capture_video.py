"""
Author: Konstantinos Angelopoulos
Date: 04/02/2020
All rights reserved.
Feel free to use and modify and if you like it give it a star.

Simultaneously capture video and store it.
"""


import cv2
import os


# Initialize Video Writer
def init(path, frame_width, frame_height):
    assert len(path) > 1, 'Length must be more than one letter!'
    assert frame_width != 0, 'Frame width must be greater than 0!'
    assert frame_height != 0, 'Frame height must be greater than 0!'
    try:
        video_writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 15, (frame_width, frame_height))
        return video_writer
    except Exception as e:
        print(f'[CAPTURE VIDEO]: {e}')
        return e


# Save color video from kinect
def write(video_writer, frame):
    assert frame is not None, 'Frame is Empty'
    try:
        video_writer.write(frame)
    except Exception as e:
        print(f'[CAPTURE VIDEO]: {e}')
        return e


if __name__ == '__main__':
    ABSOLUTE_FILE_PATH = os.path.dirname(os.path.realpath(__file__))
    # Open Laptop Camera
    cap = cv2.VideoCapture(0)
    # If no camera exists fail
    if not cap.isOpened():
        raise Exception('[CAPTURE VIDEO]: Unable to read from camera')
    # Video frame size
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    # Create instance of video writer
    out = cv2.VideoWriter(os.path.join(ABSOLUTE_FILE_PATH, 'Videos/out.avi'), cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (frame_width, frame_height))
    # Main Loop
    while True:
        # Read frames
        ret, frame = cap.read()
        # If frame returned
        if ret:
            # Write frames
            out.write(frame)
            # Show frames
            cv2.imshow('FRAME', frame)
            # Break loop with q
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
        else:
            print('[CAPTURE VIDEO]: Cant get frames from camera')
            pass
    # Release camera and writer and close all windows
    cap.release()
    out.release()
    cv2.destroyAllWindows()
