""" Test the kinect's rgb and depth view to find the optimum location """


from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import cv2
import numpy as np


kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color)

while True:
    color_frame = kinect.get_last_color_frame()
    colorImage = color_frame.reshape((kinect.color_frame_desc.Height, kinect.color_frame_desc.Width, 4)).astype(np.uint8)
    colorImage = cv2.flip(colorImage, 1)
    cv2.imshow('Test Color View', cv2.resize(colorImage, (int(1920/2.5), int(1080/2.5))))
    depth_frame = kinect.get_last_depth_frame()
    depth_img = depth_frame.reshape((kinect.depth_frame_desc.Height, kinect.depth_frame_desc.Width)).astype(np.uint8)
    depth_img = cv2.flip(depth_img, 1)
    cv2.imshow('Test Depth View', depth_img)
    # Quit using q
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cv2.destroyAllWindows()
