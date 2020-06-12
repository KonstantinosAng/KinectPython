"""
Capture background and store it for fabric localization
"""
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import cv2
import numpy as np
import os

# Create Kinect
kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color)
shot = True  # Flag for shot

# Loop until photo taken
while shot is True:
    # wait until frame is returned
    if kinect.has_new_color_frame():
        color_frame = kinect.get_last_color_frame()
        colorImage = color_frame.reshape((kinect.color_frame_desc.Height, kinect.color_frame_desc.Width, 4)).astype(np.uint8)
        cv2.imwrite(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'images/background.png'), colorImage)
        shot = False
