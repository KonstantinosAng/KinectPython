""" Snap ad save photos using kinect v2 """


from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import ctypes
import pygame
import cv2
import numpy as np


frame_rate = 30  # Frame rate of updating screen
counter = 0
# Flags for breaking loop after saving photo
depth_shot = False
ir_shot = False
color_shot = False


# Main Class for Kinect Object
class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1),
                                               pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

        pygame.display.set_caption("Color Image")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body | PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Infrared)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface(
            (self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data
        self._bodies = None

    def draw_color_frame(self, frame, target_surface):
        # Map color frame on screen
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def run(self):
        global counter, depth_shot, color_shot, ir_shot
        counter = 0
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get():  # User did something
                if event.type == pygame.QUIT:  # If user clicked close
                    self._done = True  # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE:  # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'],
                                                           pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

            # --- Getting frames and drawing
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size)
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));

            # Take color image and write it as png then quit
            if self._kinect.has_new_color_frame() and not color_shot:
                color_frame = self._kinect.get_last_color_frame()
                color_img = color_frame.reshape((self._kinect.color_frame_desc.Height, self._kinect.color_frame_desc.Width, 4)).astype(np.uint8)
                cv2.imshow('Color Photo', color_img)
                cv2.waitKey(2000)
                cv2.imwrite('images/image9.png', color_img)
                color_shot = True

            # Take ir image and write it as png
            if self._kinect.has_new_infrared_frame() and not ir_shot:
                # Kinect IR Stream
                ir_frame = self._kinect.get_last_infrared_frame()
                ir_img = ir_frame.reshape((self._kinect.infrared_frame_desc.Height, self._kinect.infrared_frame_desc.Width)).astype(np.uint16)
                cv2.imshow('Infrared Photo', ir_img)
                cv2.waitKey(2000)
                cv2.imwrite('images/image10.png', ir_img)
                ir_shot = True

            # Take depth image and write it as png
            if self._kinect.has_new_depth_frame() and not depth_shot:
                depth_frame = self._kinect.get_last_depth_frame()
                depth_img = depth_frame.reshape((self._kinect.depth_frame_desc.Height, self._kinect.depth_frame_desc.Width)).astype(np.uint8)
                cv2.imshow('Depth Photo', depth_img)
                cv2.waitKey(2000)
                cv2.imwrite('images/image11.png', depth_img)
                depth_shot = True

            if depth_shot and color_shot and ir_shot:
                self._done = True
            self._screen.blit(surface_to_draw, (0, 0))
            surface_to_draw = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(frame_rate)
            counter += 1
        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()


__main__ = "Calibration Photos"
game = BodyGameRuntime()
# For user
print('Loaded 100%, Starting...')
game.run()
