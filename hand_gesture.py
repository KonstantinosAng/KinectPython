from numba import jit, cuda


# Classify Hand Gesture based on Convex Hull and RGBD Data from Kinect v2
class HandGestureClassifier(object):

    def __init__(self, show_track_bars=False):
        """
        :param show_track_bars: boolean to show trackbars of manipulating hsv and rgb limits of skin detection
        """
        # Import libraries here for optimization
        import numpy as np
        self.radius = 130  # pixels
        self.kernel = np.ones((3, 3), np.uint8)  # kernel for morphological transformation on rgb image
        # Create track bars with values for skin detection
        if show_track_bars:
            self._window = cv2.namedWindow("Settings")
            self.lower_skin_b = cv2.createTrackbar("L_B", "Settings", 70, 255, self.nothing)
            self.lower_skin_g = cv2.createTrackbar("L_G", "Settings", 20, 255, self.nothing)
            self.lower_skin_r = cv2.createTrackbar("L_R", "Settings", 0, 255, self.nothing)
            self.upper_skin_b = cv2.createTrackbar("U_B", "Settings", 255, 255, self.nothing)
            self.upper_skin_g = cv2.createTrackbar("U_G", "Settings", 255, 255, self.nothing)
            self.upper_skin_r = cv2.createTrackbar("U_R", "Settings", 20, 255, self.nothing)
            self._show = True
        else:
            self._show = False
            self.lower_skin = np.array([0, 20, 70], dtype=np.uint8)  # lower limit for skin color
            self.upper_skin = np.array([20, 255, 255], dtype=np.uint8)  # upper limit for skin color
        self.state = ''  # Hand State Open || Closed
        self.fingers = 0  # Number of Fingers
        self._done = False  # Flag to close detector

    # Just a callable function that does nothing
    def nothing(self, value):
        """
        A function that does nothing only for the track bar callback
        :param value: track bar return value
        :return: None
        """
        pass

    def roi(self, frame, x, y, z):
        """
        Calculate region of interest and crop out the operator's  hand from the rgb image
        :param frame: 1920x1080 rgb image as array
        :param x: x color pixel coordinate of hand
        :param y: y color pixel coordinate of hand
        :param z: z depth distance of operator's hand
        :return: rgb image with the operator's cropped hand
        """
        frame_height, frame_width, frame_channels = frame.shape
        # Compute region of interest on color image base on the location of the tracked hand
        radius = int(self.radius/z)
        min_frame_x = x - radius
        min_frame_y = y - radius
        max_frame_x = x + radius
        max_frame_y = y + radius
        # Catch exceptions for image limits
        if min_frame_x < 0:
            min_frame_x = 0
            max_frame_x = int(2 * radius)
        if min_frame_x > frame_width - 2 * radius:
            min_frame_x = int(frame_width - 2 * radius)
            max_frame_x = frame_width
        if min_frame_y < 0:
            min_frame_y = 0
            max_frame_y = int(2 * radius)
        if min_frame_y > frame_height - 2 * radius:
            min_frame_y = int(frame_height - 2 * radius)
            max_frame_y = frame_height
        return frame[min_frame_y:max_frame_y, min_frame_x:max_frame_x]

    # function optimized to run on gpu
    # @jit(target="cuda")
    def detect(self, frame, x, y, z, show=False, flipped=False):
        """
        classify operator's hand based on rgb/d data
        :param frame: rgb 1920x1080 image frame as array
        :param x: x color pixel coordinate
        :param y: y color pixel coordinate
        :param z: z depth distance of hand from rgbd sensor
        :param show: boolean to show the classified hand state
        :param flipped: boolean to flip the frame to match the pixel coordinates
        :return: None
        """
        # Import libraries here for optimization
        import cv2
        import numpy as np
        # Loop until flag
        if not self._done:
            try:
                if flipped:
                    frame = cv2.flip(frame, 1)  # If the image is flipped flip it again for better accuracy
                roi = self.roi(frame, x, y, z)  # Compute area of hand only
                # cv2.imshow('ROI', roi)
                # cv2.imwrite('images/roi1.png', roi)
                # Change track bars position
                if self._show:
                    self.lower_skin = np.array([cv2.getTrackbarPos("L_R", "Settings"), cv2.getTrackbarPos("L_G", "Settings"), cv2.getTrackbarPos("L_B", "Settings")], dtype=np.uint8)  # lower limit for skin color
                    self.upper_skin = np.array([cv2.getTrackbarPos("U_R", "Settings"), cv2.getTrackbarPos("U_G", "Settings"), cv2.getTrackbarPos("U_B", "Settings")], dtype=np.uint8)  # upper limit for skin color
                # Convert from BGR to HSV
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                # cv2.imshow('HSV', hsv)
                # cv2.imwrite('images/roi2.png', hsv)
                # Apply mask for skin detection
                mask = cv2.inRange(hsv, self.lower_skin, self.upper_skin)
                # cv2.imshow('Skin Detection', mask)
                # cv2.imwrite('images/roi3.png', mask)
                # Dilate image to fill black spots and spots inside hand
                mask = cv2.dilate(mask, self.kernel, iterations=4)
                # cv2.imshow('Dilate', mask)
                # cv2.imwrite('images/roi4.png', mask)
                # Apply Gaussian Blur to reduce noise
                mask = cv2.GaussianBlur(mask, (5, 5), 100)
                # cv2.imshow('GaussianBlur', mask)
                # cv2.imwrite('images/roi5.png', mask)
                # Compute contours in remaining image
                contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                # Find max contour in given area
                max_contour = max(contours, key=lambda x: cv2.contourArea(x))

                # Find useful data of these contour like center of mass
                # M = cv2.moments(max_contour)
                # cx = int(M['m10']/M['m00'])
                # cy = int(M['m01']/M['m00'])
                # cv2.circle(roi, (int(cx), int(cy)), 3, [255, 0, 255], -1)
                # Find minimum enclosing area
                # (circle_x, circle_y), radius = cv2.minEnclosingCircle(max_contour)
                # center = (int(circle_x), int(circle_y))
                # radius = int(radius)
                # cv2.circle(roi, center, radius, [128, 0, 128], 2)
                # Find maximum distance of contour
                # distance = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
                # cv2.circle(roi, center, int(np.amax(distance)), [255, 0, 0], 2)
                # cv2.imshow('DISTANCES', distance)
                # cv2.drawContours(roi, [max_contour], 0, (0, 0, 255), 2)
                # cv2.imshow('Max Contour', roi)
                # cv2.imwrite('images/roi6.png', roi)
                # Compute contour and convex hull area to compare
                epsilon = 0.0005 * cv2.arcLength(max_contour, True)
                approx = cv2.approxPolyDP(max_contour, epsilon, True)
                hull = cv2.convexHull(max_contour)
                area_hull = cv2.contourArea(hull)
                area_contour = cv2.contourArea(max_contour)
                area_ratio = ((area_hull - area_contour) / area_contour) * 100
                # print(area_ratio)
                hull = cv2.convexHull(approx, returnPoints=False)
                defects = cv2.convexityDefects(approx, hull)
                fingers = 0
                # Search for defects on convex hull of max contour area found
                for i in range(defects.shape[0]):
                    # Get starting point, ending point, farthest point and distance to farthest point for each line
                    s, e, f, d = defects[i, 0]
                    """
                    The triangles are generated when two fingers are apart and the area_hull - area_contour leaves a triangle shape behind.
                    The upside triangle between two fingers apart
                    """
                    # Find sides of triangle
                    start = approx[s][0]
                    end = approx[e][0]
                    far = approx[f][0]
                    # Calculate the length of each triangle line
                    a = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
                    b = np.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
                    c = np.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
                    # Calculate the semi - perimeter of the triangle
                    semi_perimeter = (a + b + c) / 2
                    # Calculate the area of the triangle using Heron's formula that uses only the lengths of the triangle
                    ar = np.sqrt(semi_perimeter * (semi_perimeter - a) * (semi_perimeter - b) * (semi_perimeter - c))
                    # Then from the area of the triangle we can find the height assuming that the triangle is upside down, so the a side is always he base.
                    # The height is the distance of the farthest point from the convexhull
                    d = (2 * ar) / a
                    # Cosine rule for triangle angles
                    angle = np.arccos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c)) * (180/np.pi)
                    # if the angle is greater than 90 then there is no defect and the line is on the perimeter of the hand
                    # if the distance of the farthest point is too close then the point is inside the hand and its just noise.
                    if angle <= 90 and d > 30:
                        fingers += 1  # find defects on convex hull ( defects usually are fingers )
                        if show:
                            cv2.circle(roi, tuple(far), 3, [255, 0, 0], -1)
                    if show:
                        cv2.line(roi, tuple(start), tuple(end), [0, 255, 0], 2)

                fingers += 1
                # Update state based on number of defects/edges found in convex hull
                if fingers == 1:
                    if area_ratio < 26 / z:
                        if show:
                            cv2.putText(roi, 'Closed', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 3, cv2.LINE_AA)
                        self.state = 'CLOSED'
                        self.fingers = 0
                    else:
                        if show:
                            cv2.putText(roi, 'Open', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 3, cv2.LINE_AA)
                        self.state = 'OPEN'
                        self.fingers = 1
                elif fingers == 2:
                    if area_ratio < 26 / z:
                        if show:
                            cv2.putText(roi, 'Closed', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 3, cv2.LINE_AA)
                        self.state = 'CLOSED'
                        self.fingers = 0
                    else:
                        if show:
                            cv2.putText(roi, 'Open', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 3, cv2.LINE_AA)
                        self.state = 'OPEN'
                        self.fingers = 2
                elif fingers == 3:
                    if show:
                        cv2.putText(roi, '3', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 3, cv2.LINE_AA)
                    self.state = 'OPEN'
                    self.fingers = 3
                elif fingers == 4:
                    if show:
                        cv2.putText(roi, '4', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 3, cv2.LINE_AA)
                    self.state = 'OPEN'
                    self.fingers = 4
                elif fingers == 5:
                    if show:
                        cv2.putText(roi, '5', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 3, cv2.LINE_AA)
                    self.state = 'OPEN'
                    self.fingers = 5
                else:
                    if show:
                        cv2.putText(roi, 'UNKNOWN', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 3, cv2.LINE_AA)
                    self.state = 'UNKNOWN'
                if show:
                    # Show ROI of Color Frame
                    cv2.imshow('Hand Classifier', roi)
                """
                # Capture using q or space
                if cv2.waitKey(1) & 0xff == ord('q') or cv2.waitKey(1) % 256 == 32:
                    print(f'[GESTURE CLASSIFIER]: SAVED IMAGE')
                    cv2.imwrite('images/roi1.png', roi)
                cv2.imwrite('images/roi7.png', roi)
                print(f'[FIN]')
                import time
                time.sleep(1)
                """
            # Handle any exception
            except Exception as e:
                print(f'[GESTURE CLASSIFIER]: {e}')

        # Destroy all windows
        else:
            cv2.destroyAllWindows()

    def close(self):
        """
        Stop classification loop
        :return: None
        """
        self._done = True

    def __del__(self):
        """
        delete instance
        :return: None
        """
        self._done = True

    def __exit__(self):
        """
        Exit class
        :return: None
        """
        self._done = True

    def __delete__(self):
        """
        Delete class instance
        :return: None
        """
        self._done = True


if __name__ == '__main__':
    # Test functionality
    from pykinect2 import PyKinectV2
    from pykinect2 import PyKinectRuntime
    import cv2
    import numpy as np

    bodies = None
    kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)
    gesture_classifier = HandGestureClassifier()

    while True:
        color_frame = kinect.get_last_color_frame()
        colorImage = color_frame.reshape((kinect.color_frame_desc.Height, kinect.color_frame_desc.Width, 4)).astype(np.uint8)
        colorImage = cv2.flip(colorImage, 1)
        cv2.imshow("Color", cv2.resize(colorImage, (int(960/1.5), int(540/1.5))))

        # --- Get Body Frame
        if kinect.has_new_body_frame():
            bodies = kinect.get_last_body_frame()

        if bodies is not None:
            for i in range(0, kinect.max_body_count):
                body = bodies.bodies[i]
                if not body.is_tracked:
                    continue

                joints = body.joints
                hand_right_state = body.hand_right_state
                if hand_right_state == PyKinectV2.HandState_Open:
                    hand_right_state = 'OPEN'
                elif hand_right_state == PyKinectV2.HandState_Closed:
                    hand_right_state = 'CLOSED'
                else:
                    hand_right_state = 'UNKNOWN'
                color_joint_points = kinect.body_joints_to_color_space(joints)
                color_hand_right_x = color_joint_points[PyKinectV2.JointType_HandRight].x  # pixels
                color_hand_right_y = color_joint_points[PyKinectV2.JointType_HandRight].y  # pixels
                world_hand_right_z = joints[PyKinectV2.JointType_HandRight].Position.z * 1000  # mm
                try:
                    gesture_classifier.detect(colorImage, int(color_hand_right_x), int(color_hand_right_y), world_hand_right_z / 1000, show=True, flipped=True)
                    print(f"[GESTURE CLASSIFIER]: {gesture_classifier.state} [KINECT STATE]: {hand_right_state}")
                except Exception as e:
                    print(f"[GESTURE CLASSIFIER] {e}")
        # Quit using q
        if cv2.waitKey(1) & 0xff == ord('q'):
            break

    cv2.destroyAllWindows()
