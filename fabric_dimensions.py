"""
Author: Konstantinos Angelopoulos
Date: 04/02/2020
All rights reserved.
Feel free to use and modify and if you like it give it a star.

 LOCALIZATION OF FABRIC
"""


# Make this a function to return the dimensions of fabric the rotation and the corner points
def find_dimensions(colorImage, color_focal_length):
    """
    LOCATE FABRIC AND FIND ITS DIMENSIONS
    :param colorImage: 1920x1080 rgb image as array
    :param color_focal_length: color focal length of the rgb sensor
    :return: fabric points, width, height, angle and fabric found
    """
    # Import libraries only for this function to minimize memory usage
    import cv2
    import numpy as np
    from matplotlib import pyplot as plt
    import matplotlib.patches as patches
    # Camera Parameters
    """

     depth_focal_length = 365.7  # focal length for depth map from pdf https://www.uv.es/imaging3/PDFs/2016_OEng_56_41305.pdf
     color_focal_length = 329.1  # focal length for colormap from pdf https://www.uv.es/imaging3/PDFs/2016_OEng_56_41305.pdf
     depth_FOV_horizontal = 70.6  # in degrees http://www.smeenk.com/webgl/kinectfovexplorer.html
     depth_FOV_vertical = 60  # in degrees https://weekly-geekly.github.io/articles/272629/index.html
     color_FOV_horizontal = 84.1  # in degrees http://www.smeenk.com/webgl/kinectfovexplorer.html
     color_FOV_vertical = 53.8  # in degrees https://weekly-geekly.github.io/articles/272629/index.html
     depth_focal_length = (524*0.5)/tan(depth_FOV_horizontal*0.5*pi/180);  # https://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/
     color_focal_length = (1920*0.5)/tan(color_FOV_horizontal*0.5*pi/180);  # https://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/
     depth_focal_length_x = 391.096;  # https://threeconstants.wordpress.com/2014/11/09/kinect-v2-depth-camera-calibration/
     color_focal_length_x = 1062.48808;  # https://www.semanticscholar.org/paper/A-new-method-in-simultaneous-estimation-of-sensor-Safaei-Fazli/3944c7f5e39090f63ccbc4ac5664519003a3cedf/figure/4
     color_focal_length_y = 1065.01486;  # https://www.semanticscholar.org/paper/A-new-method-in-simultaneous-estimation-of-sensor-Safaei-Fazli/3944c7f5e39090f63ccbc4ac5664519003a3cedf/figure/4

    """

    depth_FOV_horizontal = 71  # in pixels http://www.smeenk.com/webgl/kinectfovexplorer.html
    color_FOV_horizontal = 84  # in pixels http://www.smeenk.com/webgl/kinectfovexplorer.html
    depth_focal_length_x = 388.198  # https://www.researchgate.net/figure/Parameters-from-the-calibration-of-the-Kinect-v2-by-rounding_tbl4_321048476
    depth_focal_length_y = 389.033  # https://www.researchgate.net/figure/Parameters-from-the-calibration-of-the-Kinect-v2-by-rounding_tbl4_321048476
    color_focal_length_x = 1144.361  # https://www.researchgate.net/figure/Parameters-from-the-calibration-of-the-Kinect-v2-by-rounding_tbl4_321048476
    color_focal_length_y = 1147.337  # https://www.researchgate.net/figure/Parameters-from-the-calibration-of-the-Kinect-v2-by-rounding_tbl4_321048476
    fabric_ret = True  # flag for finding object
    print('Searching for Fabric')
    # Find Bounding box and dimensions
    img = colorImage
    # Testing
    # img = cv2.imread('images/img2.jpg')
    img = cv2.medianBlur(img, 5)  # Median Blur to reduce noise
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Transform to HSV ( Hue Saturation Value ) to extract a specific color
    COLOR_MIN = np.array([20, 80, 50], np.uint8)  # color min value for threshold
    COLOR_MAX = np.array([40, 255, 255], np.uint8)  # color max for threshold
    frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)  # Threshold image
    # imgray = frame_threshed  # Gray image
    ret, thresh = cv2.threshold(frame_threshed, 127, 255, 0)  # Threshold for specific range
    # ret, thresh = cv2.threshold(frame_threshed, 220, 255, 0, cv2.THRESH_BINARY)  # Threshold for specific range
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # find contours
    areas = [cv2.contourArea(c) for c in contours]  # find areas in contour
    max_index = np.argmax(areas)  # Find max area
    cnt = contours[max_index]  # save area

    # This is for non-rotated rectangle
    # x, y, w, h = cv2.boundingRect(cnt)  # in pixels
    # cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # Calculate bounding rect for fabric
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    '''
    
    box = [  x_bottom_right ,  y_bottom_right  ] Bottom Right Corner
          [  x_bottom_left  ,  y_bottom_left   ] Bottom Left Corner
          [  x_top_left     ,  y_top_left      ] Top Left Corner
          [  x_top_right    ,  y_top_right     ] Top Right Corner
          
    '''
    # Dimensions in pixels
    # height, width = rect[1]
    width = np.sqrt((box[0, 0] - box[1, 0])**2 + (box[0, 1] - box[1, 1])**2)  # in pixels
    height = np.sqrt((box[1, 0] - box[2, 0])**2 + (box[1, 1] - box[2, 1])**2)  # in pixels
    angle = rect[2]  # in degrees

    '''
    fig, ax = plt.subplots(1)
    rectan = patches.Rectangle((0, 0), world_width, world_height, linewidth=1, edgecolor='r', facecolor='r', alpha=0.70)
    t = patches.transforms.Affine2D().rotate_deg(-world_angle) + ax.transData
    rectan.set_transform(t)
    ax.set(xlim=(-world_height-10, world_height+10), ylim=(-10-world_height, world_height+10))
    ax.add_patch(rectan)
    plt.grid(True)
    plt.show()
    '''
    # Catch exception where no fabric is found
    if box.all() == 0:
        fabric_ret = False

    # Draw Dimensions
    if fabric_ret:
        cv2.putText(img, 'Width: ' + str(width) + ' px', (int(box[1, 0] - 20 + width/2), int(box[1, 1] + height + 40)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), lineType=cv2.LINE_AA)
        cv2.putText(img, 'Height: ' + str(height) + ' px', (int(box[1, 0] - 20 + width/2), int(box[1, 1] + height + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), lineType=cv2.LINE_AA)
        cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
        res_img = cv2.resize(img, (int(1920/3.5), int(1080/3.5)))
        cv2.imshow('Fabric Dimensions', res_img)
        # Get perspective view transformed image
        perspective_transform(img, box, color_focal_length)
        cv2.waitKey(14000)
        cv2.destroyAllWindows()

    return box, width, height, angle, fabric_ret


# Find the fabric world points using the extrinsics parameters of the camera
def world(box, depth, color_focal_length_x, color_focal_length_y, color_principal_point_offset_xo, color_principal_point_offset_yo):
    """
    TRANSFORM COLOR POINTS TO WORLD COORDINATES
    :param box: list fabric color points
    :param depth: list fabric depth distance from rgb sensor
    :param color_focal_length_x: float rgb sensor focal length in x axis
    :param color_focal_length_y: float rgb sensor focal length in y axis
    :param color_principal_point_offset_xo: float rgb sensor principal point offset in x axis
    :param color_principal_point_offset_yo: float rgb sensor principal point offset in y axis
    :return: fabric world points, width, height, angle
    """
    import numpy as np
    world_box = np.ndarray(shape=(4, 3), dtype=float)
    for i in range(len(box)):
        world_box[i, 0] = (box[i, 0] - color_principal_point_offset_xo) * depth[i] / color_focal_length_x
        world_box[i, 1] = -(box[i, 1] - color_principal_point_offset_yo) * depth[i] / color_focal_length_y
        world_box[i, 2] = depth[i]
    xmin, ymin, zmin = world_box.min(axis=0)
    xmax, ymax, zmax = world_box.max(axis=0)
    # world_width = math.sqrt((world_box[0, 0] - world_box[1, 0]) ** 2 + (world_box[0, 1] - world_box[1, 1]) ** 2 + (world_box[0, 2] - world_box[1, 2]) ** 2)  # in mm
    world_width = np.sqrt((xmax - xmin) ** 2)  # in mm
    # world_height = math.sqrt((world_box[1, 0] - world_box[2, 0]) ** 2 + (world_box[1, 1] - world_box[2, 1]) ** 2 + (world_box[1, 2] - world_box[2, 2]) ** 2)  # in mm
    world_height = np.sqrt((zmax - zmin) ** 2)  # in mm
    if world_box[0, 0] == world_box[1, 0]:
        world_angle = 90
    else:
        world_angle = np.degrees(np.arctan(np.fabs(world_box[0, 1] - world_box[1, 1]) / np.fabs(world_box[0, 0] - world_box[1, 0])))
    return world_box, world_width, world_height, world_angle


# Find the fabric width, height orientation
def world_dimensions(points):
    """
    FIND FABRIC WORLD DIMENSIONS
    :param points: list with fabric world points
    :return: list with world points, width, height and angle
    """
    import numpy as np
    xmin, ymin, zmin = points.min(axis=0)
    xmax, ymax, zmax = points.max(axis=0)
    world_width = np.sqrt((xmax - xmin)**2)
    world_height = np.sqrt((zmax - zmin)**2)
    if points[0, 0] == points[1, 0]:
        world_angle = 90
    else:
        world_angle = np.degrees(np.arctan(np.fabs(points[0, 1] - points[1, 1]) / np.fabs(points[0, 0] - points[1, 0])))
    return points, world_width, world_height, world_angle


# Rearrange the fabric points to ensure that they are in the correct order
def arrange(points):
    """
    The correct order should always be ["BR", "BL", "TL", "TR"]
    and BR and BL are the edges closer to the kinect's and sorted
    by the kinect's point of view.
    REARRANGE FABRIC POINTS TO CORRECT ORDER
    :param points: list with world fabric points
    :return list with arranged fabric points
    """
    import numpy as np
    _z = points[points[:, 2].argsort()]
    arranged_points = np.ndarray(shape=(4, 3))
    if _z[0, 0] <= _z[1, 0]:
        arranged_points[0] = _z[0]
        arranged_points[1] = _z[1]
    else:
        arranged_points[0] = _z[1]
        arranged_points[1] = _z[0]
    if _z[2, 0] <= _z[3, 0]:
        arranged_points[3] = _z[2]
        arranged_points[2] = _z[3]
    else:
        arranged_points[3] = _z[3]
        arranged_points[2] = _z[2]
    return arranged_points


# Find the Perspective Transformation matrix to calculate the real fabric dimensions:
def perspective_transform(image, points, focal_length):
    """
    DOCUMENTATION
    https://stackoverflow.com/questions/33497736/opencv-adjusting-photo-with-skew-angle-tilt
    https://www.programcreek.com/python/example/89422/cv2.warpPerspective
    https://www.microsoft.com/en-us/research/uploads/prod/2016/11/Digital-Signal-Processing.pdf
    https://pysource.com/2018/02/14/perspective-transformation-opencv-3-4-with-python-3-tutorial-13/
    https://programtalk.com/python-examples/cv2.getPerspectiveTransform/
    https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html
    https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/
    https://math.stackexchange.com/questions/1339924/compute-ratio-of-a-rectangle-seen-from-an-unknown-perspective
    https://stackoverflow.com/questions/17087446/how-to-calculate-perspective-transform-for-opencv-from-rotation-angles
    https://stackoverflow.com/questions/38285229/calculating-aspect-ratio-of-perspective-transform-destination-image
    TODO: Find the perspective transformation given the angle of the taken image
    :param image: rgb image as array
    :param points: rgb fabric points
    :param focal_length: sensor rgb focal length (float)
    :return: perspective transformation matrix
    """
    # Import here for optimization
    import cv2
    import numpy as np
    import scipy.spatial.distance

    (rows, cols, _) = image.shape
    # Image center
    u0 = cols/2.0
    v0 = rows/2.0

    # detected corners
    corners = [(points[2, 0], points[2, 1]), (points[3, 0], points[3, 1]), (points[1, 0], points[1, 1]), (points[0, 0], points[0, 1])]

    # Widths and heights
    w1 = scipy.spatial.distance.euclidean(corners[0], corners[1])
    w2 = scipy.spatial.distance.euclidean(corners[2], corners[3])

    h1 = scipy.spatial.distance.euclidean(corners[0], corners[2])
    h2 = scipy.spatial.distance.euclidean(corners[1], corners[3])

    w = max(w1, w2)
    h = max(h1, h2)

    ar_vis = float(w)/float(h)

    # make numpy arrays and append 1 for linear algebra
    m1 = np.array((corners[0][0], corners[0][1], 1)).astype('float32')
    m2 = np.array((corners[1][0], corners[1][1], 1)).astype('float32')
    m3 = np.array((corners[2][0], corners[2][1], 1)).astype('float32')
    m4 = np.array((corners[3][0], corners[3][1], 1)).astype('float32')

    # calculate the focal distance
    k2 = np.dot(np.cross(m1, m4), m3) / np.dot(np.cross(m2, m4), m3)
    k3 = np.dot(np.cross(m1, m4), m2) / np.dot(np.cross(m3, m4), m2)

    n2 = k2 * m2 - m1
    n3 = k3 * m3 - m1

    n21 = n2[0]
    n22 = n2[1]
    n23 = n2[2]

    n31 = n3[0]
    n32 = n3[1]
    n33 = n3[2]

    # f = math.sqrt(np.abs((1.0 / (n23 * n33)) * ((n21 * n31 - (n21 * n33 + n23 * n31) * u0 + n23 * n33 * u0 * u0) + (n22 * n32 - (n22 * n33 + n23 * n32) * v0 + n23 * n33 * v0 * v0))))
    f = focal_length
    a = np.array([[f, 0, u0], [0, f, v0], [0, 0, 1]]).astype('float32')
    at = np.transpose(a)
    ati = np.linalg.inv(at)
    ai = np.linalg.inv(a)

    # calculate the real aspect ratio
    ar_real = np.sqrt(np.dot(np.dot(np.dot(n2, ati), ai), n2) / np.dot(np.dot(np.dot(n3, ati), ai), n3))

    if ar_real < ar_vis:
        W = int(w)
        H = int(W / ar_real)
    else:
        H = int(h)
        W = int(ar_real * H)

    pts1 = np.array(corners).astype('float32')
    pts2 = np.float32([[0, 0], [W, 0], [0, H], [W, H]])

    # project the image with the new w/h
    M = cv2.getPerspectiveTransform(pts1, pts2)

    dst = cv2.warpPerspective(image, M, (W, H))

    cv2.imshow('Perspective Transform', cv2.flip(cv2.resize(dst, (int(1920/3.5), int(1080/3.5))), 1))


# Find fabric by background extraction
def fabric_extraction(image, show=False):
    """
    :param image: Color Image with fabric as array
    :param show: Parameter to show fabric as image
    :return: fabric location in Color Space
    """
    # Import here for optimization
    import cv2
    import os
    background = cv2.imread('images/background.png')
    cv2.imwrite('sub.png', image)
    image = cv2.imread('sub.png')
    os.remove('sub.png')
    fabric = cv2.subtract(background, image)
    if show:
        cv2.imshow('Extracted Fabric', cv2.resize(fabric, (int(1920/3.5), int(1080/3.5))))
        cv2.waitKey(100)
    return fabric


# Find fabric by identifying blobs
def blob_detection(image):
    """
    Takes color image and detects blobs
    :param image: color image 1920x1080 from camera
    :return: image with written blobs
    """
    import cv2
    import numpy as np
    # Convert to Gray
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # DETECTOR PARAMETERS
    params = cv2.SimpleBlobDetector_Params()
    # Change threshold
    params.minThreshold = 10;
    params.maxThreshold = 200;
    # Filter by Area
    params.filterByArea = True
    params.minArea = 1500
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    # Set detector with custom parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs
    keypoints = detector.detect(image)
    # Draw blobs
    image_with_blobs = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # Show blobs
    cv2.imshow('Blobs detected', cv2.resize(image_with_blobs, (int(1920/3.5), int(1080/3.5))))
    cv2.waitKey(4000)
    cv2.destroyAllWindows()
    return image_with_blobs


# Identify rectangles in image
def rectangle_detector(image):
    """
    Source: https://gist.github.com/anku255/03dc35c5233a3fc59d60fdf62c3cda24
    Source: https://gist.github.com/benmarwick/2b250d8ef3dbe36f817fbe2bf14aaa55
    Find rectangles in an image
    :param image: color image in 1920x1080 from kinect as array
    :return: image with found rectangle
    """
    import cv2
    import numpy as np
    _width = 1920.0
    _height = 1080.0
    _margin = 0.0
    corners = np.array([[[_margin, _margin]],
                        [[_margin, _height+_margin]],
                        [[_width+_margin, _height+_margin]],
                        [[_width+_margin, _margin]]])
    pts_dst = np.array(corners, np.float32)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 11, 17, 17)
    # gray = cv2.bilateralFilter(gray, 1, 10, 120)
    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    # edges = cv2.Canny(gray, 10, 250)
    kernel = np.ones((5, 5), np.uint8)
    erosion = cv2.erode(gray, kernel, iterations=2)
    kernel = np.ones((4, 4), np.uint8)
    dilation = cv2.dilate(erosion, kernel, iterations=2)
    edges = cv2.Canny(dilation, 30, 200)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    contours, h = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) > 5000:
            arc_len = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04*arc_len, True)
            if len(approx) == 4:
                pts_src = np.array(approx, np.float32)
                h, status = cv2.findHomography(pts_src, pts_dst)
                out = cv2.warpPerspective(image, h, (int(_width+_margin*2), int(_height+_margin*2)))
                cv2.drawContours(image, [approx], -1, (255, 0, 0), 2)
            else:
                pass

    cv2.imshow('edges', cv2.resize(edges, (int(1920/3.5), int(1080/3.5))))
    cv2.imshow('Rectangles', cv2.resize(image, (int(1920/3.5), int(1080/3.5))))
    try:
        cv2.imshow('out', cv2.resize(out, (int(1920/3.5), int(1080/3.5))))
    except Exception as e:
        print(f'[RECTANGLE DETECTOR]: {e}')
    cv2.waitKey(4000)
    cv2.destroyAllWindows()


# Find the 4x4 warp matrix that transforms an image to the 'bird view' given the width, height, axis rotation theta, out of plane phi, and vertical field of view
def rotation_transformation_matrix(width, height, theta, phi, vFOV):
    """
    Source: https://stackoverflow.com/questions/17087446/how-to-calculate-perspective-transform-for-opencv-from-rotation-angles
    M = F(width, height, theta, phi, vFOV)
    F = P*T*Rphi*Rtheta
    where:
        Rtheta = [[cos(theta), -sin(theta), 0, 0], [sin(theta), cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        Rphi = [[1, 0, 0, 0], [0, cos(phi), -sin(phi), 0], [0, sin(phi), cos(phi), 0], [0, 0, 0, 1]]
        T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -h], [0, 0, 0, 1]]
        P = [[cot(vFOV/2), 0, 0, 0], [0, cot(vFOV/2), 0, 0)], [0, 0, -(f+n)/(f-n), -2(f*n)/(f-n)], [0, 0, -1, 0]]

    :param width: fabric width ( in pixels )
    :param height: fabric height ( in pixels )
    :param theta: angle between the x,y axis of fabric and camera ( radians )
    :param phi: angle between the z axis of fabric and camera ( radians )
    :param v_FOV: Camera's color vertical field of view ( radians )
    :return: transformation matrix (F) to get 90 degrees 'TOP VIEW aka bird view'
    """
    # Import here for optimization
    import numpy as np
    d = np.sqrt((width*width+height*height))
    h = d / (2 * np.sin(vFOV/2))
    n = h-d/2
    f = h+d/2

    Rtheta = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                       [np.sin(theta), np.cos(theta), 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

    Rphi = np.array([[1, 0, 0, 0],
                     [0, np.cos(phi), -np.sin(phi), 0],
                     [0, np.sin(phi), np.cos(phi), 0],
                     [0, 0, 0, 1]])

    T = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, -h],
                  [0, 0, 0, 1]])
    P = np.array([[np.tan(np.pi/2 - vFOV/2), 0, 0, 0],
                  [0, np.tan(np.pi/2 - vFOV/2), 0, 0],
                  [0, 0, -(f+n)/(f-n), -2*(f*n)/(f-n)],
                  [0, 0, -1, 0]])
    F = np.dot(np.dot(np.dot(P, T), Rphi), Rtheta)
    return F


def box(image, color_focal_length):
    """
    Calculates fabric points by extracting background image.
    Check if background image is in Images
    :param image: Color Image from kinect (1920, 1080)
    :param color_focal_length: Focal length of RGB Camera
    :return: fabric points in color pixels, fabric width in color pixels, fabric height in color pixels, if fabric was found
    """
    # Import here for optimization
    import cv2
    import numpy as np
    fabric_ret = True  # flag for finding object
    # Find fabric
    subtraction = fabric_extraction(image)
    # Convert to Gray
    gray = cv2.cvtColor(subtraction, cv2.COLOR_BGR2GRAY)
    # Median Blur
    gray = cv2.medianBlur(gray, 5)
    # Mask background noise after subtraction
    gray[0:400, :] = 0
    gray[:, 0:500] = 0
    gray = np.where(gray > 35, gray, 0)
    cv2.waitKey()
    # Find Contours
    contours, hier = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    areas = [cv2.contourArea(c) for c in contours]  # find areas in contour
    max_index = np.argmax(areas)  # Find max area
    cnt = contours[max_index]  # save area

    # Calculate bounding rect for fabric
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    # Catch exception where no fabric is found
    if box.all() == 0:
        fabric_ret = False

    # Dimensions in pixels
    # height, width = rect[1]
    width = np.sqrt((box[0, 0] - box[1, 0]) ** 2 + (box[0, 1] - box[1, 1]) ** 2)  # in pixels
    height = np.sqrt((box[1, 0] - box[2, 0]) ** 2 + (box[1, 1] - box[2, 1]) ** 2)  # in pixels
    angle = rect[2]  # in degrees

    # Draw Dimensions
    if fabric_ret:
        cv2.putText(subtraction, 'Width: ' + str(width) + ' px', (int(box[1, 0] - 20 + width / 2), int(box[1, 1] + height + 40)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), lineType=cv2.LINE_AA)
        cv2.putText(subtraction, 'Height: ' + str(height) + ' px', (int(box[1, 0] - 20 + width / 2), int(box[1, 1] + height + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), lineType=cv2.LINE_AA)
        cv2.drawContours(subtraction, [box], 0, (0, 0, 255), 2)
        res_img = cv2.resize(subtraction, (int(1920 / 3.5), int(1080 / 3.5)))
        res_img = cv2.flip(res_img, 1)
        cv2.imshow('Fabric Dimensions', res_img)
        # Get perspective view transformed image
        perspective_transform(subtraction, box, color_focal_length)
        cv2.waitKey(1500)
        cv2.destroyAllWindows()

    return box, width, height, angle, fabric_ret


def box_hull(image, color_focal_length):
    """
    Calculates fabric points by extracting background image.
    Check if background image is in Images
    :param image: Color Image from kinect (1920, 1080)
    :param color_focal_length: Focal length of RGB Camera
    :return: fabric points in color pixels, fabric width in color pixels, fabric height in color pixels, if fabric was found
    """
    # Import here for optimization
    import cv2
    import numpy as np
    fabric_ret = True  # flag for finding object
    # cv2.imwrite('images/fab1.png', image)
    # Find fabric
    subtraction = fabric_extraction(image)
    # cv2.imwrite('images/fab2.png', subtraction)
    # Convert to Gray
    gray = cv2.cvtColor(subtraction, cv2.COLOR_BGR2GRAY)
    # cv2.imwrite('images/fab3.png', gray)
    # Median Blur
    gray = cv2.medianBlur(gray, 5)
    # cv2.imwrite('images/fab4.png', gray)
    # Mask background noise after subtraction
    gray[0:400, :] = 0
    gray[:, 0:500] = 0
    gray = np.where(gray > 35, gray, 0)
    cv2.waitKey()
    # cv2.imwrite('images/fab5.png', gray)
    # Find Contours
    contours, hier = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    areas = [cv2.contourArea(c) for c in contours]  # find areas in contour
    max_index = np.argmax(areas)  # Find max area
    cnt = contours[max_index]  # save area

    # approximate contour
    counter = 0
    coefficient = .05
    while True:
        epsilon = coefficient * cv2.arcLength(cnt, True)
        poly_approx = cv2.approxPolyDP(cnt, epsilon, True)
        hull = cv2.convexHull(poly_approx)
        if len(hull) == 4:
            break
        else:
            if len(hull) > 4:
                coefficient += .01
            else:
                coefficient -= .01
        if counter >= 30:
            break
        counter += 1

    points = np.ndarray(shape=(4, 2), dtype=np.int)
    for i in range(len(hull)):
        center = hull[i]
        points[i] = [center[0][0], center[0][1]]
        cv2.circle(subtraction, (center[0][0], center[0][1]), 25, (0, 255, 0))
    # ======================

    if points.all() == 0:
        fabric_ret = False

    # Dimensions in pixels
    width = np.sqrt((points[0, 0] - points[1, 0]) ** 2 + (points[0, 1] - points[1, 1]) ** 2)  # in pixels
    height = np.sqrt((points[1, 0] - points[2, 0]) ** 2 + (points[1, 1] - points[2, 1]) ** 2)  # in pixels
    # Calculate bounding rect for fabric
    rect = cv2.minAreaRect(cnt)
    angle = rect[2]  # in degrees

    # Draw Dimensions
    if fabric_ret:
        cv2.putText(subtraction, 'Width: ' + str(width) + ' px', (int(points[1, 0] - 20 + width / 2), int(points[1, 1] + height + 40)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), lineType=cv2.LINE_AA)
        cv2.putText(subtraction, 'Height: ' + str(height) + ' px', (int(points[1, 0] - 20 + width / 2), int(points[1, 1] + height + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), lineType=cv2.LINE_AA)
        cv2.drawContours(subtraction, [hull], 0, (0, 0, 255), 2)
        # cv2.imwrite('images/fab6.png', subtraction)
        res_img = cv2.resize(subtraction, (int(1920 / 3.5), int(1080 / 3.5)))
        res_img = cv2.flip(res_img, 1)
        cv2.imshow('Fabric Dimensions', res_img)
        # Get perspective view transformed image
        perspective_transform(subtraction, points, color_focal_length)
        cv2.waitKey(1500)
        cv2.destroyAllWindows()

    return points, width, height, angle, fabric_ret
