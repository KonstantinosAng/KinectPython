""" Handles the orientation of the operator's hand and the fabric's orientation """


def orientation(hand_right_points, wrist_right_points):
    """
    Calculate orientation of hand from axis based on the derivative of the line
    :param hand_right_points: list with operator's right hand coordinates in space
    :param wrist_right_points: list with operator's right wrist coordinates in space
    :return: rotation of hand in space theta_1, theta_2, theta_3
    """
    # Import libraries only for this function to minimize memory usage
    import numpy as np
    x_axis = [1, 0, 0]  # X axis vector
    y_axis = [0, 1, 0]  # Y axis vector
    z_axis = [0, 0, 1]  # Z axis vector
    line_v = [(hand_right_points[0] - wrist_right_points[0]),
              (hand_right_points[1] - wrist_right_points[1]),
              (hand_right_points[2] - wrist_right_points[2])]  # Hand line Vector
    rot_x = (np.arccos(np.dot(line_v, x_axis)/(np.linalg.norm(line_v)*np.linalg.norm(x_axis))))*(180/np.pi)  # in degrees
    rot_y = (np.arccos(np.dot(line_v, y_axis)/(np.linalg.norm(line_v)*np.linalg.norm(y_axis))))*(180/np.pi)  # in degrees
    rot_z = (np.arccos(np.dot(line_v, z_axis)/(np.linalg.norm(line_v)*np.linalg.norm(z_axis))))*(180/np.pi)  # in degrees
    if line_v[0] < 0:
        rot_x = -rot_x
    elif line_v[1] < 0:
        rot_y = -rot_y
    elif line_v[2] < 0:
        rot_z = -rot_z
    return rot_x, rot_y, rot_z


def fabric_orientation(fabric_points):
    """
    Calculate the edges of the fabric (2D) as vectors to find the orientation of the fabric compared to the human hand
    :param fabric_points: list with fabric edge points in space
    :return: fabric edge vertices, and center point
    """
    # Import libraries only for this function to minimize memory usage
    import numpy as np
    x_axis = [1, 0]  # X axis vector
    y_axis = [0, 1]  # Y axis vector
    b_line = [fabric_points[0][0] - fabric_points[1][0], fabric_points[0][1] - fabric_points[1][1]]  # Bottom line vector
    l_line = [fabric_points[1][0] - fabric_points[2][0], fabric_points[1][1] - fabric_points[2][1]]  # Left line vector
    t_line = [fabric_points[2][0] - fabric_points[3][0], fabric_points[2][1] - fabric_points[3][1]]  # Top line vector
    r_line = [fabric_points[3][0] - fabric_points[0][0], fabric_points[3][1] - fabric_points[0][1]]  # Right line vector
    b_rot_x = (np.arccos(np.dot(b_line, x_axis)/(np.linalg.norm(b_line)*np.linalg.norm(x_axis))))*(180/np.pi)  # in degrees
    b_rot_y = (np.arccos(np.dot(b_line, y_axis)/(np.linalg.norm(b_line)*np.linalg.norm(y_axis))))*(180/np.pi)  # in degrees
    l_rot_x = (np.arccos(np.dot(l_line, x_axis)/(np.linalg.norm(l_line)*np.linalg.norm(x_axis))))*(180/np.pi)  # in degrees
    l_rot_y = (np.arccos(np.dot(l_line, y_axis)/(np.linalg.norm(l_line)*np.linalg.norm(y_axis))))*(180/np.pi)  # in degrees
    t_rot_x = (np.arccos(np.dot(t_line, x_axis)/(np.linalg.norm(t_line)*np.linalg.norm(x_axis))))*(180/np.pi)  # in degrees
    t_rot_y = (np.arccos(np.dot(t_line, y_axis)/(np.linalg.norm(t_line)*np.linalg.norm(y_axis))))*(180/np.pi)  # in degrees
    r_rot_x = (np.arccos(np.dot(r_line, x_axis)/(np.linalg.norm(r_line)*np.linalg.norm(x_axis))))*(180/np.pi)  # in degrees
    r_rot_y = (np.arccos(np.dot(r_line, y_axis)/(np.linalg.norm(r_line)*np.linalg.norm(y_axis))))*(180/np.pi)  # in degrees
    fabric_edges = np.array([[b_rot_x, b_rot_y], [l_rot_x, l_rot_y], [t_rot_x, t_rot_y], [r_rot_x, r_rot_y]])
    # Calculate fabric center point
    xmin, ymin, zmin = fabric_points.min(axis=0)
    xmax, ymax, zmax = fabric_points.max(axis=0)
    fabric_center = np.array([(xmax+xmin)/2, (ymax+ymin)/2, (zmax+zmin)/2])
    return fabric_edges, fabric_center
