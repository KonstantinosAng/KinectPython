"""
Author: Konstantinos Angelopoulos
Date: 04/02/2020
All rights reserved.
Feel free to use and modify and if you like it give it a star.

DECISION MODEL TO CALCULATE THE HUMAN DESIRED MOVEMENT
"""


import numpy as np
"""

State Values, default = IDLE/OPEN

robot_state = 'IDLE', 'HUMAN', 'COOP'
gripper_state = 'CLOSED', 'OPEN', 'GRIPPED'
human_state = 'IDLE', 'MOVING', 'PREPARING', 'READY'

"""
# TOLERANCE OF HAND from fabric to decide robot corner and robot angle and human state and robot state
DISTANCE_TOLERANCE_X = 55  # mm
DISTANCE_TOLERANCE_Y = 90  # mm
MONITOR_DISTANCE_TOLERANCE_X = 70  # mm
MONITOR_DISTANCE_TOLERANCE_Y = 120  # mm
MONITOR_DISTANCE_TOLERANCE_Z = 100  # mm
ANGLE_TOLERANCE = 15.0  # degrees
MOVING_DISTANCE = 1.6  # mm
MOVING_DISTANCE_HAND = 0.5  # mm
FORCE_TOLERANCE = 2  # Newton
FORCE_MOVE = 1.5  # mm
MOVING_THRESHOLD_X = 47.0  # mm
MOVING_THRESHOLD_Y = 7.0  # mm
MOVING_THRESHOLD_Z = 47.0  # mm
FORCE_GAIN = 1.35


def robot_orientation(mirror_line, hand_points, wrist_points):
    """
    Calculate hand orientation with respect to mirror lines
    :param mirror_line:
    :param hand_points:  list with [x, y, z] coordinates of the operator's hand
    :param wrist_points:  list with [x, y, z] coordinates of the operator's wrist
    :return: list with the hand orientation angles with each axis [theta_x, theta_y, theta_z]
    """
    # Hand Vectors
    hand_vector = [(hand_points[0] - wrist_points[0]),
                   (hand_points[1] - wrist_points[1])]
    mirror_vector = [(mirror_line[0, 0] - mirror_line[1, 0]),
                     (mirror_line[0, 1] - mirror_line[1, 1])]
    hand_slope = hand_vector[1] / hand_vector[0]
    mirror_slope = mirror_vector[1] / mirror_vector[0]
    human_rot = np.arctan((hand_slope-mirror_slope)/(1+hand_slope*mirror_slope))*(180/np.pi)  # in degrees
    # human_rot = (np.arccos(np.dot(hand_vector, mirror_vector)/(np.linalg.norm(hand_vector)*np.linalg.norm(mirror_vector))))*(180/np.pi)  # in degrees
    return human_rot


def mirror(fabric):
    """
    Calculate mirror lines of robot with reference to fabric
    :param fabric: fabric world points
    :return:  list with horizontal and vertical axis vectors
    """
    # Find horizontal mirror
    right = np.array([(fabric[0, 0] + fabric[3, 0])/2, (fabric[0, 1] + fabric[3, 1])/2])
    left = np.array([(fabric[1, 0] + fabric[2, 0])/2, (fabric[1, 1] + fabric[2, 1])/2])
    h_line = np.array([[right[0], right[1]],
                       [left[0], left[1]]])
    # Find vertical mirror
    bottom = np.array([(fabric[0, 0] + fabric[1, 0])/2, (fabric[0, 1] + fabric[1, 1])/2])
    top = np.array([(fabric[2, 0] + fabric[3, 0])/2, (fabric[2, 1] + fabric[3, 1])/2])
    v_line = np.array([[bottom[0], bottom[1]],
                       [top[0], top[1]]])
    return h_line, v_line


def close_edges(world_robot_position_base, fabric_points, kinect_distance, RDK):
    """
    Calculate which fabric edges are closer to robot reach
    The origin of all reference frames is the kinect's reference frame where z is the depth (y in RoboDK) and y is the height(z in RoboDK)
    :param world_robot_position_base:  list with the robot's base coordinations [x, y, z]
    :param fabric_points:  fabric world points
    :param kinect_distance:  distance of kinect from the fabric corneres
    :param RDK:  connection to the RoboDK instance
    :return:  list with the two closer edges to the robot
    """
    import robotSim
    """
    fabric_points = [[BR], [BL], [TL], [TR]]
    TODO: calculate edges closer to robot reach
    :return: closer_edges = [edge1, edge2]
    """
    # Find distances of fabric from robot
    distances = []
    for i in range(len(fabric_points)):
        distances[i] = np.sqrt((world_robot_position_base[i] - fabric_points[i][0])**2 + (world_robot_position_base[i] - fabric_points[i][1])**2)
    # Sort list from low to high value
    sorted_distances = sorted(distances)
    # Find if there are duplicates in list (same distance) and their index
    if len(distances) != len(set(distances)):
        if distances[0] == distances[1]:
            duplicates_id = [0, 1]
        elif distances[0] == distances[2]:
            duplicates_id = [0, 2]
        elif distances[0] == distances[3]:
            duplicates_id = [0, 3]
        elif distances[1] == distances[2]:
            duplicates_id = [1, 2]
        elif distances[1] == distances[3]:
            duplicates_id = [1, 3]
        elif distances[2] == distances[3]:
            duplicates_id = [2, 3]
        duplicates = True
    else:
        duplicates = False
    # Solve for exception of two distances being the same
    # Find where the width and height corresponds ( robot only grabs the closest edges, but they have to correspond to the biggest dimension of fabric)
    """
        if dimension1 is bigger it means that the right and left dimensions of fabric are bigger than the bottom and top and vice versa.
            Simple Examples (Might be rotated, or not exactly rectangle, due to less accurate fabric detection:
                                           
            dimension1 > dimension2:    +-------+           dimension1 < dimension2:    +-----------------------+                
                                        |       |                                       |                       |
                                        |       |                                       |                       |
                                        |       |                                       |                       |
                                        |       |                                       +-----------------------+
                                        |       |
                                        +-------+
    """
    dimension1 = np.sqrt((fabric_points[0][0] - fabric_points[3][0])**2 + (fabric_points[0][1] - fabric_points[3][1])**2)
    dimension2 = np.sqrt((fabric_points[0][0] - fabric_points[1][0])**2 + (fabric_points[0][1] - fabric_points[1][1])**2)
    # Find bigger dimension
    if dimension1 > dimension2:
        bigger_dimension = 'LR'
    elif dimension1 < dimension2:
        bigger_dimension = 'BT'
    else:
        bigger_dimension = 'EQUAL'

    if not duplicates:
        # Find what the corners are if there are no duplicates
        for i in range(len(distances)):
            if sorted_distances[0] == distances[i]:
                first_edge = i
            if sorted_distances[1] == distances[i]:
                second_edge = i
    else:
        for i in range(len(distances)):
            if sorted_distances[0] == distances[i]:
                first_edge = i
        # If there are two equal distances chose the one with the less joint movement
        target1 = [fabric_points[duplicates_id[0]][0], fabric_points[duplicates_id[0]][1], kinect_distance]
        target2 = [fabric_points[duplicates_id[1]][0], fabric_points[duplicates_id[1]][1], kinect_distance]
        angles1 = robotSim.target_joints(RDK, target1)
        angles2 = robotSim.target_joints(RDK, target2)
        if sum(angles1)/len(angles1) < sum(angles2)/len(angles2):
            second_edge = duplicates_id[0]
        else:
            second_edge = duplicates_id[1]

    # Return corners
    if first_edge == 0:
        edge1 = 'BR'
    elif first_edge == 1:
        edge1 = 'BL'
    elif first_edge == 2:
        edge1 = 'TL'
    else:
        edge1 = 'TR'

    if second_edge == 0:
        edge2 = 'BR'
    elif second_edge == 1:
        edge2 = 'BL'
    elif second_edge == 2:
        edge2 = 'TL'
    else:
        edge2 = 'TR'

    return [edge1, edge2]


def distance(points, counter):
    """
    Calculate distance between points in space
    :param points: list with points in space [[x, y, z], .... [x, y, z]]
    :param counter: counter to desired list position
    :return: distance from previous point in space
    """
    return np.sqrt((points[counter][0] - points[counter-1][0])**2 + (points[counter][1] - points[counter-1][1])**2 + (points[counter][2] - points[counter-1][2])**2)


def distance_x_axis(points, counter):
    """
    Calculate distance between points in the x axis
    :param points: list with points in space [[x, y, z], .... [x, y, z]]
    :param counter: counter to desired list position
    :return: distance from previous point in space
    """
    return np.sqrt((points[counter][0] - points[counter-1][0])**2)


def distance_y_axis(points, counter):
    """
    Calculate distance between points in the y axis
    :param points: list with points in space [[x, y, z], .... [x, y, z]]
    :param counter: counter to desired list position
    :return: distance from previous point in space
    """
    return np.sqrt((points[counter][1] - points[counter-1][1])**2)


def distance_z_axis(points, counter):
    """
    Calculate distance between points in the z axis
    :param points: list with points in space [[x, y, z], .... [x, y, z]]
    :param counter: counter to desired list position
    :return: distance from previous point in space
    """
    return np.sqrt((points[counter][2] - points[counter-1][2])**2)


# Calculate euclidean distance of two points in space
def euclidean_distance(p1, p2):
    """
    Calculate distance between points in space
    :param p1: list with x y z coordinates of a point
    :param p2: list with x y z coordinates of a point
    :return: distance from previous point in space
    """
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)


# Make decision for Robot Move
def decision(kalman_points, counter, fabric_points, hand_right_confidence, hand_right_state):
    """
    Update the human state
    :param kalman_points: list with kalman world points [[x, y, z], ..., [x, y, z]]
    :param counter: pointer to a point position
    :param fabric_points: list with fabric points
    :param hand_right_confidence: string with kinect's confidence in tracking the hand
    :param hand_right_state: operator's hand right state
    :return: human state
    """
    # Import libraries only for this function to minimize memory usage
    if counter > 0:
        if distance(kalman_points, counter) >= MOVING_DISTANCE:
            # If the distance of previous point is greater than 1.6 mm then the hand is moving
            human_state = 'MOVING'

        else:
            # If the distance of previous point is less than 1.6 mm then the hand is moving
            human_state = 'IDLE'

        for i in range(4):
            if (np.fabs(kalman_points[counter][0] - fabric_points[i][0]) <= DISTANCE_TOLERANCE_X) and (np.fabs(kalman_points[counter][1] - fabric_points[i][1]) <= DISTANCE_TOLERANCE_Y):
                if hand_right_state == 'CLOSED':
                    # If the hand is close to one of the fabrics corner and the tracking is with confidence and the hand is closed (holding the fabric)
                    # then the human is ready
                    human_state = 'READY'
                else:
                    # If the hand is close to one of the fabrics corner and the tracking is with confidence but the hand is not closed (not holding the fabric)
                    # then the human is preparing for handling
                    human_state = 'PREPARING'
                    #########################
                    # For testing
                    #human_state = 'READY'
                    #########################
    else:
        # If counter = 0, use default value of IDLE
        human_state = 'IDLE'

    return human_state


"""
                  Mirror Line                                                                    [TL][L]        Mirror Line      [TR][T]
      [TL] [L]         •     [R] [TR]                 # [#] is the orientation       Example:    [Robot]  ↓          •           [Robot]
        [T] +----------¦----------+ [T]               # [##] is the corner                                +----------¦----------+ ←
            |          •          |                                                                       |          •          |  
            |          ¦          |                                                                       |          ¦          |
      •-•-•-•-•-•-•-•-•••-•-•-•-•-•-•-•-•     ROBOT MUST MIRROR THE OPERATOR'S HAND   ~~~~~~~~>     •-•-•-•-•-•-•-•-•••-•-•-•-•-•-•-•-•         Mirror Line
            |          ¦          |                                                                       |          ¦          |   ( for projection of hand to robot hand )
            |          •          |                                                                       |          •          |
        [B] +----------¦----------+ [B]                                                                   +----------¦----------+ ←
      [BL] [L]         •    [R] [BR]                                                         [Human Hand] ↑          •           [Human Hand]
                                                                                               [BL][L]                            [BR][B]
"""


# Calculate robot movement using kalman values
def coordination(kalman_points, counter, fabric_points, rot_x, rot_y, fabric_orientation):
    """
    Find the human and robot starting and stopping corner
    :param kalman_points: list with kalman world points [[x, y, z], ..., [x, y, z]]
    :param counter: pointer to a point position
    :param fabric_points: list with fabric points
    :param rot_x: rotation of hand in the x axis
    :param rot_y: rotation of hand in the y axis
    :param fabric_orientation: fabric orientation on the world coordinate system
    :return: human and robot starting and stopping corners
    """
    # Find what corner is grabbed ( We can scale the units for better precision )
    for i in range(4):
        if (np.fabs(kalman_points[counter][0] - fabric_points[i][0]) <= DISTANCE_TOLERANCE_X) and (np.fabs(kalman_points[counter][1] - fabric_points[i][1]) <= DISTANCE_TOLERANCE_Y):
            if i == 0:
                corner = 'BR'  # Bottom Right
                break
            elif i == 1:
                corner = 'BL'  # Bottom Left
                break
            elif i == 2:
                corner = 'TL'  # Top Left
                break
            else:
                corner = 'TR'  # Top Right
                break

    # We need to combine this with the orientation of the hand to be able to calculate the gripping point for the robot
    if (np.fabs(rot_x - fabric_orientation[0, 0]) <= ANGLE_TOLERANCE) and (np.fabs(rot_y - fabric_orientation[0, 1]) <= ANGLE_TOLERANCE):
        direction = 'B'  # Bottom
    elif (np.fabs(rot_x - fabric_orientation[1, 0]) <= ANGLE_TOLERANCE) and (np.fabs(rot_y - fabric_orientation[1, 1]) <= ANGLE_TOLERANCE):
        direction = 'L'  # Left
    elif (np.fabs(rot_x - fabric_orientation[2, 0]) <= ANGLE_TOLERANCE) and (np.fabs(rot_y - fabric_orientation[2, 1]) <= ANGLE_TOLERANCE):
        direction = 'T'  # Top
    elif (np.fabs(rot_x - fabric_orientation[3, 0]) <= ANGLE_TOLERANCE) and (np.fabs(rot_y - fabric_orientation[3, 1]) <= ANGLE_TOLERANCE):
        direction = 'R'  # Right
    # Calculate the rotation if its not entirely parallel to fabric edge
    elif np.fabs(rot_x - fabric_orientation[0, 0]) <= np.fabs(rot_y - fabric_orientation[0, 1]):
        direction = 'RR'  # Right to Right
    elif np.fabs(rot_x - fabric_orientation[0, 0]) > np.fabs(rot_y - fabric_orientation[0, 1]):
        direction = 'RL'  # Right to Left
    elif np.fabs(rot_x - fabric_orientation[1, 0]) <= np.fabs(rot_y - fabric_orientation[1, 1]):
        direction = 'LR'  # Left to Right
    elif np.fabs(rot_x - fabric_orientation[1, 0]) > np.fabs(rot_y - fabric_orientation[1, 1]):
        direction = 'LL'  # Left to Left
    elif np.fabs(rot_x - fabric_orientation[2, 0]) <= np.fabs(rot_y - fabric_orientation[2, 1]):
        direction = 'LL'  # Left to Left
    elif np.fabs(rot_x - fabric_orientation[2, 0]) > np.fabs(rot_y - fabric_orientation[2, 1]):
        direction = 'LR'  # Left to Right
    elif np.fabs(rot_x - fabric_orientation[3, 0]) <= np.fabs(rot_y - fabric_orientation[3, 1]):
        direction = 'RL'  # Right to Left
    elif np.fabs(rot_x - fabric_orientation[3, 0]) <= np.fabs(rot_y - fabric_orientation[3, 1]):
        direction = 'RR'  # Right to Right
    # Calculate robot gripping point
    # Keep in mind that the robot has to mirror the position of the human hand, thus the direction of Top and Bottom ('T' and 'B') stays the same
    # but the direction Left and Right ('L' and 'R') have to be in the opposite direction so that the robot faces the human hand as a mirror
    if corner == 'BR':
        robot_corner = 'TR'
        if direction == 'B':
            robot_direction = 'T'
        else:
            robot_direction = 'R'
    elif corner == 'BL':
        robot_corner = 'TL'
        if direction == 'B':
            robot_direction = 'T'
        else:
            robot_direction = 'L'
    elif corner == 'TL':
        robot_corner = 'BL'
        if direction == 'T':
            robot_direction = 'B'
        else:
            robot_direction = 'L'
    elif corner == 'TR':
        robot_corner = 'BR'
        if direction == 'T':
            robot_direction = 'B'
        else:
            robot_direction = 'R'
    # Find remaining corners that will be the stop corners
    remaining_corners = []
    sum_of_corners = ['BR', 'BL', 'TL', 'TR']
    for c in sum_of_corners:
        if c == corner or c == robot_corner:
            continue
        remaining_corners.append(c)
    # Find what corners belong to each
    for c in remaining_corners:
        if c[0] == corner[0] or c[1] == corner[1]:
            corner_stop = c
        if c[0] == robot_corner[0] or c[1] == robot_corner[1]:
            robot_corner_stop = c

    return corner, direction, robot_corner, robot_direction, corner_stop, robot_corner_stop


# Updated the coordination function with new strategy
def corners(kalman_world_hand_right, kalman_world_wrist_right, kalman_world_hand_tip_right, fabric_points):
    """
    Find the human and robot starting and stopping corner
    :param kalman_world_hand_right: list with kalman world points [[x, y, z], ..., [x, y, z]] of the operator's hand
    :param kalman_world_wrist_right: list with kalman world points [[x, y, z], ..., [x, y, z]] of the operator's wrist
    :param kalman_world_hand_tip_right: list with kalman world points [[x, y, z], ..., [x, y, z]] of the operator's hand tip
    :param fabric_points: list with fabric points
    :return: human and robot starting and stopping corners
    """
    for i in range(4):
        if (np.fabs(kalman_world_hand_tip_right[0] - fabric_points[i][0]) <= DISTANCE_TOLERANCE_X) and (np.fabs(kalman_world_hand_tip_right[1] - fabric_points[i][1]) <= DISTANCE_TOLERANCE_Y):
            if i == 0:
                human_corner = 'BR'  # Bottom Right
                break
            elif i == 1:
                human_corner = 'BL'  # Bottom Left
                break
            elif i == 2:
                human_corner = 'TL'  # Top Left
                break
            else:
                human_corner = 'TR'  # Top Right
                break
    hand_line = [kalman_world_hand_right[0] - kalman_world_wrist_right[0], kalman_world_hand_right[1] - kalman_world_wrist_right[1]]
    if human_corner == 'BR':
        fabric_line = [fabric_points[1][0] - fabric_points[0][0], fabric_points[1][1] - fabric_points[0][1]]
        angle = (np.arccos(np.dot(fabric_line, hand_line) / (np.linalg.norm(fabric_line) * np.linalg.norm(hand_line)))) * (180 / np.pi)
        if -30 <= angle <= 60.0:
            robot_corner = 'BL'
            human_stop_corner = 'TR'
        else:
            robot_corner = 'TR'
            human_stop_corner = 'BL'
        robot_stop_corner = 'TL'
    elif human_corner == 'BL':
        fabric_line = [fabric_points[0][0] - fabric_points[1][0], fabric_points[0][1] - fabric_points[1][1]]
        angle = (np.arccos(np.dot(fabric_line, hand_line) / (np.linalg.norm(fabric_line) * np.linalg.norm(hand_line)))) * (180 / np.pi)
        if -30.0 <= angle <= 60.0:
            robot_corner = 'BR'
            human_stop_corner = 'TL'
        else:
            robot_corner = 'TL'
            human_stop_corner = 'BR'
        robot_stop_corner = 'TR'
    elif human_corner == 'TL':
        fabric_line = [fabric_points[3][0] - fabric_points[2][0], fabric_points[3][1] - fabric_points[2][1]]
        angle = (np.arccos(np.dot(fabric_line, hand_line) / (np.linalg.norm(fabric_line) * np.linalg.norm(hand_line)))) * (180 / np.pi)
        if -30.0 <= angle <= 60.0:
            robot_corner = 'TR'
            human_stop_corner = 'BL'
        else:
            robot_corner = 'BL'
            human_stop_corner = 'TR'
        robot_stop_corner = 'BR'
    else:
        fabric_line = [fabric_points[2][0] - fabric_points[3][0], fabric_points[2][1] - fabric_points[3][1]]
        angle = (np.arccos(np.dot(fabric_line, hand_line) / (np.linalg.norm(fabric_line) * np.linalg.norm(hand_line)))) * (180 / np.pi)
        if -30.0 <= angle <= 60.0:
            robot_corner = 'TL'
            human_stop_corner = 'BR'
        else:
            robot_corner = 'BR'
            human_stop_corner = 'TL'
        robot_stop_corner = 'BL'

    #####################################
    # testing only
    # robot_corner = 'TL'
    # human_stop_corner = 'BR'
    # robot_stop_corner = 'BL'
    ####################################

    return human_corner, human_stop_corner, robot_corner, robot_stop_corner


# Take into consideration the force feedback from sensor
def forces_consideration(fx, fy, fz):
    """
    When the operator wants to "drag/pull" the robot to a certain direction he must use the
    force/torque sensor feedback values.
    FORCE FEEDBACK CONTROL
    :param fx: force value in x axis
    :param fy: force  value in y axis
    :param fz: force  value in z axis
    :return x, y, z force feedback
    """
    if np.abs(fx) >= FORCE_TOLERANCE:
        if fx > 0:
            z = -fx*FORCE_GAIN
        else:
            z = fx*FORCE_GAIN
    else:
        z = 0
    if np.abs(fy) >= FORCE_TOLERANCE:
        if fy > 0:
            x = -fy*FORCE_GAIN
        else:
            x = fy*FORCE_GAIN
    else:
        x = 0
    if np.abs(fz) >= FORCE_TOLERANCE:
        if fz > 0:
            y = -fz*FORCE_GAIN
        else:
            y = fz*FORCE_GAIN
    else:
        y = 0
    return x, y, z


# Decide when the manipulation is done and make the robot drop the fabric
def monitor(fabric_points, hand_pos, human_corner_stop, robot_corner_stop):
    """
    fabric_points = ["BR", "BL", "TL", "TR"]
    Monitor if human has reached the stopping corner to stop the collaboration
    :param fabric_points: list with fabric points
    :param hand_pos: list with operator's hand position
    :param human_corner_stop: human stop corner by string ID
    :param robot_corner_stop: robot stop corner by string ID
    :return Boolean
    """
    # Find human stop corner by String ID
    if human_corner_stop == 'BR':
        h_stop = fabric_points[0]
    elif human_corner_stop == 'BL':
        h_stop = fabric_points[1]
    elif human_corner_stop == 'TL':
        h_stop = fabric_points[2]
    else:
        h_stop = fabric_points[3]
    """
    # Find robot stop corner by String ID
    if robot_corner_stop == 'BR':
        r_stop = fabric_points[0]
    elif robot_corner_stop == 'BL':
        r_stop = fabric_points[1]
    elif robot_corner_stop == 'TL':
        r_stop = fabric_points[2]
    else:
        r_stop = fabric_points[3]
    """
    # print("[DECISION] {} {} {}".format("%.3f" % hand_pos[0], "%.3f" % h_stop[0], human_corner_stop))
    print("[DECISION] {} {} {}".format("%.3f" % np.fabs(hand_pos[0] - h_stop[0]), "%.3f" % np.fabs(hand_pos[1] - h_stop[1]), "%3.f" % np.fabs(hand_pos[2] - h_stop[2])))
    if np.fabs(hand_pos[0] - h_stop[0]) <= MONITOR_DISTANCE_TOLERANCE_X and np.fabs(hand_pos[1] - h_stop[1]) <= MONITOR_DISTANCE_TOLERANCE_Y and np.fabs(hand_pos[2] - h_stop[2]) <= MONITOR_DISTANCE_TOLERANCE_Z:
        return True
    return False


def collaborate(kalman_world_hand_right, counter, human_corner, human_corner_stop, hand_right_confidence):
    """
    Move robot according to operator's hand
    :param kalman_world_hand_right: list with kalman world hand points [[x, y, z], ..., [x, y, z]]
    :param counter: pointer to the last point
    :param human_corner: human starting corner by string ID
    :param human_corner_stop: human stopping corner by string ID
    :param hand_right_confidence: kinect's hand right confidence in tracking
    :return: list with distance moved in each axis [dx, dy, dz]
    """
    if counter > 0:
        if distance_x_axis(kalman_world_hand_right, counter) > MOVING_DISTANCE_HAND:
            if (kalman_world_hand_right[counter][0] - kalman_world_hand_right[counter - 1][0]) > 0:
                x = 5.0
            else:
                x = -5.0
        else:
            x = 0
        if distance_y_axis(kalman_world_hand_right, counter) > MOVING_DISTANCE_HAND:
            if (kalman_world_hand_right[counter][1] - kalman_world_hand_right[counter - 1][1]) > 0:
                y = 2
            else:
                y = -1.1
        else:
            y = 0
        if distance_z_axis(kalman_world_hand_right, counter) > MOVING_DISTANCE_HAND:
            if (kalman_world_hand_right[counter][2] - kalman_world_hand_right[counter - 1][2]) > 0:
                z = 5.0
            else:
                z = -5.0
        else:
            z = 0

        # Ensure smoothness on path
        if human_corner == 'BR':
            if human_corner_stop == 'TR':
                z *= 3
                x = x/2
            else:
                z = z/2
                x *= 3
        elif human_corner == 'BL':
            if human_corner_stop == 'TL':
                z *= 3
                x = x/2
            else:
                z = z/2
                x *= 3
        elif human_corner == 'TL':
            if human_corner_stop == 'BL':
                z *= 3
                x /= 2
            else:
                z /= 2
                x *= 3
        else:
            if human_corner_stop == 'BR':
                z *= 3
                x /= 2
            else:
                z /= 2
                x *= 3

        if hand_right_confidence == "HIGH":
            return x, y, z

    return 0, 0, 0


def _collaborate(kalman_world_hand_right, counter, human_corner, human_corner_stop, hand_right_confidence):
    """

    :param kalman_world_hand_right: list with kalman world hand points [[x, y, z], ..., [x, y, z]]
    :param counter: pointer to the last point
    :param human_corner: human starting corner by string ID
    :param human_corner_stop: human stopping corner by string ID
    :param hand_right_confidence: kinect's hand right confidence in tracking
    :return: list with distance moved in each axis [dx, dy, dz]
    """
    if counter > 0:
        if distance_x_axis(kalman_world_hand_right, counter) > MOVING_DISTANCE_HAND:
            x = (kalman_world_hand_right[counter][0] - kalman_world_hand_right[counter - 1][0])
            if np.abs(x) > MOVING_THRESHOLD_X:
                if x > 0:
                    x = MOVING_THRESHOLD_X
                else:
                    x = -MOVING_THRESHOLD_X
        else:
            x = 0
        if distance_y_axis(kalman_world_hand_right, counter) > MOVING_DISTANCE_HAND:
            y = (kalman_world_hand_right[counter][1] - kalman_world_hand_right[counter - 1][1])
            if np.abs(y) > MOVING_THRESHOLD_Y:
                if y > 0:
                    y = MOVING_THRESHOLD_Y
                else:
                    y = -MOVING_THRESHOLD_Y/2
        else:
            y = 0
        if distance_z_axis(kalman_world_hand_right, counter) > MOVING_DISTANCE_HAND:
            z = (kalman_world_hand_right[counter][2] - kalman_world_hand_right[counter - 1][2])
            if np.abs(z) > MOVING_THRESHOLD_Z:
                if z > 0:
                    z = MOVING_THRESHOLD_Z
                else:
                    z = -MOVING_THRESHOLD_Z
        else:
            z = 0

        if hand_right_confidence == 'HIGH':
            return x, y, z

    return 0.0, 0.0, 0.


# Function to plot fabric human position and robot gripping point
# Only for testing to see if code works
def visualize(kalman_points, counter, fabric_points, world_wrist_right, human_corner, human_direction, robot_corner, robot_direction):
    """
    Plot fabric corners, human starting and stopping corners.
    :param kalman_points: list with kalman hand points in world coordinates
    :param counter: pointer to last kalman world point
    :param fabric_points: list world fabric points
    :param world_wrist_right: list with kalman world wrist points
    :param human_corner: string with human_corner
    :param human_direction: string with human moving direction
    :param robot_corner: string with robot starting corner
    :param robot_direction: string with robot moving direction
    :return: None
    """
    # Import libraries only for this function to minimize memory usage
    from matplotlib import pyplot as plt
    import numpy as np
    # Plot only kalman_points and fabric_points
    # Use robot_direction, human_direction, human_corner and robot_corner to see if its correct
    # If correct start defining the robot movement and plot it too
    h_line, v_line = mirror(fabric_points)
    human_rotation = robot_orientation(h_line, kalman_points[counter], world_wrist_right[counter])
    rect = fabric_points
    rect = np.append(rect, [[fabric_points[0, 0], fabric_points[0, 1], fabric_points[0, 2]]], axis=0)

    fig, ax = plt.subplots(1)

    # Plot fabric as box and Human hand
    ax.plot(rect[:, 0], rect[:, 1], 'b', kalman_points[counter][0], kalman_points[counter][1], 'ro', linewidth=3)
    ax.plot(world_wrist_right[counter][0], world_wrist_right[counter][0], 'ro', linewidth=3)

    # Find Coordinates of Robot Corner
    if robot_corner == 'BR':
        robot_points = np.array([fabric_points[0, 0], fabric_points[0, 1]])
    elif robot_corner == 'BL':
        robot_points = np.array([fabric_points[1, 0], fabric_points[1, 1]])
    elif robot_corner == 'TL':
        robot_points = np.array([fabric_points[2, 0], fabric_points[2, 1]])
    else:
        robot_points = np.array([fabric_points[3, 0], fabric_points[3, 1]])

    # Plot Robot Corner
    ax.plot(robot_points[0], robot_points[1], 'go', linewidth=3)

    # Plot Mirror Line
    ax.plot(h_line[:, 0], h_line[:, 1], 'm--', linewidth=3)
    ax.plot(v_line[:, 0], v_line[:, 1], 'm--', linewidth=3)

    # Figure Details
    ax.title.set_text('Plot Human Robot Space')
    fig.text(0.12, 0.04, 'Human corner: ' + human_corner, ha='center', fontweight='bold', color='red')  # Human Corner
    fig.text(0.37, 0.04, 'Human direction: ' + human_direction, ha='center', fontweight='bold', color='red')  # Human Direction
    fig.text(0.62, 0.04, 'Robot corner: ' + robot_corner, ha='center', fontweight='bold', color='green')  # Robot Corner
    fig.text(0.85, 0.04, 'Robot direction: ' + robot_direction, ha='center', fontweight='bold', color='green')  # Robot Direction
    fig.text(0.40, 0.10, 'Human Rotation: ' + str(human_rotation), ha='center', fontweight='bold', color='red')  # Human Rotation
    fig.text(0.40, 0.20, 'Robot Rotation: ' + str(human_rotation), ha='center', fontweight='bold', color='green')  # Robot Rotation

    plt.grid(True)
    plt.show()
