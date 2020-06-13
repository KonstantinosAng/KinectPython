# Offset of camera center from the origin of the coordinate system of kinect
OFFSET_RGB = 95  # mm
OFFSET_DEPTH = 65  # mm
CLOUD_RESET_RADIUS = 100  # mm
PARTICLE_RADIUS = 30  # mm
# KUKA LWR IV+ Joint angle limits
ORIGINAL_LOWER_LIMITS = [-170, -30, -170, -120, -170, -120, -170]
ORIGINAL_UPPER_LIMITS = [170,  210, 170,  120,  170,  120, 170]
MODIFIED_LOWER_LIMITS = [-170, -30, -170, -170, -170, -170, -170]
MODIFIED_UPPER_LIMITS = [170,  210, 170,  170,  170, 170, 170]


def init(speed, focal_length, field_of_view, fabric_center, fabric_points, fabric_width, fabric_height, world_kalman_points, kinect_distance_from_plane, kinect_tilt, check_col=True):
    """
    Reset and initialize RoboDK workspace
    :param speed: float robot iso speed
    :param focal_length: float kinect focal length
    :param field_of_view: float kinect field of view
    :param fabric_center: list with fabric center coordinates
    :param fabric_points: list with fabric edge point coordinates
    :param fabric_width: float fabric width
    :param fabric_height: float fabric height
    :param world_kalman_points: list with world kalman coordinates
    :param kinect_distance_from_plane: float kinect height from floor plane
    :param kinect_tilt: float kinect tilt angle from floor plane
    :param check_col: boolean to check for collisions
    :return: link to the RoboDK instance, Item robot in robodk, Item pose, Item hand, Item kinect, Item gripper, Item bottom right, Item bottom left, Item top left, Item top right
    """
    # Import libraries here for optimization
    from robolink.robolink import Robolink, ITEM_TYPE_ROBOT
    from robodk.robodk import KUKA_2_Pose, Pose_2_KUKA, transl, rotx
    import sys
    import numpy as np
    import os

    # Interactions with Robot
    RDK = Robolink()
    # Turn off automatic rendering (faster simulation)
    RDK.Render(False)

    # Initialize robot
    # Search all items for a valid robot type (assumes only one robot is loaded)
    robot = None
    for station in RDK.ItemList():
        for item in station.Childs():
            if item.type == ITEM_TYPE_ROBOT:
                robot = item
                break
            if robot is not None:
                break

    if not robot.Valid():
        print("[ROBODK]: No robot is loaded to the station")
        raise Exception("[ROBODK]: No Robot found!!")

    # Check Home, Targets and Items Loaded
    home = RDK.Item('Home')  # Home
    pose = RDK.Item('Target')  # Target
    sheet = RDK.Item('Sheet')  # Sheet item
    br = RDK.Item('Bottom Right')  # Bottom Right Corner of Fabric
    bl = RDK.Item('Bottom Left')  # Bottom Left Corner of Fabric
    tl = RDK.Item('Top Left')  # Top Left Corner of Fabric
    tr = RDK.Item('Top Right')  # Top Right Corner of Fabric
    kinect = RDK.Item('Kinect')  # Get Kinect
    hand = RDK.Item('Hand')  # Get Hand model
    gripper = RDK.Item('Gripper')  # Get Gripper model
    light = RDK.Item('Light Source')  # Get Light Source

    # Check if everything is loaded correctly
    if home.type == -1 or pose.type == -1 or sheet.type == -1 or bl.type == -1 or tl.type == -1 or tr.type == -1 or br.type == -1 or kinect.type == -1 or hand.type == -1 or gripper.type == -1 or light.type == -1:
        sys.exit('[ROBODK]: Cannot obtain simulation items: Check if everything is loaded')

    # Always start from home
    try:
        # Move to Home
        robot.setJoints([0, 90, 0, 0, 0, 0])
    except Exception as e:
        print(f'[ROBODK]: Cant Move Robot Home\n{e}')

    # Initialize Kinect
    # Check Kinect Tilt angle
    temp_kinect = Pose_2_KUKA(kinect.Pose())
    kinect.setPose(KUKA_2_Pose(list((temp_kinect[0], temp_kinect[1], temp_kinect[2], temp_kinect[3], temp_kinect[4], -90 + kinect_tilt))))
    # Check kinect height
    actual_height = Pose_2_KUKA(kinect.Pose())
    if (actual_height[2] - kinect_distance_from_plane) <= 50:
        # kinect.setPose(KUKA_2_Pose(list((690, -1770, kinect_distance_from_plane, 0, 0, -90 + kinect_tilt))))
        kinect.setPose(KUKA_2_Pose(list((temp_kinect[0], temp_kinect[1], temp_kinect[2], temp_kinect[3], temp_kinect[4], -90 + kinect_tilt))))
    else:
        # kinect.setPose(KUKA_2_Pose(list((690, -1770, 2150-kinect_distance_from_plane/100, 0, 0, -90 + kinect_tilt))))
        kinect.setPose(KUKA_2_Pose(list((temp_kinect[0], temp_kinect[1], temp_kinect[2], temp_kinect[3], temp_kinect[4], -90 + kinect_tilt))))

    # Reset Sheet
    sheet.Delete()
    sheet = RDK.AddFile(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'Models/Sheet.stl'))
    sheet.setPose(KUKA_2_Pose(list((635, 0, 770, 90, 0, 0))))
    sheet.setColor([255/255, 170/255, 127/255, 1])

    # Reset Fabric Points
    """!---------Targets are with respect to the Robot Base Reference so its better to use setPoseAbs() to keep everything in the universal coordinate system---------!"""
    br.setPoseAbs(KUKA_2_Pose(list((785, -250, 770, 45, -180, -15))))
    bl.setPoseAbs(KUKA_2_Pose(list((485, -250, 770, -45, 180, -15))))
    tl.setPoseAbs(KUKA_2_Pose(list((485, 250, 770, 45, 0, 180-15))))
    tr.setPoseAbs(KUKA_2_Pose(list((785, 250, 770, -45, 0, 180-15))))

    # Reset Hand
    hand.setPose(KUKA_2_Pose(list((500, 1000, 1400, -90, 0, 0))))

    # Reset Target
    pose.setPose(KUKA_2_Pose(list((600, 250, 750, 0, 0, -90))))

    # It is important to provide the reference frame and the tool frames when generating programs offline
    robot.setPoseFrame(robot.PoseFrame())
    robot.setPoseTool(robot.PoseTool())

    # Define Light Source
    light.setPoseAbs(KUKA_2_Pose(list((1000, 0, 2500, 0, 205, 0))))

    # Define Camera
    # Close all cameras just in case
    RDK.Cam2D_Close()
    # Pick which items will act as the camera
    # camref = RDK.ItemUserPick('Select the camera')

    # Use the GLSL files for more realistic camera view from inside the RoboDK software
    file_shader_fragment = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RoboDK/KUKA/Camera-Shaders/shader_fragment.glsl')
    file_shader_vertex = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RoboDK/KUKA/Camera-Shaders/shader_vertex.glsl')

    # Add Camera
    """
    # -------------Set parameters in mm and degrees-------------------------
    #  FOV: Field of view in degrees (atan(0.5*height/distance) of the sensor
    #  FOCAL_LENGTH: focal length in mm
    #  FAR_LENGTH: maximum working distance (in mm)
    #  SIZE: size of the window in pixels (fixed) (width x height)
    #  SNAPSHOT: size of the snapshot image in pixels (width x height)
    #  BG_COLOR: background color (rgb color or named color: AARRGGBB)
    #  LIGHT_AMBIENT: ambient color (rgb color or named color: AARRGGBB)
    #  LIGHT_SPECULAR: specular color (rgb color or named color: AARRGGBB)
    #  LIGHT_DIFFUSE: diffuse color (rgb color or named color: AARRGGBB)
    #  DEPTH: Add this flag to create a 32 bit depth map (white=close, black=far)
    #  NO_TASKBAR: Don't add the window to the task bar
    #  MINIMIZED: Show the window minimized
    #  ALWAYS_VISIBLE: Keep the window on top of all other windows
    #  SHADER_VERTEX: File to a vertex shader (GLSL file)
    #  SHADER_FRAGMENT: File to a fragment shader (GLSL file)
    """
    # cam_id = RDK.Cam2D_Add(kinect, 'FOCAL_LENGTH=' + str(focal_length) + ' FOV=' + str(field_of_view) + ' FAR_LENGTH=4500 SIZE=548x308 BG_COLOR=black LIGHT_AMBIENT=red LIGHT_DIFFUSE=black LIGHT_SPECULAR=white ALWAYS_VISIBLE')
    # RDK.Cam2D_Add(kinect, 'FOCAL_LENGTH=' + str(focal_length) + ' FOV=' + str(field_of_view) + ' FAR_LENGTH=4500 SIZE=548x308 BG_COLOR=black LIGHT_AMBIENT=red LIGHT_DIFFUSE=black LIGHT_SPECULAR=white TASKBAR ALWAYS_VISIBLE')
    RDK.Cam2D_Add(kinect, 'FOCAL_LENGTH=' + str(focal_length) + ' FOV=' + str(field_of_view) + ' FAR_LENGTH=4500 SIZE=512x424 BG_COLOR=black SHADER_FRAGMENT=' + file_shader_fragment + ' SHADER_VERTEX=' + file_shader_vertex + ' MINIMIZED')
    # Special command to retrieve the window ID:
    # win_id = RDK.Command("CamWinID", str(cam_id))
    # Take snapshot
    # RDK.Cam2D_Snapshot(RDK.getParam('PATH_OPENSTATION') + "/Kinect_snapshot.png", cam_id)

    """
    !------------------THE POSE OF AN ITEM/OBJECT/TARGET IS WITH RESPECT TO ITS PARENT------------------!
    """
    # Define fabric origins
    kinect_pos = Pose_2_KUKA(kinect.Pose())

    # Initialise hand origin as detected from kinect camera
    # hand.setPose(KUKA_2_Pose(list((kinect_pos[0] - world_kalman_points[0], kinect_pos[1] + world_kalman_points[2], kinect_pos[2] + world_kalman_points[1] + np.sin(kinect_tilt*np.pi/180)*world_kalman_points[2], -90, 0, 0))))
    hand_temp = Pose_2_KUKA(kinect.Pose()*transl(-world_kalman_points[0] + OFFSET_DEPTH, -world_kalman_points[1], world_kalman_points[2]))
    hand.setPose(KUKA_2_Pose(list((hand_temp[0], hand_temp[1], hand_temp[2], -90, 0, 0))))
    # Original Position = (660, 240, 426, 90, 0, 0)
    # sheet.setPose(KUKA_2_Pose(list((kinect_pos[0] - fabric_center[0], kinect_pos[1] + fabric_center[2], kinect_pos[2] + fabric_center[1] + np.sin(kinect_tilt*np.pi/180)*fabric_center[2], 90, 0, 0))))
    sheet_temp = Pose_2_KUKA(kinect.Pose()*transl(-fabric_center[0] + OFFSET_DEPTH, -fabric_center[1], fabric_center[2]))
    sheet.setPose(KUKA_2_Pose(list((sheet_temp[0], sheet_temp[1], sheet_temp[2], 90, 0, 0))))
    # sheet.setPose(KUKA_2_Pose(list((kinect_pos[0] - fabric_center[0], kinect_pos[1] + fabric_center[2], kinect_pos[2] + fabric_center[1], 90, 0, 0))))
    # Scale to match measurements
    # Sheet CAD: thickness (Z axis) = 0.1 mm, Length (Y axis) = 500 mm, Width (X Axis) = 300 mm
    # sheet.Scale(scale_uniform)
    # Calculate scale
    scale_x = fabric_height/500
    scale_y = fabric_width/300
    scale_z = 1
    sheet.Scale([scale_x, scale_y, scale_z])  # Scale with different factor in each direction

    # Define Fabric Corners
    """!---------Targets are with respect to the Robot Base Reference so its better to use setPoseAbs() to keep everything in the universal coordinate system---------!"""

    """
    br.setPoseAbs(KUKA_2_Pose(list((kinect_pos[0] - fabric_points[0, 0], kinect_pos[1] + fabric_points[0, 2], kinect_pos[2] + fabric_points[0, 1] + np.sin(kinect_tilt*np.pi/180)*fabric_points[0, 2], 0, -180, 0))))
    bl.setPoseAbs(KUKA_2_Pose(list((kinect_pos[0] - fabric_points[1, 0], kinect_pos[1] + fabric_points[1, 2], kinect_pos[2] + fabric_points[1, 1] + np.sin(kinect_tilt*np.pi/180)*fabric_points[1, 2], -90, 180, 0))))
    tl.setPoseAbs(KUKA_2_Pose(list((kinect_pos[0] - fabric_points[2, 0], kinect_pos[1] + fabric_points[2, 2], kinect_pos[2] + fabric_points[2, 1] + np.sin(kinect_tilt*np.pi/180)*fabric_points[2, 2], 0, 0, 180))))
    tr.setPoseAbs(KUKA_2_Pose(list((kinect_pos[0] - fabric_points[3, 0], kinect_pos[1] + fabric_points[3, 2], kinect_pos[2] + fabric_points[3, 1] + np.sin(kinect_tilt*np.pi/180)*fabric_points[3, 2], -90, 0, 180))))
    """

    br_temp = Pose_2_KUKA(kinect.Pose()*transl(-fabric_points[0, 0] + OFFSET_DEPTH, -fabric_points[0, 1], fabric_points[0, 2]))
    bl_temp = Pose_2_KUKA(kinect.Pose()*transl(-fabric_points[1, 0] + OFFSET_DEPTH, -fabric_points[1, 1], fabric_points[1, 2]))
    tl_temp = Pose_2_KUKA(kinect.Pose()*transl(-fabric_points[2, 0] + OFFSET_DEPTH, -fabric_points[2, 1], fabric_points[2, 2]))
    tr_temp = Pose_2_KUKA(kinect.Pose()*transl(-fabric_points[3, 0] + OFFSET_DEPTH, -fabric_points[3, 1], fabric_points[3, 2]))
    br.setPoseAbs(KUKA_2_Pose(list((br_temp[0], br_temp[1], sheet_temp[2], 45, -180,0))))
    bl.setPoseAbs(KUKA_2_Pose(list((bl_temp[0], bl_temp[1], sheet_temp[2], -45, 180, 0))))
    tl.setPoseAbs(KUKA_2_Pose(list((tl_temp[0], tl_temp[1], sheet_temp[2], 45, 0, 180))))
    tr.setPoseAbs(KUKA_2_Pose(list((tr_temp[0], tr_temp[1], sheet_temp[2], -45, 0, 180))))

    # enable render and speed up simulation (big data for neural)
    RDK.Render(True)
    RDK.setSimulationSpeed(5)  # x faster (5 default)
    robot.setZoneData(3)  # Set the rounding parameter (Also known as: CNT, APO/C_DIS, ZoneData, Blending radius, cornering, ...)
    RDK.ShowRoboDK()  # Show RoboDK window
    RDK.setWindowState(2)  # Show RoboDK window
    """
        # Script execution types
        RUNMODE_SIMULATE = 1                      # performs the simulation moving the robot (default)
        RUNMODE_QUICKVALIDATE = 2                 # performs a quick check to validate the robot movements
        RUNMODE_MAKE_ROBOTPROG = 3                # makes the robot program
        RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD = 4     # makes the robot program and updates it to the robot
        RUNMODE_MAKE_ROBOTPROG_AND_START = 5      # makes the robot program and starts it on the robot (independently from the PC)
        RUNMODE_RUN_ROBOT = 6                     # moves the real robot from the PC (PC is the client, the robot behaves like a server)
    """
    RDK.setRunMode(1)  # Simulate only
    # robot.setSpeed(speed_linear=speed*1000, speed_joints=110.0, accel_linear=-1, accel_joints=-1)  # Set linear speed in mm/s (-1 = no change)
    robot.setSpeed(speed_linear=(0.8*speed)*1000, speed_joints=-1, accel_linear=-1, accel_joints=-1)  # Set linear speed in mm/s (-1 = no change)
    # robot.setSpeedJoints(-1, 110.0, -1, -1)  # Set joint speed in deg/s for rotary joints and mm/s for linear joints
    # robot.setAcceleration()  # Set linear acceleration in mm/s2
    # robot.setAccelerationJoints()  # Set joint acceleration in deg/s2 for rotary joints and mm/s2 for linear joints
    # robot.setParamRobotTool(tool_mass=5, tool_cog=None)  # set tool_mass in kg as float, tool_cog = list(x, y, z)  tool center of gravity as [x,y,z] with respect to the robot flange
    if check_col:
        RDK.setCollisionActive(check_state=1)  # Enable collision check
        RDK.Command('CollisionMethod', value='CUDA')
    RDK.Command('DisplayCurves', value='0')
    # RDK.Command('SetSize3D', value='650x610')
    # RDK.Command('Trace', value='On')
    # RDK.setFlagsRoboDK(flags=[FLAG_ROBODK_TREE_ACTIVE])
    RDK.setViewPose(KUKA_2_Pose(list((-635, -500, -6000, 0, 0, -40))))

    return RDK, robot, pose, hand, kinect, gripper, br, bl, tl, tr


def _init(speed, focal_length, field_of_view, fabric_center, fabric_points, fabric_width, fabric_height, world_kalman_points, kinect_distance_from_plane, kinect_tilt, check_col=True):
    """
    Reset and initialize RoboDK workspace
    :param speed: float robot speed
    :param focal_length: float kinect color focal length
    :param field_of_view: float kinect color field of view
    :param fabric_center: list with fabric ceter coordinates
    :param fabric_points: list with fabric edge points
    :param fabric_width: float fabric width
    :param fabric_height: float fabric height
    :param world_kalman_points: list with world kalman points
    :param kinect_distance_from_plane: float kinect height from floor
    :param kinect_tilt: float kinect tilt from floor plane
    :param check_col: boolean to check collisions
    :return: Item robot in RoboDK, Item pose in RoboDK, Item hand in RoboDK, Item kinect in RoboDK, Item gripper in RoboDK, Item bottom right, Item bottom left, Item top left, Item top right
    """
    # Import libraries here for optimization
    from robolink.robolink import Robolink, ITEM_TYPE_ROBOT
    from robodk.robodk import KUKA_2_Pose, Pose_2_KUKA, transl, rotx
    import sys
    import numpy as np
    import os

    # Interactions with Robot
    RDK = Robolink()
    # Turn off automatic rendering (faster simulation)
    RDK.Render(False)

    # Initialize robot
    # Search all items for a valid robot type (assumes only one robot is loaded)
    robot = None
    for station in RDK.ItemList():
        for item in station.Childs():
            if item.type == ITEM_TYPE_ROBOT:
                robot = item
                break
            if robot is not None:
                break

    if not robot.Valid():
        print("[ROBODK]: No robot is loaded to the station")
        raise Exception("[ROBODK]: No Robot found!!")

    # Check Home, Targets and Items Loaded
    home = RDK.Item('Home')  # Home
    pose = RDK.Item('Target')  # Target
    sheet = RDK.Item('Sheet')  # Sheet item
    br = RDK.Item('Bottom Right')  # Bottom Right Corner of Fabric
    bl = RDK.Item('Bottom Left')  # Bottom Left Corner of Fabric
    tl = RDK.Item('Top Left')  # Top Left Corner of Fabric
    tr = RDK.Item('Top Right')  # Top Right Corner of Fabric
    kinect = RDK.Item('Kinect')  # Get Kinect
    hand = RDK.Item('Hand')  # Get Hand model
    gripper = RDK.Item('Gripper')  # Get Gripper model
    light = RDK.Item('Light Source')  # Get Light Source

    # Check if everything is loaded correctly
    if home.type == -1 or pose.type == -1 or sheet.type == -1 or bl.type == -1 or tl.type == -1 or tr.type == -1 or br.type == -1 or kinect.type == -1 or hand.type == -1 or gripper.type == -1 or light.type == -1:
        sys.exit('[ROBODK]: Cannot obtain simulation items: Check if everything is loaded')

    # Initialize Kinect
    # Check Kinect Tilt angle
    temp_kinect = Pose_2_KUKA(kinect.Pose())
    kinect.setPoseAbs(KUKA_2_Pose(list((temp_kinect[0], temp_kinect[1], temp_kinect[2], temp_kinect[3], temp_kinect[4], -90 + kinect_tilt))))
    # Check kinect height
    actual_height = Pose_2_KUKA(kinect.Pose())
    if (actual_height[2] - kinect_distance_from_plane) <= 50:
        # kinect.setPose(KUKA_2_Pose(list((690, -1770, kinect_distance_from_plane, 0, 0, -90 + kinect_tilt))))
        kinect.setPoseAbs(KUKA_2_Pose(list((temp_kinect[0], temp_kinect[1], temp_kinect[2], temp_kinect[3], temp_kinect[4], -90 + kinect_tilt))))
    else:
        # kinect.setPose(KUKA_2_Pose(list((690, -1770, 2150-kinect_distance_from_plane/100, 0, 0, -90 + kinect_tilt))))
        kinect.setPoseAbs(KUKA_2_Pose(list((temp_kinect[0], temp_kinect[1], temp_kinect[2], temp_kinect[3], temp_kinect[4], -90 + kinect_tilt))))

    # Define Camera
    # Use the GLSL files for more realistic camera view from inside the RoboDK software
    file_shader_fragment = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RoboDK/KUKA/Camera-Shaders/shader_fragment.glsl')
    file_shader_vertex = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RoboDK/KUKA/Camera-Shaders/shader_vertex.glsl')

    # Add Camera
    """
    # -------------Set parameters in mm and degrees-------------------------
    #  FOV: Field of view in degrees (atan(0.5*height/distance) of the sensor
    #  FOCAL_LENGTH: focal length in mm
    #  FAR_LENGTH: maximum working distance (in mm)
    #  SIZE: size of the window in pixels (fixed) (width x height)
    #  SNAPSHOT: size of the snapshot image in pixels (width x height)
    #  BG_COLOR: background color (rgb color or named color: AARRGGBB)
    #  LIGHT_AMBIENT: ambient color (rgb color or named color: AARRGGBB)
    #  LIGHT_SPECULAR: specular color (rgb color or named color: AARRGGBB)
    #  LIGHT_DIFFUSE: diffuse color (rgb color or named color: AARRGGBB)
    #  DEPTH: Add this flag to create a 32 bit depth map (white=close, black=far)
    #  NO_TASKBAR: Don't add the window to the task bar
    #  MINIMIZED: Show the window minimized
    #  ALWAYS_VISIBLE: Keep the window on top of all other windows
    #  SHADER_VERTEX: File to a vertex shader (GLSL file)
    #  SHADER_FRAGMENT: File to a fragment shader (GLSL file)
    """
    # cam_id = RDK.Cam2D_Add(kinect, 'FOCAL_LENGTH=' + str(focal_length) + ' FOV=' + str(field_of_view) + ' FAR_LENGTH=4500 SIZE=548x308 BG_COLOR=black LIGHT_AMBIENT=red LIGHT_DIFFUSE=black LIGHT_SPECULAR=white ALWAYS_VISIBLE')
    # RDK.Cam2D_Add(kinect, 'FOCAL_LENGTH=' + str(focal_length) + ' FOV=' + str(field_of_view) + ' FAR_LENGTH=4500 SIZE=548x308 BG_COLOR=black LIGHT_AMBIENT=red LIGHT_DIFFUSE=black LIGHT_SPECULAR=white TASKBAR ALWAYS_VISIBLE')
    RDK.Cam2D_Add(kinect, 'FOCAL_LENGTH=' + str(focal_length) + ' FOV=' + str(field_of_view) + ' FAR_LENGTH=4500 SIZE=512x424 BG_COLOR=black SHADER_FRAGMENT=' + file_shader_fragment + ' SHADER_VERTEX=' + file_shader_vertex + ' MINIMIZED')
    # Special command to retrieve the window ID:
    # win_id = RDK.Command("CamWinID", str(cam_id))
    # Take snapshot
    # RDK.Cam2D_Snapshot(RDK.getParam('PATH_OPENSTATION') + "/Kinect_snapshot.png", cam_id)

    """
    !------------------THE POSE OF AN ITEM/OBJECT/TARGET IS WITH RESPECT TO ITS PARENT------------------!
    """
    # Define fabric origins

    # Initialise hand origin as detected from kinect camera
    hand_temp = Pose_2_KUKA(kinect.Pose() * transl(-world_kalman_points[0] + OFFSET_DEPTH, -world_kalman_points[1], world_kalman_points[2]))
    hand.setPoseAbs(KUKA_2_Pose(list((hand_temp[0], hand_temp[1], hand_temp[2], -90, 0, 0))))
    # Original Position = (660, 240, 426, 90, 0, 0)
    sheet_temp = Pose_2_KUKA(kinect.Pose() * transl(-fabric_center[0] + OFFSET_DEPTH, -fabric_center[1], fabric_center[2]))
    sheet.setPoseAbs(KUKA_2_Pose(list((sheet_temp[0], sheet_temp[1], sheet_temp[2], 90, 0, 0))))
    # Scale to match measurements
    # Sheet CAD: thickness (Z axis) = 0.1 mm, Length (Y axis) = 500 mm, Width (X Axis) = 300 mm
    # sheet.Scale(scale_uniform)
    # Calculate scale
    scale_x = fabric_height / 500
    scale_y = fabric_width / 300
    scale_z = 1
    sheet.Scale([scale_x, scale_y, scale_z])  # Scale with different factor in each direction

    # Define Fabric Corners
    br_temp = Pose_2_KUKA(kinect.Pose() * transl(-fabric_points[0, 0] + OFFSET_DEPTH, -fabric_points[0, 1], fabric_points[0, 2]))
    bl_temp = Pose_2_KUKA(kinect.Pose() * transl(-fabric_points[1, 0] + OFFSET_DEPTH, -fabric_points[1, 1], fabric_points[1, 2]))
    tl_temp = Pose_2_KUKA(kinect.Pose() * transl(-fabric_points[2, 0] + OFFSET_DEPTH, -fabric_points[2, 1], fabric_points[2, 2]))
    tr_temp = Pose_2_KUKA(kinect.Pose() * transl(-fabric_points[3, 0] + OFFSET_DEPTH, -fabric_points[3, 1], fabric_points[3, 2]))
    br.setPoseAbs(KUKA_2_Pose(list((br_temp[0], br_temp[1], sheet_temp[2], 45, -180, 0))))
    bl.setPoseAbs(KUKA_2_Pose(list((bl_temp[0], bl_temp[1], sheet_temp[2], -45, 180, 0))))
    tl.setPoseAbs(KUKA_2_Pose(list((tl_temp[0], tl_temp[1], sheet_temp[2], 45, 0, 180))))
    tr.setPoseAbs(KUKA_2_Pose(list((tr_temp[0], tr_temp[1], sheet_temp[2], -45, 0, 180))))

    # enable render and speed up simulation (big data for neural)
    RDK.Render(True)
    """
        # Script execution types
        RUNMODE_SIMULATE = 1                      # performs the simulation moving the robot (default)
        RUNMODE_QUICKVALIDATE = 2                 # performs a quick check to validate the robot movements
        RUNMODE_MAKE_ROBOTPROG = 3                # makes the robot program
        RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD = 4     # makes the robot program and updates it to the robot
        RUNMODE_MAKE_ROBOTPROG_AND_START = 5      # makes the robot program and starts it on the robot (independently from the PC)
        RUNMODE_RUN_ROBOT = 6                     # moves the real robot from the PC (PC is the client, the robot behaves like a server)
    """
    # robot.setSpeed(speed_linear=speed*1000, speed_joints=110.0, accel_linear=-1, accel_joints=-1)  # Set linear speed in mm/s (-1 = no change)
    robot.setSpeed(speed_linear=(0.8 * speed) * 1000, speed_joints=-1, accel_linear=-1, accel_joints=-1)  # Set linear speed in mm/s (-1 = no change)
    # robot.setSpeedJoints(-1, 110.0, -1, -1)  # Set joint speed in deg/s for rotary joints and mm/s for linear joints
    # robot.setAcceleration()  # Set linear acceleration in mm/s2
    # robot.setAccelerationJoints()  # Set joint acceleration in deg/s2 for rotary joints and mm/s2 for linear joints
    # robot.setParamRobotTool(tool_mass=5, tool_cog=None)  # set tool_mass in kg as float, tool_cog = list(x, y, z)  tool center of gravity as [x,y,z] with respect to the robot flange
    if check_col:
        RDK.setCollisionActive(check_state=1)  # Enable collision check
        RDK.Command('CollisionMethod', value='CUDA')
    # RDK.Command('SetSize3D', value='650x610')
    # RDK.Command('Trace', value='On')
    # RDK.setFlagsRoboDK(flags=[FLAG_ROBODK_TREE_ACTIVE])

    return robot, pose, hand, kinect, gripper, br, bl, tl, tr


def reset(robot_ip, port, simulation=True):
    """
    Resets the RoboDK's Items position
    :param robot_ip: string IP of the robot controller
    :param port: integer port of the robot controller
    :param simulation: boolean to enable simulation or move the real robot
    :return: RDK: Connection to the RoboDK application
    """
    # Import libraries here for optimization
    from robolink.robolink import ITEM_TYPE_ROBOT, Robolink
    from robodk.robodk import KUKA_2_Pose
    import sys
    import os
    RDK = Robolink()
    # Turn off automatic rendering (faster simulation)
    RDK.Render(False)

    # Initialize robot
    # Search all items for a valid robot type (assumes only one robot is loaded)
    robot = None
    for station in RDK.ItemList():
        for item in station.Childs():
            if item.type == ITEM_TYPE_ROBOT:
                robot = item
                break
            if robot is not None:
                break

    if not robot.Valid():
        print("[ROBODK]: No robot is loaded to the station")
        raise Exception("[ROBODK]: No Robot found!!")

    # Check Home, Targets and Items Loaded
    pose = RDK.Item('Target')  # Target
    sheet = RDK.Item('Sheet')  # Sheet item
    br = RDK.Item('Bottom Right')  # Bottom Right Corner of Fabric
    bl = RDK.Item('Bottom Left')  # Bottom Left Corner of Fabric
    tl = RDK.Item('Top Left')  # Top Left Corner of Fabric
    tr = RDK.Item('Top Right')  # Top Right Corner of Fabric
    hand = RDK.Item('Hand')  # Get Hand model
    light = RDK.Item('Light Source')  # Get Light Source

    # Check if everything is loaded correctly
    if pose.type == -1 or sheet.type == -1 or bl.type == -1 or tl.type == -1 or tr.type == -1 or br.type == -1 or hand.type == -1 or light.type == -1:
        sys.exit('[ROBODK]: Cannot obtain simulation items: Check if everything is loaded')

    # Always start from home
    try:
        # Move to Home
        robot.setJoints([0, 90, 0, 0, 0, 0])
    except Exception as e:
        print(f'[ROBODK]: Cant Move Robot Home\n{e}')

    # Reset Sheet
    sheet.Delete()
    sheet = RDK.AddFile(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'Models/Sheet.stl'))
    sheet.setPoseAbs(KUKA_2_Pose(list((635, 0, 770, 90, 0, 0))))
    sheet.setColor([255 / 255, 170 / 255, 127 / 255, 1])
    # Reset Fabric Points
    """!---------Targets are with respect to the Robot Base Reference so its better to use setPoseAbs() to keep everything in the universal coordinate system---------!"""
    br.setPoseAbs(KUKA_2_Pose(list((785, -250, 770, 45, -180, 0))))
    bl.setPoseAbs(KUKA_2_Pose(list((485, -250, 770, -45, 180, 0))))
    tl.setPoseAbs(KUKA_2_Pose(list((485, 250, 770, 45, 0, 180))))
    tr.setPoseAbs(KUKA_2_Pose(list((785, 250, 770, -45, 0, 180))))
    # Reset Hand
    hand.setPoseAbs(KUKA_2_Pose(list((500, 1000, 1400, -90, 0, 0))))
    # Reset Target
    pose.setPoseAbs(KUKA_2_Pose(list((600, 250, 770, 0, 0, -90))))
    # It is important to provide the reference frame and the tool frames when generating programs offline
    robot.setPoseFrame(robot.PoseFrame())
    robot.setPoseTool(robot.PoseTool())
    # Define Light Source
    light.setPoseAbs(KUKA_2_Pose(list((1000, 0, 2500, 0, 205, 0))))
    # Close all cameras just in case
    RDK.Cam2D_Close()
    # enable render and speed up simulation (big data for neural)
    RDK.Render(True)
    RDK.setSimulationSpeed(5)  # x faster (5 default)
    robot.setZoneData(3)  # Set the rounding parameter (Also known as: CNT, APO/C_DIS, ZoneData, Blending radius, cornering, ...)
    RDK.ShowRoboDK()  # Show RoboDK window
    RDK.setWindowState(2)  # Show RoboDK window
    """
        # Script execution types
        RUNMODE_SIMULATE = 1                      # performs the simulation moving the robot (default)
        RUNMODE_QUICKVALIDATE = 2                 # performs a quick check to validate the robot movements
        RUNMODE_MAKE_ROBOTPROG = 3                # makes the robot program
        RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD = 4     # makes the robot program and updates it to the robot
        RUNMODE_MAKE_ROBOTPROG_AND_START = 5      # makes the robot program and starts it on the robot (independently from the PC)
        RUNMODE_RUN_ROBOT = 6                     # moves the real robot from the PC (PC is the client, the robot behaves like a server)
    """
    if simulation:
        RDK.setRunMode(1)  # simulate only
    else:
        RDK.setRunMode(6)  # move real robot and simulation
        connect(robot_ip, port, robot)
    # robot.setSpeed(speed_linear=speed*1000, speed_joints=110.0, accel_linear=-1, accel_joints=-1)  # Set linear speed in mm/s (-1 = no change)
    # robot.setSpeedJoints(-1, 110.0, -1, -1)  # Set joint speed in deg/s for rotary joints and mm/s for linear joints
    # robot.setAcceleration()  # Set linear acceleration in mm/s2
    # robot.setAccelerationJoints()  # Set joint acceleration in deg/s2 for rotary joints and mm/s2 for linear joints
    # robot.setParamRobotTool(tool_mass=5, tool_cog=None)  # set tool_mass in kg as float, tool_cog = list(x, y, z)  tool center of gravity as [x,y,z] with respect to the robot flange
    RDK.Command('DisplayCurves', value='0')
    # RDK.Command('SetSize3D', value='650x610')
    # RDK.Command('Trace', value='On')
    # RDK.setFlagsRoboDK(flags=[FLAG_ROBODK_TREE_ACTIVE])
    RDK.setViewPose(KUKA_2_Pose(list((-635, -500, -6000, 0, 0, -40))))
    return RDK


def connect(ip, port, robot):
    """
    Make connection with robot
    :param ip: string IP of the robot controller
    :param port: port of the robot controller
    :param robot: Item robot in RoboDK
    :return: None
    """
    try:
        # Set the connection parameters
        robot.setConnectionParams(ip, port, '/', 'anonymous', '')
        # Connect to the robot, 0 = failed, 1 = success
        success = robot.Connect()
        #  Check connection
        if success == 0:
            print('[ROBODK]: Failed to Connect to Robot')
            robot.Disconnect()
        else:
            print('[ROBODK]: Connection Successful')
            # It is important to provide the reference frame and the tool frames when generating programs offline
            robot.setPoseFrame(robot.PoseFrame())
            robot.setPoseTool(robot.PoseTool())

        status, status_msg = robot.ConnectedState()
        # Robot connection status
        # ROBOTCOM_PROBLEMS = -3
        # ROBOTCOM_DISCONNECTED = -2
        # ROBOTCOM_NOT_CONNECTED = -1
        # ROBOTCOM_READY = 0
        # ROBOTCOM_WORKING = 1
        # ROBOTCOM_WAITING = 2
        # ROBOTCOM_UNKNOWN = -1000
        # Print Robot status
        print("[ROBODK]: Robot Status: " + status_msg)
    except Exception as e:
        print(f'[ROBODK]: {e}')


def target_joints(RDK, target):
    """
    Move robot to a Target and get the joints
    :param RDK: link to the RDK instance
    :param target: Item target in RoboDk
    :return: list with robot angles [J1, J2, J3, J4, J5, J6, J7]
    """
    from robolink.robolink import ITEM_TYPE_ROBOT
    from robodk.robodk import tr, transl
    # Faster Calculations ( Robot not moving )
    RDK.Render(False)
    RDK.setRunMode(1)  # Simulate only
    # Initialize robot
    # Search all items for a valid robot type (assumes only one robot is loaded)
    robot = None
    for station in RDK.ItemList():
        for item in station.Childs():
            if item.type == ITEM_TYPE_ROBOT:
                robot = item
                break
            if robot is not None:
                break
    if not robot.Valid():
        print("[ROBODK]: No robot is loaded to the station")
        raise Exception("[ROBODK]: No Robot found!!")
    # Calculate pose
    pose = robot.Pose()*transl(target[0], target[1], target[2])
    # Find angles
    angles = robot.SolveIK(pose, robot.Joints(), robot.PoseTool(), robot.PoseFrame())
    RDK.Render(True)
    return angles


def simulation(RDK, x, y, z, rot_x, rot_y, rot_z, robot, pose, hand_model, hand_state, gripper_model, gripper_state, waitGripper, kinect_model, kinect_tilt, world_kalman_hand_tip_right, human_state, robot_corner, b_right, b_left, t_left, t_right, startSim, check_col=True):
    """
    Update robot, hand, gripper, fabric, kinect and cloud skeleton
    :param RDK: link to RoboDK instance
    :param x: float hand distance in x axis
    :param y: float hand distance in y axis
    :param z: float hand distance in z axis
    :param rot_x: float hand angle in x axis
    :param rot_y: float hand angle in y axis
    :param rot_z: float hand angle in z axis
    :param robot: Item robot in RoboDK
    :param pose: Item target in RoboDK
    :param hand_model: Item hand in RoboDK
    :param hand_state: string hand state
    :param gripper_model: Item gripper in RoboDK
    :param gripper_state: string gripper state
    :param waitGripper: boolean to wait for gripper to grab fabric
    :param kinect_model: Item kinect in RoboDK
    :param kinect_tilt: float angle of tiltness of Kinect from floor plane
    :param world_kalman_hand_tip_right: list with world kalman points of the operator's hand tip
    :param human_state: string with human state
    :param robot_corner: string with robot's starting corner
    :param b_right: Item bottom right in RoboDK
    :param b_left: Item bottom left in RoboDK
    :param t_left: Item top left in RoboDK
    :param t_right: Item top right in RoboDK
    :param startSim: boolean to start simulation after fabric is grabbed
    :param check_col: boolean to check for collisions
    :return: string with robot_state, boolean that simulation started, boolean to wait for gripper to grab the fabric
    """
    # Import libraries only for this function to minimize memory usage
    from robodk.robodk import transl, rotx, roty, rotz, TxyzRxyz_2_Pose, Pose_2_TxyzRxyz, KUKA_2_Pose, Pose_2_KUKA, tr
    import numpy as np
    import os

    RDK.Render(False)  # Faster Calculations
    """
    if check_col:
        # Check for current collisions
        check_collisions = RDK.Collisions()
        # if collision then move to home
        if check_collisions != 0:
            print("Collision detected. Moving to Home Position")
            # robot.setJoints([0, 90, 0, 0, 0, 0])
    """
    # Move Robot to Target
    # joints = robot.SolveIK(x, y, z)
    # robot.setJoints([joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]])
    # Calculate Robot Pose to move to (x, y, z = mm, rotations = radians
    # rot_x, rot_y, rot_z = rot_x * np.pi/180, rot_y * np.pi/180, rot_z * np.pi/18
    # pose = robot.Pose()*transl(int(x), int(y), int(z))*rotx(int(rot_x))*roty(int(rot_y))*rotz(int(rot_z))
    kinect_pos = Pose_2_KUKA(kinect_model.Pose())
    hand_tar = Pose_2_KUKA(hand_model.Pose())
    # tar = Pose_2_KUKA(pose.Pose())
    # tar = list((tar[0] + int(x), tar[1] + int(z), tar[2] + int(y), int(rot_x), int(rot_z), int(rot_y)))
    # tar = list((tar[0] + int(x), tar[1] + int(y), tar[2] + int(z), int(rot_y), int(rot_x), int(rot_z)))
    # tar = list((tar[0] - int(x), tar[1] + int(z), tar[2] + int(y), int(rot_z*180/np.pi), int(rot_y*180/np.pi), int(rot_x*180/np.pi)))
    # tar = list((tar[0] - int(x), tar[1] + int(z), tar[2] + int(y), rot_x, tar[4], tar[5]))
    # tar = list((tar[0] - int(x), tar[1] + int(z), tar[2] + int(y), int(rot_z), tar[4], int(rot_y)))
    # tar = list((tar[0] - int(x), tar[1] + int(z), tar[2] + int(y), int(rot_z), int(rot_x), int(rot_y)))
    # tar[4] could be the wrist rotation from quaternions

    # pose = TxyzRxyz_2_Pose([int(x)/2, int(y)/2, int(z)/2, 0, 0, 0])
    # pose = robot.Pose().setPos([int(x), int(y), int(z)])

    # Update hand pose in simulation Space
    # hand_model.setPose(KUKA_2_Pose(list((kinect_pos[0] - int(world_kalman_hand_tip_right[0]), kinect_pos[1] + int(world_kalman_hand_tip_right[2]), kinect_pos[2] + int(world_kalman_hand_tip_right[1]) + np.sin(kinect_tilt*np.pi/180)*world_kalman_hand_tip_right[2], int(rot_z), 0, int(rot_y)))))
    hand_pose = Pose_2_KUKA(kinect_model.Pose()*transl(-world_kalman_hand_tip_right[0], -world_kalman_hand_tip_right[1], world_kalman_hand_tip_right[2]))
    # pose.setPose(KUKA_2_Pose(tar))
    hand_model.setPose(KUKA_2_Pose(list((hand_pose[0] + OFFSET_DEPTH, hand_pose[1], hand_pose[2], rot_z, 0, rot_y))))
    # Update Hand State
    # -------------- Color = [r, g, b, a] (from 0 to 1 scale) (a = opacity, 0=transparent, 1=solid)
    # GREEN = Closed, RED = UNKNOWN/NOT TRACKED, NUDE = OPEN
    if hand_state == 'OPEN':
        hand_model.setColor([255/255, 170/255, 127/255, 1])
    elif hand_state == 'CLOSED':
        hand_model.setColor([85/255, 255/255, 0/255, 1])
    else:
        hand_model.setColor([255/255, 0/255, 0/255, 1])
    # Update Gripper State
    # -------------- Color = [r, g, b, a] (from 0 to 1 scale) (a = opacity, 0=transparent, 1=solid)
    # GREEN = Closed, BLACK = OPEN
    if gripper_state == 'OPEN':
        gripper_model.setColor([15/255, 15/255, 15/255, 1])
    elif gripper_state == 'CLOSED':
        gripper_model.setColor([85/255, 255/255, 0/255, 1])

    RDK.Render(True)  # Enable simulation again
    # RDK.Update()
    # Try moving to point if inside reach else stay to previous point
    if human_state == 'READY' and not waitGripper:
        if startSim:
            # Break Loop
            startSim = False
            # Wait for Gripper to catch the fabric
            # Move once and wait for gripper
            waitGripper = True
            # Calculate robot corner to move to
            # Keep the orientation the same and change the xyz coordinates to match the corner
            pose_tar = Pose_2_KUKA(pose.Pose())
            if robot_corner == 'BR':
                """
                br_tar = Pose_2_KUKA(b_right.Pose())
                pose.setPose(KUKA_2_Pose(list((br_tar[0], br_tar[1], br_tar[2], br_tar[3], br_tar[4], br_tar[5]))))
                """
                pose.setPose(b_right.Pose())
            elif robot_corner == 'BL':
                """
                bl_tar = Pose_2_KUKA(b_left.Pose())
                pose.setPose(KUKA_2_Pose(list((bl_tar[0], bl_tar[1], bl_tar[2], bl_tar[3], bl_tar[4], bl_tar[5]))))
                """
                pose.setPose(b_left.Pose())
            elif robot_corner == 'TL':
                """
                tl_tar = Pose_2_KUKA(t_left.Pose())
                pose.setPose(KUKA_2_Pose(list((tl_tar[0], tl_tar[1], tl_tar[2], tl_tar[3], tl_tar[4], tl_tar[5]))))
                """
                pose.setPose(t_left.Pose())
            elif robot_corner == 'TR':
                """
                tr_tar = Pose_2_KUKA(t_right.Pose())
                pose.setPose(KUKA_2_Pose(list((tr_tar[0], tr_tar[1], tr_tar[2], tr_tar[3], tr_tar[4], tr_tar[5]))))
                """
                pose.setPose(t_right.Pose())
        try:
            """
            # check if move is free of collisions
            issues = robot.MoveJ_Test(robot.Joints(), pose.Joints())
            can_move_joints = (issues == 0)
            if can_move_joints and len(tr(pose.Joints())) == 7:
                # Move Robot
                robot.MoveJ(pose)
                # Update robot state
                robot_state = 'MOVING'
            else:
                robot_state = 'IDLE'
            """
            # Move Robot on Simulation Only
            # RDK.setRunMode(1)  # Simulate only
            # robot.MoveJ(pose)
            # Move real robot
            RDK.setRunMode(6)
            # change robot's joint limits to the original ones to avoid joint limitations
            robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
            tar = tr(pose.Joints())
            """ 
                Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
                KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
            """
            robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
            # change robot's joint limits to the modified ones to compute the real joint angles
            robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
            # Update robot state
            robot_state = 'MOVING'
            if not waitGripper:
                tar = Pose_2_KUKA(pose.Pose())
                # tar = list((tar[0] - int(x), tar[1] + int(z), tar[2] + int(y), int(rot_z), tar[4], int(rot_y)))
                tar = list((tar[0] - int(x), tar[1] + int(z), tar[2] + int(y), tar[3], tar[4], tar[5]))
                pose.setPose(KUKA_2_Pose(tar))
                """
                with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data.txt'), 'a') as file:
                    file.write('{}, {}, {}\n'.format(tar[0], tar[1], tar[2]))
                """
            if check_col:
                # Check for current collisions
                check_collisions = RDK.Collisions()
                # if collision then move to home
                if check_collisions != 0:
                    print("[ROBODK]: Collision detected. Moving to Home Position")
                    robot.setJoints([0, 90, 0, 0, 0, 0])
                    # Update robot state
                    robot_state = 'IDLE'
                    waitGripper = False
                    startSim = True
        except Exception as e:
            print(f'[ROBODK]: {e}')
            # Update robot state
            robot_state = 'IDLE'
            waitGripper = False
            startSim = True
            robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
    else:
        robot_state = 'IDLE'

    return robot_state, startSim, waitGripper


def _simulation(RDK, x, y, z, rot_y, rot_z, robot, pose, hand_model, hand_state, gripper_model, gripper_state, waitGripper, kinect_model, world_kalman_hand_tip_right, human_state, robot_corner, b_right, b_left, t_left, t_right, startSim, robot_path, check_col=True, sim=True):
    """
    Update robot, hand, gripper, fabric, kinect and cloud skeleton
    :param RDK: link to RoboDK instance
    :param x: float hand distance in x axis
    :param y: float hand distance in y axis
    :param z: float hand distance in z axis
    :param rot_y: float hand angle in x axis
    :param rot_z: float hand angle in z axis
    :param robot: Item robot in RoboDK
    :param pose: Item pose in RoboDK
    :param hand_model: Item hand in RoboDK
    :param hand_state: string with hand state
    :param gripper_model: Item gripper in RoboDK
    :param gripper_state: string with gripper state
    :param waitGripper: boolean to wait for gripper to grab fabric
    :param kinect_model: Item kinect in RoboDK
    :param world_kalman_hand_tip_right: list with kalman world hand tip coordinates
    :param human_state: string with human state
    :param robot_corner: string with robot's starting corner
    :param b_right: Item bottom right in RoboDK
    :param b_left: Item bottom left in RoboDK
    :param t_left: Item top left in RoboDk
    :param t_right: Item top right in RoboDK
    :param startSim: boolean to start updating models and view
    :param robot_path: list with robot movement
    :param check_col: boolean to check for collisions
    :param sim: boolean to simulate
    :return: string with robot state, boolean that simmulation stared, boolean to wait for gripper to grab fabric, list with robot movement poits
    """
    # Import libraries only for this function to minimize memory usage
    from robodk.robodk import transl, KUKA_2_Pose, Pose_2_KUKA, tr
    RDK.Render(False)  # Faster Calculations
    # Update hand pose in simulation Space
    hand_pose = Pose_2_KUKA(kinect_model.Pose()*transl(-world_kalman_hand_tip_right[0], -world_kalman_hand_tip_right[1], world_kalman_hand_tip_right[2]))
    hand_model.setPoseAbs(KUKA_2_Pose(list((hand_pose[0] + OFFSET_DEPTH, hand_pose[1], hand_pose[2], rot_z, 0, rot_y))))
    # Update Hand State
    # -------------- Color = [r, g, b, a] (from 0 to 1 scale) (a = opacity, 0=transparent, 1=solid)
    # GREEN = Closed, RED = UNKNOWN/NOT TRACKED, NUDE = OPEN
    if hand_state == 'OPEN':
        hand_model.setColor([255/255, 170/255, 127/255, 1])
    elif hand_state == 'CLOSED':
        hand_model.setColor([85/255, 255/255, 0/255, 1])
    else:
        hand_model.setColor([255/255, 0/255, 0/255, 1])
    # Update Gripper State
    # -------------- Color = [r, g, b, a] (from 0 to 1 scale) (a = opacity, 0=transparent, 1=solid)
    # GREEN = Closed, BLACK = OPEN
    if gripper_state == 'OPEN':
        gripper_model.setColor([15/255, 15/255, 15/255, 1])
    elif gripper_state == 'CLOSED':
        gripper_model.setColor([85/255, 255/255, 0/255, 1])
    RDK.Render(True)  # Enable simulation again
    # Try moving to point if inside reach else stay to previous point
    if human_state == 'READY' and not waitGripper:
        if startSim:
            # Break Loop
            startSim = False
            # Wait for Gripper to catch the fabric
            # Move once and wait for gripper
            waitGripper = True
            # Calculate robot corner to move to
            # Keep the orientation the same and change the xyz coordinates to match the corner
            if robot_corner == 'BR':
                pose.setPoseAbs(b_right.PoseAbs())
            elif robot_corner == 'BL':
                pose.setPoseAbs(b_left.PoseAbs())
            elif robot_corner == 'TL':
                pose.setPoseAbs(t_left.PoseAbs())
            elif robot_corner == 'TR':
                pose.setPoseAbs(t_right.PoseAbs())
        try:
            # change robot's joint limits to the original ones to avoid joint limitations
            robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
            if RDK.RunMode() == 1:
                tar = tr(pose.Joints())
                # Move Robot on Simulation Only
                robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 2], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6]])
            elif RDK.RunMode() == 6:
                # Move real robot
                tar = tr(pose.Joints())
                """
                    Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
                    KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
                """
                robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
            else:
                pass
            # change robot's joint limits to the modified ones to compute the real joint angles
            robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)

            # store robot path
            robot_xyz = Pose_2_KUKA(pose.PoseAbs())
            robot_path.append([robot_xyz[0], robot_xyz[1], robot_xyz[2]])

            # Update robot state
            robot_state = 'MOVING'
            if not waitGripper:
                tar = Pose_2_KUKA(pose.PoseAbs())
                tar = list((tar[0] - int(x), tar[1] + int(z), tar[2] + int(y), tar[3], tar[4], tar[5]))
                pose.setPoseAbs(KUKA_2_Pose(tar))
            if check_col:
                # Check for current collisions
                check_collisions = RDK.Collisions()
                # if collision then move to home
                if check_collisions != 0:
                    print("[ROBODK]: Collision detected. Moving to Home Position")
                    robot.setJoints([0, 90, 0, 0, 0, 0])
                    # Update robot state
                    robot_state = 'IDLE'
                    waitGripper = False
                    startSim = True
        except Exception as e:
            print('[ROBODK]: {}'.format(e))
            # Update robot state
            robot_state = 'IDLE'
            waitGripper = False
            startSim = True
            robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
    else:
        robot_state = 'IDLE'

    return robot_state, startSim, waitGripper, robot_path


def move_up(RDK, robot, pose):
    """
    Move robot up after the fabric is grabbed
    :param RDK: link to RoboDK instance
    :param robot: Item robot in RoboDK
    :param pose: Item pose in RoboDK
    :return: Boolean
    """
    from robodk.robodk import KUKA_2_Pose, Pose_2_KUKA, tr
    # Move real robot
    # change robot's joint limits to the original ones to avoid joint limitations
    robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
    tar = Pose_2_KUKA(pose.PoseAbs())
    tar = list((tar[0], tar[1], tar[2] + 60, tar[3], tar[4], tar[5]))
    pose.setPoseAbs(KUKA_2_Pose(tar))
    RDK.Render(False)
    tar = tr(pose.Joints())
    """ Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
        KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1] """
    robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
    # change robot's joint limits to the modified ones to compute the real joint angles
    robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
    return True


def collaborate(RDK, x, y, z, robot, pose, robot_path, kinect_model, hand_model, gripper_model, hand_state, gripper_state, world_kalman_hand_tip_right, rot_y, rot_z):
    """
    Human robot collaboration fuction to move robot according to human operator
    :param RDK: link to robodk instance
    :param x: distance moved in x axis
    :param y: distance moved in y axis
    :param z: distance moved in z axis
    :param robot: Item robot in RoboDK
    :param pose: Item pose in RoboDK
    :param robot_path: list with robot movement
    :param kinect_model: Item kinect in RoboDK
    :param hand_model: Item hand in RoboDK
    :param gripper_model: Item gripper in RoboDK
    :param hand_state: string with operator's hand state
    :param gripper_state: string with gripper's state
    :param world_kalman_hand_tip_right: list with kalman world coordinates of the operator's hand tip
    :param rot_y: float rotation in y axis
    :param rot_z: float rotation in z axis
    :return: string with robot state, list with robot movements
    """
    from robodk.robodk import transl, rotx, roty, rotz, TxyzRxyz_2_Pose, Pose_2_TxyzRxyz, KUKA_2_Pose, Pose_2_KUKA, tr
    import numpy as np

    RDK.Render(False)  # Faster Calculations
    # Try moving to point if inside reach else stay to previous point
    try:
        # Move real robot
        # change robot's joint limits to the original ones to avoid joint limitations
        robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
        tar = Pose_2_KUKA(pose.PoseAbs())
        tar = list((tar[0] - int(x), tar[1] + int(z), tar[2] + int(y), tar[3], tar[4], tar[5]))
        pose.setPoseAbs(KUKA_2_Pose(tar))
        # Simulation only
        if RDK.RunMode() == 1:
            RDK.Render(False)  # Faster Calculations
            # Update hand pose in simulation Space
            hand_pose = Pose_2_KUKA(kinect_model.Pose() * transl(-world_kalman_hand_tip_right[0], -world_kalman_hand_tip_right[1], world_kalman_hand_tip_right[2]))
            hand_model.setPoseAbs(KUKA_2_Pose(list((hand_pose[0] + OFFSET_DEPTH, hand_pose[1], hand_pose[2], rot_z, 0, rot_y))))
            # Update Hand State
            # -------------- Color = [r, g, b, a] (from 0 to 1 scale) (a = opacity, 0=transparent, 1=solid)
            # GREEN = Closed, RED = UNKNOWN/NOT TRACKED, NUDE = OPEN
            if hand_state == 'OPEN':
                hand_model.setColor([255 / 255, 170 / 255, 127 / 255, 1])
            elif hand_state == 'CLOSED':
                hand_model.setColor([85 / 255, 255 / 255, 0 / 255, 1])
            else:
                hand_model.setColor([255 / 255, 0 / 255, 0 / 255, 1])
            # Update Gripper State
            # -------------- Color = [r, g, b, a] (from 0 to 1 scale) (a = opacity, 0=transparent, 1=solid)
            # GREEN = Closed, BLACK = OPEN
            if gripper_state == 'OPEN':
                gripper_model.setColor([15 / 255, 15 / 255, 15 / 255, 1])
            elif gripper_state == 'CLOSED':
                gripper_model.setColor([85 / 255, 255 / 255, 0 / 255, 1])
            tar = tr(pose.Joints())
            # Move Robot on Simulation Only
            robot.MoveL([tar[0, 0], tar[0, 1], tar[0, 2], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6]])
            RDK.Render(True)  # Enable simulation again
        # Real Robot movement
        elif RDK.RunMode() == 6:
            RDK.Render(False)
            tar = tr(pose.Joints())
            """ Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
                KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1] """
            robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
        else:
            pass
        # change robot's joint limits to the modified ones to compute the real joint angles
        robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
        # store robot path
        robot_xyz = Pose_2_KUKA(pose.PoseAbs())
        robot_path.append([robot_xyz[0], robot_xyz[1], robot_xyz[2]])
        # Update robot state
        robot_state = 'MOVING'
    except Exception as e:
        print('[ROBODK]: {}'.format(e))
        # Update robot state
        robot_state = 'IDLE'
        robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
    return robot_state, robot_path


def arrowControl(RDK):
    """
    Function to control the RoboDK robot using the arrow keys of the computer
    :param RDK: link to robodk instance
    :return: None
    """
    """	Requires the RDK with the link to the Robot to optimize and run faster """
    import keyboard
    from robolink.robolink import ITEM_TYPE_ROBOT
    from robodk.robodk import Pose_2_KUKA, KUKA_2_Pose
    RDK.Render(False)  # Faster computations
    robot = RDK.ItemUserPick('Pick a robot to control', ITEM_TYPE_ROBOT)
    target = RDK.Item('Position')
    if target.type == -1:
        target = RDK.AddTarget('Position')
    target.setAsCartesianTarget()
    target.setPose(KUKA_2_Pose(list((600, 250, 750, 0, 0, -90))))
    # It is important to provide the reference frame and the tool frames when generating programs offline
    robot.setPoseFrame(robot.PoseFrame())
    robot.setPoseTool(robot.PoseTool())
    # enable render and speed up simulation (big data for neural)
    RDK.setSimulationSpeed(5)  # x faster (5 default)
    robot.setZoneData(3)  # Set the rounding parameter (Also known as: CNT, APO/C_DIS, ZoneData, Blending radius, cornering, ...)
    RDK.ShowRoboDK()  # Show RoboDK window
    RDK.setWindowState(2)  # Show RoboDK window
    RDK.setRunMode(6)  # Simulate only
    robot.setSpeed(200)  # Set linear speed in mm/s
    RDK.setCollisionActive(check_state=1)  # Enable collision check
    right_arrow = 10
    left_arrow = -10
    up_arrow = 10
    down_arrow = -10
    a_button = 10
    z_button = -10
    # Instructions for user
    print('+--------------------------------------------------------------------+')
    print('  Control the RDK Robot using the arrow keys and the a and z buttons  ')
    print('                   ← (-1)     X axis     (+1) →                       ')
    print('                   ↓ (-1)     Y axis     (+1) ↑                       ')
    print('                   z (-1)     Z axis     (+1) a                       ')
    print('                       Enter = Home Position                          ')
    print('+--------------------------------------------------------------------+')
    while True:  # making a loop
        tar = Pose_2_KUKA(target.Pose())
        # Listen for keys
        if keyboard.is_pressed('q'):  # if key 'q' is pressed
            print('[ROBODK]: Exiting...')
            print('[ROBODK]: Exit with code 0')
            break  # finishing the loop
        elif keyboard.is_pressed('a'):
            RDK.Render(False)
            target.setPose(KUKA_2_Pose(list((tar[0], tar[1], tar[2] + a_button, tar[3], tar[4], tar[5]))))
            try:
                robot.MoveJ(target)
            except:
                target.setPose(KUKA_2_Pose(list((tar[0], tar[1], tar[2] - a_button, tar[3], tar[4], tar[5]))))
                pass
            RDK.Render(True)
        elif keyboard.is_pressed('z'):
            RDK.Render(False)
            target.setPose(KUKA_2_Pose(list((tar[0], tar[1], tar[2] + z_button, tar[3], tar[4], tar[5]))))
            try:
                robot.MoveJ(target)
            except:
                target.setPose(KUKA_2_Pose(list((tar[0], tar[1], tar[2] - z_button, tar[3], tar[4], tar[5]))))
                pass
            RDK.Render(True)
        elif keyboard.is_pressed('up'):
            RDK.Render(False)
            target.setPose(KUKA_2_Pose(list((tar[0], tar[1] + up_arrow, tar[2], tar[3], tar[4], tar[5]))))
            try:
                robot.MoveJ(target)
            except:
                target.setPose(KUKA_2_Pose(list((tar[0], tar[1] - up_arrow, tar[2], tar[3], tar[4], tar[5]))))
                pass
            RDK.Render(True)
        elif keyboard.is_pressed('down'):
            RDK.Render(False)
            target.setPose(KUKA_2_Pose(list((tar[0], tar[1] + down_arrow, tar[2], tar[3], tar[4], tar[5]))))
            try:
                robot.MoveJ(target)
            except:
                target.setPose(KUKA_2_Pose(list((tar[0], tar[1] - down_arrow, tar[2], tar[3], tar[4], tar[5]))))
                pass
            RDK.Render(True)
        elif keyboard.is_pressed('right'):
            RDK.Render(False)
            target.setPose(KUKA_2_Pose(list((tar[0] + right_arrow, tar[1], tar[2], tar[3], tar[4], tar[5]))))
            try:
                robot.MoveJ(target)
            except:
                target.setPose(KUKA_2_Pose(list((tar[0] - right_arrow, tar[1], tar[2], tar[3], tar[4], tar[5]))))
                pass
            RDK.Render(True)
        elif keyboard.is_pressed('left'):
            RDK.Render(False)
            target.setPose(KUKA_2_Pose(list((tar[0] + left_arrow, tar[1], tar[2], tar[3], tar[4], tar[5]))))
            try:
                robot.MoveJ(target)
            except:
                target.setPose(KUKA_2_Pose(list((tar[0] - left_arrow, tar[1], tar[2], tar[3], tar[4], tar[5]))))
                pass
            RDK.Render(True)
        elif keyboard.is_pressed('enter'):
            RDK.Render(True)
            try:
                robot.setJoints([0, 90, 0, 0, 0, 0, 0])
            except:
                pass
        else:
            continue


"""

It can be better if more particle models are included. Two ways to improve it:

1. Using the Body Index Frame that has cells containing 0 for where the body
is and 255 for everything else. With a good graphics card each of these cells 
or some of them can be represented with a particle in RoboDK. You would have
to use the mapper file to map each of these points from depth space to world 
space and then use the world coordinates to simulate all points in RoboDK.

2. A simpler way would be to compute the lines from one joint to another and
divide the length of that line by a number and compute the joint coordinates
of the in between particles. For example if the line from wrist to elbow joint
is 150 mm and you divide it by 50 mm then you can place two more particles
in between with the first having a 50 mm distance from the elbow and the second
100 mm distance from the elbow and the third will be the wrist joint that has a
150 mm distance from elbow.

"""


# Class to represent the Operator
class CloudSkeleton(object):

    def __init__(self, RDK):
        super().__init__()
        # Import here for optimization
        from robodk.robodk import KUKA_2_Pose

        # ---------- Store RoboDK link
        self._RDK = RDK
        # Kinect
        self.KINECT_MODEL = RDK.Item("Kinect")
        # ------- Initialize the operator Skeleton --------

        # HEAD
        self.HEAD = RDK.Item('HEAD')
        self.NECK = RDK.Item('NECK')

        # SPINE
        self.SPINE_SHOULDER = RDK.Item('SPINE_SHOULDER')
        self.SPINE_MID = RDK.Item('SPINE_MID')
        self.SPINE_BASE = RDK.Item('SPINE_BASE')

        # RIGHT ARM
        self.SHOULDER_RIGHT = RDK.Item('SHOULDER_RIGHT')
        self.ELBOW_RIGHT = RDK.Item('ELBOW_RIGHT')
        self.WRIST_RIGHT = RDK.Item('WRIST_RIGHT')
        self.HAND_RIGHT = RDK.Item('HAND_RIGHT')
        self.THUMB_RIGHT = RDK.Item('THUMB_RIGHT')
        self.HAND_TIP_RIGHT = RDK.Item('HAND_TIP_RIGHT')

        # LEFT ARM
        self.SHOULDER_LEFT = RDK.Item('SHOULDER_LEFT')
        self.ELBOW_LEFT = RDK.Item('ELBOW_LEFT')
        self.WRIST_LEFT = RDK.Item('WRIST_LEFT')
        self.HAND_LEFT = RDK.Item('HAND_LEFT')
        self.THUMB_LEFT = RDK.Item('THUMB_LEFT')
        self.HAND_TIP_LEFT = RDK.Item('HAND_TIP_LEFT')

        # LEG RIGHT
        self.HIP_RIGHT = RDK.Item('HIP_RIGHT')
        self.KNEE_RIGHT = RDK.Item('KNEE_RIGHT')
        self.ANKLE_RIGHT = RDK.Item('ANKLE_RIGHT')
        self.FOOT_RIGHT = RDK.Item('FOOT_RIGHT')

        # LEG LEFT
        self.HIP_LEFT = RDK.Item('HIP_LEFT')
        self.KNEE_LEFT = RDK.Item('KNEE_LEFT')
        self.ANKLE_LEFT = RDK.Item('ANKLE_LEFT')
        self.FOOT_LEFT = RDK.Item('FOOT_LEFT')

        # Full Body and All Joints
        self.Body = RDK.Item('CloudSkeletonParticles')
        self.Joints = RDK.Item('CloudSkeletonParticles').Childs()

        # Reset the position of all the particles for easier translation
        self.Body.setPose(KUKA_2_Pose(list((0, 1500, 1000, 0, 0, 0))))

        # Set the Skeleton's Color
        # -------------- Color = [r, g, b, a] (from 0 to 1 scale) (a = opacity, 0=transparent, 1=solid)
        for joints in self.Joints:
            for particle in joints.Childs():
                particle.setColor([255 / 255, 0 / 255, 255 / 255, 1])

    def Circle(self):
        """
        Make joint particles arrange in a circle position
        :return: None
        """
        # Import here for optimization
        from robodk.robodk import KUKA_2_Pose
        import numpy as np

        # ---------- Reset the Position of the Cloud Skeleton making a Circle ----------
        self._RDK.Render(False)  # Faster calculations
        # Find Number of particles
        n_particles = 0
        for joints in self.Joints:
            n_particles += len(joints.Childs())
        # Reset the Position of the Cloud Skeleton
        counter = 0
        for joints in self.Joints:
            for particle in joints.Childs():
                particle.setPose(KUKA_2_Pose(list((CLOUD_RESET_RADIUS * np.cos(counter * 2 * np.pi / n_particles), 0, CLOUD_RESET_RADIUS * np.sin(counter * 2 * np.pi / n_particles), 0, 0, 0))))
                counter += 1
        self._RDK.Render(True)  # Enable simulation again

    def Torus(self):
        """
        Make joint particles arrange in a torus position
        :return: None
        """
        # Import here for optimization
        from robodk.robodk import KUKA_2_Pose
        import numpy as np

        # ---------- Reset the Position of the Cloud Skeleton making a torus ----------
        self._RDK.Render(False)  # Faster Calculations
        # Find Number of particles
        n_particles = 0
        for joints in self.Joints:
            n_particles += len(joints.Childs())
        # Reset the Position of the Cloud Skeleton
        counter = 0
        for joints in self.Joints:
            for particle in joints.Childs():
                particle.setPose(KUKA_2_Pose(list((PARTICLE_RADIUS * np.cos(counter * 2 * np.pi / n_particles), 0, PARTICLE_RADIUS * np.sin(counter * 2 * np.pi / n_particles), 0, 0, 0))))
                counter += 1
        self._RDK.Render(True)  # Enable Simulation again

    def Cluster(self):
        """
        Make joint particles arrange in a cluster position
        :return:
        """
        # Import here for optimization
        from robodk.robodk import KUKA_2_Pose

        # ---------- Reset the Position of the Cloud Skeleton making a torus ----------
        self._RDK.Render(False)  # Faster Calculations

        n_particles = 0
        for joints in self.Joints:
            n_particles += len(joints.Childs())

        # Reset the Position of the Cloud Skeleton
        counter = 0
        for joints in self.Joints:
            for particle in joints.Childs():
                particle.setPose(KUKA_2_Pose(list((0, 0, 0, 0, 0, 0))))
                counter += 1
        self._RDK.Render(True)  # Enable Simulation again

    def Visible(self):
        """
        Make skeleton visible
        :return: None
        """
        self._RDK.Render(False)  # Faster Calculations
        # --------- Make CloudSkeleton Visible
        for joints in self.Joints:
            for particle in joints.Childs():
                particle.setVisible(1)
        self._RDK.Render(True)  # Enable Simulation again

    def Invisible(self):
        """
        Make skeleton invisible
        :return: None
        """
        self._RDK.Render(False)  # Faster Calculations
        # --------- Make CloudSkeleton Invisible
        for joints in self.Joints:
            for particle in joints.Childs():
                particle.setVisible(0)
        self._RDK.Render(True)  # Enable Simulation again

    def testPose(self):
        """
        Update skeleton position for testing purposes only
        :return: None
        """
        # Import here for optimization
        from robodk.robodk import KUKA_2_Pose
        # DISABLE RENDER FOR FASTER CALCULATIONS
        self._RDK.Render(False)
        # TEST POINTS TO MAKE MORE REALISTIC SKELETON FOR PRESENTATION MODE
        head = [635.0, 1240.0, 2540.0]
        neck = [635.0, 1240.0, 2400.0]
        spine_shoulder = [630.0, 1240.0, 2320.0]
        spine_mid = [625.0, 1240.0, 2087.0]
        spine_base = [615.0, 1200.0, 1770.0]
        shoulder_right = [430.0, 1245.0, 2290.0]
        elbow_right = [210.0, 1210.0, 2205.0]
        wrist_right = [11.0, 1134.0, 2180.0]
        hand_right = [-50.0, 1120.0, 2207.0]
        thumb_right = [-100.0, 1090.0, 2185.0]
        hand_tip_right = [-120.0, 1095.0, 2230.0]
        shoulder_left = [835.0, 1225.0, 2290.0]
        elbow_left = [1055.0, 1195.0, 2190.0]
        wrist_left = [1260.0, 1125.0, 2175.0]
        hand_left = [1355.0, 1100.0, 2190.0]
        thumb_left = [1370.0, 1060.0, 2235.0]
        hand_tip_left = [1420.0, 1065.0, 2195.0]
        hip_right = [535.0, 1165.0, 1780.0]
        knee_right = [540.0, 1210.0, 1400.0]
        ankle_right = [545.0, 1205.0, 1010.0]
        foot_right = [490.0, 1090.0, 960.0]
        hip_left = [695.0, 1150.0, 1780.0]
        knee_left = [715.0, 1212.0, 1400.0]
        ankle_left = [710.0, 1205.0, 1010.0]
        foot_left = [690.0, 1090.0, 960.0]
        # UPDATE POSITIONS FOR EACH JOINT VALUE
        self.HEAD.setPoseAbs(KUKA_2_Pose(list((head[0], head[1], head[2], 0, 0, 0))))
        self.NECK.setPoseAbs(KUKA_2_Pose(list((neck[0], neck[1], neck[2], 0, 0, 0))))
        self.SPINE_SHOULDER.setPoseAbs(KUKA_2_Pose(list((spine_shoulder[0], spine_shoulder[1], spine_shoulder[2], 0, 0, 0))))
        self.SPINE_MID.setPoseAbs(KUKA_2_Pose(list((spine_mid[0], spine_mid[1], spine_mid[2], 0, 0, 0))))
        self.SPINE_BASE.setPoseAbs(KUKA_2_Pose(list((spine_base[0], spine_base[1], spine_base[2], 0, 0, 0))))
        self.SHOULDER_RIGHT.setPoseAbs(KUKA_2_Pose(list((shoulder_right[0], shoulder_right[1], shoulder_right[2], 0, 0, 0))))
        self.ELBOW_RIGHT.setPoseAbs(KUKA_2_Pose(list((elbow_right[0], elbow_right[1], elbow_right[2], 0, 0, 0))))
        self.WRIST_RIGHT.setPoseAbs(KUKA_2_Pose(list((wrist_right[0], wrist_right[1], wrist_right[2], 0, 0, 0))))
        self.HAND_RIGHT.setPoseAbs(KUKA_2_Pose(list((hand_right[0], hand_right[1], hand_right[2], 0, 0, 0))))
        self.THUMB_RIGHT.setPoseAbs(KUKA_2_Pose(list((thumb_right[0], thumb_right[1], thumb_right[2], 0, 0, 0))))
        self.HAND_TIP_RIGHT.setPoseAbs(KUKA_2_Pose(list((hand_tip_right[0], hand_tip_right[1], hand_tip_right[2], 0, 0, 0))))
        self.SHOULDER_LEFT.setPoseAbs(KUKA_2_Pose(list((shoulder_left[0], shoulder_left[1], shoulder_left[2], 0, 0, 0))))
        self.ELBOW_LEFT.setPoseAbs(KUKA_2_Pose(list((elbow_left[0], elbow_left[1], elbow_left[2], 0, 0, 0))))
        self.WRIST_LEFT.setPoseAbs(KUKA_2_Pose(list((wrist_left[0], wrist_left[1], wrist_left[2], 0, 0, 0))))
        self.HAND_LEFT.setPoseAbs(KUKA_2_Pose(list((hand_left[0], hand_left[1], hand_left[2], 0, 0, 0))))
        self.THUMB_LEFT.setPoseAbs(KUKA_2_Pose(list((thumb_left[0], thumb_left[1], thumb_left[2], 0, 0, 0))))
        self.HAND_TIP_LEFT.setPoseAbs(KUKA_2_Pose(list((hand_tip_left[0], hand_tip_left[1], hand_tip_left[2], 0, 0, 0))))
        self.HIP_RIGHT.setPoseAbs(KUKA_2_Pose(list((hip_right[0], hip_right[1], hip_right[2], 0, 0, 0))))
        self.KNEE_RIGHT.setPoseAbs(KUKA_2_Pose(list((knee_right[0], knee_right[1], knee_right[2], 0, 0, 0))))
        self.ANKLE_RIGHT.setPoseAbs(KUKA_2_Pose(list((ankle_right[0], ankle_right[1], ankle_right[2], 0, 0, 0))))
        self.FOOT_RIGHT.setPoseAbs(KUKA_2_Pose(list((foot_right[0], foot_right[1], foot_right[2], 0, 0, 0))))
        self.HIP_LEFT.setPoseAbs(KUKA_2_Pose(list((hip_left[0], hip_left[1], hip_left[2], 0, 0, 0))))
        self.KNEE_LEFT.setPoseAbs(KUKA_2_Pose(list((knee_left[0], knee_left[1], knee_left[2], 0, 0, 0))))
        self.ANKLE_LEFT.setPoseAbs(KUKA_2_Pose(list((ankle_left[0], ankle_left[1], ankle_left[2], 0, 0, 0))))
        self.FOOT_LEFT.setPoseAbs(KUKA_2_Pose(list((foot_left[0], foot_left[1], foot_left[2], 0, 0, 0))))
        # RENDER RESULT
        self._RDK.Render(True)

    def Update(self, joints):
        """
        Update the cloud skeleton joint points
        :param joints: list with body joints cordinates
        :return: None
        """
        # Import here for optimization
        from robodk.robodk import KUKA_2_Pose, Pose_2_KUKA, transl
        from pykinect2 import PyKinectV2

        # Update CloudSkeletonParticles with real world coordinates of joints
        self._RDK.Render(False)  # Faster Calculations
        # HEAD
        HEAD_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_Head].Position.x * 1000, -joints[PyKinectV2.JointType_Head].Position.y * 1000, joints[PyKinectV2.JointType_Head].Position.z * 1000))
        NECK_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_Neck].Position.x * 1000, -joints[PyKinectV2.JointType_Neck].Position.y * 1000, joints[PyKinectV2.JointType_Neck].Position.z * 1000))
        self.HEAD.setPoseAbs(KUKA_2_Pose(list((HEAD_POSE[0], HEAD_POSE[1], HEAD_POSE[2], 0, 0, 0))))
        self.NECK.setPoseAbs(KUKA_2_Pose(list((NECK_POSE[0], NECK_POSE[1], NECK_POSE[2], 0, 0, 0))))

        # SPINE
        SPINE_SHOULDER_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_SpineShoulder].Position.x * 1000, -joints[PyKinectV2.JointType_SpineShoulder].Position.y * 1000, joints[PyKinectV2.JointType_SpineShoulder].Position.z * 1000))
        SPINE_MID_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_SpineMid].Position.x * 1000, -joints[PyKinectV2.JointType_SpineMid].Position.y * 1000, joints[PyKinectV2.JointType_SpineMid].Position.z * 1000))
        SPINE_BASE_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_SpineBase].Position.x * 1000, -joints[PyKinectV2.JointType_SpineBase].Position.y * 1000, joints[PyKinectV2.JointType_SpineBase].Position.z * 1000))
        self.SPINE_SHOULDER.setPoseAbs(KUKA_2_Pose(list((SPINE_SHOULDER_POSE[0], SPINE_SHOULDER_POSE[1], SPINE_SHOULDER_POSE[2], 0, 0, 0))))
        self.SPINE_MID.setPoseAbs(KUKA_2_Pose(list((SPINE_MID_POSE[0], SPINE_MID_POSE[1], SPINE_MID_POSE[2], 0, 0, 0))))
        self.SPINE_BASE.setPoseAbs(KUKA_2_Pose(list((SPINE_BASE_POSE[0], SPINE_BASE_POSE[1], SPINE_BASE_POSE[2], 0, 0, 0))))

        # RIGHT ARM
        SHOULDER_RIGHT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_ShoulderRight].Position.x * 1000, -joints[PyKinectV2.JointType_ShoulderRight].Position.y * 1000, joints[PyKinectV2.JointType_ShoulderRight].Position.z * 1000))
        ELBOW_RIGHT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_ElbowRight].Position.x * 1000, -joints[PyKinectV2.JointType_ElbowRight].Position.y * 1000, joints[PyKinectV2.JointType_ElbowRight].Position.z * 1000))
        WRIST_RIGHT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_WristRight].Position.x * 1000, -joints[PyKinectV2.JointType_WristRight].Position.y * 1000, joints[PyKinectV2.JointType_WristRight].Position.z * 1000))
        HAND_RIGHT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_HandRight].Position.x * 1000, -joints[PyKinectV2.JointType_HandRight].Position.y * 1000, joints[PyKinectV2.JointType_HandRight].Position.z * 1000))
        THUMB_RIGHT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_ThumbRight].Position.x * 1000, -joints[PyKinectV2.JointType_ThumbRight].Position.y * 1000, joints[PyKinectV2.JointType_ThumbRight].Position.z * 1000))
        HAND_TIP_RIGHT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_HandTipRight].Position.x * 1000, -joints[PyKinectV2.JointType_HandTipRight].Position.y * 1000, joints[PyKinectV2.JointType_HandTipRight].Position.z * 1000))
        self.SHOULDER_RIGHT.setPoseAbs(KUKA_2_Pose(list((SHOULDER_RIGHT_POSE[0], SHOULDER_RIGHT_POSE[1], SHOULDER_RIGHT_POSE[2], 0, 0, 0))))
        self.ELBOW_RIGHT.setPoseAbs(KUKA_2_Pose(list((ELBOW_RIGHT_POSE[0], ELBOW_RIGHT_POSE[1], ELBOW_RIGHT_POSE[2], 0, 0, 0))))
        self.WRIST_RIGHT.setPoseAbs(KUKA_2_Pose(list((WRIST_RIGHT_POSE[0], WRIST_RIGHT_POSE[1], WRIST_RIGHT_POSE[2], 0, 0, 0))))
        self.HAND_RIGHT.setPoseAbs(KUKA_2_Pose(list((HAND_RIGHT_POSE[0], HAND_RIGHT_POSE[1], HAND_RIGHT_POSE[2], 0, 0, 0))))
        self.THUMB_RIGHT.setPoseAbs(KUKA_2_Pose(list((THUMB_RIGHT_POSE[0], THUMB_RIGHT_POSE[1], THUMB_RIGHT_POSE[2], 0, 0, 0))))
        self.HAND_TIP_RIGHT.setPoseAbs(KUKA_2_Pose(list((HAND_TIP_RIGHT_POSE[0], HAND_TIP_RIGHT_POSE[1], HAND_TIP_RIGHT_POSE[2], 0, 0, 0))))

        # LEFT ARM
        SHOULDER_LEFT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_ShoulderLeft].Position.x * 1000, -joints[PyKinectV2.JointType_ShoulderLeft].Position.y * 1000, joints[PyKinectV2.JointType_ShoulderLeft].Position.z * 1000))
        ELBOW_LEFT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_ElbowLeft].Position.x * 1000, -joints[PyKinectV2.JointType_ElbowLeft].Position.y * 1000, joints[PyKinectV2.JointType_ElbowLeft].Position.z * 1000))
        WRIST_LEFT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_WristLeft].Position.x * 1000, -joints[PyKinectV2.JointType_WristLeft].Position.y * 1000, joints[PyKinectV2.JointType_WristLeft].Position.z * 1000))
        HAND_LEFT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_HandLeft].Position.x * 1000, -joints[PyKinectV2.JointType_HandLeft].Position.y * 1000, joints[PyKinectV2.JointType_HandLeft].Position.z * 1000))
        THUMB_LEFT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_ThumbLeft].Position.x * 1000, -joints[PyKinectV2.JointType_ThumbLeft].Position.y * 1000, joints[PyKinectV2.JointType_ThumbLeft].Position.z * 1000))
        HAND_TIP_LEFT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_HandTipLeft].Position.x * 1000, -joints[PyKinectV2.JointType_HandTipLeft].Position.y * 1000, joints[PyKinectV2.JointType_HandTipLeft].Position.z * 1000))
        self.SHOULDER_LEFT.setPoseAbs(KUKA_2_Pose(list((SHOULDER_LEFT_POSE[0], SHOULDER_LEFT_POSE[1], SHOULDER_LEFT_POSE[2], 0, 0, 0))))
        self.ELBOW_LEFT.setPoseAbs(KUKA_2_Pose(list((ELBOW_LEFT_POSE[0], ELBOW_LEFT_POSE[1], ELBOW_LEFT_POSE[2], 0, 0, 0))))
        self.WRIST_LEFT.setPoseAbs(KUKA_2_Pose(list((WRIST_LEFT_POSE[0], WRIST_LEFT_POSE[1], WRIST_LEFT_POSE[2], 0, 0, 0))))
        self.HAND_LEFT.setPoseAbs(KUKA_2_Pose(list((HAND_LEFT_POSE[0], HAND_LEFT_POSE[1], HAND_LEFT_POSE[2], 0, 0, 0))))
        self.THUMB_LEFT.setPoseAbs(KUKA_2_Pose(list((THUMB_LEFT_POSE[0], THUMB_LEFT_POSE[1], THUMB_LEFT_POSE[2], 0, 0, 0))))
        self.HAND_TIP_LEFT.setPoseAbs(KUKA_2_Pose(list((HAND_TIP_LEFT_POSE[0], HAND_TIP_LEFT_POSE[1], HAND_TIP_LEFT_POSE[2], 0, 0, 0))))

        # LEG RIGHT
        HIP_RIGHT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_HipRight].Position.x * 1000, -joints[PyKinectV2.JointType_HipRight].Position.y * 1000, joints[PyKinectV2.JointType_HipRight].Position.z * 1000))
        KNEE_RIGHT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_KneeRight].Position.x * 1000, -joints[PyKinectV2.JointType_KneeRight].Position.y * 1000, joints[PyKinectV2.JointType_KneeRight].Position.z * 1000))
        ANKLE_RIGHT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_AnkleRight].Position.x * 1000, -joints[PyKinectV2.JointType_AnkleRight].Position.y * 1000, joints[PyKinectV2.JointType_AnkleRight].Position.z * 1000))
        FOOT_RIGHT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_FootRight].Position.x * 1000, -joints[PyKinectV2.JointType_FootRight].Position.y * 1000, joints[PyKinectV2.JointType_FootRight].Position.z * 1000))
        self.HIP_RIGHT.setPoseAbs(KUKA_2_Pose(list((HIP_RIGHT_POSE[0], HIP_RIGHT_POSE[1], HIP_RIGHT_POSE[2], 0, 0, 0))))
        self.KNEE_RIGHT.setPoseAbs(KUKA_2_Pose(list((KNEE_RIGHT_POSE[0], KNEE_RIGHT_POSE[1], KNEE_RIGHT_POSE[2], 0, 0, 0))))
        self.ANKLE_RIGHT.setPoseAbs(KUKA_2_Pose(list((ANKLE_RIGHT_POSE[0], ANKLE_RIGHT_POSE[1], ANKLE_RIGHT_POSE[2], 0, 0, 0))))
        self.FOOT_RIGHT.setPoseAbs(KUKA_2_Pose(list((FOOT_RIGHT_POSE[0], FOOT_RIGHT_POSE[1], FOOT_RIGHT_POSE[2], 0, 0, 0))))

        # LEG LEFT
        HIP_LEFT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_HipLeft].Position.x * 1000, -joints[PyKinectV2.JointType_HipLeft].Position.y * 1000, joints[PyKinectV2.JointType_HipLeft].Position.z * 1000))
        KNEE_LEFT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_KneeLeft].Position.x * 1000, -joints[PyKinectV2.JointType_KneeLeft].Position.y * 1000, joints[PyKinectV2.JointType_KneeLeft].Position.z * 1000))
        ANKLE_LEFT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_AnkleLeft].Position.x * 1000, -joints[PyKinectV2.JointType_AnkleLeft].Position.y * 1000, joints[PyKinectV2.JointType_AnkleLeft].Position.z * 1000))
        FOOT_LEFT_POSE = Pose_2_KUKA(self.KINECT_MODEL.Pose() * transl(-joints[PyKinectV2.JointType_FootLeft].Position.x * 1000, -joints[PyKinectV2.JointType_FootLeft].Position.y * 1000, joints[PyKinectV2.JointType_FootLeft].Position.z * 1000))
        self.HIP_LEFT.setPoseAbs(KUKA_2_Pose(list((HIP_LEFT_POSE[0], HIP_LEFT_POSE[1], HIP_LEFT_POSE[2], 0, 0, 0))))
        self.KNEE_LEFT.setPoseAbs(KUKA_2_Pose(list((KNEE_LEFT_POSE[0], KNEE_LEFT_POSE[1], KNEE_LEFT_POSE[2], 0, 0, 0))))
        self.ANKLE_LEFT.setPoseAbs(KUKA_2_Pose(list((ANKLE_LEFT_POSE[0], ANKLE_LEFT_POSE[1], ANKLE_LEFT_POSE[2], 0, 0, 0))))
        self.FOOT_LEFT.setPoseAbs(KUKA_2_Pose(list((FOOT_LEFT_POSE[0], FOOT_LEFT_POSE[1], FOOT_LEFT_POSE[2], 0, 0, 0))))
        self._RDK.Render(True)  # Enable Simulation again

    def UpdateView(self):
        """
        Update robodk view to follow cloud skeleton
        :return: None
        """
        from robodk.robodk import Pose_2_KUKA, KUKA_2_Pose

        # Update CloudSkeletonParticles with real world coordinates of joints
        self._RDK.Render(False)  # Faster Calculations
        # HEAD
        view = Pose_2_KUKA(self.HEAD.PoseAbs())
        self._RDK.setViewPose(KUKA_2_Pose(list((view[0], view[1] - 650, -view[2] - 2000, 180, 0, 20))))
        self._RDK.Render(True)


def pointcloud(cloud_points, skip_bits=0):
    """
    Import a text file with xyz values as x, y, z in every row as a point cloud in RoboDK
    :param cloud_points: list with point coordinates
    :param skip_bits: integer to skip points for faster calculation
    :return: None
    """
    import os
    from robodk.robodk import KUKA_2_Pose
    from robolink.robolink import Robolink
    RDK = Robolink()
    RDK.Render(False)
    # Remove file with points if it exists
    if os.path.exists('Models/PointCloud/points.txt'):
        os.remove('Models/PointCloud/points.txt')
    # Write new files with points
    with open('Models/PointCloud/points.txt', 'w') as points:
        for point in range(len(cloud_points)):
            row = '{}, {}, {}'.format(cloud_points[point, 0]*1000, cloud_points[point, 1]*1000, cloud_points[point, 2]*1000)
            points.write(row)
            points.write('\n')
            point += skip_bits
    robodk_points = RDK.Item('points')
    # Check if previous points are loaded
    if robodk_points.type != -1:
        robodk_points.Delete()
    # Import file in RoboDK
    RDK.AddFile(r'D:\Downloads\Kinect python\Models\PointCloud\points.txt')
    robodk_points = RDK.Item('points')
    robodk_points.setValue('Display', 'POINTSIZE=50 COLOR=#FF771111')
    RDK.setViewPose(KUKA_2_Pose(list((0, -100, -600, 0, 0, -90))))
    RDK.Render(True)


if __name__ == '__main__':

    # -------------- Test PointCloud from Kinect to RoboDK
    from robolink.robolink import Robolink
    import mapper
    import time
    from pykinect2.PyKinectV2 import *
    from pykinect2 import PyKinectV2
    from pykinect2 import PyKinectRuntime
    import numpy as np
    import cv2
    import os
    import ctypes
    kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body | PyKinectV2.FrameSourceTypes_BodyIndex)
    RDK = Robolink()
    RDK.Render(False)

    body, body_index, start = None, None, False

    while True:

        if kinect.has_new_body_frame():
            _body = kinect.get_last_body_frame()
            for i in range(0, kinect.max_body_count):
                body = _body.bodies[i]
                if not body.is_tracked:
                    continue
                start = True

        if kinect.has_new_body_index_frame():
            body_index = kinect.get_last_body_index_frame()
            body_index_image = body_index.reshape((kinect.depth_frame_desc.Height, kinect.depth_frame_desc.Width)).astype(np.uint8)

        if body is not None and body_index is not None and start:
            body_index_image = body_index.reshape((kinect.depth_frame_desc.Height, kinect.depth_frame_desc.Width)).astype(np.uint8)
            cv2.imshow('Body Index Stream', body_index_image)
            world_points = mapper.depth_2_world(kinect, kinect._depth_frame_data, _CameraSpacePoint)
            world_points = ctypes.cast(world_points, ctypes.POINTER(ctypes.c_float))
            world_points = np.ctypeslib.as_array(world_points, shape=(kinect.depth_frame_desc.Height * kinect.depth_frame_desc.Width, 3))
            # store points
            point_cloud = np.ndarray(shape=(len(world_points), 3), dtype=np.float32)
            point_cloud[:, 0] = world_points[:, 0] * 1000
            point_cloud[:, 1] = world_points[:, 2] * 1000
            point_cloud[:, 2] = world_points[:, 1] * 1000
            point_cloud = point_cloud[np.all(point_cloud != float('-inf'), axis=1)]
            pointcloud(point_cloud, skip_bits=0)
            RDK.Render(True)

        # Quit using q
        if cv2.waitKey(1) & 0xff == ord('q'):
            break

    cv2.destroyAllWindows()
