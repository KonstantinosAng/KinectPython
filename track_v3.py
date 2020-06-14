"""
Author: Konstantinos Angelopoulos
Date: 04/02/2020
All rights reserved.
Feel free to use and modify and if you like it give it a star.

MAIN FILE
"""


import os
import sys
import threading
from functools import wraps
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import numpy as np
import json, codecs
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import ctypes
import cv2
import time
import camera_calibrate
import QuaternionOperations
import transformations
import KalmanFilter
import mapper
import fabric_dimensions
import decision
import orientation
import robot_handle
import robotSim
from robotSim import CloudSkeleton
from hand_gesture import HandGestureClassifier
from Gripper.ros_handler import Client, graspModel, touch
from ATI_FT.ati_ft_sensor import AtiFtClient
# Flag for pygame to hide greeting message
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame


elapsed_time = time.time()
# Global Variables
ABSOLUTE_FILE_PATH = os.path.dirname(os.path.realpath(__file__))

"""====================== Camera Settings =================="""
# This values are calculated from the calibration of the camera

# Open file with calibrated configurations
with open(os.path.join(ABSOLUTE_FILE_PATH, 'calibrate/IR/config.json')) as config_file:
    data = json.load(config_file)
    DEPTH_OPTIMAL_CAMERA_MATRIX = data['Optimal Camera Matrix']
    DEPTH_OLD_CAMERA_MATRIX = data['Old Camera Matrix']

# Open file with calibrated configurations
with open(os.path.join(ABSOLUTE_FILE_PATH, 'calibrate/Color/config.json')) as config_file:
    data = json.load(config_file)
    COLOR_OPTIMAL_CAMERA_MATRIX = data['Optimal Camera Matrix']
    COLOR_OLD_CAMERA_MATRIX = data['Old Camera Matrix']

# Open file with transformation coefficients
with open(os.path.join(ABSOLUTE_FILE_PATH, 'mapper/matrix.json')) as config_file:
    data = json.load(config_file)
    RGB2D_TRANSFORMATION_MATRIX = np.array(data['Transformation Matrix'])

DEPTH_FOV_HORIZONTAL = 70.6  # in degrees http://www.smeenk.com/webgl/kinectfovexplorer.html
DEPTH_FOV_VERTICAL = 60  # in degrees https://weekly-geekly.github.io/articles/272629/index.html
DEPTH_FIELD_OF_VIEW = np.arctan(0.5 * DEPTH_FOV_HORIZONTAL / DEPTH_FOV_VERTICAL) * 180 / np.pi
COLOR_FOV_HORIZONTAL = 84.1  # in degrees http://www.smeenk.com/webgl/kinectfovexplorer.html
COLOR_FOV_VERTICAL = 53.8  # in degrees https://weekly-geekly.github.io/articles/272629/index.html
COLOR_FIELD_OF_VIEW = np.arctan(0.5 * COLOR_FOV_HORIZONTAL / COLOR_FOV_VERTICAL) * 180 / np.pi
DEPTH_FOCAL_LENGTH = (512 * 0.5) / np.tan(DEPTH_FOV_HORIZONTAL * 0.5 * np.pi / 180)  # https://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/
COLOR_FOCAL_LENGTH = (1920 * 0.5) / np.tan(COLOR_FOV_HORIZONTAL * 0.5 * np.pi / 180)  # https://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/
DEPTH_SCALE = 0.001  # Default kinect depth scale where 1 unit = 0.001 m = 1 mm
DEPTH_FOCAL_LENGTH_X = DEPTH_OLD_CAMERA_MATRIX[0][0]  # mm
DEPTH_FOCAL_LENGTH_Y = DEPTH_OLD_CAMERA_MATRIX[1][1]  # mm
DEPTH_PRINCIPAL_POINT_OFFSET_XO = DEPTH_OLD_CAMERA_MATRIX[0][2]  # pixels
DEPTH_PRINCIPAL_POINT_OFFSET_YO = DEPTH_OLD_CAMERA_MATRIX[1][2]  # pixels
COLOR_FOCAL_LENGTH_X = COLOR_OLD_CAMERA_MATRIX[0][0]  # mm
COLOR_FOCAL_LENGTH_Y = COLOR_OLD_CAMERA_MATRIX[1][1]  # mm
COLOR_PRINCIPAL_POINT_OFFSET_XO = COLOR_OLD_CAMERA_MATRIX[0][2]  # pixels
COLOR_PRINCIPAL_POINT_OFFSET_YO = COLOR_OLD_CAMERA_MATRIX[1][2]  # pixels
DEPH_PIXEL_WIDTH = DEPTH_FOCAL_LENGTH * 2 * np.tan(DEPTH_FOV_HORIZONTAL * 0.5 * np.pi / 180) / 512  # pixels
DEPTH_PIXEL_HEIGHT = DEPTH_FOCAL_LENGTH * 2 * np.tan(DEPTH_FOV_VERTICAL * 0.5 * np.pi / 180) / 424  # pixels
COLOR_PIXEL_WIDTH = COLOR_FOCAL_LENGTH * 2 * np.tan(COLOR_FOV_HORIZONTAL * 0.5 * np.pi / 180) / 1920  # pixels
COLOR_PIXEL_HEIGHT = COLOR_FOCAL_LENGTH * 2 * np.tan(COLOR_FOV_VERTICAL * 0.5 * np.pi / 180) / 1080  # pixels

"""======================== ROBOT CONFIGS ==========================="""
""" The robot is a KUKA LWR IV+ 7-DOF robot """
# Connection Parameters to Connect to KRC2 of KUKA LWR IV+ using KUKAVARPROXY server
ROBOT_IP = '169.254.98.120'  # KRC2 LAN IP
ROBOT_PORT = 7000  # KRC2 LAN port
# States for cooperation ( see decision for more details )
robot_state = 'IDLE'
human_state = 'IDLE'
gripper_state = 'OPEN'
robot_corner = 'BR'
# Flag for Simulation with RoboDK
robot_end_effector_path = []  # store robot's movements
movements_rdk = []  # movements calculated from operator's hand

""" ========== Always have these flags at Default values even if you do not want the simulation to run ========== """
startSim = True  # Flag to move robot to corner (Always at True)
startGripper = False  # Flag to move the gripper after the robot and human is ready in position (always at False)
waitGripper = False  # Flag to wait for the gripper to catch the cloth (Always at False)
startSensor = False  # Flag to start reading from the ForceTorque Sensor when robot has grasped the cloth (always at False)
graspBias = True  # Flag to bias the Force Torque Sensor when the robot catches the fabric (Always at True)
graspUnbias = False  # Flag to unbias the Force Torque Sensor (Always at False)
kalman_ft_first_time = True  # Flag to initialize first values for force torque kalman filtering (Always at True)
robotInit = True  # Flag for Robot Initialisation of Space (always at True)
monitorStop = False  # Flag to monitor the distance between the hand and the stopping corner (Always at False)
gripperRelease = False  # Flag to release the fabric after folding it (Always at False)
gripper_thread = False  # Flag to start using the gripper in thread in order to catch the fabric (Always at False)
daq_thread = False  # Flag to stsart using the daq in thread (Always at False)
start_thread = True  # Flag to start thread one time (Always at True)
fabric_folded = False  # Flag that fabric is folded and close loop (Always at False)
# Init Variables and Flags for __main__ ( Main Program Loop in def run inside class KinectMain)

""" =========== Change the below flags if you do not want some parts of the code to execute =========== """
dim = True  # Flag for finding the fabric's dimensions
cal = False  # Flag for calibrating the camera
Sim = True  # Flag for starting RoboDK
RealMovement = True  # Flag to move the real robot with RoboDK
gestureInit = True  # Flag for custom Gesture classifier
gripperInit = True  # Flag to connect to gripper
sensorInit = True  # Flag to connect to the ATI FT Sensor
kalmanInit = True  # Flag for drawing kalman on screen
skeletonInit = True  # Flag for drawing kinect's skeleton tracking on screen
cloudInit = False  # Flag for Cloud Skeleton Visualize in RoboDK
cloudPointInit = False  # Flag to import the workspace as a pointCloud in RoboDK
full_screen = False  # flag to open pygame in fullscreen

"""============================= Gripper Configs ==========================="""
""" 
Gripper Model is ReFlex 1 and it is only compatible with ROS Jade (ros_handler.py) or Indigo (gripper.py) on Ubuntu 14.04.6 (Trusty) LTS.
So a VM with Ubuntu is running ROS and has a static IP in the same network with the Laptop and the Robot Controller.
The VM is running an instance of ROS with the gripper software and a python Server to communicate with the Windows.
A client on windows is controlling the gripper state through the python server.
"""
VM_IP = '192.168.56.2'  # Vm with Ubuntu Host only Static IP
VM_PORT = 20000  # Port to communicate with Ubuntu running ROS
# Flag to communicate with server (True = using encryption (SHA-256), False = Plain Text communication)
# Both the Server and the Client must have the same flag, either both True or either both False
VM_SERVER_ENCRYPTION = True

"""========================== ATI FT Sensor Configs ====================="""
"""
ATI FT Sensor configurations.
ATI FT Sensor communication is established using the ATI FT controller and an adapter 
to convert serial to usb. The python code has a sampling rate at approximately 150 Hz.
To match the frequency of the kinect we restrict the sampling rate to 30 Hz.
"""
ATI_FT_IP = 'localhost'
ATI_FT_PORT = 10000
ATI_FT_SERVER_ENCRYPTION = True
ATI_FT_COM_PORT_WINDOWS = 'COM1'  # Port that the DAQ ATI is connected to the windows computer
ATI_FT_COM_PORT_LINUX = '/dev/ttyUSB0'  # Port that the ATI Controller FT is connected to the linux computer

"""=========================== Kalman Settings ========================="""
SHOW_KALMAN_KINECT_GRAPHS = True  # Flag to show the comparison of kalman and kinect values at the end of program
SHOW_FINAL_PLOTS = True  # Flag to plot the robot's and operator's path
ex, dt, frame_rate = KalmanFilter.initialise()  # Initialize Kalman Filters
ex_6 = KalmanFilter.initialise(force_torque=True)  # Initialize Kalman Filters for Forces and Torques

# Variables for storing kinect and kalman values
kinect_world_hand_right = []
kinect_color_hand_right = []
kalman_world_hand_right = []
kalman_color_hand_right = []
kinect_world_wrist_right = []
kinect_color_wrist_right = []
kalman_world_wrist_right = []
kalman_color_wrist_right = []
kinect_world_hand_tip_right = []
kinect_color_hand_tip_right = []
kalman_world_hand_tip_right = []
kalman_color_hand_tip_right = []
sensor_forces = []
kalman_forces = []
forces_control = []
sum_time = []  # store time for plots
history = []  # store absolute time for plots

# colors for drawing different bodies
SKELETON_COLORS = [pygame.color.THECOLORS["red"], pygame.color.THECOLORS["blue"], pygame.color.THECOLORS["green"],
                   pygame.color.THECOLORS["orange"], pygame.color.THECOLORS["purple"], pygame.color.THECOLORS["yellow"],
                   pygame.color.THECOLORS["violet"], pygame.color.THECOLORS['black'], pygame.color.THECOLORS['deepskyblue'],
                   pygame.color.THECOLORS['cyan2'], pygame.color.THECOLORS['aquamarine2'], pygame.color.THECOLORS['darkgoldenrod']]


def plot(sum_time, kinect_world_hand_right, kalman_world_hand_right, kinect_color_hand_right, kalman_color_hand_right):
    """
    Plots kinect and kalman movements in 2D or 3D
    :param sum_time: list with time of each loop
    :param kinect_world_hand_right: list with kinect's world tracking points
    :param kalman_world_hand_right: list with kalman's world tracking points
    :param kinect_color_hand_right: list with kinect's color tracking points
    :param kalman_color_hand_right: list with kalman's color tracking points
    :return: None
    """
    global SHOW_KALMAN_KINECT_GRAPHS
    if SHOW_KALMAN_KINECT_GRAPHS:
        # Plots 1D
        fig, axs = plt.subplots(3, sharex='all', gridspec_kw={'hspace': 0.4})
        l1 = axs[0].plot(sum_time, extract_x(kinect_world_hand_right), 'r-', sum_time, extract_x(kalman_world_hand_right), 'b-', linewidth=2)
        axs[0].set_title('X AXIS', fontweight='bold', fontsize=16)
        l2 = axs[1].plot(sum_time, extract_y(kinect_world_hand_right), 'r-', sum_time, extract_y(kalman_world_hand_right), 'b-', linewidth=2)
        axs[1].set_title('Y AXIS', fontweight='bold', fontsize=16)
        l3 = axs[2].plot(sum_time, extract_z(kinect_world_hand_right), 'r-', sum_time, extract_z(kalman_world_hand_right), 'b-', linewidth=2)
        axs[2].set_title('Z AXIS', fontweight='bold', fontsize=16)
        fig.legend([l1, l2, l3], labels=["Kinect Values", "Kalman Values"], loc="lower right", borderaxespad=0.1, prop={'weight': 'bold'})
        fig.text(0.5, 0.04, 'Time (sec)', ha='center', fontweight='bold', fontsize=16)  # X Axis Label
        fig.text(0.04, 0.5, 'Position of Right Hand (mm)', va='center', rotation='vertical', fontweight='bold', fontsize=16)  # Y Axis Label
        # Hide x labels and tick labels for all but bottom plot.
        for ax in axs:
            ax.tick_params(axis='both', which='major', labelsize=14)
            ax.tick_params(axis='both', which='minor', labelsize=14)
            ax.label_outer()
        # Plots 3D
        fig3 = plt.figure()
        ax = Axes3D(fig3)
        ax.plot(extract_x(kinect_color_hand_right), extract_y(kinect_color_hand_right), zs=extract_z(kinect_color_hand_right), color='red')
        ax.plot(extract_x(kalman_color_hand_right), extract_y(kalman_color_hand_right), zs=extract_z(kalman_color_hand_right), color='blue')
        ax.set_title('Kinect and Kalman Values in 3D space', fontsize=14)
        ax.set_axis_on()
        ax.set_xlabel('X Axis (mm)', fontsize=14)
        ax.set_ylabel('Y Axis (mm)', fontsize=14)
        ax.set_zlabel('Z Axis (mm)', fontsize=14)
        # ax.clabel(cset, fontsize=9, inline=1)
        plt.show()  # Show plots


def final_plots(time, kinect, kalman, robot, movements, history):
    """
    Plots with kinect, kalman and robot movement
    :param time: list with time on each loop
    :param kinect: list with kinect's world tracking points
    :param kalman: list with kalman's world tracking points
    :param robot: list with robot's movement points
    :param movements: list with distance moved in each axis
    :param history: list with time of every loop
    :return: None
    """
    global SHOW_FINAL_PLOTS
    if SHOW_FINAL_PLOTS:
        time, kinect, kalman, movements, history = time[-len(robot):], kinect[-len(robot):], kalman[-len(robot):], movements[-len(robot):], history[-len(robot):]
        # Plots 1D
        fig, axs = plt.subplots(3, sharex='all', gridspec_kw={'hspace': 0.4})
        l1 = axs[0].plot(time, extract_x(kinect), 'r-', time, extract_x(kalman), 'b-', time, extract_x(robot), 'g--', linewidth=2)
        axs[0].set_title('X AXIS', fontweight='bold', fontsize=16)
        l2 = axs[1].plot(time, extract_z(kinect), 'r-', time, extract_z(kalman), 'b-', time, extract_y(robot), 'g--', linewidth=2)
        axs[1].set_title('Y AXIS', fontweight='bold', fontsize=16)
        l3 = axs[2].plot(time, extract_y(kinect), 'r-', time, extract_y(kalman), 'b-', time, extract_z(robot), 'g--', linewidth=2)
        axs[2].set_title('Z AXIS', fontweight='bold', fontsize=16)
        fig.legend([l1, l2, l3], labels=["Kinect Values", "Kalman Values", "Robot"], loc="lower right", borderaxespad=0.1, prop={'weight': 'bold'})
        fig.text(0.5, 0.04, 'Time (sec)', ha='center', fontweight='bold', fontsize=16)  # X Axis Label
        fig.text(0.04, 0.5, 'Position of Right Hand (mm)', va='center', rotation='vertical', fontweight='bold', fontsize=16)  # Y Axis Label
        # Hide x labels and tick labels for all but bottom plot.
        for ax in axs:
            ax.tick_params(axis='both', which='major', labelsize=14)
            ax.tick_params(axis='both', which='minor', labelsize=14)
            ax.label_outer()
        plt.show()


def reader():
    """
    Open saved data and return the values in lists
    :return: list with time, list with kinect values, list with kalman values, list with robot movements, list with distance moved, list with time in each loop
    """
    # Delete old files
    if not os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/time.txt')):
        return False
    if not os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kinect.txt')):
        return False
    if not os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kalman.txt')):
        return False
    if not os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/robot.txt')):
        return False
    if not os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/movements.txt')):
        return False
    if not os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/history.txt')):
        return False
    # write new files
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/time.txt'), 'rb') as file:
        t = json.loads(file.read())
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kinect.txt'), 'rb') as file:
        kin = json.loads(file.read())
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kalman.txt'), 'rb') as file:
        kal = json.loads(file.read())
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/robot.txt'), 'rb') as file:
        rob = json.loads(file.read())
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/movements.txt'), 'rb') as file:
        move = json.loads(file.read())
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/history.txt'), 'rb') as file:
        hist = json.loads(file.read())
    return t, kin, kal, rob, move, hist


def _write(sum_time, kinect_world_hand_right, kalman_world_hand_right, robot_end_effector_path, movements, history, sensor_forces, kalman_forces, force_control):
    """
    Write lists in files for plotting
    :param sum_time: list with time in each loop
    :param kinect_world_hand_right: list with kinect tracked coordinates
    :param kalman_world_hand_right: list with kalman tracked coordinates
    :param robot_end_effector_path: list with robot end effector movements
    :param movements: list with distance moved in each axis
    :param history: list with time on each loop
    :param sensor_forces: list with sensor raw forces
    :param kalman_forces: list with kalman predictions of sensor forces
    :param force_control: list with force control feedback
    :return: boolean
    """
    # Delete old files
    if os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/time.txt')):
        os.remove(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/time.txt'))
    if os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kinect.txt')):
        os.remove(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kinect.txt'))
    if os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kalman.txt')):
        os.remove(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kalman.txt'))
    if os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/robot.txt')):
        os.remove(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/robot.txt'))
    if os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/movements.txt')):
        os.remove(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/movements.txt'))
    if os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/history.txt')):
        os.remove(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/history.txt'))
    if os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/ati_ft.txt')):
        os.remove(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/ati_ft.txt'))
    if os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kalman_ati_ft.txt')):
        os.remove(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kalman_ati_ft.txt'))
    if os.path.exists(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/control_ati_ft.txt')):
        os.remove(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/control_ati_ft.txt'))
    # write new files
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/time.txt'), 'wb') as file:
        json.dump(sum_time, codecs.getwriter('utf-8')(file), ensure_ascii=False)
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kinect.txt'), 'wb') as file:
        json.dump(kinect_world_hand_right, codecs.getwriter('utf-8')(file), ensure_ascii=False)
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kalman.txt'), 'wb') as file:
        json.dump(kalman_world_hand_right, codecs.getwriter('utf-8')(file), ensure_ascii=False)
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/robot.txt'), 'wb') as file:
        json.dump(robot_end_effector_path, codecs.getwriter('utf-8')(file), ensure_ascii=False)
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/movements.txt'), 'wb') as file:
        json.dump(movements, codecs.getwriter('utf-8')(file), ensure_ascii=False)
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/history.txt'), 'wb') as file:
        json.dump(history, codecs.getwriter('utf-8')(file), ensure_ascii=False)
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/ati_ft.txt'), 'wb') as file:
        json.dump(sensor_forces, codecs.getwriter('utf-8')(file), ensure_ascii=False)
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/kalman_ati_ft.txt'), 'wb') as file:
        json.dump(kalman_forces, codecs.getwriter('utf-8')(file), ensure_ascii=False)
    with open(os.path.join(ABSOLUTE_FILE_PATH, 'CasePlots/control_ati_ft.txt'), 'wb') as file:
        json.dump(force_control, codecs.getwriter('utf-8')(file), ensure_ascii=False)
    return True


def extract_x(lst):
    """
    Extract x coordinate from list with x, y, z coordinates
    :param lst: list with [[x, y, z], ..., [x, y, z]]
    :return: list with x coordinates [x, ..., x]
    """
    return [item[0] for item in lst]  # Extract x values for plots


def extract_y(lst):
    """
    Extract y coordinate from list with x, y, z coordinates
    :param lst: list with [[x, y, z], ..., [x, y, z]]
    :return: list with y coordinates [x, ..., x]
    """
    return [item[1] for item in lst]  # Extract y values for plots


def extract_z(lst):
    """
    Extract z coordinate from list with x, y, z coordinates
    :param lst: list with [[x, y, z], ..., [x, y, z]]
    :return: list with z coordinates [x, ..., x]
    """
    return [item[2] for item in lst]  # Extract z values for plots


def time_func(func):
    """
    Time any function
    :param func: any python function
    :return: None
    """
    @wraps(func)
    def Timer(*args, **kwargs):
        """
        execute function and time it
        :param args: any argument
        :param kwargs: any keyword argument
        :return: None
        """
        started_time = time.time()
        try:
            return func(*args, **kwargs)
        finally:
            end = time.time() - started_time
            print("[TIMER] Finished {} seconds".format(end if end > 0 else 0))
    return Timer


def Gesture_Classifier():
    """
    Initialize hand gesture classifier
    :return: instance of class HandGestureClassifier
    """
    global gestureInit
    # Initialize classifier only one time
    if gestureInit:
        return HandGestureClassifier()


class Clients:

    def __init__(self):
        self.rdk = None  # link to robodk application instance
        self.daq = None  # link to ati ft controller
        self.gripper_client = None  # link to the gripper API

    def __exit__(self):
        # Open gripper
        self.gripper_client.send('hello')
        self.gripper_client.receive()
        self.gripper_client.send('pub')
        self.gripper_client.send('open')
        self.gripper_client.receive()
        self.gripper_client.close()
        self.daq.close()

    def ati_client_connect(self):
        """
        Connect to the ati ft controller
        :return: boolean sensor initialized, sensor started pulling data
        """
        global sensorInit
        """========= Initialize connection to the DAQ Sensor =========="""
        if sensorInit is True:
            try:
                self.daq = AtiFtClient(ATI_FT_IP, ATI_FT_PORT, encrypt_flag=ATI_FT_SERVER_ENCRYPTION)
                self.daq.send('hello')
                self.daq.receive()
                self.daq.send('status')
                status = self.daq.receive(ret=True)
                if status == 'INACTIVE':
                    self.daq.send('hello')
                    self.daq.receive()
                    self.daq.send('start')
                    self.daq.receive()
                sensorInit = False
                startSensor = True
                return sensorInit, startSensor
            except Exception as e:
                print("[ATI FT CLIENT]: {}".format(e))

    def gripper_client_connect(self):
        """
        Connect to the gripper ROS API
        :return: boolean gripepr initialized, boolean gripper connected, string gripper status
        """
        global gripperInit
        """========== Initialize connection to the robot's gripper"""
        if gripperInit:
            try:
                print('+-------------------+')
                print('Connecting to Gripper Server')
                # Connect to Gripper Server running on Ubuntu with ROS
                self.gripper_client = Client(VM_IP, VM_PORT, encrypt_flag=VM_SERVER_ENCRYPTION)
                # Always start with Hello ( see Gripper/gripper.py for more info or send 'Hello' and then 'help' to see available commands )
                self.gripper_client.send('hello')
                # Always receive after commands ( except some exceptions!!! )
                self.gripper_client.receive()
                self.gripper_client.send('status')
                ros_status = self.gripper_client.receive(ret=True)
                # Check to see if ROS has started
                if ros_status == 'INACTIVE':
                    self.gripper_client.send('Hello')
                    self.gripper_client.receive()
                    self.gripper_client.send('start')
                    self.gripper_client.receive()
                # Open gripper
                self.gripper_client.send('hello')
                self.gripper_client.receive()
                self.gripper_client.send('pub')
                self.gripper_client.send('open')
                self.gripper_client.receive()
                # Update gripper status
                gripper_status = 'OPEN'
                gripperInit = False
                startGripper = True
                return gripperInit, startGripper, gripper_status
            # Handle exceptions
            except Exception as e:
                print("[GRIPPER CLIENT]: {}".format(e))

    def robodk_client_connect(self):
        """
        Connect to RoboDK and KUKA robot
        :return: connection to the RoboDK instance
        """
        global Sim, RealMovement
        if Sim:
            self.rdk = robotSim.reset(ROBOT_IP, ROBOT_PORT, simulation=False if RealMovement else True)

    def close_clients(self):
        self.__exit__()


class ReflexOne(Clients):

    def __init__(self, clients):
        super().__init__()
        self._done = False  # flag to stop gripper
        self.rdk = clients.rdk  # link to robodk
        self.daq = clients.daq # link to ati controller
        self.gripper_client = clients.gripper_client  # link to ROS API
        self.gripper_state = "OPEN"  # gripper status
        self.startGripper = startGripper
        self.waitGripper = waitGripper
        self.monitorStop = monitorStop
        self.graspBias = graspBias
        self.daq_thread = daq_thread

    def grasp(self):
        """
        Initialize grasping model to grasp fabric from the table
        :return: boolean gripper started, boolean wait for gripper to grab, boolean start monitoring hand position, boolean to bias ft sensor, boolean gripper thread started
        """
        global waitGripper, startGripper, daq_thread, gripper_state, monitorStop, graspBias
        """ ================== Gripper Reflex Logic ================= """
        try:
            """
            # gripper_client.start_grasping(self.rdk)
            if graspModel(self.rdk, self.daq, self.gripper_client):
                pass
            """

            if touch(self.rdk, self.daq, self.gripper_client):
                pass

            self.startGripper = False
            self.waitGripper = False
            self.gripper_state = 'CLOSED'
            self.monitorStop = True
            self.graspBias = True
            self.daq_thread = False
            self._done = True
            return startGripper, waitGripper, gripper_state, monitorStop, graspBias, daq_thread
        except Exception as e:
            print("[GRIPPER CLIENT]: {}".format(e))

    def close(self):
        self.__exit__()


class RoboDK_CloudSkeleton(Clients):

    def __init__(self, clients):
        super().__init__()
        self.skeleton = None  # skeleton class
        self.rdk = clients.rdk  # link to RoboDK
        self.daq = clients.daq  # link to ati controller
        self.gripper_client = clients.gripper_client  # link to gripper ROS API

    def reset(self):
        """
        Reset Cloud Skeleton in RoboDK
        :return: None
        """
        # Initialize CloudSkeleton in RoboDK
        self.skeleton = CloudSkeleton(self.rdk)
        if cloudInit:
            self.skeleton.Visible()
            self.skeleton.Circle()
        else:
            self.skeleton.Invisible()

    def RoboDK_CloudSkeleton_Update(self, joints):
        """
        Update cloud skeleton joint position
        :param joints: list with body joints data
        :return: None
        """
        self.skeleton.Update(joints)
        self.skeleton.UpdateView()


# Main Class for Kinect Object
class KinectMain(Clients):

    def __init__(self, clients, operator):
        super().__init__()
        self.cloud_skeleton = operator  # Cloud Skeleton class in RoboDK
        self.rdk = clients.rdk  # connection to RoboDK
        self.daq = clients.daq  # connection to ati ft controller
        self.gripper_client = clients.gripper_client  # connection to ROS API
        pygame.init()  # initialize pygame
        self._clock = pygame.time.Clock()  # Used to manage how fast the screen updates
        pygame.display.set_caption("Kalman Kinect Tracking")  # Pygame Window Name
        self._done = False  # Flag for when the user clicks the close button.
        # Kinect runtime object
        # self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body | PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_BodyIndex | PyKinectV2.FrameSourceTypes_Infrared | PyKinectV2.FrameSourceTypes_Audio)
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body | PyKinectV2.FrameSourceTypes_Depth)
        # Set the width and height of the screen [width, height]
        if full_screen:
            # self._infoObject = pygame.display.Info()
            # self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)
            self._screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)
        else:
            self._screen = pygame.display.set_mode((int(self._kinect.color_frame_desc.Width / 3.5), int(self._kinect.color_frame_desc.Height / 3.5)), pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)
        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)
        self._bodies = None  # Skeleton Data
        self._bodies_index = None  # Body Index Data
        self._body_index_image = None  # Body Index Image
        self._depth = None  # Depth Frame Data
        self._depth_image = None  # Depth Image
        self._color = None  # Color Frame Data
        self._color_image = None  # Color Image
        self._ir = None  # IR Frame Data
        self._ir_image = None  # IR Image
        self._color_fabric_points = None  # Color Fabric Points
        self.rgb_focal_length = COLOR_FOCAL_LENGTH  # Kinect's color_focal_length
        self.rgb_field_of_view = COLOR_FIELD_OF_VIEW  # Kinect's color field of view
        self._points = np.ndarray(shape=(4, 3), dtype=float)  # Fabric world points
        self._counter = 0  # Counter for storing loops
        self.distance = []  # distance of hand on every loop
        self.velocity_x = []  # velocity of hand on x axis
        self.velocity_y = []  # velocity of hand on y axis
        self.velocity_z = []  # velocity of hand on z axis
        # self.Limit = 1800 * frame_rate  # Limit to break loop
        self.Limit = 300 * frame_rate  # Limit to test kalman figure plots
        self.body_id = -1  # store operators body id to track only him
        self.mass_x = 0.0  # Mass Center Coordinates for the human body
        self.mass_y = 0.0  # Mass Center Coordinates for the human body
        self.mass_z = 0.0  # Mass Center Coordinates for the human body
        # Store points to calculate the transformation matrix for depth to color space
        self.mapper_color_points = []
        self.mapper_depth_points = []
        # estimate of initial position variance (covariance matrix) for all values
        self.world_hand_right_p = ex
        self.color_hand_right_p = ex
        self.world_wrist_right_p = ex
        self.color_wrist_right_p = ex
        self.world_hand_tip_right_p = ex
        self.color_hand_tip_right_p = ex
        self.forces_p = ex_6
        self.mass_init = True  # To only wait once for the body to enter robot Space
        self.hand_right_state = 'UNKNOWN'  # Right Hand State
        self.hand_right_confidence = 'LOW'  # Right Hand confidence
        self.joints = None  # store body joints of the tracked person

    def draw_body_bone(self, joint_points, color, joint0, joint1):
        """
        Draw skeleton on pygame screen
        :param joint_points: joint data
        :param color: string pygame color
        :param joint0: string link joint 1
        :param joint1: string link joint 2
        :return: None
        """
        # both joints are not tracked
        if (self.joints[joint0].TrackingState == PyKinectV2.TrackingState_NotTracked) or (self.joints[joint1].TrackingState == PyKinectV2.TrackingState_NotTracked):
            return

        # both joints are not *really* tracked
        if (self.joints[joint0].TrackingState == PyKinectV2.TrackingState_Inferred) and (self.joints[joint1].TrackingState == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good
        start = (joint_points[joint0].x, joint_points[joint0].y)
        end = (joint_points[joint1].x, joint_points[joint1].y)

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 20)
        except Exception as e:  # need to catch it due to possible invalid positions (with inf)
            print("[MAIN - DRAW SKELETON]: {}".format(e))

    def draw_body(self, joint_points, color):
        """
        Draw Body joints
        :param joint_points: body joint data
        :param color: pygame color
        :return: None
        """
        # Right Arm
        self.draw_body_bone(joint_points, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight)
        self.draw_body_bone(joint_points, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight)
        self.draw_body_bone(joint_points, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight)

    def draw_color_frame(self, frame, target_surface):
        """
        Map color frame on screen
        :param frame: frame array
        :param target_surface: pygame surface
        :return: None
        """
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def draw_kinect(self, points, color):
        """
        Draw Kalman prediction on Screen
        :param points: list with color pixel coordinates
        :param color: pygame color
        :return: None
        """
        try:
            pygame.draw.circle(self._frame_surface, color, (points[0], points[1]), 40, 15)
        except Exception as e:  # need to catch it due to possible invalid positions (with inf)
            print("[MAIN - DRAW KALMAN PREDICTION]: {}".format(e))

    def draw_infrared_frame(self, frame, target_surface):
        """
        Draw infrared frame on surface
        :param frame: infrared frame
        :param target_surface: pygame surface
        :return: None
        """
        if frame is None:  # some usb hub do not provide the infrared image. it works with Kinect studio though
            return
        target_surface.lock()
        f8 = np.uint8(frame.clip(1, 4000) / 16.)
        frame8bit = np.dstack((f8, f8, f8))
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame8bit.ctypes.data, frame8bit.size)
        del address
        target_surface.unlock()

    def draw_fabric(self, f_points, p_color):
        """
        Draw fabric edges on pygame screen
        :param f_points: list with fabric color pixel coordinates
        :param p_color: pygame color
        :return: None
        """
        try:
            for point in f_points:
                pygame.draw.circle(self._frame_surface, p_color, (point[0], point[1]), 40, 15)
        except Exception as e:
            print("MAIN - DRAW FABRIC]: {}".format(e))

    def find_fabric(self, dim):
        """
        locate fabric with image process
        :param dim: boolean flag to locate image
        :return: boolean, list with fabric world points, list with fabric center coordinates, float fabric width, float fabric height, float quasi state robot speed
        """
        # Color Space Coordinates
        # fabric_points, fabric_width, fabric_height, fabric_angle, fabric_ret = fabric_dimensions.find_dimensions(self._color_image, self.rgb_focal_length)
        fabric_points, fabric_width, fabric_height, fabric_angle, fabric_ret = fabric_dimensions.box_hull(self._color_image, self.rgb_focal_length)
        self._color_fabric_points = fabric_points
        # Flag to ensure that a valid fabric is found
        if fabric_ret:
            # Run mapper here to avoid severe segmentation errors
            # Map Color Points to Depth Space, using kinect mapper, for each fabric point
            world_points = mapper.color_2_world(self._kinect, self._kinect._depth_frame_data, _CameraSpacePoint)
            world_points = ctypes.cast(world_points, ctypes.POINTER(ctypes.c_float))
            world_points = np.ctypeslib.as_array(world_points, shape=(self._kinect.color_frame_desc.Height * self._kinect.color_frame_desc.Width, 3))
            world_points *= 1000  # transform to mm

            for i, point in enumerate(fabric_points):
                fabric_depth_x, fabric_depth_y = mapper.color_point_2_depth_point(self._kinect, _DepthSpacePoint, self._kinect._depth_frame_data, point)  # pixel
                cv2.circle(self._depth_image, (fabric_depth_x, fabric_depth_y), 7, (255, 0, 0))
                # cv2.imwrite('images/fab7.png', self._depth_image)
                self._points[i] = world_points[point[1] * 1920 + point[0]]

            cv2.imshow('Depth', cv2.flip(self._depth_image, 1))
            cv2.waitKey(1500)
            cv2.destroyAllWindows()
            # World Coordinates
            fabric_points, fabric_width, fabric_height, fabric_angle = fabric_dimensions.world_dimensions(self._points)
            fabric_points = fabric_dimensions.arrange(fabric_points)  # always arrange to correct way
            # Calculate fabric orientations
            fabric_orientation, fabric_center = orientation.fabric_orientation(fabric_points)
            """ ================================ TEST POINTS =============================== """
            # fabric_points = np.asarray([[-785, 250, 760], [-485, 250, 760], [-485, -250, 760], [-785, -250, 760]])
            # fabric_center = np.asarray([-635, 0, 760])
            # fabric_width, fabric_height = np.asarray([785 - 485]), np.asarray([250 + 250])
            """ ============================================================================ """
            """ ========================= Robot ISO Parameters ============================= """
            # max_speed for Quasi-static contact and Transient contact ( see robot_handle.iso for more details )
            q_relev_max_speed, t_relev_max_speed, relev_speed = robot_handle.iso(fabric_width, fabric_height)
            # Find edges closer to robot
            print('+-------------------+')
            print('   Fabric Detected   ')
            print('   Width (mm): {}\n   Height (mm): {}\n World Center Point (XYZ in mm): {}'.format(fabric_width, fabric_height, fabric_center))
            print(' Color2World Fabric Points (mm): ')
            print(str(fabric_points[0]))
            print(str(fabric_points[1]))
            print(str(fabric_points[2]))
            print(str(fabric_points[3]))
            print(' ISO Speed Calculated')
            print('+-------------------+')
            print('+-------------------+')
            print('  Starting Tracking  ')
            print('+-------------------+')
            dim = False
        else:
            print('+-------------------+')
            print(" Fabric not detected properly")
            print('+-------------------+')

        return dim, fabric_points, fabric_center, fabric_width, fabric_height, q_relev_max_speed

    def update_cloud_skeleton(self):
        """
        Update cloud skeleton position in RoboDK
        :return:
        """
        self.cloud_skeleton.RoboDK_CloudSkeleton_Update(self.joints)

    def run(self):
        """
        Main loop for human robot collaboration
        :return: None
        """

        """================= Global Variables ================"""
        global human_state, robot_state, gripper_state, robot_corner, monitorStop, gripperRelease, cloudPointInit, fabric_folded
        global graspUnbias, frame_rate, dt, kalman_ft_first_time, waitGripper, graspBias, Sim, dim, cloudInit, start_thread
        global gestureInit, kalmanInit, sensorInit, gripperInit, startGripper, startSensor, robotInit, cal, startSim, MOVING_DISTANCE
        global gesture_classifier, operator, gripper, robot_end_effector_path, gripper_thread, daq_thread, skeletonInit

        """================== Main Program Loop ================="""
        while not self._done:

            # time frames
            time_it = time.time()

            """===== Listen for user actions ====="""
            for event in pygame.event.get():  # User did something
                if event.type == pygame.QUIT:  # If user clicked close
                    self._done = True  # Flag that we are done so we exit this loop
                # Check for screen resizing and update the surface
                elif event.type == pygame.VIDEORESIZE:  # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

            """======== Color Frame - Update the surface ====="""
            if self._kinect.has_new_color_frame():
                self._color = self._kinect.get_last_color_frame()
                self._color_image = self._color.reshape((self._kinect.color_frame_desc.Height, self._kinect.color_frame_desc.Width, 4)).astype(np.uint8)
                self.draw_color_frame(self._color, self._frame_surface)

            """=============== Infrared frames ========"""
            """
            if self._kinect.has_new_infrared_frame():
                self._ir = self._kinect.get_last_infrared_frame()
                self._ir_image = self._ir.reshape((self._kinect.infrared_frame_desc.Height, self._kinect.infrared_frame_desc.Width)).astype(np.uint16)
                ir_img = cv2.flip(ir_img, 1)
                cv2.imshow('Kinect IR Stream', self._ir_image)
            """

            """=============== Depth Frame ============="""
            if self._kinect.has_new_depth_frame():
                self._depth = self._kinect.get_last_depth_frame()
                self._depth_image = self._depth.reshape((self._kinect.depth_frame_desc.Height, self._kinect.depth_frame_desc.Width)).astype(np.uint8)
                """
                    self._depth_image = cv2.flip(depth_image, 1)
                    cv2.imshow('Kinect Depth Stream', self._depth_image)
                """

            """=============== Body Frame =============="""
            if self._kinect.has_new_body_frame():
                self._bodies = self._kinect.get_last_body_frame()

            """============= Body Index Frame ========"""
            """
            if self._kinect.has_new_body_index_frame():
                self._bodies_index = self._kinect.get_last_body_index_frame()
                self._body_index_image = self._bodies_index.reshape((self._kinect.depth_frame_desc.Height, self._kinect.depth_frame_desc.Width)).astype(np.uint8)
                cv2.imshow('Body Index Stream', self._body_index_image)
            """
            """======== Import Cloud Point in RoboDK ============="""
            if cloudPointInit and self._depth is not None:
                world_points = mapper.depth_2_world(self._kinect, self._kinect._depth_frame_data, _CameraSpacePoint)
                world_points = ctypes.cast(world_points, ctypes.POINTER(ctypes.c_float))
                world_points = np.ctypeslib.as_array(world_points, shape=(self._kinect.depth_frame_desc.Height * self._kinect.depth_frame_desc.Width, 3))
                # store points
                point_cloud = np.ndarray(shape=(len(world_points), 3), dtype=np.float32)
                point_cloud[:, 0] = world_points[:, 0] * 1000
                point_cloud[:, 1] = world_points[:, 2] * 1000
                point_cloud[:, 2] = world_points[:, 1] * 1000
                point_cloud = point_cloud[np.all(point_cloud != float('-inf'), axis=1)]

            """============== Calibrate Camera ================="""
            if  cal and self._kinect.has_new_color_frame():
                print('Starting Calibration')
                ret_cal = camera_calibrate.calibrate('calibrate/Color/*.png')
                if ret_cal:
                    print("Calibration was successful")
                    print("Continuing with Tracking...")
                    print("Tracking...")
                    cal = False
                else:
                    print('Something went wrong...Calibration unsuccessful')
                # You can also get the Depth Camera Intrinsics from the mapper
                # Make sure that you have at least one full depth frame, if the
                # kinect does not have a full depth frame all values will be 0
                if self._depth is not None:
                    intrinsics_matrix = mapper.intrinsics(self._kinect, write=True)

            """============ Track only one person and within a certain distance from kinect origin"""
            if self._bodies is not None and self.body_id == -1 and not dim and not cal and not sensorInit and not gripperInit:
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked:
                        continue
                    # Only wait for the human to enter robot space once, then track him all the time
                    if self.mass_init:
                        # Compute mass center for the tracked human
                        self.joints = body.joints
                        for j in range(0, PyKinectV2.JointType_Count):
                            self.mass_x += self.joints[j].Position.x
                            self.mass_y += self.joints[j].Position.y
                            self.mass_z += self.joints[j].Position.z

                        self.mass_x = self.mass_x / PyKinectV2.JointType_Count
                        self.mass_y = self.mass_y / PyKinectV2.JointType_Count
                        self.mass_z = self.mass_z / PyKinectV2.JointType_Count
                        # Map mass center to color for drawing
                        color_mass = mapper.world_point_2_color(self._kinect, _CameraSpacePoint, np.asarray([self.mass_x, self.mass_y, self.mass_z]))
                        # Draw mass center
                        self.draw_kinect([color_mass[0], color_mass[1]], SKELETON_COLORS[8])
                        # print(np.abs(self.mass_x*1000), np.abs(self.mass_y*1000))
                        # If mass center is less than 250 mm from x and y axis track him
                        if np.abs(self.mass_x * 1000) <= 400 and np.abs(self.mass_y * 1000) <= 700:
                            self.body_id = i
                            self.mass_init = False
                            print('     Found Body      ')
                            print('+-------------------+')
                    else:
                        self.body_id = i

            """============ Main Loop =========="""
            if self._bodies is not None and self.body_id != -1 and not dim and not cal and not sensorInit and not gripperInit:
                # Initialize body
                body = self._bodies.bodies[self.body_id]

                """====== Escape Loop if no body is tracked"""
                if not body.is_tracked:
                    self.body_id = -1
                    print('      Lost Body      ')
                    print('+-------------------+')
                    continue

                floor_vector = self._bodies.floor_clip_plane
                self.joints = body.joints

                # Kinect's Distance from Floor
                kinect_distance_from_plane = floor_vector.w * 1000  # mm
                kinect_tilt = np.arctan(floor_vector.z / floor_vector.y) * (180.0 / np.pi)  # degrees
                # Real world coordinates in meters
                world_hand_right_x = self.joints[PyKinectV2.JointType_HandRight].Position.x * 1000  # mm
                world_hand_right_y = self.joints[PyKinectV2.JointType_HandRight].Position.y * 1000  # mm
                world_hand_right_z = self.joints[PyKinectV2.JointType_HandRight].Position.z * 1000  # mm

                world_wrist_right_x = self.joints[PyKinectV2.JointType_WristRight].Position.x * 1000  # mm
                world_wrist_right_y = self.joints[PyKinectV2.JointType_WristRight].Position.y * 1000  # mm
                world_wrist_right_z = self.joints[PyKinectV2.JointType_WristRight].Position.z * 1000  # mm

                world_hand_tip_right_x = self.joints[PyKinectV2.JointType_HandTipRight].Position.x * 1000  # mm
                world_hand_tip_right_y = self.joints[PyKinectV2.JointType_HandTipRight].Position.y * 1000  # mm
                world_hand_tip_right_z = self.joints[PyKinectV2.JointType_HandTipRight].Position.z * 1000  # mm

                # convert joint coordinates to color space
                color_joint_points = self._kinect.body_joints_to_color_space(self.joints)
                joint_points_depth = self._kinect.body_joints_to_depth_space(self.joints)

                color_hand_right_x = color_joint_points[PyKinectV2.JointType_HandRight].x  # pixels
                color_hand_right_y = color_joint_points[PyKinectV2.JointType_HandRight].y  # pixels
                depth_hand_right_x = joint_points_depth[PyKinectV2.JointType_HandRight].x  # pixels
                depth_hand_right_y = joint_points_depth[PyKinectV2.JointType_HandRight].y  # pixels
                """---------------------------------------------------------------------------------------
                There is no guaranty that a depth value will have a corresponding depth pixel, because the
                skeleton joints frame is obtained after bodyIndex frame going through a complex algorithm.
                ---------------------------------------------------------------------------------------"""
                if (int(depth_hand_right_y) * 512 + int(depth_hand_right_x)) < 512 * 424:
                    color_hand_right_z = float(self._depth[int(depth_hand_right_y) * 512 + int(depth_hand_right_x)])  # mm
                else:
                    # If it exceeds return the last value to catch overflow
                    color_hand_right_z = float(self._depth[int((512 * 424) - 1)])  # mm

                color_wrist_right_x = color_joint_points[PyKinectV2.JointType_WristRight].x  # pixels
                color_wrist_right_y = color_joint_points[PyKinectV2.JointType_WristRight].y  # pixels
                depth_wrist_right_x = joint_points_depth[PyKinectV2.JointType_WristRight].x  # pixels
                depth_wrist_right_y = joint_points_depth[PyKinectV2.JointType_WristRight].y  # pixels

                if (int(depth_wrist_right_y) * 512 + int(depth_wrist_right_x)) < 512 * 424:
                    color_wrist_right_z = float(self._depth[int(depth_wrist_right_y) * 512 + int(depth_wrist_right_x)])  # mm
                else:
                    # If it exceeds return the last value to catch overflow
                    color_wrist_right_z = float(self._depth[int((512 * 424) - 1)])  # mm

                color_hand_tip_right_x = color_joint_points[PyKinectV2.JointType_HandTipRight].x  # pixels
                color_hand_tip_right_y = color_joint_points[PyKinectV2.JointType_HandTipRight].y  # pixels
                depth_hand_tip_right_x = joint_points_depth[PyKinectV2.JointType_HandTipRight].x  # pixels
                depth_hand_tip_right_y = joint_points_depth[PyKinectV2.JointType_HandTipRight].y  # pixels

                if (int(depth_hand_tip_right_y) * 512 + int(depth_hand_tip_right_x)) < 512 * 424:
                    color_hand_tip_right_z = float(self._depth[int(depth_hand_tip_right_y) * 512 + int(depth_hand_tip_right_x)])  # mm
                else:
                    # If it exceeds return the last value to catch overflow
                    color_hand_tip_right_z = float(self._depth[int((512 * 424) - 1)])  # mm
                """
                                # --------------------------- Solve equation to find transformation matrix from color to depth -----------------
                                if self._counter % 3 <= 2:
                                    if self._counter % 3 == 0:
                                        cc = 0
                                    # Mapper Points ( Store points to find the transformation matrix for depth to color )
                                    self.mapper_color_points[cc] = [color_hand_right_x, color_hand_right_y]  # pixels
                                    self.mapper_depth_points[cc] = [depth_hand_right_x, depth_hand_right_y]  # pixels
                                    if self._counter % 3 == 2:
                                        transformation_matrix, transformation_matrix_ret = mapper.transform_color_2_depth(self.mapper_color_points, self.mapper_depth_points)
                                        if transformation_matrix_ret is True:
                                            with open('mapper/data.csv', mode='a') as csv_file:
                                                row = [transformation_matrix[0, 0], transformation_matrix[0, 1], transformation_matrix[0, 2], transformation_matrix[1, 0], transformation_matrix[1, 1], transformation_matrix[1, 2]]
                                                csv_file.write("{}, {}, {}, {}, {}, {}".format(row[0], row[1], row[2], row[3], row[4], row[5]))
                                                csv_file.write("\n")
                                            print(transformation_matrix)
                                    cc += 1
                                """

                """
                ------------------------------------------------------------------------------------------
                # Keep in mind that z is not defined as the distance of the object from the sensor origins,
                # but rather the distance of the object from the kinect level

                                      Kinect Level
                                            |          depth   +------------------+
                                            |<---------------->| (Tracked Object) | 
                                            |                  +------------------+
                                            o 
                                   (Kinect) o
                                            o 
                                            |
                                            |
                                            |     
                -------------------------------------------------------------------------------------------
                """
                # Find Joint Quaternions of Wrist Right
                # qx = body.joint_orientations[PyKinectV2.JointType_WristRight].Orientation.x
                # qy = body.joint_orientations[PyKinectV2.JointType_WristRight].Orientation.y
                # qz = body.joint_orientations[PyKinectV2.JointType_WristRight].Orientation.z
                # qw = body.joint_orientations[PyKinectV2.JointType_WristRight].Orientation.w

                # Calculate pitch(X), yaw(Y), roll(Z) of wrist right from quaternions
                # https://pterneas.com/2017/05/28/kinect-joint-rotation/
                # wrist_right_orientation = QuaternionOperations.quaternion2rot([qw, qx, qy, qz])
                # (temp_roll, temp_pitch, temp_yaw) = import transformations.euler_from_quaternion([qx, qy, qz, qw])
                
                # Find Confidence of right hand ( 0 = Not Tracked, 1 = Inferred, 2 = Tracked )
                self.hand_right_confidence = self.joints[PyKinectV2.JointType_HandRight].TrackingState
                if self.hand_right_confidence == PyKinectV2.TrackingState_Tracked:
                    self.hand_right_confidence = 'HIGH'
                else:
                    self.hand_right_confidence = 'LOW'

                # Find Right Hand State ( 0 = Unknown, 1 = NotTracked, 2 = Open, 3 = Closed, 4 = Lasso )
                # We only care about Open or Closed
                self.hand_right_state = body.hand_right_state
                if self.hand_right_state == PyKinectV2.HandState_Open:
                    self.hand_right_state = 'OPEN'
                elif self.hand_right_state == PyKinectV2.HandState_Closed:
                    self.hand_right_state = 'CLOSED'
                else:
                    self.hand_right_state = 'UNKNOWN'

                """============= Gesture Classifier based on kinect rgb image"""
                if gestureInit and self._counter % 3 == 0:
                    gesture_classifier.detect(self._color_image, int(color_hand_right_x), int(color_hand_right_y), world_hand_right_z / 1000, show=True)
                    choices = [self.hand_right_state, gesture_classifier.state]
                    weights = [0.1, 0.9]
                    # Classify hand state ( classifier has more weight than kinect's state )
                    self.hand_right_state = np.random.choice(choices, p=weights)

                """================= Kalman Initialize first point at the start"""
                if self._counter == 0:
                    # Calculate Velocity and Acceleration of hand
                    self.distance.append([0, 0, 0])

                    self.velocity_x.append(self.distance[-1][0] / dt)
                    self.velocity_y.append(self.distance[-1][1] / dt)
                    self.velocity_z.append(self.distance[-1][2] / dt)

                    # Kalman Initialization of first point for wrist hand and Tip of Right hand
                    # ---------------------------- World --------------------------------------
                    world_hand_right_q_estimate = np.array([[world_hand_right_x],
                                                            [world_hand_right_y],
                                                            [world_hand_right_z],
                                                            [self.velocity_x[-1]],
                                                            [self.velocity_y[-1]],
                                                            [self.velocity_z[-1]]])

                    world_wrist_right_q_estimate = np.array([[world_wrist_right_x],
                                                             [world_wrist_right_y],
                                                             [world_wrist_right_z],
                                                             [self.velocity_x[-1]],
                                                             [self.velocity_y[-1]],
                                                             [self.velocity_z[-1]]])

                    world_hand_tip_right_q_estimate = np.array([[world_hand_tip_right_x],
                                                                [world_hand_tip_right_y],
                                                                [world_hand_tip_right_z],
                                                                [self.velocity_x[-1]],
                                                                [self.velocity_y[-1]],
                                                                [self.velocity_z[-1]]])

                    # ---------------------------- Color --------------------------------------
                    if kalmanInit:
                        color_hand_right_q_estimate = np.array([[color_hand_right_x],
                                                                [color_hand_right_y],
                                                                [color_hand_right_z],
                                                                [self.velocity_x[-1]],
                                                                [self.velocity_y[-1]],
                                                                [self.velocity_z[-1]]])

                        color_wrist_right_q_estimate = np.array([[color_wrist_right_x],
                                                                 [color_wrist_right_y],
                                                                 [color_wrist_right_z],
                                                                 [self.velocity_x[-1]],
                                                                 [self.velocity_y[-1]],
                                                                 [self.velocity_z[-1]]])

                        color_hand_tip_right_q_estimate = np.array([[color_hand_tip_right_x],
                                                                    [color_hand_tip_right_y],
                                                                    [color_hand_tip_right_z],
                                                                    [self.velocity_x[-1]],
                                                                    [self.velocity_y[-1]],
                                                                    [self.velocity_z[-1]]])
                else:
                    self.distance.append([(world_hand_right_x - kinect_world_hand_right[self._counter - 1][0]),
                                          (world_hand_right_y - kinect_world_hand_right[self._counter - 1][1]),
                                          (world_hand_right_z - kinect_world_hand_right[self._counter - 1][2])])

                    self.velocity_x.append(self.distance[-1][0] / dt)
                    self.velocity_y.append(self.distance[-1][1] / dt)
                    self.velocity_z.append(self.distance[-1][2] / dt)

                # !------------------------ Kalman Prediction of World Coordinates -------------------------------------!

                # Kalman Prediction of Hand Right
                world_hand_right_q_loc_meas = np.array([[world_hand_right_x],
                                                        [world_hand_right_y],
                                                        [world_hand_right_z]])

                world_hand_right_q_estimate, self.world_hand_right_p = KalmanFilter.estimate(self.world_hand_right_p, world_hand_right_q_loc_meas, world_hand_right_q_estimate)

                # Kalman Prediction of Wrist Right
                world_wrist_right_q_loc_meas = np.array([[world_wrist_right_x],
                                                         [world_wrist_right_y],
                                                         [world_wrist_right_z]])

                world_wrist_right_q_estimate, self.world_wrist_right_p = KalmanFilter.estimate(self.world_wrist_right_p, world_wrist_right_q_loc_meas, world_wrist_right_q_estimate)

                # Kalman Prediction of Hand Tip Right
                world_hand_tip_right_q_loc_meas = np.array([[world_hand_tip_right_x],
                                                            [world_hand_tip_right_y],
                                                            [world_hand_tip_right_z]])

                world_hand_tip_right_q_estimate, self.world_hand_tip_right_p = KalmanFilter.estimate(self.world_hand_tip_right_p, world_hand_tip_right_q_loc_meas, world_hand_tip_right_q_estimate)

                # !------------------------ Kalman Prediction of Color Coordinates --------------------------------------!
                if kalmanInit:
                    # Kalman Prediction of Hand Right
                    color_hand_right_q_loc_meas = np.array([[color_hand_right_x],
                                                            [color_hand_right_y],
                                                            [color_hand_right_z]])

                    color_hand_right_q_estimate, self.color_hand_right_p = KalmanFilter.estimate(self.color_hand_right_p, color_hand_right_q_loc_meas, color_hand_right_q_estimate)

                    # Kalman Prediction of Wrist Right
                    color_wrist_right_q_loc_meas = np.array([[color_wrist_right_x],
                                                             [color_wrist_right_y],
                                                             [color_wrist_right_z]])

                    color_wrist_right_q_estimate, self.color_wrist_right_p = KalmanFilter.estimate(self.color_wrist_right_p, color_wrist_right_q_loc_meas, color_wrist_right_q_estimate)

                    # Kalman Prediction of Hand Tip Right
                    color_hand_tip_right_q_loc_meas = np.array([[color_hand_tip_right_x],
                                                                [color_hand_tip_right_y],
                                                                [color_hand_tip_right_z]])

                    color_hand_tip_right_q_estimate, self.color_hand_tip_right_p = KalmanFilter.estimate(self.color_hand_tip_right_p, color_hand_tip_right_q_loc_meas, color_hand_tip_right_q_estimate)

                # -------------------------------------- Store Values -------------------------------------------------------

                # --------------------------------------- Hand Right --------------------------------------------------------

                kinect_world_hand_right.append([world_hand_right_x, world_hand_right_y, world_hand_right_z])  # mm
                kinect_color_hand_right.append([color_hand_right_x, color_hand_right_y, color_hand_right_z])  # pixels
                kalman_world_hand_right.append([float(world_hand_right_q_estimate[0]), float(world_hand_right_q_estimate[1]), float(world_hand_right_q_estimate[2])])  # mm
                if kalmanInit:
                    kalman_color_hand_right.append([float(color_hand_right_q_estimate[0]), float(color_hand_right_q_estimate[1]), float(color_hand_right_q_estimate[2])])  # pixels

                # --------------------------------------- Wrist Right -------------------------------------------------------

                kinect_world_wrist_right.append([world_wrist_right_x, world_wrist_right_y, world_wrist_right_z])  # mm
                kinect_color_wrist_right.append([color_wrist_right_x, color_wrist_right_y, color_wrist_right_z])  # pixels
                kalman_world_wrist_right.append([float(world_wrist_right_q_estimate[0]), float(world_wrist_right_q_estimate[1]), float(world_wrist_right_q_estimate[2])])  # mm
                if kalmanInit:
                    kalman_color_wrist_right.append([float(color_wrist_right_q_estimate[0]), float(color_wrist_right_q_estimate[1]), float(color_wrist_right_q_estimate[2])])  # pixels

                # --------------------------------------- Hand Tip Right ----------------------------------------------------

                kinect_world_hand_tip_right.append([world_hand_tip_right_x, world_hand_tip_right_y, world_hand_tip_right_z])  # mm
                kinect_color_hand_tip_right.append([color_hand_tip_right_x, color_hand_tip_right_y, color_hand_tip_right_z])  # pixels
                kalman_world_hand_tip_right.append([float(world_hand_tip_right_q_estimate[0]), float(world_hand_tip_right_q_estimate[1]), float(world_hand_tip_right_q_estimate[2])])  # mm
                if kalmanInit:
                    kalman_color_hand_tip_right.append([float(color_hand_tip_right_q_estimate[0]), float(color_hand_tip_right_q_estimate[1]), float(color_hand_tip_right_q_estimate[2])])  # pixels

                """
                Formula to compensate for the radial and tangential distortion:
                Radial:
                            xCorrect = x(1 + k1*r^2 + k2*r^4 + k3*r^6)
                            yCorrect = y(1 + k1*r^2 + k2*r^4 + k3*r^6)

                Tangential: 
                            xCorrect = x + [2*p1*x*y + p2*(r^2 + 2*x^2)]
                            yCorrect = y + [2*p2*x*y + p1*(r^2 + 2*y^2)]

                Where (k1 k2 p1 p2 k3 ) are the distortion parameters from the camera calibration and r = squareRoot(x^2 + y^2)
                !----( see /calibrate/IR/config.json or /calibrate/Color/config.json )

                Conversion to World Coordinates using the camera calibration parameters:

                [ x ]    [ focalLengthX         0        principalPointOffsetX0 ]   [ X ]
                [ y ]  = [      0          focalLengthY  principalPointOffsetY0 ] * [ Y ]
                [ w ]    [      0               0                   1           ]   [ Z ]

                To transform from color or depth coordinates follow this formula:

                world_x = (color_x - COLOR_PRINCIPAL_POINT_OFFSET_XO) * (world_z / COLOR_FOCAL_LENGTH_X)
                world_y = -(color_y - COLOR_PRINCIPAL_POINT_OFFSET_YO) * (world_z / COLOR_FOCAL_LENGTH_Y)

                world_x = (depth_x - DEPTH_PRINCIPAL_POINT_OFFSET_XO) * (world_z / DEPTH_FOCAL_LENGTH_X)
                world_y = -(depth_y - DEPTH_PRINCIPAL_POINT_OFFSET_YO) * (world_z / DEPTH_FOCAL_LENGTH_Y)

                """

                # Find orientation
                rot_x, rot_y, rot_z = orientation.orientation(kalman_world_hand_right[-1], kalman_world_wrist_right[-1])

                # Call Decision function
                human_state = decision.decision(kalman_world_hand_tip_right, self._counter, fabric_points, self.hand_right_confidence, self.hand_right_state)

                # Call coordination function if human is ready
                if human_state == 'READY' and gripper_state != "CLOSED":
                    human_corner, human_corner_stop, robot_corner, robot_corner_stop = decision.corners(kalman_world_hand_right[-1], kalman_world_wrist_right[-1], kalman_world_hand_tip_right[-1], fabric_points)
                    # decision.visualize(kalman_world_hand_right, self._counter, fabric_points, kalman_world_wrist_right, human_corner, human_direction, robot_corner, robot_direction)

                # Simulate Robot in RoboDK
                if Sim:

                    if robotInit:
                        # Initialise Robot Space at start
                        robot, pose, hand_model, kinect_model, gripper_model, br, bl, tl, tr = robotSim._init(q_relev_max_speed, self.rgb_focal_length, self.rgb_field_of_view, fabric_center, fabric_points, fabric_width, fabric_height,
                                                                                                              kalman_world_hand_right[-1], kinect_distance_from_plane, kinect_tilt, check_col=False)

                        robotInit = False
                    else:
                        """ ================== ATI FT Sensor Logic ================== """
                        if startSensor and not daq_thread:
                            try:
                                if gripper.graspBias:
                                    # Zero the sensor values
                                    self.daq.send('hello')
                                    self.daq.receive()
                                    self.daq.send('bias')
                                    self.daq.receive()
                                    gripper.graspBias = False
                                if graspUnbias:
                                    # Unbias sensor values
                                    self.daq.send('hello')
                                    self.daq.receive()
                                    self.daq.send('unbias')
                                    self.daq.receive()
                                    graspUnbias = False
                                self.daq.send('hello')
                                m = self.daq.receive(ret=True)
                                self.daq.send('forces')
                                forces = self.daq.receive(ret=True)
                                try:
                                    forces = [float(x) for x in forces.split(',')]
                                except Exception as e:
                                    """
                                    Catch exception where server has not initialized and fixed the correct
                                    sequence for the ATI FT Sensor and fixing sequence is transmitted.
                                    """
                                    print("[ATI FT CLIENT]: SERVER NOT INITIALISED \n {}".format(e))
                                    forces = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                                # Initialize Kalman Filters
                                if kalman_ft_first_time:
                                    forces_q_estimate = np.array([[forces[0]], [forces[1]], [forces[2]], [forces[3]], [forces[4]], [forces[5]]])
                                    kalman_ft_first_time = False
                                # Calculate Kalman Filters
                                forces_loc_meas = np.array([[forces[0]], [forces[1]], [forces[2]]])
                                forces_q_estimate, self.forces_p = KalmanFilter.estimate(self.forces_p, forces_loc_meas, forces_q_estimate, force_torque=True)
                                # Calculate changes in forces
                                fx, fy, fz = decision.forces_consideration(forces_q_estimate[0], forces_q_estimate[1], forces_q_estimate[3])
                                sensor_forces.append([float(forces[0]), float(forces[1]), float(forces[2])])
                                kalman_forces.append([float(forces_q_estimate[0]), float(forces_q_estimate[1]), float(forces_q_estimate[3])])
                                forces_control.append([float(fx), float(fz)])
                                # Update robot movement
                                x = x + fx
                                # y = y + fy
                                z = z + fz
                                # print(x, y, z)
                            except Exception as e:
                                print("[ATI FT CLIENT]: {}".format(e))

                        # If startSim is false robot has moved to corner and is ready
                        if not startSim:
                            human_state = 'READY'

                        if human_state == 'READY':
                            x, y, z = decision._collaborate(kalman_world_hand_right, self._counter, human_corner, human_corner_stop, self.hand_right_confidence)
                            movements_rdk.append([x, y, z])
                        else:
                            x = (kalman_world_hand_right[self._counter][0] - kalman_world_hand_right[self._counter - 1][0]) / 1.1
                            y = (kalman_world_hand_right[self._counter][1] - kalman_world_hand_right[self._counter - 1][1]) / 1.3
                            z = (kalman_world_hand_right[self._counter][2] - kalman_world_hand_right[self._counter - 1][2]) / 3

                        # print("[KINECT] X: {}, Y: {}, Z: {}".format("%.3f" % x, "%.3f" % y, "%.3f" % z))
                        print("[KINECT] DX: {}, DY: {}, DZ: {}".format("%.3f" % x, "%.3f" % y, "%.3f" % z))
                        print("[KINECT] FX: {}, FY: {}, FZ: {}".format("%.3f" % x, "%.3f" % y, "%.3f" % z))

                        if gripper.gripper_state == "CLOSED" and gripper._done:
                            gripper._done = False
                            startGripper = gripper.startGripper
                            waitGripper = gripper.waitGripper
                            gripper_state = gripper.gripper_state
                            monitorStop = gripper.monitorStop
                            graspBias = gripper.graspBias
                            daq_thread = gripper.daq_thread
                            thread_gripper.join()
                            del thread_gripper

                        """ start thread with gripper here """
                        if human_state == 'READY' and waitGripper and not startSim and startGripper and startSensor:
                            gripper_thread = True
                            daq_thread = True
                            if start_thread:
                                print("[KINECT] Started gripper thread")
                                thread_gripper = threading.Thread(target=gripper.grasp)
                                thread_gripper.start()
                                start_thread = False

                        """ ============= Monitor Distance to STOP corner =========== """
                        # Monitor distance between the operator's hand and the stopping corner
                        if monitorStop and decision.monitor(fabric_points, kalman_world_hand_tip_right[-1], human_corner_stop, robot_corner_stop):
                            print('[KINECT] Fabric folded')
                            self.gripper_client.send('hello')
                            self.gripper_client.receive()
                            self.gripper_client.send('pub')
                            self.gripper_client.send('open')
                            self.gripper_client.receive()
                            if robotSim.move_up(self.rdk, robot, pose):
                                pass
                            gripper_state = "OPEN"
                            self._done = True
                            human_state = "IDLE"
                            fabric_folded = True

                        """ ===================== RoboDK Logic ====================== """
                        # Start Cloud Skeleton Visualize
                        if cloudInit and not daq_thread and self._counter % 15 == 0:
                            self.cloud_skeleton.skeleton.Update(self.joints)
                            self.cloud_skeleton.skeleton.UpdateView()
                            # too slow for the main algorithm
                            # threading.Thread(target=self.update_cloud_skeleton).start()

                        # Move Robot until gripped
                        if self._counter % 7 == 0 and not gripper_thread and gripper_state != 'CLOSED' and not fabric_folded:
                            robot_state = 'IDLE'
                            robot_state, startSim, waitGripper, robot_end_effector_path = robotSim._simulation(self.rdk, x, y, z, rot_y, rot_z, robot, pose, hand_model, self.hand_right_state, gripper_model, gripper_state, waitGripper, kinect_model, kalman_world_hand_tip_right[-1],
                                                                                                               human_state, robot_corner, br, bl, tl, tr, startSim, robot_end_effector_path, check_col=False)
                        # move robot after the gripper catches the fabric
                        if gripper_state == "CLOSED" and self._counter % 1 == 0 and not fabric_folded:
                            robot_state = 'IDLE'
                            robot_state, robot_end_effector_path = robotSim.collaborate(self.rdk, x, y, z, robot, pose, robot_end_effector_path, kinect_model, hand_model, gripper_model, self.hand_right_state, gripper_state, kalman_world_hand_tip_right[-1], rot_y, rot_z)


                """ ======================================== LOGS for DEBUGGING ======================================= """
                # print(waitGripper, gripper_state, startSim, startGripper)
                # print(human_state, robot_state, gripper_state, fabric_points[1, 0], fabric_points[1, 1], world_kalman_points[self._counter][0], world_kalman_points[self._counter][1],
                #      self.hand_right_confidence, self.hand_right_state, handRightOrientation, temp_roll*(180/np.pi), temp_pitch*(180/np.pi), temp_yaw*(180/np.pi), rot_x, rot_y, rot_z)
                # print(human_state, robot_state, gripper_state, int(fabric_points[1, 0]), int(fabric_points[1, 1]), int(world_kalman_points[self._counter][0]), int(world_kalman_points[self._counter][1]),
                #      int(rot_x), int(rot_y), int(rot_z), self.hand_right_confidence, self.hand_right_state, handRightOrientation)
                # print(human_state, robot_state, gripper_state, int(fabric_points[0, 0]), int(fabric_points[0, 1]), int(kalman_world_hand_tip_right[self._counter][0]),
                #       int(kalman_world_hand_tip_right[self._counter][1]), self.hand_right_confidence, self.hand_right_state, gesture_classifier.state, int(rot_x), int(rot_y), int(rot_z))
                # print(human_state, robot_state, gripper_state, startSim, self.hand_right_confidence, self.hand_right_state, gesture_classifier.state)
                # print((kalman_world_hand_right[self._counter][0] - kalman_world_hand_right[self._counter-1][0])*9, (kalman_world_hand_right[self._counter][1] - kalman_world_hand_right[self._counter-1][1])*9, (kalman_world_hand_right[self._counter][2] - kalman_world_hand_right[self._counter-1][2])*9)
                # print(human_state, robot_state, gripper_state, int(fabric_points[0, 0]), int(fabric_points[0, 1]), int(kalman_world_hand_tip_right[self._counter][0]),
                #       int(kalman_world_hand_tip_right[self._counter][1]), self.hand_right_confidence, self.hand_right_state, gesture_classifier.state, gripperRelease, int(x), int(y), int(z))
                # print(human_state, robot_state, gripper_state,
                #       [np.abs(kalman_world_hand_tip_right[self._counter][0] - fabric_points[0][0]), np.abs(kalman_world_hand_tip_right[self._counter][1] - fabric_points[0][1])],
                #       [np.abs(kalman_world_hand_tip_right[self._counter][0] - fabric_points[1][0]), np.abs(kalman_world_hand_tip_right[self._counter][1] - fabric_points[1][1])],
                #       [np.abs(kalman_world_hand_tip_right[self._counter][0] - fabric_points[2][0]), np.abs(kalman_world_hand_tip_right[self._counter][1] - fabric_points[2][1])],
                #       [np.abs(kalman_world_hand_tip_right[self._counter][0] - fabric_points[3][0]), np.abs(kalman_world_hand_tip_right[self._counter][1] - fabric_points[3][1])])
                """ =================================================================================================== """

                """ ========================== RoboDK Cloud Point =============================="""
                # Import space as a pointcloud (be sure to not use the above robodk space and check off the appropriate flags)
                # This option requires a lot of computing power
                if cloudPointInit:
                    robotSim.pointcloud(point_cloud, self._depth, skip_bits=0)
                    cloudPointInit = False

                """ =============================== Update Panel =============================== """
                # panel.panel(kinect_world_hand_right[-1], kalman_world_hand_right[-1], human_state, robot_state, gripper_state, rot_x, rot_y, rot_z)
                # if self._counter % 2 == 0:
                #    visio.page(kinect_world_hand_right[-1], kalman_world_hand_right[-1], human_state, robot_state, gripper_state, rot_x, rot_y, rot_z, frame_rate)

                """ ================================= Pygame Screen Draw ====================="""
                # Draw Skeleton track and Kalman prediction to screen (AquaMarine = Kalman , Other = Kinect)
                if skeletonInit:
                    self.draw_body(color_joint_points, SKELETON_COLORS[self.body_id])  # Draw kinect skeleton track

                # Print kalman prediction on screen
                if kalmanInit:
                    self.draw_kinect([color_hand_right_q_estimate[0], color_hand_right_q_estimate[1], color_hand_right_q_estimate[2]], SKELETON_COLORS[10])  # Draw Kalman estimation of hand right
                    self.draw_kinect([color_wrist_right_q_estimate[0], color_wrist_right_q_estimate[1], color_wrist_right_q_estimate[2]], SKELETON_COLORS[10])  # Draw Kalman estimation of wrist right
                    self.draw_kinect([color_hand_tip_right_q_estimate[0], color_hand_tip_right_q_estimate[1], color_hand_tip_right_q_estimate[2]], SKELETON_COLORS[10])  # Draw kalman estimation of hand tip right

                # Store Time for plots
                sum_time.append((1 / frame_rate) * self._counter)
                history.append(time.time() - time_it)

                # Update Counter
                self._counter += 1
                if self._counter == self.Limit:
                    self._done = True

                """ =================================== Graphs and Plots ========================="""
                # Break inner loop if limit is reached
                if self._done:
                    _write(sum_time, kinect_world_hand_right, kalman_world_hand_right, robot_end_effector_path, movements_rdk, history, sensor_forces, kalman_forces, forces_control)
                    plot(sum_time, kinect_world_hand_right, kalman_world_hand_right, kinect_color_hand_right, kalman_color_hand_right)
                    _time, kinect, kalman, robot, movements, _history = reader()
                    final_plots(_time, kinect, kalman, robot, movements, _history)
                    break

            """================================ Pygame Screen Update ======================"""
            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size)
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height))
            surface_to_draw = pygame.transform.flip(surface_to_draw, True, False)  # Flip horizontally

            """============================ Fabric Extraction ============================="""
            # Find Bounding box, dimensions, orientation and ISO parameters
            if self._color is not None and self._depth is not None and dim and not cal and not sensorInit and not gripperInit:
                try:
                    dim, fabric_points, fabric_center, fabric_width, fabric_height, q_relev_max_speed = self.find_fabric(dim)
                except Exception as e:
                    print("[FABRIC EXTRACTION]: {}".format(e))

            self._screen.blit(surface_to_draw, (0, 0))
            """
            if not dim and self._counter <= 50*self.Limit:
                print("[MAIN]: Save Image {}".format(self._counter))
                pygame.image.save(surface_to_draw, 'images/orientation3.png')
                self._counter += 1
            """
            surface_to_draw = None
            pygame.display.update()
            # --- Update the screen
            pygame.display.flip()
            # --- Limit frame rates
            self._clock.tick(frame_rate)
            # print('Frames: {}'.format(1/(time.time() - time_it)))
            # sys.stdout.flush()

        # Close Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()


if __name__ == '__main__':
    # For user
    print('+-----------------------------+')
    print('[MAIN] Elapsed Time: {} seconds'.format("%3.f" % (time.time() - elapsed_time)))
    print('[MAIN] Loaded: 100%')
    print('[MAIN] Starting...')
    print('+-----------------------------+')
    # Initialize the Clients Class
    clients = Clients()
    # Start ATI Connection
    if sensorInit:
        try:
            sensorInit, startSensor = clients.ati_client_connect()
        except Exception as e:
            print("[MAIN] ATI Sensor not Connected")
            print(e)
    # Start Gripper Connection
    if gripperInit:
        try:
            gripperInit, startGripper, gripper_status = clients.gripper_client_connect()
        except Exception as e:
            print('[MAIN] Gripper not connected')
            print(e)
    # Reset RoboDK
    if Sim:
        clients.robodk_client_connect()
    # Reset RoboDK cloud skeleton
    operator = RoboDK_CloudSkeleton(clients)
    if Sim:
        operator.reset()
    # Start Gesture Classifier
    if gestureInit:
        gesture_classifier = Gesture_Classifier()
    # Initialize ReflexOne and run thread
    gripper = ReflexOne(clients)
    KinectApp = KinectMain(clients, operator)
    KinectApp.run()
    # Destroy any remaining windows
    cv2.destroyAllWindows()
