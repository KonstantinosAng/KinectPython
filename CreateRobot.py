"""

Import the Robot's Step Files and Color/Scale/Assemble them using the instructions in /RoboDK/KUKA/KUKA LWR IV+ Description
(for Original=kuka_lwr_model_description.json, for custom=custom_kuka_lwr_model_description, for custom2=custom_kuka_lwr_model_description_2) 
before running the code to complete the robot model.

#########################################################################
######### To quickly color and scale use the next lines of code #########
#########################################################################

from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox

RDK = Robolink()

for station in RDK.ItemList():
    for item in station.Childs():
        item.Scale(1000)
        item.setColor(255/255, 85/255, 0/255, 255/255)

########################################################################
#### For custom2 run these commands before assembling the stl files ####
########################################################################

from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
RDK = Robolink()


for station in RDK.ItemList():
    for item in station.Childs():
        item.setGeometryPose(item.Pose()*rotz(np.pi))
        item.Scale(1000)
        item.setColor(255/255, 85/255, 0/255, 255/255)

and after building the mechanism and import it, in order to rotate the robot run:

from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox

RDK = Robolink()

ref = RDK.Item('reference2')
ref.setPoseAbs(ref.Pose()*rotz(pi))

##############################################################################################
##### The original option is just the robot model without any inverted sense and joints ######
##### home are [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ###########################################
##############################################################################################
##### The custom robot is the real model that has the same limitations, home joints and ######
##### senses as the REAl KUKA LWR but the X and Y axis system are inverted ###################
##############################################################################################
##### The custom2 robot is the same as the custom option but with the X and Y axis being #####
##### the same as the REAL KUKA ROBOT ########################################################
##############################################################################################

"""

# Start the RoboDK API
from robolink.robolink import *
from robodk.robodk import *
import json
import os


# ORIGINAL ROBOT DATA
with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RoboDK/KUKA/KUKA LWR IV+ Description/kuka_lwr_model_description.json')) as config_file:
    data = json.load(config_file)
    original_robot_name = data['Robot name']
    original_robot_dof = data['DOF']
    original_robot_joint1 = data['Joint 1']
    original_robot_joint2 = data['Joint 2']
    original_robot_joint3 = data['Joint 3']
    original_robot_joint4 = data['Joint 4']
    original_robot_joint5 = data['Joint 5']
    original_robot_joint6 = data['Joint 6']
    original_robot_joint7 = data['Joint 7']
    original_robot_joints_build = [original_robot_joint1["Build joints"], original_robot_joint2["Build joints"], original_robot_joint3["Build joints"],
                                   original_robot_joint4["Build joints"], original_robot_joint5["Build joints"], original_robot_joint6["Build joints"],
                                   original_robot_joint7["Build joints"]]

    original_robot_joints_home = [original_robot_joint1["Home"], original_robot_joint2["Home"], original_robot_joint3["Home"],
                                   original_robot_joint4["Home"], original_robot_joint5["Home"], original_robot_joint6["Home"], original_robot_joint7["Home"]]

    original_robot_parameters = [data["d1"], data["d3"], data["d5"], data["d7"], data["dtheta1"], data["dtheta2"], data["dtheta3"], data["dtheta4"],
                                 data["dtheta5"], data["dtheta6"], data["dtheta7"]]

    original_robot_joint_senses = [original_robot_joint1["Invert Sense"], original_robot_joint2["Invert Sense"], original_robot_joint3["Invert Sense"],
                                   original_robot_joint4["Invert Sense"], original_robot_joint5["Invert Sense"], original_robot_joint6["Invert Sense"],
                                   original_robot_joint7["Invert Sense"]]

    original_robot_joint_lower_limit = [original_robot_joint1["Minimum limit"], original_robot_joint2["Minimum limit"], original_robot_joint3["Minimum limit"],
                                        original_robot_joint4["Minimum limit"], original_robot_joint5["Minimum limit"], original_robot_joint6["Minimum limit"],
                                        original_robot_joint7["Minimum limit"]]

    original_robot_joint_upper_limit = [original_robot_joint1["Maximum limit"], original_robot_joint2["Maximum limit"], original_robot_joint3["Maximum limit"],
                                        original_robot_joint4["Maximum limit"], original_robot_joint5["Maximum limit"], original_robot_joint6["Maximum limit"],
                                        original_robot_joint7["Maximum limit"]]

    original_robot_base_pose = data["Base shift"]
    original_robot_tool_pose = data["End-effector shift"]

# CUSTOM ROBOT DATA
with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RoboDK/KUKA/KUKA LWR IV+ Description/custom_lwr_model_description.json')) as config_file:
    data = json.load(config_file)
    custom_robot_name = data['Robot name']
    custom_robot_dof = data['DOF']
    custom_robot_joint1 = data['Joint 1']
    custom_robot_joint2 = data['Joint 2']
    custom_robot_joint3 = data['Joint 3']
    custom_robot_joint4 = data['Joint 4']
    custom_robot_joint5 = data['Joint 5']
    custom_robot_joint6 = data['Joint 6']
    custom_robot_joint7 = data['Joint 7']
    custom_robot_joints_build = [custom_robot_joint1["Build joints"], custom_robot_joint2["Build joints"], custom_robot_joint3["Build joints"],
                                   custom_robot_joint4["Build joints"], custom_robot_joint5["Build joints"], custom_robot_joint6["Build joints"],
                                   custom_robot_joint7["Build joints"]]

    custom_robot_joints_home = [custom_robot_joint1["Home"], custom_robot_joint2["Home"], custom_robot_joint3["Home"],
                                custom_robot_joint4["Home"], custom_robot_joint5["Home"], custom_robot_joint6["Home"], custom_robot_joint7["Home"]]

    custom_robot_parameters = [data["d1"], data["d3"], data["d5"], data["d7"], data["dtheta1"], data["dtheta2"], data["dtheta3"], data["dtheta4"],
                               data["dtheta5"], data["dtheta6"], data["dtheta7"]]

    custom_robot_joint_senses = [custom_robot_joint1["Invert Sense"], custom_robot_joint2["Invert Sense"], custom_robot_joint3["Invert Sense"],
                                 custom_robot_joint4["Invert Sense"], custom_robot_joint5["Invert Sense"], custom_robot_joint6["Invert Sense"],
                                 custom_robot_joint7["Invert Sense"]]

    custom_robot_joint_lower_limit = [custom_robot_joint1["Minimum limit"], custom_robot_joint2["Minimum limit"], custom_robot_joint3["Minimum limit"],
                                      custom_robot_joint4["Minimum limit"], custom_robot_joint5["Minimum limit"], custom_robot_joint6["Minimum limit"],
                                      custom_robot_joint7["Minimum limit"]]

    custom_robot_joint_upper_limit = [custom_robot_joint1["Maximum limit"], custom_robot_joint2["Maximum limit"], custom_robot_joint3["Maximum limit"],
                                      custom_robot_joint4["Maximum limit"], custom_robot_joint5["Maximum limit"], custom_robot_joint6["Maximum limit"],
                                      custom_robot_joint7["Maximum limit"]]

    custom_robot_base_pose = data["Base shift"]
    custom_robot_tool_pose = data["End-effector shift"]

# CUSTOM 2 ROBOT DATA
with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RoboDK/KUKA/KUKA LWR IV+ Description/custom_lwr_model_description_2.json')) as config_file:
    data = json.load(config_file)
    custom_2_robot_name = data['Robot name']
    custom_2_robot_dof = data['DOF']
    custom_2_robot_joint1 = data['Joint 1']
    custom_2_robot_joint2 = data['Joint 2']
    custom_2_robot_joint3 = data['Joint 3']
    custom_2_robot_joint4 = data['Joint 4']
    custom_2_robot_joint5 = data['Joint 5']
    custom_2_robot_joint6 = data['Joint 6']
    custom_2_robot_joint7 = data['Joint 7']
    custom_2_robot_joints_build = [custom_2_robot_joint1["Build joints"], custom_2_robot_joint2["Build joints"], custom_2_robot_joint3["Build joints"],
                                   custom_2_robot_joint4["Build joints"], custom_2_robot_joint5["Build joints"], custom_2_robot_joint6["Build joints"],
                                   custom_2_robot_joint7["Build joints"]]

    custom_2_robot_joints_home = [custom_2_robot_joint1["Home"], custom_2_robot_joint2["Home"], custom_2_robot_joint3["Home"],
                                   custom_2_robot_joint4["Home"], custom_2_robot_joint5["Home"], custom_2_robot_joint6["Home"], custom_2_robot_joint7["Home"]]

    custom_2_robot_parameters = [data["d1"], data["d3"], data["d5"], data["d7"], data["dtheta1"], data["dtheta2"], data["dtheta3"], data["dtheta4"],
                                 data["dtheta5"], data["dtheta6"], data["dtheta7"]]

    custom_2_robot_joint_senses = [custom_2_robot_joint1["Invert Sense"], custom_2_robot_joint2["Invert Sense"], custom_2_robot_joint3["Invert Sense"],
                                   custom_2_robot_joint4["Invert Sense"], custom_2_robot_joint5["Invert Sense"], custom_2_robot_joint6["Invert Sense"],
                                   custom_2_robot_joint7["Invert Sense"]]

    custom_2_robot_joint_lower_limit = [custom_2_robot_joint1["Minimum limit"], custom_2_robot_joint2["Minimum limit"], custom_2_robot_joint3["Minimum limit"],
                                        custom_2_robot_joint4["Minimum limit"], custom_2_robot_joint5["Minimum limit"], custom_2_robot_joint6["Minimum limit"],
                                        custom_2_robot_joint7["Minimum limit"]]

    custom_2_robot_joint_upper_limit = [custom_2_robot_joint1["Maximum limit"], custom_2_robot_joint2["Maximum limit"], custom_2_robot_joint3["Maximum limit"],
                                        custom_2_robot_joint4["Maximum limit"], custom_2_robot_joint5["Maximum limit"], custom_2_robot_joint6["Maximum limit"],
                                        custom_2_robot_joint7["Maximum limit"]]

    custom_2_robot_base_pose = data["Base shift"]
    custom_2_robot_tool_pose = data["End-effector shift"]

RDK = Robolink()
custom = False
custom2 = True

if custom:
    robot_name = custom_robot_name
    DOFs = custom_robot_dof
    joints_build = custom_robot_joints_build
    joints_home = custom_robot_joints_home
    parameters = custom_robot_parameters
    joints_senses = custom_robot_joint_senses  # -1 = Inverted, +1 = Not Inverted
    lower_limits = custom_robot_joint_lower_limit
    upper_limits = custom_robot_joint_upper_limit
    base_pose = xyzrpw_2_pose(custom_robot_base_pose)
    tool_pose = xyzrpw_2_pose(custom_robot_tool_pose)
    list_objects = []

elif custom2:
    robot_name = custom_2_robot_name
    DOFs = custom_2_robot_dof
    joints_build = custom_2_robot_joints_build
    joints_home = custom_2_robot_joints_home
    parameters = custom_2_robot_parameters
    joints_senses = custom_2_robot_joint_senses  # -1 = Inverted, +1 = Not Inverted
    lower_limits = custom_2_robot_joint_lower_limit
    upper_limits = custom_2_robot_joint_upper_limit
    base_pose = xyzrpw_2_pose(custom_2_robot_base_pose)
    tool_pose = xyzrpw_2_pose(custom_2_robot_tool_pose)
    list_objects = []

else:
    robot_name = original_robot_name
    DOFs = original_robot_dof
    joints_build = original_robot_joints_build
    joints_home = original_robot_joints_home
    parameters = original_robot_parameters
    joints_senses = original_robot_joint_senses  # -1 = Inverted, +1 = Not Inverted
    lower_limits = original_robot_joint_lower_limit
    upper_limits = original_robot_joint_upper_limit
    base_pose = xyzrpw_2_pose(original_robot_base_pose)
    tool_pose = xyzrpw_2_pose(original_robot_tool_pose)
    list_objects = []


for i in range(DOFs + 1):
    if i == 0:
        itm = RDK.Item('base', ITEM_TYPE_OBJECT)
    else:
        itm = RDK.Item('link_'+str(i), ITEM_TYPE_OBJECT)
    list_objects.append(itm)

new_robot = RDK.BuildMechanism(MAKE_ROBOT_7DOF, list_objects, parameters, joints_build, joints_home, joints_senses, lower_limits, upper_limits, base_pose, tool_pose, robot_name)

if not new_robot.Valid():
    print("Failed to create the robot. Check input values.")
else:
    print("Robot/mechanism created: " + new_robot.Name())
