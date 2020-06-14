"""
Author: Konstantinos Angelopoulos
Date: 04/02/2020
All rights reserved.
Feel free to use and modify and if you like it give it a star.

Script to Handle the Robot movement based on the decision result
"""


"""

Safety parameters as specified by ISO/TS 15066 in a human-robot collaboration scenario
where the hand is the only body part that could collide with the end effector and absorb
kinetic energy, thus harming the operator.

----------------- Omron Adept Cobra S800
Payload: rated=2kg, max=5.5 kg
Mass: 43 kg
Volume of working envelope: 0.027552 m3
Pose repeatability (ISO 9283): xy = ± 0.017 mm, z = ± 0.003 mm, theta = ± 0.019°
Max. Reach: 800 mm

--------------- KUKA LWR IV+
Payload: 7 kg
Mass: 16 kg ( excluding controller )
Volume of working envelope: 1.84 m3
Pose repeatability (ISO 9283): ± 0.05 mm
Max. Reach: 790 mm

------------ Gripper ReFlex  1 (https://www.labs.righthandrobotics.com/reflexhand)
Number of fingers: 3
Degrees of freedom:	3 x bending (1 in each finger), 2 x rotation (1 in each of 2 fingers)
Actuators:	3 x Dynamixel AX-12A motors, 2 x Dynamixel XL-320 motors
Sensors per finger:	0
Power:	12 V DC
Communications interface:	USB
Weight:	0.8 kg

------------- ATI FT Gamma 65-5 ( https://www.ati-ia.com/products/ft/ft_models.aspx?id=Gamma )
Force - Torque Sensor
f ~= 150 Hz
Weight = 0.255 kg
Diameter = 75.4 mm
Height = 33.3 mm

============== Tool weight is defined as ATIGamma_payload + Gripper_payload + MountingBase_payload + AdaptorPlate_payload
"""


def iso(fabric_width, fabric_height):
    """
    Calculate robot iso speed
    :param fabric_width: float fabric width
    :param fabric_height: float fabric height
    :return: float quasi state robot speed, float transient state robot speed, float relevant speed
    """
    # Import here for optimization
    import numpy as np
    # Every value in S.I.
    k = 75000  # (N/m) Effective spring constant for hand from ISO/TS 15066
    E = 0.49  # (Joule) for hand from ISO/TS 15066 ( E = (force**2) / (2 * k) = ( (Area**2)*(pressure**2) ) / (2*k) )
    mh = 0.6  # (kg) effective mass of hand as specified by ISO/TS 15066
    robot_payload = 7.0  # (kg) as specified by the manufacturer
    tool_payload = 1.441  # (kg) ( tool is defined above ) as weighted ( approximation )
    # Lightweight clothes have 30-150 gr per squared meter (median = 90 gr per m2), so we need the area of the cloth
    # https://blog.fabricuk.com/understanding-fabric-weight/
    # https://www.onlineclothingstudy.com/2018/09/what-is-gsm-in-fabric.html
    # https://slideplayer.com/slide/4134611/
    piece_payload = 0.5 * 0.15 * fabric_width * fabric_height  # (kg) of theoretical piece of cloth
    ml = tool_payload + piece_payload  # (kg) the effective payload of the robot system, including tooling and workpiece
    robot_mass = 16  # (kg) robot mass as given by the manufacturer
    M = robot_mass * 0.9  # (kg) total mass of the moving parts of the robot (assuming that only the 90% of the robot is moving)
    mr = (M/2) + ml  # (kg) effective mass of robot as a function of robot posture and motion
    m = ((1/mh) + (1/mr))**(-1)  # (kg) reduced mass of the two-body system

    # Quasi-static contact ( When the robot crushes the hand against a fixed object )
    q_max_force = 140  # (Newton)
    q_max_press = 2000000  # (N/m2) assuming dominant hand ( non dominant = 1.9 * 10^6 N/m2)
    q_relev_max_speed = q_max_force / np.sqrt(m*k)  # (m/sec) relevant max speed limit for robots in quasi-static contact

    # Transient contact ( When the robot comes in contact with the hand but the hand can move freely )
    t_max_force = 2 * q_max_force  # (Newton)
    t_max_press = 2 * q_max_press  # (N/m2) assuming dominant hand
    t_relev_max_speed = t_max_force / np.sqrt(m*k)  # (m/sec) relevant max speed limit for robots in transient contact

    # normal robot speed
    relev_speed = np.sqrt((2*E)/m)  # (m/sec) relevant speed speed between the robot and the human body region
    # we will assume that the relev_speed is the acceptable speed, since is given by the maximum acceptable energy
    # transferred by the robot to the operator's hand
    # 1 m/sec = 1000 mm/sec
    return q_relev_max_speed, t_relev_max_speed, relev_speed
