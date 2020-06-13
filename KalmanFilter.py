"""
NON LINEAR KALMAN FILTER ON HAND MOVEMENT
"""

import numpy as np
# Linear Kalman Filtering of Hand Movement

"""

    Kalman Prediction Init
    xt+1 = xt + Vxt + 0.5*ax*t^2 for one dimension
    Vxt+1 = Vxt + axt
   [  x ]       [1, 0, 0, dt, 0, 0]     [  x ]    [dt^2/2]
   [  y ]       [0, 1, 0, 0, dt, 0]     [  y ]    [dt^2/2]
   [  z ]   =   [0, 0, 1, 0, 0, dt]  *  [  z ]  + [dt^2/2] * u
   [ Vx ]       [0, 0, 0, 1, 0, 0 ]     [ Vx ]    [  dt  ]
   [ Vy ]       [0, 0, 0, 0, 1, 0 ]     [ Vy ]    [  dt  ]
   [ Vz ] t+1   [0, 0, 0, 0, 0, 1 ]     [ Vz ] t  [  dt  ]
   +----+       +-----------------+     +----+    +------+
    xt+1                A                 xt          B

    zt = C * [x ; y ; z ; Vx ; Vy ; Vz]
    
"""
# GLOBAL KALMAN SETTINGS
frame_rate = 30  # Frame Rate for updating the screen
dt = 4.0 / frame_rate  # Time step for kalman
u = 0.008  # Velocity for Kalman prediction
noise_mag = 4  # noise for velocity how fast is speeding up

# ----------------- For 3 axis x, y, z position of hand from kinect
noise_x = 4  # Noise in kinect x values
noise_y = 4  # Noise in kinect y values  # rule of thumb ---> more noise is better approximation to real value
noise_z = 4  # Noise in kinect z values
velocity_x = 1  # Velocity in kinect x axis
velocity_y = 1  # Velocity in kinect y axis
velocity_z = 1  # Velocity in kinect z axis

ez = np.array([[noise_x, 0, 0],
               [0, noise_y, 0],
               [0, 0, noise_z]])  # noise matrix for x y z position measurements

ex = np.array([[(dt ** 4) / 4, 0, 0, (dt ** 3) / 2, 0, 0],
               [0, (dt ** 4) / 4, 0, 0, (dt ** 3) / 2, 0],
               [0, 0, (dt ** 4) / 4, 0, 0, (dt ** 3) / 2],
               [(dt ** 3) / 2, 0, 0, (dt ** 2), 0, 0],
               [0, (dt ** 3) / 2, 0, 0, (dt ** 2), 0],
               [0, 0, (dt ** 3) / 2, 0, 0, (dt ** 2)]])

ex = ex * (noise_mag ** 2)  # Convert process noise (stdv) into covariance matrix

a = np.array([[1, 0, 0, dt, 0, 0],
              [0, 1, 0, 0, dt, 0],
              [0, 0, 1, 0, 0, dt],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1]])  # State matrix for position in 3D space

inva = np.array([[1, 0, 0, 0, 0, 0],
                 [0, 1, 0, 0, 0, 0],
                 [0, 0, 1, 0, 0, 0],
                 [dt, 0, 0, 1, 0, 0],
                 [0, dt, 0, 0, 1, 0],
                 [0, 0, dt, 0, 0, 1]])  # Inverse of table a

b = np.array([[(dt ** 2) / 2],
              [(dt ** 2) / 2],
              [(dt ** 2) / 2],
              [dt],
              [dt],
              [dt]])  # State matrix

invc = np.array([[1, 0, 0],
                 [0, 1, 0],
                 [0, 0, 1],
                 [0, 0, 0],
                 [0, 0, 0],
                 [0, 0, 0]])  # Inverse of table c

c = np.array([[1, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0]])  # We multiply this to get the next predicted state only for position

# ----------------- For 3 axis x, y, z and two values for each axis Forces and Torques in each axis
""" !!!!!!!!!!!!!!!!!!!!!!!------------------------- DOES NOT WORK -----------------------------!!!!!!!!!!!!!!!!!!!!!!!!!!"""
force_x = 50  # Noise in forces on x values
force_y = 50  # Noise in forces on y values  # rule of thumb ---> more noise is better approximation to real value
force_z = 50  # Noise in forces z values
torque_x = 1  # Velocity in torques x axis
torque_y = 1  # Velocity in torques y axis
torque_z = 1  # Velocity in torques z axis
u_6 = 0.008  # Velocity for Kalman prediction
noise_mag_6 = 4  # noise for velocity how fast is speeding up
dt_6 = 3 / frame_rate

ex_6 = np.array([[(dt_6 ** 4) / 4, 0, 0, (dt_6 ** 3) / 2, 0, 0],
                 [0, (dt_6 ** 4) / 4, 0, 0, (dt_6 ** 3) / 2, 0],
                 [0, 0, (dt_6 ** 4) / 4, 0, 0, (dt_6 ** 3) / 2],
                 [(dt_6 ** 3) / 2, 0, 0, (dt_6 ** 2), 0, 0],
                 [0, (dt_6 ** 3) / 2, 0, 0, (dt_6 ** 2), 0],
                 [0, 0, (dt_6 ** 3) / 2, 0, 0, (dt_6 ** 2)]])

ex_6 = ex_6 * (noise_mag_6 ** 2)  # Convert process noise (stdv) into covariance matrix

ez_6 = np.array([[force_x, 0, 0],
                 [0, force_y, 0],
                 [0, 0, force_z]])  # noise matrix for x y z position measurements

invc_6 = np.array([[1, 0, 0],
                   [0, 1, 0],
                   [0, 0, 1],
                   [0, 0, 0],
                   [0, 0, 0],
                   [0, 0, 0]])  # Inverse of table c

c_6 = np.array([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0]])  # We multiply this to get the next predicted state only for position


# Initialise Kalman Values and Matrices
def initialise(force_torque=False):
    """
    Initialize frame rate, frequency and covariance matrix
    :param force_torque: boolean to return the covariance matrix for forces that has different noise values
    :return: array with covariance matrix, loop time, and frame rate
    """
    if not force_torque:
        # Return values for initializing different Kalman for different joints
        return ex, dt, frame_rate
    else:
        return ex_6


# Estimate Kalman Values
def estimate(p, location_measure, estimation, force_torque=False):
    """
    Estimate kalman movement
    :param p: array of covariance
    :param location_measure: list with measured state values [x, y, z, vx, vy, vz]
    :param estimation: list with previous estimation state [x, y, z, vx, vy, vz]
    :param force_torque: boolean to calculate for force/torque oe for position
    :return: list with estimated state values [x, y, z, vx, vy, vz], array of covariance p
    """
    if not force_torque:
        # Kalman Prediction of 3axis movement
        estimation = np.dot(a, estimation) + b * u
        p = np.dot(np.dot(a, p), inva) + ex
        invk = np.linalg.inv(np.dot(np.dot(c, p), invc) + ez)
        k = np.dot(np.dot(p, invc), invk)
        estimation = estimation + np.dot(k, (location_measure - np.dot(c, estimation)))
        p = np.dot((np.identity(6) - np.dot(k, c)), p)
        # Return Estimation and Position Deviation for given joint
        return estimation, p
    else:
        # Kalman Prediction of Hand Right
        estimation = np.dot(a, estimation) + b * u_6
        p = np.dot(np.dot(a, p), inva) + ex_6
        invk = np.linalg.inv(np.dot(np.dot(c_6, p), invc_6) + ez_6)
        k = np.dot(np.dot(p, invc_6), invk)
        estimation = estimation + np.dot(k, (location_measure - np.dot(c_6, estimation)))
        p = np.dot((np.identity(6) - np.dot(k, c_6)), p)
        # Return Estimation and Position Deviation for given joint
        return estimation, p


if __name__ == '__main__':
    # Test functionality
    import numpy as np
    meas = np.array([[2], [3], [4]])
    est = np.array([[2], [3], [4], [5], [6], [7]])
    ex, dt, frame_rate = initialise()  # Initialize Kalman Filters
    loc_p = ex
    est, loc_p = estimate(loc_p, meas, est, force_torque=False)
    test = 1
