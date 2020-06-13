# Different utilities to perform quaternion operations


def quaternion2matrix(quaternion):
    """
    Calculate the rotation matrix based on a quaternion
    :param quaternion: list quaternion coefficient [qw, qx, qy, qz]
    :return: transformation matrix
    """
    import numpy as np
    # Calculate rotation matrix
    qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    matrix = np.array([[1 - 2 * qy * qy - 2 * qz * qz, 2 * qx * qy + 2 * qw * qz, 2 * qx * qz - 2 * qw * qy],
                       [2 * qx * qy - 2 * qw * qz, 1 - 2 * qx * qx - 2 * qz * qz, 2 * qy * qz + 2 * qw * qx],
                       [2 * qx * qz + 2 * qw * qy, 2 * qy * qz - 2 * qw * qx, 1 - 2 * qx * qx - 2 * qy * qy]])
    return matrix


def quaternion2rot(quaternion):
    """
    Calculate the orientation of a joint in 3 axis (pitch, yaw, roll) given the quaternion
    :param quaternion: list with quaternion coefficient [qw, qx, qy, qz]
    :return: array with [pitch, yaw, roll]
    """
    # Import libraries only for this function to minimize memory usage
    import numpy as np

    """
    # http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    transformation matrix  =  [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy + 2*qw*qz, 2*qx*qz - 2*qw*qy],
                              [2*qx*qy - 2*qw*qz, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz + 2*qw*qx],
                              [2*qx*qz + 2*qw*qy, 2*qy*qz - 2*qw*qx, 1 - 2*qx*qx - 2*qy*qy]
    """
    qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    # http://www.gregslabaugh.net/publications/euler.pdf  <------ Better
    # https://www.learnopencv.com/rotation-matrix-to-euler-angles/
    singularity = 2 * (qx * qy + qz * qw)

    # Check for singularities around north and south pole
    if singularity == 1:
        pitch = np.arctan2(2 * (qy * qz + qx * qw), 1 - 2 * qx * qx - 2 * qy * qy) * (180.0 / np.pi)  # degrees (ψ)
        yaw = np.arcsin((-1) * (2 * qx * qz - 2 * qy * qw)) * (180.0 / np.pi)  # degrees   (θ)
        roll = 0  # degrees  (φ)
    elif singularity == -1:
        pitch = np.arctan2((-1) * (2 * qy * qz - 2 * qx * qw), 1 - 2 * qx * qx - 2 * qz * qz) * (180.0 / np.pi)  # degrees
        yaw = np.arcsin((-1) * (2 * qx * qz - 2 * qy * qw)) * (180.0 / np.pi)  # degrees
        roll = 0  # degrees
    else:
        pitch = np.arctan2(2 * qy * qz + 2 * qx * qw, 1 - 2 * qx * qx - 2 * qy * qy) * (180.0 / np.pi)  # degrees (ψ)
        yaw = np.arcsin((-1) * (2 * qx * qz - 2 * qy * qw)) * (180.0 / np.pi)  # degrees   (θ)
        roll = np.arctan2(2 * qx * qy + 2 * qz * qw, 1 - 2 * qy * qy - 2 * qz * qz) * (180.0 / np.pi)  # degrees  (φ)

    return np.array([pitch, yaw, roll])


def rot2quaternion(rot_x, rot_y, rot_z):
    """
    Define a quaternion to rotate a vector with specific angles
    Angle Inputs as radians!!
    :param rot_x: float angle in x axis
    :param rot_y: float angle in y axis
    :param rot_z: float angle in z axis
    :return: list with rotation quaternnion
    """
    # Import libraries only for this function to minimize memory usage
    import numpy as np

    # Calculate quaternion based on rotations
    c1 = np.cos(rot_x/2)
    c2 = np.cos(rot_y/2)
    c3 = np.cos(rot_z/2)
    s1 = np.sin(rot_x/2)
    s2 = np.sin(rot_y/2)
    s3 = np.sin(rot_z/2)
    qw = c1*c2*c3 - s1*s2*s3
    qx = s1*s2*c3 + c1*c2*s3
    qy = s1*c2*c3 + c1*s2*s3
    qz = c1*s2*c3 - s1*c2*s3
    return np.array([qw, qx, qy, qz])


def rotate(quat, vec):
    """
    Rotate a vector based on a specific quaternion
    :param quat: list with quaternion coefficient [qw, qx, qy, qz]
    :param vec: list with vector coefficients
    :return: list with rotated vector coefficients
    """
    # Import libraries only for this function to minimize memory usage
    import numpy as np
    # Calculate rotation matrix
    qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
    matrix = np.array([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy + 2*qw*qz, 2*qx*qz - 2*qw*qy],
                       [2*qx*qy - 2*qw*qz, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz + 2*qw*qx],
                       [2*qx*qz + 2*qw*qy, 2*qy*qz - 2*qw*qx, 1 - 2*qx*qx - 2*qy*qy]])
    # Calculate rotated vector
    vector = np.dot(matrix, vec)
    return vector


def show(quaternion, point):
    """
    Show original and rotated vector
    :param quaternion: list with quaternion coefficient [qw, qx, qy, qz]
    :param point: list with vector coefficients
    :return: None
    """
    # Import libraries only for this function to minimize memory usage
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    # Define axis vectors
    x, y, z = np.array([[-1, -1, -1], [-1, -1, -1], [-1, -1, -1]])  # Origins of axis vectors
    u, v, w = np.array([[[1], [0], [0]], [[0], [1], [0]], [[0], [0], [1]]])  # Length of Axis vectors on each direction
    # Define rotated point
    p = rotate(quaternion, point)
    # Plot Vectors
    fig = plt.figure(figsize=(8, 5))
    ax = Axes3D(fig)
    ax.quiver(0, 0, 0, point[0], point[1], point[2], color='orange', label='Original Vector')
    ax.quiver(0, 0, 0, p[0], p[1], p[2], color='purple', label='Rotated Vector')
    ax.quiver(x[0], y[0], z[0], u[0], v[0], w[0], color='red')
    ax.quiver(x[1], y[1], z[1], u[1], v[1], w[1], color='blue')
    ax.quiver(x[2], y[2], z[2], u[2], v[2], w[2], color='green')
    ax.legend()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    plt.show()
