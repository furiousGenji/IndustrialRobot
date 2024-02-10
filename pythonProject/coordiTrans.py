import numpy as np
from sympy import *

# RadialDistortion
k1 = -0.00712053357274876
k2 = 0.891640846161531
k3 = -1.70499476265774
# TangentialDistortion
p1 = -0.00149750334823676
p2 = -0.000244325493978436
# IntrinsicMatrix
fx = 706.706716497219
fy = 707.195482387565
cx = 323.322034105302
cy = 254.215721655113
epsilon = -0.696601265990033

# Aberration coefficient matrix
dist = np.array([k1, k2, p1, p2, k3])
# Camera Internal Reference Matrix
mtx = np.array([[fx, epsilon, cx],
                [0, fy, cy],
                [0, 0, 1]])

inverseIntrinsicMatrix = np.array([[1 / fx, - epsilon / (fx * fy), ((epsilon * cy) / (fx * fy)) - (cx / fx)], [0, 1 / fy, - cy / fy], [0, 0, 1]])


def uv_to_camera(u, v, z):
    """

    :param u: x of pixel
    :param v: z of pixel
    :param z: depth of object(along the z axis of camera coord)
    :return: position in camera coordinates
    """
    image_point = z * np.array([[u], [v], [1]])
    # return np.linalg.inv(intrinsicMatrix) @ image_point 使用np.linalg.inv计算结果错误
    return inverseIntrinsicMatrix @ image_point


def transform_matrix(rotate_x, rotate_y, rotate_z, x, y, w):
    """

    :param rotate_x: Camera coordinate system rotated around the x-axis in radians, clockwise, relative to the world coordinate system.
    :param rotate_y: Camera coordinate system rotated around the y-axis in radians, clockwise, relative to the world coordinate system.
    :param rotate_z: Camera coordinate system rotated around the z-axis in radians, clockwise, relative to the world coordinate system.
    :param x: The coordinates of the origin of the camera coordinate system x
    :param y: The coordinates of the origin of the camera coordinate system y
    :param w: The coordinates of the origin of the camera coordinate system z
    :return: World Coordinate System to Camera Coordinate System Transformation Matrix
    """
    rotX = np.array(
        [[1, 0, 0, 0], [0, cos(rotate_x), -sin(rotate_x), 0], [0, sin(rotate_x), cos(rotate_x), 0], [0, 0, 0, 1]])
    rotY = np.array(
        [[cos(rotate_y), 0, sin(rotate_y), 0], [0, 1, 0, 0], [-sin(rotate_y), 0, cos(rotate_y), 0], [0, 0, 0, 1]])
    rotZ = np.array(
        [[cos(rotate_z), -sin(rotate_z), 0, 0], [sin(rotate_z), cos(rotate_z), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    pos = np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, w], [0, 0, 0, 1]])
    transformMatrix = pos @ rotZ @ rotY @ rotX
    return transformMatrix


def camera_to_world(u, v, z, rotate_x, rotate_y, rotate_z, x, y, w):
    temp = np.vstack([uv_to_camera(u, v, z), 1])
    result = transform_matrix(rotate_x, rotate_y, rotate_z, x, y, w) @ temp
    return result
