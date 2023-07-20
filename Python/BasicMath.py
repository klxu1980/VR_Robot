import numpy as np
import math


def euler2rotation(euler):
    """
    Calculates Rotation Matrix given euler angles. Angles are defined by degree.
    :param euler: 1-by-3 list [rx, ry, rz] angle in degree
    :return: rotation matrix
    """
    euler = [degree * math.pi / 180.0 for degree in euler]
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(euler[0]), -math.sin(euler[0])],
                    [0, math.sin(euler[0]), math.cos(euler[0])]])
    R_y = np.array([[math.cos(euler[1]), 0, math.sin(euler[1])],
                    [0, 1, 0],
                    [-math.sin(euler[1]), 0, math.cos(euler[1])]])
    R_z = np.array([[math.cos(euler[2]), -math.sin(euler[2]), 0],
                    [math.sin(euler[2]), math.cos(euler[2]), 0],
                    [0, 0, 1]])
    return np.dot(R_z, np.dot(R_x, R_y))


def rodrigues_rotation(r, theta):
    """
    罗德里格斯公式求旋转矩阵
    :param r: 旋转轴
    :param theta: 逆时针旋转theta
    :return: 旋转矩阵
    """
    normalized_n = r / np.linalg.norm(r)
    n = normalized_n.reshape(3, 1)
    nx, ny, nz = n[:, 0]
    K = np.array([
        [0, -nz, ny],
        [nz, 0, -nx],
        [-ny, nx, 0]])
    I = np.eye(3)
    R = np.add(I, np.add(K * math.sin(theta), (1 - math.cos(theta)) * np.dot(K, K)))
    return R
