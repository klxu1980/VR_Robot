from BasicMath import *


class Trans_Camera_VR(object):
    """
    TransMatrix calculates and contains all the transforming matrix between camera and VR.
    """
    def __init__(self, pos, euler, g_cam):
        """
        :param pos: 头盔、右手柄、左手柄在VR坐标系中的坐标
        :param euler: 头盔、右手柄、左手柄在VR坐标系中的姿态角
        :param g_cam: 相机坐标系中的重力向量
        """
        # 计算头盔、手柄坐标系到VR坐标系的变换阵
        self.head2vr = self.__augmented_matrix(euler2rotation(euler[0]), pos[0])
        self.r_handle2vr = self.__augmented_matrix(euler2rotation(euler[1]), pos[1])
        self.l_handle2vr = self.__augmented_matrix(euler2rotation(euler[2]), pos[2])

        # 计算相机坐标系到VR坐标系的变换矩阵，以及VR坐标系到相机坐标系的变换矩阵
        norm_g_cam = g_cam / np.linalg.norm(g_cam)
        norm_g_vr = np.array([0, 1, 0])
        n = np.cross(norm_g_cam, norm_g_vr)
        theta = math.acos(np.dot(norm_g_cam, norm_g_vr))
        r_vr2cam = rodrigues_rotation(n, theta)
        self.cam2vr = self.__augmented_matrix(r_vr2cam, pos[0])
        self.vr2cam = np.linalg.inv(self.cam2vr)

        # 手柄坐标系到相机坐标系的变换矩阵
        self.r_handle2cam = np.dot(self.vr2cam, self.r_handle2vr)
        self.l_handle2cam = np.dot(self.vr2cam, self.l_handle2vr)

    @staticmethod
    def __augmented_matrix(r, p):
        """
        将旋转阵(3*3)和位移向量(3*1)合并为增广矩阵(4*4)
        :param r: B坐标系到A坐标系的旋转矩阵3*3
        :param p: B坐标系原点在A坐标系中的坐标，1维的
        :return: B坐标系到A坐标系的旋转平移矩阵(4*4)
        """
        return np.append(np.insert(r, 3, 0, axis=0), np.insert(p, 3, 1, axis=0).reshape(4, 1), axis=1)