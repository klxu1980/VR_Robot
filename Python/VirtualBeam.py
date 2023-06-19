import numpy as np
import cupy as cp
import cv2
import time
import math
from Interface import Interface


class VirtualBeam(object):
    def __init__(self, cam_left, cam_right, cam_trans, cam_rotate):
        """
        :param cam_left: 双目相机左镜头内参
        :param cam_right: 双目相机右镜头内参
        :param cam_trans: 相机右镜头到左镜头坐标系的平移向量(1*3)
        :param cam_rotate: 相机右镜头到左镜头坐标系的旋转矩阵(3*3)
        """
        # 各个阶段耗时
        self.frame_time = 0
        self.process_time = 0
        self.read_write_frame_time = 0
        self.calc_images_time = 0

        # 内参矩阵、旋转矩阵和平移向量
        self.cam_left = cam_left
        self.cam_right = cam_right
        self.cam_trans = cam_trans
        self.cam_rotate = cam_rotate

        # 手柄颜色、长度(mm)
        self.color_x = (255, 20, 20)
        self.color_y = (20, 255, 20)
        self.color_z = (20, 20, 255)
        self.handle_len = 0.1
        self.hand_size = 10

        # 虚拟光束颜色、标准差阈值
        self.color_beam = (255, 20, 20)
        self.beam_std = 0.1
        self.beam_size = 10

    @staticmethod
    def __eulerAngles2rotationMat(euler):
        """
        Calculates Rotation Matrix given euler angles. Angles are defined by degree.
        :param euler: 1-by-3 list [rx, ry, rz] angle in degree
        :return:
        RYP角，是ZXY欧拉角，依次绕定轴ZXY转动[rx, ry, rz]
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

    @staticmethod
    def find_beam_end_uv(pos, r_mat, point_cloud, sigma):
        """
        根据直线方程计算虚拟直线透射到uv坐标系的区域
        :param pos: 手柄相对头盔的位置坐标
        :param rot_mat: 手柄的旋转矩阵
        :param point_cloud: 相机点云
        :param sigma: 标准差范围，反应在投射区域大小
        :return: 投射区域的uv坐标
        """
        # 光束起始点
        pos = cp.asarray(pos, dtype='float32')

        # 光束方向
        # 注意：这里需要考虑分母为0时的计算
        dir_cos = np.dot(r_mat, np.array([0, 0, 1]))
        if dir_cos[0] == 0 or dir_cos[1] == 0 or dir_cos[2] == 0:
            return None

        m_inv = 1 / dir_cos[0]
        n_inv = 1 / dir_cos[1]
        p_inv = 1 / dir_cos[2]
        mnp = cp.asarray([m_inv, n_inv, p_inv])

        # 计算检查点云中每个点是否在虚拟光束直线方程上
        cloud_cp = cp.asarray(point_cloud)
        t = cp.subtract(cloud_cp, pos)
        t = cp.multiply(t, mnp)
        t_std = cp.std(t, axis=2)

        uv = cp.where(~cp.isnan(t_std) & ~cp.isinf(t_std) & (t_std < sigma))
        uv_idx = (cp.asnumpy(uv[0]), cp.asnumpy(uv[1]))
        return uv_idx  # 元组形式返回([v1, v2, ···], [u1, u2, ···])

    def calc_points_uv(self, points):
        """
        计算三维空间中的点集在左右相机中的像素位置。
        :param points: 三维空间中的点集，维度为(N * 3)，每一行是一个点的三维坐标。
        :return: 每个点在左右相机中的像素坐标
        """
        if points.ndim == 1:
            points = points.reshape(1, 3)

        # 计算三维点在右相机坐标系中的坐标
        points_r = np.dot(self.cam_rotate, points.T).T + self.cam_trans

        # 计算点在左、右相机中的像素位置
        uvw_l = np.dot(self.cam_left, points.T).T
        uvw_r = np.dot(self.cam_right, points_r.T).T
        uv_l = (uvw_l[:, 0:2] / uvw_l[:, 2].reshape(-1, 1)).astype(np.int32)
        uv_r = (uvw_r[:, 0:2] / uvw_r[:, 2].reshape(-1, 1)).astype(np.int32)

        return uv_l, uv_r

    def draw_handle_beam(self, pos, euler, point_cloud, img_l, img_r):
        # 计算手柄原点和X,Y,Z方向在左右图像中的像素位置
        r_mat = self.__eulerAngles2rotationMat(euler)
        xyz = np.dot(r_mat, np.eye(3) * self.handle_len).T + pos
        xyz = np.append(xyz, pos.reshape(1, 3), axis=0)

        # 计算虚拟光束末端的三维坐标
        beam_ends = self.find_beam_end_uv(pos, r_mat, point_cloud, self.beam_std)
        if beam_ends is not None and beam_ends[0].size != 0:  # 如果虚拟直线投射在uv坐标系中
            beam_uv = (np.median(beam_ends[0]).astype(np.int32), np.median(beam_ends[1]).astype(np.int32))
            end_point = point_cloud[beam_uv[0], beam_uv[1], :]  # 光线末端三维坐标
            xyz = np.append(xyz, end_point.reshape(1, 3), axis=0)

        # 计算三维点在左右图像中的像素位置
        uv_l, uv_r = self.calc_points_uv(xyz)

        # 绘制虚拟光束
        if uv_l.shape[0] == 5:
            cv2.circle(img_l, uv_l[4], self.beam_size, self.color_beam, -1)
            cv2.line(img_l, uv_l[2], uv_l[4], self.color_beam, self.beam_size)

            cv2.circle(img_r, uv_r[4], self.beam_size, self.color_beam, -1)
            cv2.line(img_r, uv_r[2], uv_r[4], self.color_beam, self.beam_size)

        # 绘制手柄
        cv2.line(img_l, uv_l[3], uv_l[0], self.color_x, self.hand_size)
        cv2.line(img_l, uv_l[3], uv_l[1], self.color_y, self.hand_size)
        cv2.line(img_l, uv_l[3], uv_l[2], self.color_z, self.hand_size)

        cv2.line(img_r, uv_r[3], uv_r[0], self.color_x, self.hand_size)
        cv2.line(img_r, uv_r[3], uv_r[1], self.color_y, self.hand_size)
        cv2.line(img_r, uv_r[3], uv_r[2], self.color_z, self.hand_size)


class BeamTestInterface(Interface):
    def __init__(self):
        Interface.__init__(self)
        self.virtual_beam = VirtualBeam(cam_left=self.cam_inner_l,
                                        cam_right=self.cam_inner_r,
                                        cam_rotate=self.transform_mat[:, 0:3],
                                        cam_trans=self.transform_mat[:, 3].T)

    def process(self):
        # 获取头部和手柄位姿
        self.shmm.read_pos_gesture()

        # 右手相对头部位置
        rHand_head = np.subtract(self.shmm.rHand_pos, self.shmm.head_pos)

        # 手柄位置在VR中的显示
        self.virtual_beam.draw_handle_beam(pos=rHand_head, euler=self.shmm.rHand_euler, point_cloud=self.point_cloud,
                                           img_l=self.proc_image_l, img_r=self.proc_image_r)


if __name__ == '__main__':
    beam_test = BeamTestInterface()
    beam_test.run()