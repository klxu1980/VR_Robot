import numpy as np
import cv2
import time
import threading
import logging
from ZEDCamera import ZEDCamera
from SharedMem import SharedMemory


class CameraThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        cameras = ZEDCamera.enum_cameras()
        self.camera = ZEDCamera(cameras[0], resolution=720, camera_fps=30, depth_min=100, depth_max=5000)
        self.img_left = None
        self.img_right = None
        self.point_cloud = None
        self.__event_cam_read = threading.Event()
        self.__event_get_frame = threading.Event()
        self.__terminated = False
        self.frame_time = 0

    def run(self) -> None:
        self.__event_cam_read.set()
        while not self.__terminated:
            if self.__event_cam_read.wait(timeout=0.1):
                # 相机读完一帧图像后，清除相机读事件(__event_cam_read)，不再更新图像，直至外部调用者再次启动图像更新
                self.__event_cam_read.clear()

                # 读相机，更新数据
                begin_time = time.perf_counter()
                self.camera.refresh()
                self.img_left = self.camera.get_RGBimage()
                self.img_right = self.camera.get_RGBimage_right()
                self.point_cloud = self.camera.get_point_cloud()
                self.frame_time = int((time.perf_counter() - begin_time) * 1000)

                # 通知外部调用者读取更新后的数据
                self.__event_get_frame.set()

    def get_one_frame(self):
        if self.__event_get_frame.wait(timeout=0.1):
            self.__event_get_frame.clear()

            img_left = self.img_left.copy()
            img_right = self.img_right.copy()
            point_cloud = self.point_cloud.copy()
            data = (img_left, img_right, point_cloud, self.frame_time)
            self.__event_cam_read.set()
            return data
        else:
            return None

    def close(self):
        self.__terminated = True
        self.join()


class Interface(object):
    def __init__(self):
        # 相机线程和共享内存
        self.cam_thread = CameraThread()
        self.cam_thread.start()
        self.shmm = SharedMemory(mem_name="ShareForUnity", mem_size=2208 * 1242 * 3 + 1024, encode_jpg=True)

        # 左右相机内参矩阵
        self.cam_params = self.cam_thread.camera.cam_params
        self.cam_inner_l = np.array([[self.cam_params[0], 0, self.cam_params[2]],
                                     [0, self.cam_params[1], self.cam_params[3]],
                                     [0, 0, 1]], dtype=np.float32)
        self.cam_inner_r = np.array([[self.cam_params[4], 0, self.cam_params[6]],
                                     [0, self.cam_params[5], self.cam_params[7]],
                                     [0, 0, 1]], dtype=np.float32)
        self.transform_mat = self.cam_thread.camera.transform_mat      # 相机左到右旋转平移矩阵 3*4

        # 从相机里读出的图像和点云
        self.cam_image_l = None
        self.cam_image_r = None
        self.point_cloud = None

        # 用于叠加到实际图像上的显示画面
        self.proc_image_l = None
        self.proc_image_r = None
        self.proc_image_alpha = 0.5

        # 运行日志
        logging.basicConfig(filemode="Log.txt",
                            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s-%(funcName)s',
                            level=logging.INFO)
        logging.info("程序启动")

    def __del__(self):
        if self.cam_thread is not None:
            self.cam_thread.close()

    @staticmethod
    def proc_time(begin_time):
        return int((time.perf_counter() - begin_time) * 1000)

    def process(self):
        """
        通过重载该函数实现对图像的具体处理
        :return:
        """
        pass

    def run(self):
        while True:
            # 统计一个周期的总计算耗时
            cycle_start_time = time.perf_counter()

            # 从ZED中读取一帧图像和点云数据，统计读取图像耗时
            begin_time = time.perf_counter()
            data = self.cam_thread.get_one_frame()
            if data is None:
                continue
            self.cam_image_l = data[0]
            self.cam_image_r = data[1]
            self.point_cloud = data[2]
            camera_time = data[3]
            read_frame_time = self.proc_time(begin_time)

            # 处理数据，绘制处理结果图像
            begin_time = time.perf_counter()
            self.proc_image_l = np.zeros(self.cam_image_l.shape, dtype=np.uint8)
            self.proc_image_r = np.zeros(self.cam_image_r.shape, dtype=np.uint8)
            self.process()
            process_time = self.proc_time(begin_time)

            # 将处理结果图像和相机图像结合
            begin_time = time.perf_counter()
            img_l = cv2.addWeighted(self.cam_image_l, 1, self.proc_image_l, self.proc_image_alpha, gamma=1)
            img_r = cv2.addWeighted(self.cam_image_l, 1, self.proc_image_r, self.proc_image_alpha, gamma=1)
            self.shmm.write_images(images=[img_l, img_r])
            write_img_time = self.proc_time(begin_time)

            # 显示各个步骤的时间
            cycle_time = self.proc_time(cycle_start_time)
            info = "Time consumption: Cycle = %d, camera = %d, frame = %d, process = %d, image write = %d" % (cycle_time, camera_time, read_frame_time, process_time, write_img_time)
            logging.info(info)