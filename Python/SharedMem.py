import mmap
import numpy as np
import cv2
import time
import sys
import struct
from namedmutex import NamedMutex
from ZEDCamera import ZEDCamera


class SharedMemory(object):
    def __init__(self, mem_name, mem_size, encode_jpg=True):
        """
        :param mem_name: 共享内存命名
        :param mem_size: 共享内存大小
        """
        self.mem_name = mem_name
        self.mem_size = mem_size
        self.shared_mem = mmap.mmap(fileno=0, length=mem_size, tagname=mem_name, access=mmap.ACCESS_DEFAULT)
        self.mutex = NamedMutex(mem_name + "_mutex")

        # 图像以jpg格式编码
        self.encode_jpg = encode_jpg

        # 头部、左右手柄位姿
        self.head_pos = None
        self.head_quaternion = None
        self.head_euler = None
        self.lHand_pos = None
        self.lHand_quaternion = None
        self.lHand_euler = None
        self.rHand_pos = None
        self.rHand_quaternion = None
        self.rHand_euler = None

    def __del__(self):
        self.shared_mem.close()

    def write_bytes(self, data, begin=0):
        self.mutex.acquire()
        self.shared_mem.seek(begin)
        self.shared_mem.write(data)
        self.mutex.release()

    def read_bytes(self, byte_cnt, begin=0):
        self.mutex.acquire()
        self.shared_mem.seek(begin)
        data = self.shared_mem.read(byte_cnt)
        self.mutex.release()
        return data

    def write_images(self, images, begin=1024):
        """
        Write a list of images(raw BGR format) through shared memory.
        :param images: Image list
        :return:
        """
        self.mutex.acquire()
        self.shared_mem.seek(begin)
        self.shared_mem.write(len(images).to_bytes(length=4, byteorder=sys.byteorder))
        for img in images:
            format = '.jpg' if self.encode_jpg else '.bmp'
            success, img_code = cv2.imencode(format, img)    # 将原始图像数据转码为编码形式
            if success:
                img_bytes = img_code.tobytes()       # 将编码转换为字节数组
                len_bytes = len(img_bytes).to_bytes(4, byteorder=sys.byteorder)  # 编码长度
                self.shared_mem.write(len_bytes)
                self.shared_mem.write(img_bytes)
        self.mutex.release()

    def read_images(self, begin=0):
        """
        Read a list of images(raw BGR format) from the shared memory.
        :return:
        """
        self.mutex.acquire()
        self.shared_mem.seek(begin)
        img_list = list()
        img_cnt = int.from_bytes(bytes=self.shared_mem.read(4), byteorder=sys.byteorder)
        for i in range(img_cnt):
            img_len = int.from_bytes(bytes=self.shared_mem.read(4), byteorder=sys.byteorder)
            img_data = np.frombuffer(self.shared_mem.read(img_len), dtype=np.uint8)
            img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
            img_list.append(img)
        self.mutex.release()
        return img_list

    @staticmethod
    def __write_float(value, buffer, pos):
        # 高位在前，低位在后
        bytes = struct.pack("f", value)
        buffer[pos] = bytes[0]
        buffer[pos + 1] = bytes[1]
        buffer[pos + 2] = bytes[2]
        buffer[pos + 3] = bytes[3]

    def read_pos_gesture(self):
        # 注意：应该在这里完成左手系到右手系的变换
        T_pos = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
        T_rotate = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])

        bytes = self.read_bytes(byte_cnt=10 * 4 * 3)
        head_hand_pos_gest = np.frombuffer(bytes, dtype='float32')

        # 头部位姿
        self.head_pos = np.dot(T_pos, head_hand_pos_gest[0:3])
        self.head_quaternion = head_hand_pos_gest[3:7]          # 四元数没有做右手系转换！！
        self.head_euler = np.dot(T_rotate, head_hand_pos_gest[7:10])

        # 左手位姿
        self.lHand_pos = np.dot(T_pos, head_hand_pos_gest[10:13])
        self.lHand_quaternion = head_hand_pos_gest[13:17]       # 四元数没有做右手系转换！！
        self.lHand_euler = np.dot(T_rotate, head_hand_pos_gest[17:20])

        # 右手位姿
        self.rHand_pos = np.dot(T_pos, head_hand_pos_gest[20:23])
        self.rHand_quaternion = head_hand_pos_gest[23:27]
        self.rHand_euler = np.dot(T_rotate, head_hand_pos_gest[27:])


def test_with_images():
    img1 = cv2.imread("lena.jpg")
    img2 = cv2.imread("building.jpg")
    img = (img1, img2)

    shmm = SharedMemory(mem_name="ShareForUnity", mem_size=2208 * 1242 * 3 + 1024)
    print("Ready to write shared memory")
    while True:
        shmm.write_images(img)
        time.sleep(0.05)


def test_with_ZED():
    cameras = ZEDCamera.enum_cameras()
    camera = ZEDCamera(cameras[0], resolution=1080, camera_fps=30, depth_min=400, depth_max=5000)

    shmm = SharedMemory(mem_name="ShareForUnity", mem_size=2208 * 1242 * 3 + 1024)
    print("Camera and shared memory ready")
    while True:
        # 更新相机图像
        camera.refresh()
        img_left = camera.get_RGBimage()
        img_right = camera.get_RGBimage_right()
        images = (img_left, img_right)

        # 将图像写入共享内存，从共享内存中读取头盔、手柄位姿数据
        shmm.write_images(images)
        shmm.read_pos_gesture()
        print(shmm.head_hand_pos_gest)


if __name__ == '__main__':
    # test_with_images()
    test_with_ZED()
