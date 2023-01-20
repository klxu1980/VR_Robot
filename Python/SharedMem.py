import mmap
import numpy as np
import cv2
import time
import sys
from namedmutex import NamedMutex
from ZEDCamera import ZEDCamera


class SharedMemory(object):
    def __init__(self, mem_name, mem_size):
        """
        :param mem_name: 共享内存命名
        :param mem_size: 共享内存大小
        """
        self.mem_name = mem_name
        self.mem_size = mem_size
        self.shared_mem = mmap.mmap(fileno=0, length=mem_size, tagname=mem_name, access=mmap.ACCESS_DEFAULT)
        self.mutex = NamedMutex(mem_name + "_mutex")

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

    def write_images(self, images, begin=0):
        """
        Write a list of images(raw BGR format) through shared memory.
        :param images: Image list
        :return:
        """
        self.mutex.acquire()
        self.shared_mem.seek(begin)
        self.shared_mem.write(len(images).to_bytes(length=4, byteorder=sys.byteorder))
        for img in images:
            img_size = np.array(img.shape, dtype=np.int32)
            self.shared_mem.write(img_size.tobytes())
            self.shared_mem.write(img.tobytes())
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
            img_shape = np.frombuffer(self.shared_mem.read(12), dtype=np.int32)
            img_data = self.shared_mem.read(img_shape[0] * img_shape[1] * img_shape[2])
            img = np.frombuffer(img_data, dtype=np.uint8).reshape(img_shape)
            img_list.append(img)
        self.mutex.release()
        return img_list


def test_with_images():
    img1 = cv2.imread("lena.jpg")
    img2 = cv2.imread("building.jpg")
    img = (img1, img2)

    shmm = SharedMemory(mem_name="shared_memory1", mem_size=1024 * 1024 * 3 * 2)
    print("Ready to write shared memory")
    while True:
        shmm.write_images(img)
        time.sleep(0.05)


def test_with_ZED():
    cameras = ZEDCamera.enum_cameras()
    camera = ZEDCamera(cameras[0], resolution=720, camera_fps=30, depth_min=400, depth_max=5000)

    shmm = SharedMemory(mem_name="shared_memory1", mem_size=1280 * 720 * 3 * 2 + 2 * 12 + 4)
    print("Camera and shared memory ready")
    while True:
        camera.refresh()
        img_left = camera.get_RGBimage()
        img_right = camera.get_RGBimage_right()
        images = (img_left, img_right)

        shmm.write_images(images)
        time.sleep(0.01)


if __name__ == '__main__':
    test_with_images()
