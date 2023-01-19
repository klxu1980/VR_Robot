import mmap
import numpy as np
import cv2
import time
from namedmutex import NamedMutex
from ZEDCamera import ZEDCamera


class SharedMemory(object):
    def __init__(self, mem_name, mem_size, mem_type=mmap.ACCESS_READ):
        """
        注意，初始化共享内存时，需要先建立ACCESS_WRITE实例，然后建立ACCESS_READ实例，否则会出错
        :param mem_name: 共享内存命名
        :param mem_size: 共享内存大小
        :param mem_type: 共享内存类型(读、写)，由mmap.ACCESS_READ或mmap.ACCESS_WRITE定义
        """
        self.mem_name = mem_name
        self.mem_size = mem_size
        self.shared_mem = mmap.mmap(fileno=0, length=mem_size, tagname=mem_name, access=mem_type)
        self.mutex = NamedMutex(mem_name + "_mutex")

    def __del__(self):
        self.shared_mem.close()

    def send(self, data):
        self.shared_mem.write(data)

    def read(self, byte_cnt):
        return self.shared_mem.read(byte_cnt)

    def send_images(self, images):
        """
        Send a list of images(raw BGR format) through shared memory.
        :param images: Image list
        :return:
        """
        self.mutex.acquire()
        self.shared_mem.seek(0)
        for img in images:
            img_size = np.array(img.shape, dtype=np.int32)
            self.shared_mem.write(img_size.tobytes())
            self.shared_mem.write(img.tobytes())
        empty = np.zeros(3, dtype=np.int32)
        self.shared_mem.write(empty.tobytes())
        self.mutex.release()

    def read_images(self):
        """
        Read a list of images(raw BGR format) from the shared memory.
        :return:
        """
        self.mutex.acquire()
        self.shared_mem.seek(0)
        img_list = list()
        while True:
            img_shape = self.shared_mem.read(12)
            img_shape = np.frombuffer(img_shape, dtype=np.int32)
            img_size = img_shape[0] * img_shape[1] * img_shape[2]
            if not img_size:
                break
            img_data = self.shared_mem.read(img_size)
            img = np.frombuffer(img_data, dtype=np.uint8).reshape(img_shape)
            img_list.append(img)
        self.mutex.release()
        return img_list


def test_with_images():
    img1 = cv2.imread("lena.jpg")
    img2 = cv2.imread("building.jpg")
    img = (img1, img2)

    shmm = SharedMemory(mem_name="shared_memory1", mem_size=1024 * 1024 * 3 * 2, mem_type=mmap.ACCESS_WRITE)
    while True:
        shmm.send_images(img)
        time.sleep(0.05)


def test_with_ZED():
    cameras = ZEDCamera.enum_cameras()
    camera = ZEDCamera(cameras[0], resolution=720, camera_fps=30, depth_min=400, depth_max=5000)

    shmm = SharedMemory(mem_name="shared_memory1", mem_size=1280 * 720 * 3 * 2 + 3 * 12, mem_type=mmap.ACCESS_WRITE)
    print("Camera and shared memory ready")
    while True:
        camera.refresh()
        img_left = camera.get_RGBimage()
        img_right = camera.get_RGBimage_right()
        images = (img_left, img_right)

        shmm.send_images(images)
        time.sleep(0.01)


if __name__ == '__main__':
    test_with_ZED()
