from SharedMem import SharedMemory
from Monitor3D import Monitor3D
import cv2


if __name__ == '__main__':
    shmm = SharedMemory(mem_name="ShareForUnity", mem_size=2208 * 1242 * 3 + 1024)
    monitor = Monitor3D(trans_mat_file="monitor_cali_lab.mat", screens=(0, 2))

    key = 0
    while key != ord('q') and key != ord('Q'):
        img_list = shmm.read_images()
        if len(img_list) >= 2:
            monitor.show(img1=img_list[0], img2=img_list[1])
        key = cv2.waitKey(10) & 0xFF