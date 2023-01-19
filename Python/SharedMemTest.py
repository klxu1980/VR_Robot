from SharedMem import SharedMemory
import mmap
import cv2


shmm = SharedMemory(mem_name="shared_memory1", mem_size=1280*720*3*2 + 3 * 12, mem_type=mmap.ACCESS_READ)
while True:
    img_list = shmm.read_images()
    if len(img_list) >= 2:
        cv2.imshow("1", img_list[0])
        cv2.imshow("2", img_list[1])
    cv2.waitKey(10)