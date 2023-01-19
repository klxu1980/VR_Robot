from ZEDCamera import ZEDCamera
from Monitor3D import Monitor3D
from Monitor3D import Monitor3D_RB
import cv2


if __name__ == '__main__':
    cameras = ZEDCamera.enum_cameras()
    camera = ZEDCamera(cameras[0], resolution=720, camera_fps=30, depth_min=400, depth_max=5000)

    monitor_pol = Monitor3D(trans_mat_file="monitor_cali_lab.mat", screens=(0, 2))
    monitor_rb = Monitor3D_RB()

    key = 0
    while key != ord('q') and key != ord('Q'):
        key = cv2.waitKey(50) & 0xFF

        camera.refresh()
        img_left = camera.get_RGBimage()
        img_right = camera.get_RGBimage_right()
        #cv2.putText(img_left, "Press Q to quit", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        #cv2.putText(img_right, "Press Q to quit", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        monitor_pol.show(img_left, img_right)
        monitor_rb.show(img_left, img_right)