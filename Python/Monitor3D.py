import numpy as np
import cv2
import win32api,win32con

CORNER_POS = 1.0 / 3.0
USED_ENVIRONMENT = 2        # 1 for 609, 2 for our lab


class Monitor3D(object):
    def __init__(self, trans_mat_file=None, screens=None):
        self.__init_screen(screens)
        try:
            self.load_trans_mat(trans_mat_file)
        except:
            self.__trans_mat = np.eye(3)

    def load_trans_mat(self, mat_file):
        self.__trans_mat = np.loadtxt(mat_file)

    def __init_screen(self, screens):
        # it may be needed to modify screen initialization according to the computer configuration.
        width = win32api.GetSystemMetrics(win32con.SM_CXSCREEN)
        height = win32api.GetSystemMetrics(win32con.SM_CYSCREEN)

        if screens is None:
            self.screen_size = (int(width / 2), height)
            self.twin_screen = False

            cv2.namedWindow('window', cv2.WINDOW_NORMAL)
            cv2.setWindowProperty('window', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.moveWindow('window', 0, 0)
        else:
            self.screen_size = (width, height)
            self.twin_screen = True

            cv2.namedWindow('window1', cv2.WINDOW_NORMAL)
            cv2.setWindowProperty('window1', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.moveWindow('window1', width, screens[0] * width)

            cv2.namedWindow('window2', cv2.WINDOW_NORMAL)
            cv2.setWindowProperty('window2', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.moveWindow('window2', screens[1] * width, 0)

    def __start_calibration(self):
        self.corner_supposed = list()
        self.corner_adjusted = list()

        self.corner_supposed.append((int(self.screen_size[0] * CORNER_POS),
                                     int(self.screen_size[1] * CORNER_POS)))
        self.corner_supposed.append((int(self.screen_size[0] * (1.0 - CORNER_POS)),
                                     int(self.screen_size[1] * CORNER_POS)))
        self.corner_supposed.append((int(self.screen_size[0] * (1.0 - CORNER_POS)),
                                     int(self.screen_size[1] * (1.0 - CORNER_POS))))
        self.corner_supposed.append((int(self.screen_size[0] * CORNER_POS),
                                     int(self.screen_size[1] * (1.0 - CORNER_POS))))

    def __adjust_corner(self, idx):
        x1 = x2 = self.corner_supposed[idx][0]
        y1 = y2 = self.corner_supposed[idx][1]

        while True:
            img1 = np.zeros((self.screen_size[1], self.screen_size[0], 3), dtype=np.uint8)
            img2 = np.zeros((self.screen_size[1], self.screen_size[0], 3), dtype=np.uint8)

            cv2.circle(img1, (x1, y1), radius=10, color=(0, 255, 0), thickness=-1)
            cv2.circle(img2, (x2, y2), radius=10, color=(0, 0, 255), thickness=-1)
            cv2.putText(img1, "Click 'S' when you finished adjusting", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (20, 255, 20), 1)
            self.show(img1, img2, no_correction=True)

            key = cv2.waitKeyEx()
            if key == ord('s') or key == ord('S'):
                self.corner_adjusted.append((x2, y2))
                break
            elif key == 2490368:
                y2 -= 1
            elif key == 2621440:
                y2 += 1
            elif key == 2424832:
                x2 -= 1
            elif key == 2555904:
                x2 += 1

    @staticmethod
    def __calc_trans_mat(corners_supposed, corners_adjusted):
        m = np.zeros((8, 8), dtype=np.double)
        b = np.zeros((8, 1), dtype=np.double)
        for i in range(4):
            u = corners_supposed[i][0]
            v = corners_supposed[i][1]
            x = corners_adjusted[i][0]
            y = corners_adjusted[i][1]
            m[i * 2, 0] = u
            m[i * 2, 2] = -x * u
            m[i * 2, 3] = v
            m[i * 2, 5] = -x * v
            m[i * 2, 6] = 1

            m[i * 2 + 1, 1] = u
            m[i * 2 + 1, 2] = -y * u
            m[i * 2 + 1, 4] = v
            m[i * 2 + 1, 5] = -y * v
            m[i * 2 + 1, 7] = 1

            b[i * 2, 0] = x
            b[i * 2 + 1, 0] = y

        a = np.linalg.solve(m, b)
        mat = np.zeros((3, 3), dtype=np.double)
        mat[0, 0] = a[0]
        mat[1, 0] = a[1]
        mat[2, 0] = a[2]
        mat[0, 1] = a[3]
        mat[1, 1] = a[4]
        mat[2, 1] = a[5]
        mat[0, 2] = a[6]
        mat[1, 2] = a[7]
        mat[2, 2] = 1
        return mat

    @staticmethod
    def __trans_point(trans_mat, uv):
        w = trans_mat[2, 0] * uv[0] + trans_mat[2, 1] * uv[1] + trans_mat[2, 2]
        x = (trans_mat[0, 0] * uv[0] + trans_mat[0, 1] * uv[1] + trans_mat[0, 2]) / w
        y = (trans_mat[1, 0] * uv[0] + trans_mat[1, 1] * uv[1] + trans_mat[1, 2]) / w
        return int(x), int(y)

    def __test_calibration(self):
        img1 = np.zeros((self.screen_size[1], self.screen_size[0], 3), dtype=np.uint8)
        img2 = np.zeros((self.screen_size[1], self.screen_size[0], 3), dtype=np.uint8)

        for i in range(len(self.corner_supposed)):
            x1 = x2 = int(self.corner_supposed[i][0])
            y1 = y2 = int(self.corner_supposed[i][1])

            cv2.circle(img1, (x1, y1), radius=10, color=(0, 0, 255), thickness=-1)
            cv2.circle(img2, (x2, y2), radius=10, color=(0, 255, 0), thickness=-1)
        cv2.putText(img1, "Click 'S' to save calculated transforming matrix", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (20, 255, 20), 1)
        self.show(img1, img2)

    def calibrate(self, save_file=None):
        self.__start_calibration()

        for idx in range(len(self.corner_supposed)):
            self.__adjust_corner(idx)
        self.__trans_mat = self.__calc_trans_mat(self.corner_supposed, self.corner_adjusted)

        self.__test_calibration()
        key = cv2.waitKey()
        if save_file is not None and (key == ord('s') or key == ord('S')):
            np.savetxt(save_file, self.__trans_mat)

    def test_calibrate(self):
        self.__start_calibration()
        self.__test_calibration()

    def refresh_trans_mat(self):
        self.__trans_mat = self.__calc_trans_mat(self.corner_supposed, self.corner_adjusted)

    def show(self, img1, img2, no_correction=False):
        screen1 = np.zeros((self.screen_size[1], self.screen_size[0], 3), dtype=np.uint8)
        screen2 = np.zeros((self.screen_size[1], self.screen_size[0], 3), dtype=np.uint8)

        if img1 is not None:
            # the w/h ratio of the image is larger than that of the screen
            # so width of the image is used for resizing
            if img1.shape[1] / img1.shape[0] > self.screen_size[0] / self.screen_size[1]:
                width = self.screen_size[0]
                height = int(img1.shape[0] * width / img1.shape[1])
                top = int((self.screen_size[1] - height) / 2)

                # image is resized to fit screen if its width no equal to screen
                if img1.shape[0] != self.screen_size[0]:
                    img1 = cv2.resize(img1, (width, height))
                screen1[top: top + height, :, :] = img1[:, :, :]

                if img2 is not None:
                    if img2.shape[0] != self.screen_size[0]:
                        img2 = cv2.resize(img2, (width, height))
                    screen2[top: top + height, :, :] = img2[:, :, :]
            else:
                height = self.screen_size[1]
                width = int(img1.shape[1] * height / img1.shape[0])
                left = int((self.screen_size[0] - width) / 2)

                if img1.shape[1] != self.screen_size[1]:
                    img1 = cv2.resize(img1, (width, height))
                screen1[:, left: left + width, :] = img1[:, :, :]

                if img2 is not None:
                    if img1.shape[1] != self.screen_size[1]:
                        img2 = cv2.resize(img2, (width, height))
                    screen2[:, left: left + width, :] = img2[:, :, :]

        if not no_correction:
            screen2 = cv2.warpPerspective(screen2, self.__trans_mat, self.screen_size)

        if self.twin_screen:
            screen2 = cv2.flip(screen2, 0)
            cv2.imshow('window1', screen1)
            cv2.imshow('window2', screen2)
        else:
            screen2 = cv2.flip(screen2, 1)
            screen = cv2.hconcat((screen1, screen2))
            cv2.imshow('window', screen)


class Monitor3D_RB(object):
    def __init__(self):
        width = win32api.GetSystemMetrics(win32con.SM_CXSCREEN)
        height = win32api.GetSystemMetrics(win32con.SM_CYSCREEN)

        self.screen_size = (width, height)

        cv2.namedWindow('window_rb', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty('window_rb', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.moveWindow('window_rb', 0, 0)

    def show(self, img_r, img_b):
        screen = np.zeros((self.screen_size[1], self.screen_size[0], 3), dtype=np.uint8)
        img_r = cv2.resize(img_r, self.screen_size)
        img_b = cv2.resize(img_b, self.screen_size)
        screen[:, 0:self.screen_size[0]:2, 2] = img_r[:, 0:self.screen_size[0]:2, 2]
        screen[:, 1:self.screen_size[0]:2, 0:2] = img_b[:, 1:self.screen_size[0]:2, 0:2]

        cv2.imshow("window_rb", screen)


if __name__ == "__main__":
    monitor = Monitor3D(trans_mat_file="monitor_cali.mat", screens=(0, 2))
    monitor.calibrate(save_file="monitor_cali.mat")
