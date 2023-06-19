import numpy as np
import numpy.ma as ma
import cv2
import copy
import time
from colour import Color


class DepthCamera(object):
    def __init__(self, resolution):
        self.resolution = resolution
        self.cam_params = None
        self._RGBimage = None
        self._point_cloud = None
        self.ROI = None
        self.timestamp = time.time()
        self.rot_angle = 0    # 图像有可能需要旋转90°、180°、270°
        self.flipud = False

        self.__init_color_map("blue", "red", 1000)

    def __init_color_map(self, begin, end, color_cnt):
        self.color_cnt = color_cnt
        color_map = list(Color(begin).range_to(Color(end), self.color_cnt))
        self.color_map_r = np.zeros(self.color_cnt)
        self.color_map_g = np.zeros(self.color_cnt)
        self.color_map_b = np.zeros(self.color_cnt)

        for i in range(self.color_cnt):
            self.color_map_r[i] = int(color_map[i].red * 255)
            self.color_map_g[i] = int(color_map[i].green * 255)
            self.color_map_b[i] = int(color_map[i].blue * 255)

    def _get_ROI(self, mat, ROI=None):
        if ROI is None:
            return mat
        else:
            return mat[ROI[1]: ROI[3], ROI[0]: ROI[2]]

    def __refresh_XYZmap(self):
        fx = self.cam_params[0]
        fy = self.cam_params[1]
        cx = self.cam_params[2]
        cy = self.cam_params[3]

        rows = self._Zmap.shape[0]
        cols = self._Zmap.shape[1]

        array_ux = np.repeat([np.arange(cols) - cx], repeats=rows, axis=0)  # u-cx得到的矩阵值
        array_vy = (np.repeat([np.arange(rows) - cy], repeats=cols, axis=0)).T  # v-cy得到的矩阵值

        self._Xmap = self._Zmap * array_ux / fx
        self._Ymap = self._Zmap * array_vy / fy

    def camera_name(self):
        """
        Each camera has a name string to help distinguish them.
        """
        return "None"

    def get_XYZ_ROI(self, ROI=None):
        if ROI is None:
            ROI = self.ROI
        X = self._get_ROI(self._Xmap, ROI).copy()
        Y = self._get_ROI(self._Ymap, ROI).copy()
        Z = self._get_ROI(self._Zmap, ROI).copy()
        return X, Y, Z

    def get_Z_ROI(self, ROI=None):
        if ROI is None:
            ROI = self.ROI
        Z = self._get_ROI(self.Zmap, ROI).copy()
        return Z

    def set_ROI(self, left, top, right, bottom):
        self.ROI = (left, top, right, bottom)

    def calc_point_XY(self, x_pixel, y_pixel, Z):
        fx = self.cam_params[0]
        fy = self.cam_params[1]
        cx = self.cam_params[2]
        cy = self.cam_params[3]

        X = Z * (x_pixel - cx) / fx
        Y = Z * (y_pixel - cy) / fy
        return X, Y

    def calc_X_width(self, w_pixel, Z):
        fx = self.cam_params[0]
        width = Z * w_pixel / fx
        return width

    def calc_point_pixel(self, x, y, z):
        fx = self.cam_params[0]
        fy = self.cam_params[1]
        cx = self.cam_params[2]
        cy = self.cam_params[3]

        px = x * fx / z + cx
        py = y * fy / z + cy
        return px, py

    def refresh(self):
        self._RGBimage = None
        self._RGBimage_right = None
        self._Zmap = None
        return True

    @staticmethod
    def get_minmax_depth(Zmap):
        # There are NaN and Inf in depth map, which lead to wrong max and min value.
        # To avoid this, convert Zmap into a masked array, which masks NaN and Inf values.
        mask = np.isnan(Zmap) | np.isinf(Zmap)
        mask_depth = ma.masked_array(Zmap, mask)
        max_depth = mask_depth.max()
        min_depth = mask_depth.min()
        return min_depth, max_depth

    def _get_depth_image(self, Zmap, width=None, min_depth=None, max_depth=None):
        # Convert depth into color index
        if min_depth is None or max_depth is None:
            min_depth, max_depth = self.get_minmax_depth(Zmap)
        depth = (max_depth - Zmap) * self.color_cnt / (max_depth - min_depth)
        depth[depth < 0] = 0
        depth[depth >= self.color_cnt] = self.color_cnt - 1
        depth = depth.astype(np.uint32)
        depth = self.color_cnt - 1 - depth

        # convert depth map into colored image
        image = np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)
        image[:, :, 0] = self.color_map_b[depth]
        image[:, :, 1] = self.color_map_g[depth]
        image[:, :, 2] = self.color_map_r[depth]

        # NaN points are painted black, and INF points white
        image[np.isnan(Zmap), :] = 0
        image[np.isinf(Zmap), :] = 255

        if width is not None:
            image = cv2.resize(image, (width, int(image.shape[0] * width / image.shape[1])))

        return image

    def get_RGBimage(self, ROIonly=False, width=None, mark_infeasible=False):
        img = self._get_ROI(self._RGBimage, self.ROI) if ROIonly else self._RGBimage

        if mark_infeasible:
            depth = self._get_ROI(self.Zmap, self.ROI) if ROIonly else self.Zmap
            img[np.isnan(depth), :] = 0
            img[np.isinf(depth), :] = 255

        if width is not None:
            img = cv2.resize(img, (width, int(img.shape[0] * width / img.shape[1])))
        return img

    def get_depth_image(self, ROIonly=False, width=None, min_depth=None, max_depth=None):
        Zmap = self._get_ROI(self.Zmap, self.ROI) if ROIonly else self.Zmap
        return self._get_depth_image(Zmap, width, min_depth, max_depth)

    def save_RGB(self, file_name):
        cv2.imwrite(file_name, self.get_RGBimage())

    def save_depth(self, file_name):
        xyz_map = np.vstack((self.Xmap, self.Ymap, self.Zmap))
        xyz_map = xyz_map.reshape((self.Zmap.shape[0], self.Zmap.shape[1], 3))
        np.save(file_name, xyz_map)

    def load_depth(self, file_name):
        xyz_map = np.load(file_name)
        self._Xmap = xyz_map[:, :, 0]
        self._Ymap = xyz_map[:, :, 1]
        self._Zmap = xyz_map[:, :, 2]

    def show_3Dsurface(self, ROIonly=False):
        """
        Zmap = self._get_ROI(self.Zmap, self.ROI) if ROIonly else self.Zmap
        min_depth, max_depth = self.get_minmax_depth(Zmap)

        depth = Zmap.copy()
        depth[depth < min_depth] = min_depth
        depth[depth > max_depth] = max_depth
        depth[np.isnan(depth)] = max_depth
        depth = max_depth - depth

        y = np.arange(0, depth.shape[0], 1)
        x = np.arange(0, depth.shape[1], 1)
        x, y = np.meshgrid(x, y)

        figure = plt.figure()
        ax = Axes3D(figure)
        ax.plot_surface(x, y, depth, rstride=8, cstride=8, cmap='rainbow', linewidth=2.0)

        ax.set_zlim(0, max_depth - min_depth)
        plt.show()
        """
        pass


if __name__ == '__main__':
    depth_camera = DepthCamera(width=2208, height=1242)
    depth_camera.load_depth("D:\\11.9日-小飞机照片\\2021_11_09_11_12_34_6468932_depth.npy")
    img = depth_camera.get_depth_image()
    cv2.imshow("", img)
    cv2.waitKey(0)