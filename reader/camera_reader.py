# -*- coding:utf-8 -*-
"""
@Time: 2024/2/26 19:21
@author: Junfan W
@file: camera_reader.py

"""
import math
import numpy as np
import xmltodict


class CameraReader(object):
    def __init__(self, xml_path, if_single_camera):
        """从ContextCapture格式的xml文件中读取影像元数据信息"""
        self.xml_path = xml_path
        with open(self.xml_path, 'r') as f:
            self.xml = xmltodict.parse(f.read())
        self.if_single_camera = if_single_camera
        if self.if_single_camera:
            self.photo_group = self.xml['BlocksExchange']['Block']['Photogroups']['Photogroup']
        else:
            self.photo_groups = self.xml['BlocksExchange']['Block']['Photogroups']

    @ staticmethod
    def get_image_size_multicam(group):
        """获取当前photo_group的像片尺寸"""
        w = int(group['ImageDimensions']['Width'])
        h = int(group['ImageDimensions']['Height'])
        return w, h

    def get_image_size_singlecam(self):
        """获取像片的尺寸"""
        w = int(self.photo_group['ImageDimensions']['Width'])
        h = int(self.photo_group['ImageDimensions']['Height'])
        return w, h

    def get_image_num_singlecam(self):
        return len(self.photo_group['Photo'])

    def get_image_path(self):
        """获取原始像片所在位置
        :return: img_path list[img_num]
        """
        img_path = []
        if self.if_single_camera:
            photo = self.photo_group['Photo']
            for idx in range(len(photo)):
                img_path.append(photo[idx]['ImagePath'])
        else:
            groups_num = len(self.photo_groups["Photogroup"])
            for group_idx in range(groups_num):
                group = self.photo_groups["Photogroup"][group_idx]
                photo = group['Photo']
                for idx in range(len(photo)):
                    img_path.append(photo[idx]['ImagePath'])
        return img_path

    def get_cam_intrinsic(self):
        """
        获取相机的内参
        :return:
        cam_type/相机类型/str
        cam_orient/坐标系朝向/str
        cam_fov/对角线视场角/ np.array(groups_num,1)
        cam_intrinsic/内参矩阵/np.array(groups_num,3,3)
        """
        intrinsics = []
        fovs = []
        imgs_w = []
        imgs_h = []
        img_num = 0

        if self.if_single_camera:
            img_num = self.get_image_num_singlecam()
            # images w and h (unit:pixel)
            w, h = self.get_image_size_singlecam()
            # focal length (unit:mm)
            f_mm = float(self.photo_group["FocalLength"])
            # sensor's largest dimension (unit:mm)
            sensor_size = float(self.photo_group["SensorSize"])
            # sensor's pixel size (unit:mm)
            pixel_size = sensor_size / max(w, h)
            # aspect_ratio = dx / dy
            aspect_ratio = float(self.photo_group["AspectRatio"])
            # Principal Point (x,y) (unit:pixel)
            cx = float(self.photo_group["PrincipalPoint"]["x"])
            cy = float(self.photo_group["PrincipalPoint"]["y"])
            sk_x = sk_y = float(self.photo_group["Skew"])  # skew
            # focal length (unit:pixel)
            if aspect_ratio == 1:
                fx = fy = f_mm / pixel_size
            else:
                if max(w, h) == w:
                    fx = f_mm / pixel_size
                    fy = aspect_ratio * fx
                else:
                    fy = f_mm / pixel_size
                    fx = fy / aspect_ratio

            # 为每一张照片生成一个内参
            w = img_num * [w]  # list(group_img_num)
            h = img_num * [h]  # list(group_img_num)
            imgs_w.append(w)  # list(group_img_num, group_img_num)
            imgs_h.append(h)  # list(group_img_num, group_img_num)

            k = [[fx, sk_x, cx],
                 [sk_y, fy, cy],
                 [0, 0, 1]]  # list(3,3)
            k = img_num * k  # list(group_img_num,3,3)
            intrinsics.append(k)

            fov = img_num * [2 * math.atan(sensor_size / 2 / f_mm)]  # list(group_img_num)
            fovs.append(fov)

            intrinsics = np.array(intrinsics, dtype=np.float32).reshape(img_num, 3, 3)  # np.array(img_num,3,3)
            fovs = np.array(fovs, dtype=np.float32).reshape(img_num)  # np.array(img_num)

            output = {
                "cam_type": self.photo_group["CameraModelType"],  # str
                "cam_orient": self.photo_group["CameraOrientation"],  # str
                "cam_fov": fovs,  # np.array(img_num)
                "cam_intrinsic": intrinsics,  # np.array(img_num,3,3)
                "imgs_w": w,  # list(img_num)
                "imgs_h": h  # list(img_num)
            }

        else:
            groups_num = len(self.photo_groups["Photogroup"])
            for group_idx in range(groups_num):
                group = self.photo_groups["Photogroup"][group_idx]
                group_img_num = len(self.photo_groups["Photogroup"][group_idx]["Photo"])
                img_num += group_img_num
                # images w and h (unit:pixel)
                w, h = self.get_image_size_multicam(group)
                # focal length (unit:mm)
                f_mm = float(group["FocalLength"])
                # sensor's largest dimension (unit:mm)
                sensor_size = float(group["SensorSize"])
                # sensor's pixel size (unit:mm)
                pixel_size = sensor_size / max(w, h)
                # aspect_ratio = dx / dy
                aspect_ratio = float(group["AspectRatio"])
                # Principal Point (x,y) (unit:pixel)
                cx = float(group["PrincipalPoint"]["x"])
                cy = float(group["PrincipalPoint"]["y"])
                sk_x = sk_y = float(group["Skew"])  # skew
                # focal length (unit:pixel)
                if aspect_ratio == 1:
                    fx = fy = f_mm / pixel_size
                else:
                    if max(w, h) == w:
                        fx = f_mm / pixel_size
                        fy = aspect_ratio * fx
                    else:
                        fy = f_mm / pixel_size
                        fx = fy / aspect_ratio

                # 为每一张照片生成一个内参
                w = group_img_num * [w]  # list(group_img_num)
                h = group_img_num * [h]  # list(group_img_num)
                imgs_w.append(w)  # list(group_img_num, group_img_num)
                imgs_h.append(h)  # list(group_img_num, group_img_num)

                k = [[fx, sk_x, cx],
                     [sk_y, fy, cy],
                     [0, 0, 1]]  # list(3,3)
                k = group_img_num * k  # list(group_img_num,3,3)
                intrinsics.append(k)

                fov = group_img_num * [2 * math.atan(sensor_size / 2 / f_mm)]  # list(group_img_num)
                fovs.append(fov)

            imgs_w = [item for sublist in imgs_w for item in sublist]   # list(img_num)
            imgs_h = [item for sublist in imgs_h for item in sublist]   # list(img_num)
            intrinsics = [item for sublist in intrinsics for item in sublist]  # list(img_num)
            fovs = [item for sublist in fovs for item in sublist]  # list(img_num)

            intrinsics = np.array(intrinsics, dtype=np.float32).reshape(img_num, 3, 3)  # np.array(img_num,3,3)
            fovs = np.array(fovs, dtype=np.float32).reshape(img_num)  # np.array(img_num)

            output = {
                "cam_type": self.photo_groups["Photogroup"][0]["CameraModelType"],  # str
                "cam_orient": self.photo_groups["Photogroup"][0]["CameraOrientation"],  # str
                "cam_fov": fovs,  # np.array(img_num)
                "cam_intrinsic": intrinsics,  # np.array(img_num,3,3)
                "imgs_w": imgs_w,  # list(img_num)
                "imgs_h": imgs_h   # list(img_num)
            }

        return output

    def get_cam_extrinsic(self):
        """
        获取相机的外参
        :return:
        rot_w2c/旋转矩阵/np.array(img_num,3,3)
        rot_c2w/旋转矩阵/np.array(img_num,3,3)
        cam_center_in_world/相机中心在世界坐标系的坐标/np.array(img_num,3)
        cam_extrinsic/ c2w opencv /np.array(img_num,4,4)
        """

        w2c_rot_all = []
        c2w_rot_all = []
        cam_world_all = []
        cam_extrinsic_all = []  # [R T] c2w

        if self.if_single_camera:
            photos = self.photo_group['Photo']
            for photo in photos:
                r00 = float(photo['Pose']['Rotation']['M_00'])
                r01 = float(photo['Pose']['Rotation']['M_01'])
                r02 = float(photo['Pose']['Rotation']['M_02'])
                r10 = float(photo['Pose']['Rotation']['M_10'])
                r11 = float(photo['Pose']['Rotation']['M_11'])
                r12 = float(photo['Pose']['Rotation']['M_12'])
                r20 = float(photo['Pose']['Rotation']['M_20'])
                r21 = float(photo['Pose']['Rotation']['M_21'])
                r22 = float(photo['Pose']['Rotation']['M_22'])
                t1 = float(photo['Pose']['Center']['x'])
                t2 = float(photo['Pose']['Center']['y'])
                t3 = float(photo['Pose']['Center']['z'])

                w2c_rot = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
                c2w_rot = np.linalg.inv(w2c_rot)
                cam_world = np.array([t1, t2, t3])

                cam_extrinsic = np.zeros(shape=(4, 4), dtype=np.float32)
                cam_extrinsic[:-1, :-1] = c2w_rot
                cam_extrinsic[:-1, -1] = cam_world
                cam_extrinsic[-1, :] = np.array([0.0, 0.0, 0.0, 1.0])

                w2c_rot_all.append(w2c_rot)
                c2w_rot_all.append(c2w_rot)
                cam_world_all.append(cam_world)
                cam_extrinsic_all.append(cam_extrinsic)

        else:
            groups_num = len(self.photo_groups["Photogroup"])
            for group_idx in range(groups_num):
                group = self.photo_groups["Photogroup"][group_idx]
                photos = group['Photo']
                for photo in photos:
                    r00 = float(photo['Pose']['Rotation']['M_00'])
                    r01 = float(photo['Pose']['Rotation']['M_01'])
                    r02 = float(photo['Pose']['Rotation']['M_02'])
                    r10 = float(photo['Pose']['Rotation']['M_10'])
                    r11 = float(photo['Pose']['Rotation']['M_11'])
                    r12 = float(photo['Pose']['Rotation']['M_12'])
                    r20 = float(photo['Pose']['Rotation']['M_20'])
                    r21 = float(photo['Pose']['Rotation']['M_21'])
                    r22 = float(photo['Pose']['Rotation']['M_22'])
                    t1 = float(photo['Pose']['Center']['x'])
                    t2 = float(photo['Pose']['Center']['y'])
                    t3 = float(photo['Pose']['Center']['z'])

                    w2c_rot = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
                    c2w_rot = np.linalg.inv(w2c_rot)
                    cam_world = np.array([t1, t2, t3])

                    cam_extrinsic = np.zeros(shape=(4, 4), dtype=np.float32)
                    cam_extrinsic[:-1, :-1] = c2w_rot
                    cam_extrinsic[:-1, -1] = cam_world
                    cam_extrinsic[-1, :] = np.array([0.0, 0.0, 0.0, 1.0])

                    w2c_rot_all.append(w2c_rot)
                    c2w_rot_all.append(c2w_rot)
                    cam_world_all.append(cam_world)
                    cam_extrinsic_all.append(cam_extrinsic)

        out = {
            "rot_w2c": np.stack(w2c_rot_all, axis=0),  # np.array(img_num,3,3)
            "rot_c2w": np.stack(c2w_rot_all, axis=0),  # np.array(img_num,3,3)
            "cam_center_in_world": np.stack(cam_world_all, axis=0),  # np.array(img_num,3)
            "cam_extrinsic_c2w": np.stack(cam_extrinsic_all, axis=0)
        }
        return out
