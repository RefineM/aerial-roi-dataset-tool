# -*- coding:utf-8 -*-
"""
@Time: 2024/2/26 19:21
@author: RefineM
@file: camera_reader.py

"""
import math
import numpy as np
import xmltodict
import itertools


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

    @staticmethod
    def get_group_intrinsic(group):
        """获取单个photo_group的内参"""
        # w,h (unit:pixel)
        w = int(group['ImageDimensions']['Width'])
        h = int(group['ImageDimensions']['Height'])
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
        # skew
        sk_x = sk_y = float(group["Skew"])
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
        # intrinsic (c2p,3*3)
        k = [[fx, sk_x, cx],
             [sk_y, fy, cy],
             [0, 0, 1]]
        # fov
        fov = 2 * math.atan(sensor_size / 2 / f_mm)
        return w, h, k, fov

    def get_cam_intrinsic(self):
        """获取所有像片的内参"""
        intrinsics = []
        fovs = []
        imgs_w = []
        imgs_h = []
        img_num = 0

        # 单相机
        if self.if_single_camera:
            img_num = len(self.photo_group['Photo'])
            w, h, k, fov = self.get_group_intrinsic(self.photo_group)
            w = img_num * [w]
            h = img_num * [h]
            imgs_w.extend(w)
            imgs_h.extend(h)
            intrinsics.extend(img_num * [k])
            fovs.extend(img_num * [fov])
            cam_type = self.photo_group["CameraModelType"]  # str
            cam_orient = self.photo_group["CameraOrientation"]  # str

        # 多相机
        else:
            groups_num = len(self.photo_groups["Photogroup"])
            for group_idx in range(groups_num):
                group = self.photo_groups["Photogroup"][group_idx]
                group_img_num = len(group["Photo"])
                img_num += group_img_num
                w, h, k, fov = self.get_group_intrinsic(group)
                w = group_img_num * [w]
                h = group_img_num * [h]
                imgs_w.extend(w)
                imgs_h.extend(h)
                intrinsics.extend(group_img_num * [k])
                fovs.extend(group_img_num * [fov])
            cam_type = self.photo_groups["Photogroup"][0]["CameraModelType"]  # str
            cam_orient = self.photo_groups["Photogroup"][0]["CameraOrientation"]  # str

        intrinsics = np.array(intrinsics, dtype=np.float32).reshape(img_num, 3, 3)
        fovs = np.array(fovs, dtype=np.float32).reshape(img_num)

        output = {
            "cam_type": cam_type,
            "cam_orient": cam_orient,
            "cam_fov": fovs,  # np.array(img_num)
            "cam_intrinsic": intrinsics,  # np.array(img_num,3,3)
            "imgs_w": imgs_w,  # list(img_num)
            "imgs_h": imgs_h   # list(img_num)
        }
        return output

    @ staticmethod
    def get_photo_extrinsic(photo):
        """
        获取单个像片的外参
        :return:
        rot_w2c/旋转矩阵/np.array(3,3)
        rot_c2w/旋转矩阵/np.array(3,3)
        cam_center_in_world/相机中心在世界坐标系的坐标/np.array(3)
        cam_extrinsic/ c2w opencv /np.array(4,4)
        """
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

        return w2c_rot, c2w_rot, cam_world, cam_extrinsic

    def get_cam_extrinsic(self):
        """
        获取所有相片的外参
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
                w2c_rot, c2w_rot, cam_world, cam_extrinsic = self.get_photo_extrinsic(photo)
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
                    w2c_rot, c2w_rot, cam_world, cam_extrinsic = self.get_photo_extrinsic(photo)
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
