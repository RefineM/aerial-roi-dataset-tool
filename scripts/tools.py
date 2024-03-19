# -*- coding:utf-8 -*-
"""
@Time: 2024/3/19 20:15
@author: RefineM
@file: tools.py

"""
import os
import shutil
import numpy as np
from tqdm import tqdm


def get_scene_box(p_world, tar_radius=1.0):
    """
    获取包围盒
    :param p_world: 兴趣区世界点坐标 / np.array(N,3)
    :param tar_radius: 前景球体半径 / float / 默认为1.0
    :return: bounding_box, center, radius, scale
    """
    x_min, x_max = np.min(p_world[:, 0]), np.max(p_world[:, 0])
    y_min, y_max = np.min(p_world[:, 1]), np.max(p_world[:, 1])
    z_min, z_max = np.min(p_world[:, 2]), np.max(p_world[:, 2])

    x_range = x_max - x_min
    y_range = y_max - y_min
    z_range = z_max - z_min

    bounding_box = np.array([[x_min, x_max], [y_min, y_max], [z_min, z_max]])  # (3,2)
    center = np.array([[x_min + x_range / 2], [y_min + y_range / 2], [z_min + z_range / 2]])  # (3,1)
    radius = np.sqrt(np.power(x_range, 2) + np.power(y_range, 2) + np.power(z_range, 2)) / 2
    scale = tar_radius / radius

    output = {
        "bounding_box": bounding_box,
        "center": center,
        "radius": radius,
        "scale": scale}
    return output


def get_required_img_path(ori_img_path, use_img_id, tar_folder):
    """
    获取感兴趣影像的新老地址
    :param ori_img_path: 原始影像路径 / list[str]
    :param use_img_id: 感兴趣影像id / list[int]
    :param tar_folder: 目标文件夹 / str
    :return:  new_path / list[str], old_path / list[str]
    """
    new_path = []
    old_path = [ori_img_path[i] for i in use_img_id]
    for i in range(0, len(use_img_id)):
        img_name = old_path[i].split("\\")[-1]
        tar_path = os.path.join(tar_folder, img_name)
        new_path.append(tar_path)
    return old_path, new_path


def move_images(old_path, new_path):
    """
    将感兴趣影像移动到新文件夹下
    :param old_path: list[str]
    :param new_path: list[str]
    :return:
    """
    print("move required images into target folder")
    for i in tqdm(range(0, len(old_path))):
        shutil.copy(old_path[i], new_path[i])


def get_use_img(rot, c_world, p_world, threshold, cover_ratio):
    """
    挑选出感兴趣区的影像
    :param rot: c2w矩阵 / np.array(n,3,3)
    :param c_world: 相机世界坐标 / np.array(n,3)
    :param p_world: 兴趣区世界点坐标 / np.array(N,3)
    :param threshold: 相机的视场角，用来判断世界点相对于某张影像是否可视 / np.array(n)
    :param cover_ratio: 当兴趣区的可视点大于一定比例，认为该图片符合要求 / float
    :return: use_img_id : 感兴趣影像id / list[int]
    """
    print("detect required images")
    use_img_id = []
    c_orient = rot[:, :, 2]  # np.array(img_num, 3)
    for i in tqdm(range(0, np.size(c_world, 0))):
        t = threshold[i]
        c_p = p_world - c_world[i, :]  # np.array(point_num, 3)
        c_p_norm = np.linalg.norm(c_p, axis=1)  # np.array(point_num, 1)
        c_o = np.tile(c_orient[i, :], (np.size(p_world, 0), 1))  # np.array(point_num, 3)
        c_o_norm = np.linalg.norm(c_o, axis=1)  # np.array(point_num, 1)

        cos_angle = np.sum(np.multiply(c_p, c_o), axis=1)  # np.array(point_num, 1)
        cos_angle = cos_angle / c_p_norm / c_o_norm

        angle = np.arccos(cos_angle)  # between the vector 'cam->o' and 'cam->p'
        if np.sum(angle <= t) > (cover_ratio * len(p_world)):
            use_img_id.append(i)

    return use_img_id
