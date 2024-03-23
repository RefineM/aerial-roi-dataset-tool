# -*- coding:utf-8 -*-
"""
@Time: 2024/1/30 12:22
@author: RefineM
@file: run.py
"""
import json
import os.path
import numpy as np
from PIL import Image
from reader.camera_reader import CameraReader
from reader.obj_reader import SceneReader
from scripts.tools import get_scene_box, get_use_img, get_required_img_path, move_images
from tqdm import tqdm

if __name__ == "__main__":

    """参数设置"""
    # 数据集路径
    dataset_dir = r'.\dataset\isprs_01_n'
    # CC空三解算导出的整个数据集的相机内外参文件
    xml_path = dataset_dir + r"\AT.xml"
    # CC三维建模导出的感兴趣区域的mesh路径
    obj_path = dataset_dir + r"\Model.obj"
    # CC三维建模导出的感兴趣区域的mesh元数据文件
    mesh_xml_path = dataset_dir + r"\metadata.xml"
    # 是否是单相机
    if_single_camera = False

    # 是否将包含感兴趣区域的影像移动
    if_move = True
    tar_folder = dataset_dir + r"\images"
    # 保存包含感兴趣区域的影像id的文本文件路径
    use_id_path = dataset_dir + r"\required_img_id.txt"

    # 是否要获取掩膜，裁剪影像（在感兴趣目标在影像中的尺度很小的情况下可使用）
    if_mask_crop = True
    # 裁剪后影像的目标尺寸
    tar_size_w = 1200
    tar_size_h = 1000
    # 影像裁剪后路径
    crop_folder = dataset_dir + r"\images_crop"

    # 是否将场景规范化到指定范围
    if_standardization = True
    # 将场景规范化的目标半径
    tar_radius = 1.0

    # 感兴趣区域影像的相机内外参json文件路径
    json_file_path = dataset_dir + r".\transforms.json"
    # 将mesh顶点坐标中心化、规范化后输出路径
    resized_obj_path = dataset_dir + r"\Model_resized.obj"

    """声明类的实例"""
    # 相机参数解析类
    cam_info = CameraReader(xml_path, if_single_camera)
    # 场景参数解析类
    scene_info = SceneReader(obj_path, mesh_xml_path)

    """获取影像的元数据信息"""
    cam_in = cam_info.get_cam_intrinsic()
    cam_ex = cam_info.get_cam_extrinsic()
    ori_img_path = cam_info.get_image_path()

    """获取场景信息"""
    p_world = scene_info.get_3d_points()
    scene = get_scene_box(p_world, tar_radius)
    scene_info.centered_and_scaled_3d_points(resized_obj_path)

    """获取感兴趣影像id"""
    if os.path.isfile(use_id_path):
        use_img_id = []
        for line in open(use_id_path, "r"):
            use_img_id.append(int(line.split('\n')[0]))
    else:
        use_img_id = get_use_img(cam_ex["rot_c2w"], cam_ex["cam_center_in_world"],
                                 p_world,
                                 cam_in['cam_fov'] / 2,
                                 cover_ratio=0.05)
        f = open(use_id_path, "w")
        for i in use_img_id:
            f.write(str(i) + '\n')
        f.close()

    """移动感兴趣影像"""
    old_path, new_path = get_required_img_path(ori_img_path, use_img_id, tar_folder)
    if if_move:
        move_images(old_path, new_path)

    """获取感兴趣影像的元数据"""
    file_path = []
    for i in range(len(new_path)):
        file_path.append("images/" + new_path[i].split("\\")[-1])

    intrinsic_matrix = cam_in["cam_intrinsic"][use_img_id]  # c2p (n,3,3)
    transform_matrix = cam_ex["cam_extrinsic_c2w"][use_img_id]  # c2w (n,4,4)
    w = np.array(cam_in["imgs_w"])[use_img_id]
    h = np.array(cam_in["imgs_h"])[use_img_id]

    """(可选)获取mask, 将感兴趣区域从影像中裁剪出来"""
    if if_mask_crop:
       crop_file_path = []
       print("crop images")
       for img_idx in tqdm(range(len(new_path))):
           c2p = intrinsic_matrix[img_idx, :, :]  # c2p (3,3)
           w2c = np.linalg.inv(transform_matrix[img_idx, :-1, :-1])  # w2c (3,3)
           trans = transform_matrix[img_idx, :-1, 3].reshape(1, 3)  # (3)
           trans = np.tile(trans, reps=(p_world.shape[0], 1))  # (N 3)

           # 求出世界点的像素坐标
           p_cam = (p_world - trans) @ w2c.T  # (N,3) @ (3,3) - (N,3)
           p_pixel = p_cam @ c2p.T  # (N,3) @ (3,3)
           p_pixel[:, 0] /= p_cam[:, -1]  # np(N,3)
           p_pixel[:, 1] /= p_cam[:, -1]  # np(N,3)
           p_pixel = p_pixel[:, :-1]  # np(N,2)

           # 掩膜包围盒
           bbox_min = np.clip(np.min(p_pixel, axis=0), a_min=[0, 0], a_max=[w[img_idx]-1, h[img_idx]-1])
           bbox_max = np.clip(np.max(p_pixel, axis=0), a_min=[0, 0], a_max=[w[img_idx]-1, h[img_idx]-1])
           bbox_min_x, bbox_min_y = int(bbox_min[0]), int(bbox_min[1])
           bbox_max_x, bbox_max_y = int(bbox_max[0]), int(bbox_max[1])
           box = (bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y)

           # 判断包围盒
           if bbox_min_x + tar_size_w >= w[img_idx]:
              bbox_min_x = w[img_idx] - tar_size_w

           if bbox_min_y + tar_size_h >= h[img_idx]:
              bbox_min_y = h[img_idx] - tar_size_h

           box = (bbox_min_x, bbox_min_y, bbox_min_x + tar_size_w, bbox_min_y + tar_size_h)

           # 裁剪该影像
           ori_path = os.path.join(dataset_dir, file_path[img_idx])
           img = Image.open(ori_path)
           region = img.crop(box)
           save_path = os.path.join(crop_folder, file_path[img_idx].split("/")[-1])
           crop_file_path.append(save_path)
           region.save(save_path)

           # 裁剪改了主点的cx、cy
           intrinsic_matrix[img_idx, 0, 2] -= bbox_min_x
           intrinsic_matrix[img_idx, 1, 2] -= bbox_min_y

           # 裁剪改了图像大小
           w[img_idx] = tar_size_w
           h[img_idx] = tar_size_h

       file_path = crop_file_path

    """(可选)场景中心化、规范化"""
    if if_standardization:
        # c2w矩阵中的平移量需要平移和缩放
        transform_matrix[:, :3, 3] -= scene["center"].flatten()
        transform_matrix[:, :3, 3] *= scene["scale"]
        # 场景的包围盒需要平移和缩放
        scene["bounding_box"] -= scene["center"]
        scene["bounding_box"] *= scene["scale"]
        # 场景中心变为[0, 0, 0]
        scene["center"] = np.array([0, 0, 0])
        # 场景半径变为指定半径
        scene["radius"] = tar_radius

    """输出配置文件"""
    out = {
        "camera_mode": cam_in["cam_type"],
        "camera_orientation": cam_in["cam_orient"],
        "aabb_scale": np.exp2(np.rint(np.log2(scene["radius"]))),
        "aabb_range": scene["bounding_box"].tolist(),
        "sphere_center": scene["center"].tolist(),
        "sphere_radius": scene["radius"],
        "frames": []
    }

    for i in range(len(new_path)):
        frame = {
            # 根据具体的任务设置不同的路径
            "file_path": "images/" + file_path[i].split("\\")[-1],
            "intrinsic_matrix": intrinsic_matrix[i].tolist(),
            "transform_matrix": transform_matrix[i].tolist(),
            "w": w[i].tolist(),
            "h": h[i].tolist()
        }
        out["frames"].append(frame)

    with open(json_file_path, "w") as outputfile:
        json.dump(out, outputfile, indent=2)
