# -*- coding:utf-8 -*-
"""
@Time: 2024/2/3 0:52
@author: Junfan W
@file: scene_visualizer.py

"""
import open3d as o3d
import json
import numpy as np


def get_camera_frustum(img_size, k, c2w, frustum_length=0.5):
    """
    定义视锥体的顶点、边、颜色
    ref: nerf++
    """
    color = [1, 0, 0]
    # 根据视锥体的长度，调整影像大小
    W, H = img_size
    hfov = np.rad2deg(np.arctan(W / 2. / k[0, 0]) * 2.)
    vfov = np.rad2deg(np.arctan(H / 2. / k[1, 1]) * 2.)
    half_w = frustum_length * np.tan(np.deg2rad(hfov / 2.))
    half_h = frustum_length * np.tan(np.deg2rad(vfov / 2.))
    # 定义视锥体的5个顶点、8条边
    # 在相机坐标系下：opencv格式（x->right, y->down, z->forward）
    frustum_points = np.array([[0., 0., 0.],                          # frustum origin
                               [-half_w, -half_h, frustum_length],    # top-left images corner
                               [half_w, -half_h, frustum_length],     # top-right images corner
                               [half_w, half_h, frustum_length],      # bottom-right images corner
                               [-half_w, half_h, frustum_length]])    # bottom-left images corner
    frustum_lines = np.array([[0, i] for i in range(1, 5)] + [[i, (i+1)] for i in range(1, 4)] + [[4, 1]])
    frustum_colors = np.tile(np.array(color).reshape((1, 3)), (frustum_lines.shape[0], 1))
    # 在世界坐标系下：
    frustum_points = np.dot(np.hstack((frustum_points, np.ones_like(frustum_points[:, 0:1]))), c2w.T)  # 左乘c2w转为世界坐标
    frustum_points = frustum_points[:, :3] / frustum_points[:, 3:4]  # 齐次转非齐次
    return frustum_points, frustum_lines, frustum_colors


def frustums2lineset(frustums):
    """
    创建一个构成视锥体的点线集合，便于在open3d中显示
    ref: nerf++
    """
    N = len(frustums)
    merged_points = np.zeros((N*5, 3))      # 5 vertices per frustum
    merged_lines = np.zeros((N*8, 2))       # 8 lines per frustum
    merged_colors = np.zeros((N*8, 3))      # each line gets a color
    for i, (frustum_points, frustum_lines, frustum_colors) in enumerate(frustums):
        merged_points[i*5:(i+1)*5, :] = frustum_points
        merged_lines[i*8:(i+1)*8, :] = frustum_lines + i*5
        merged_colors[i*8:(i+1)*8, :] = frustum_colors
    # Create a LineSet from given points and line indices
    lineset = o3d.geometry.LineSet()
    lineset.points = o3d.utility.Vector3dVector(merged_points)
    lineset.lines = o3d.utility.Vector2iVector(merged_lines)
    lineset.colors = o3d.utility.Vector3dVector(merged_colors)
    return lineset


def read_json(json_path):
    """从json中读取相机的位姿"""
    with open(json_path, 'r') as f:
        meta = json.load(f)

    bbox = meta['aabb_range']
    frames = meta['frames']
    k = []
    poses = []  # c2w (opencv:RDF)
    h = []
    w = []
    for i in range(len(frames)):
        k.append(frames[i]["intrinsic_matrix"])
        poses.append(frames[i]['transform_matrix'])
        h.append(frames[i]["h"])
        w.append(frames[i]["w"])

    out = {
        "w": w,
        "h": h,
        "bbox": bbox,
        "cam_intrinsic": k,  # list(n,3,3)
        "cam_poses": poses  # list(n,4,4)
    }
    return out


def visualize_scene(
        camera_meta,
        sphere_radius=1.0,
        camera_size=0.05,
        geometry_file=None,
        geometry_type='mesh'):
    """可视化场景和相机"""

    # 场景包围球（前景球，球心为原点，半径为1）
    # 首先创建构成场景包围球的三角mesh，然后保留这个三角mesh的边，达到透明效果
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius, resolution=10)
    sphere = o3d.geometry.LineSet.create_from_triangle_mesh(sphere)
    sphere.paint_uniform_color((0, 0, 1))
    # 坐标原点和坐标轴
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0., 0., 0.])
    # 包围盒
    bbox = np.array(camera_meta["bbox"])  # np.array(3,2)
    min_bound = bbox[:, 0].tolist()  # list(3,1)
    max_bound = bbox[:, 1].tolist()  # list(3,1)
    bbox_line = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    bbox_line.color = (1, 0, 0)
    # 需要绘制的几何图形
    draw = [sphere, coord_frame, bbox_line]
    # 相机位姿
    c2w = np.array(camera_meta["cam_poses"])  # np.array(n,4,4)
    k = np.array(camera_meta["cam_intrinsic"])  # np.array(n,3,3)

    frustums = []
    for i in range(np.size(c2w, 0)):
        k_i = k[i, :, :]
        c2w_i = c2w[i, :, :]
        img_size = (camera_meta['w'][i], camera_meta['h'][i])
        frustums.append(get_camera_frustum(img_size, k_i, c2w_i, camera_size))
        cameras = frustums2lineset(frustums)
        draw.append(cameras)

    # 加载场景点云或mesh
    if geometry_file is not None:
        if geometry_type == 'mesh':
            geometry = o3d.io.read_triangle_mesh(geometry_file)
            geometry.compute_vertex_normals()
        elif geometry_type == 'pointcloud':
            geometry = o3d.io.read_point_cloud(geometry_file)
        else:
            raise Exception('Unknown geometry_type: ', geometry_type)
        draw.append(geometry)

    o3d.visualization.draw_geometries(geometry_list=draw, width=1500, height=900, window_name='scene_vis')


if __name__ == '__main__':
    dataset_dir = r'../dataset/isprs_01'
    json_path = dataset_dir + r"\transforms.json"
    geometry_file = dataset_dir + r"\Model_resized.obj"

    meta = read_json(json_path)
    visualize_scene(camera_meta=meta, geometry_file=geometry_file)
