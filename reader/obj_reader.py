# -*- coding:utf-8 -*-
"""
@Time: 2024/2/26 19:21
@author: RefineM
@file: obj_reader.py

"""
import numpy as np
import xmltodict
from scripts.tools import get_scene_box


class SceneReader(object):
    def __init__(self, obj, xml):
        """由感兴趣区域mesh文件及元数据文件获取感兴趣场景信息"""
        self.srs_origin = None
        self.srs = None
        self.mesh_obj_path = obj
        self.mesh_xml_path = xml
        self.get_srs()

    def get_srs(self):
        """获取mesh的参考坐标系以及中心坐标"""
        with open(self.mesh_xml_path, 'r') as f:
            mesh_xml = xmltodict.parse(f.read())
        model_meta = mesh_xml['ModelMetadata']
        if 'SRS' not in model_meta.keys():
            self.srs = 'local system'
            self.srs_origin = [0.0, 0.0, 0.0]
        else:
            self.srs = mesh_xml['ModelMetadata']['SRS']
            ori = mesh_xml['ModelMetadata']['SRSOrigin'].split(",")
            self.srs_origin = [float(ori[0]), float(ori[1]), float(ori[2])]

    def get_3d_points(self):
        """
        获取mesh世界点坐标
        vertex/顶点坐标/np.array(N,3)
        """
        _v = []
        for line in open(self.mesh_obj_path, 'r'):
            line = line.split(' ')
            if line[0] == 'v':
                # 注意这里，obj保存的顶点的坐标是相对于中心点的坐标，要还原
                v = np.array([float(line[1]) + self.srs_origin[0],
                              float(line[2]) + self.srs_origin[1],
                              float(line[3]) + self.srs_origin[2]])
                _v.append(v)
        vertex = np.stack(_v, axis=0)
        return vertex

    def centered_and_scaled_3d_points(self, output_path):
        """
        将mesh点坐标中心化、规范化之后输出
        :param output_path: 输出mesh文件路径
        :return: 在指定路径生成mesh
        """
        _v = []
        _vt = []
        _f = []
        for line in open(self.mesh_obj_path, 'r'):
            line_split = line.split(' ')
            if line_split[0] == 'v':
                v = [float(line_split[1]) + self.srs_origin[0],
                     float(line_split[2]) + self.srs_origin[1],
                     float(line_split[3]) + self.srs_origin[2]]
                _v.append(v)
            if line_split[0] == 'vt':
                _vt.append(line)
            if line_split[0] == 'f':
                _f.append(line)

        p_world = _v
        p_world = np.array(p_world)  # np.array(n,3)
        scene = get_scene_box(p_world, tar_radius=1.0)
        center = scene['center']  # np.array(3,1)
        scale = scene['scale']  # float
        p_world = (p_world - center[:, 0]) * scale  # np.array(n,3)

        f = open(output_path, 'w')
        for i in range(np.size(p_world, 0)):
            f.write("v {0} {1} {2}\n".format(p_world[i, 0], p_world[i, 1], p_world[i, 2]))
        for i in _vt:
            f.write(i)
        for i in _f:
            f.write(i)
        f.close()
