# -*- coding: utf-8 -*-
"""drone_body —— 无人机机体几何 + 刚体变换（对应 MATLAB graphics/graphics_drone/
define_quad_body.m / define_wing_body.m + math_tools/rotate.m / translate.m）。

vertices 为 3×N（每列一个顶点，与 MATLAB 一致）；faces 为 0-based 顶点索引（MATLAB 1-based-1）。
坐标：机体坐标系；DroneViewer 再做 rotate→translate→NED→ENU。
"""
import math

import numpy as np


def define_quad_body():
    """四旋翼机体（define_quad_body.m）：21 顶点、9 面、红/灰/黑配色。"""
    myred = [1.0, 0.0, 0.0]
    myblack = [0.0, 0.0, 0.0]
    mygray = [0.7, 0.7, 0.7]

    rot_angle = math.pi / 4
    Rot = np.array([[math.cos(rot_angle), math.sin(rot_angle), 0.0],
                    [-math.sin(rot_angle), math.cos(rot_angle), 0.0],
                    [0.0, 0.0, 1.0]])

    raw = np.array([
        [1, 0, 0], [4, 3, 0], [3, 4, 0], [0, 1, 0], [-3, 4, 0], [-4, 3, 0],
        [-1, 0, 0], [-4, -3, 0], [-3, -4, 0], [0, -1, 0], [3, -4, 0], [4, -3, 0],
        [1, 0, 0], [0, 0, -1], [0, 1, 0], [0, 0, -1], [-1, 0, 0], [0, 0, -1],
        [0, -1, 0], [0, 0, -1], [1, 0, 0],
    ], dtype=float)
    vertices = Rot @ raw.T                            # (3, 21)

    faces = np.array([                                # 0-based（原 1-based）
        [1, 2, 3, 4, 1], [4, 5, 6, 7, 4], [7, 8, 9, 10, 7], [10, 11, 12, 1, 10],
        [1, 4, 7, 10, 1], [13, 14, 15, 13, 13], [15, 16, 17, 15, 15],
        [17, 18, 19, 17, 17], [19, 20, 21, 19, 19],
    ], dtype=int) - 1

    face_colors = np.array([myred, mygray, mygray, mygray, mygray,
                            myred, myblack, mygray, myblack], dtype=float)
    return vertices, faces, face_colors


def define_wing_body():
    """固定翼机体（define_wing_body.m）：10 顶点、3 面（左翼红/右翼绿/尾黑）。"""
    myred = [1.0, 0.0, 0.0]
    mygreen = [0.0, 1.0, 0.0]
    myblack = [0.0, 0.0, 0.0]

    raw = np.array([
        [2, 0, 0], [0, 4, 0], [-1, 4, 0], [0, 0, 0], [-1, -4, 0],
        [0, -4, 0], [2, 0, 0], [0.4, 0, -1], [0, 0, -1], [0, 0, 0],
    ], dtype=float)
    vertices = raw.T                                   # (3, 10)

    faces = np.array([[1, 6, 5, 4], [1, 4, 3, 2], [1, 4, 9, 8]], dtype=int) - 1
    face_colors = np.array([myred, mygreen, myblack], dtype=float)
    return vertices, faces, face_colors


def define_body(drone_type):
    """按类型返回 (vertices, faces, face_colors)；point_mass 返回空（对应 DroneViewer.define_body）。"""
    if drone_type == "fixed_wing":
        return define_wing_body()
    elif drone_type == "quadcopter":
        return define_quad_body()
    return np.zeros((3, 0)), np.zeros((0, 0), dtype=int), np.zeros((0, 3))


def rotate(pts, phi, theta, psi):
    """刚体旋转（rotate.m）：R=(R_roll·R_pitch·R_yaw)^T（body→inertial），pts=R@pts。"""
    R_roll = np.array([[1.0, 0.0, 0.0],
                       [0.0, math.cos(phi), math.sin(phi)],
                       [0.0, -math.sin(phi), math.cos(phi)]])
    R_pitch = np.array([[math.cos(theta), 0.0, -math.sin(theta)],
                        [0.0, 1.0, 0.0],
                        [math.sin(theta), 0.0, math.cos(theta)]])
    R_yaw = np.array([[math.cos(psi), math.sin(psi), 0.0],
                      [-math.sin(psi), math.cos(psi), 0.0],
                      [0.0, 0.0, 1.0]])
    R = R_roll @ R_pitch @ R_yaw          # inertial to body
    R = R.T                               # body to inertial
    return R @ pts


def translate(pts, pn, pe, pd):
    """平移（translate.m）：每列加 [pn;pe;pd]。"""
    return pts + np.array([[pn], [pe], [pd]])
