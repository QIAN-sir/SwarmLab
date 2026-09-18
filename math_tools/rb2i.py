"""Rb2i —— 机体坐标系 → 惯性坐标系(NED) 的旋转矩阵。

对应 MATLAB: math_tools/Rb2i.m
被 Swarm.get_vel_ned（以及后续 physics/control 中的坐标变换）调用。

注：MATLAB 原文件内注释写作 "Rotation matrix from inertial frame to body frame"，
但矩阵本身是标准的 body→inertial(Z-Y-X 欧拉角)旋转，get_vel_ned 用 ``Rbi * vel_xyz``
把机体系速度转到 NED。此处忠实照搬矩阵，不改动（计划 §2.3）。
"""

import math

import numpy as np


def Rb2i(phi, theta, psi):
    """返回 3×3 旋转矩阵 Rbi（机体 → 惯性/NED）。

    对应 MATLAB Rb2i.m：
        cr=cos(phi); cp=cos(theta); cy=cos(psi);
        sr=sin(phi); sp=sin(theta); sy=sin(psi);
        Rbi = [cp*cy, sr*sp*cy-cr*sy, cr*sp*cy+sr*sy;
               cp*sy, sr*sp*sy+cr*cy, cr*sp*sy-sr*cy;
               -sp,   sr*cp,          cr*cp];
    """
    cr = math.cos(phi)
    cp = math.cos(theta)
    cy = math.cos(psi)
    sr = math.sin(phi)
    sp = math.sin(theta)
    sy = math.sin(psi)

    Rbi = np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp,     sr * cp,                cr * cp],
    ])
    return Rbi
