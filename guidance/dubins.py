# -*- coding: utf-8 -*-
"""compute_dubins_param —— Dubins 路径参数（对应 MATLAB guidance/path_manager/compute_dubins_param.m）。

标准 Beard-McLain Dubins：在起终点位形间，从 4 种路径 (RSR/RSL/LSR/LSL) 取最短者。
子函数 Rz（z 轴旋转）、wrapTo2Pi 一并迁移。返回 dict（对应 MATLAB struct dubinspath）；
若起终点 2D 距离 < 2R，MATLAB 返回 []（并 disp 警告）→ 此处返回 None。
"""
import math

import numpy as np


def Rz(theta):
    """z 轴旋转矩阵（compute_dubins_param.m 子函数）。"""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]])


def wrap_to_2pi(x):
    """MATLAB wrapTo2Pi：把角度包到 [0, 2π)。"""
    return float(x) % (2.0 * math.pi)


def compute_dubins_param(start_node, end_node, R):
    """Dubins 路径参数。start_node/end_node = [wn, we, wd, chi, 0, 0]；R=最小转弯半径。

    返回 dict: {ps,chis,pe,chie,R,L, cs,lams, ce,lame, w1,q1,w2,w3,q3}；距离<2R 返回 None。
    """
    start_node = np.asarray(start_node, dtype=float).ravel()
    end_node = np.asarray(end_node, dtype=float).ravel()

    line_length = float(np.linalg.norm(start_node[0:2] - end_node[0:2]))
    if line_length < 2 * R:
        # MATLAB: disp('The distance between nodes must be larger than 2R.'); dubinspath = []
        return None

    ps = start_node[0:3].copy()
    chis = float(start_node[3])
    pe = end_node[0:3].copy()
    chie = float(end_node[3])

    e_s = np.array([math.cos(chis), math.sin(chis), 0.0])
    e_e = np.array([math.cos(chie), math.sin(chie), 0.0])
    crs = ps + R * (Rz(math.pi / 2) @ e_s)     # 起点右转圆心
    cls = ps + R * (Rz(-math.pi / 2) @ e_s)    # 起点左转圆心
    cre = pe + R * (Rz(math.pi / 2) @ e_e)     # 终点右转圆心
    cle = pe + R * (Rz(-math.pi / 2) @ e_e)    # 终点左转圆心
    pi2 = math.pi / 2
    twopi = 2.0 * math.pi

    # ---- L1: RSR ----
    ll = float(np.linalg.norm(crs - cre))
    theta = math.atan((cre[1] - crs[1]) / (cre[0] - crs[0]))
    L1 = ll + R * wrap_to_2pi(twopi + wrap_to_2pi(theta - pi2) - wrap_to_2pi(chis - pi2)) \
            + R * wrap_to_2pi(twopi + wrap_to_2pi(chie - pi2) - wrap_to_2pi(theta - pi2))

    # ---- L2: RSL ----
    ll = float(np.linalg.norm(cle - crs))
    theta = math.pi - math.atan((cle[1] - crs[1]) / (cle[0] - crs[0]))
    ratio = 2 * R / ll
    if abs(ratio) > 1:            # asin 非实 → MATLAB isreal==0
        L2 = 9999.0
    else:
        theta2 = theta - pi2 + math.asin(ratio)
        L2 = math.sqrt(ll ** 2 - 4 * R ** 2) \
            + R * wrap_to_2pi(twopi + wrap_to_2pi(theta2) - wrap_to_2pi(chis - pi2)) \
            + R * wrap_to_2pi(twopi + wrap_to_2pi(theta2 + math.pi) - wrap_to_2pi(chie + pi2))

    # ---- L3: LSR ----
    ll = float(np.linalg.norm(cre - cls))
    theta = math.atan((cre[1] - cls[1]) / (cre[0] - cls[0]))
    ratio = 2 * R / ll
    if abs(ratio) > 1:            # acos 非实
        L3 = 9999.0
    else:
        theta2 = math.acos(ratio)
        L3 = math.sqrt(ll ** 2 - 4 * R ** 2) \
            + R * wrap_to_2pi(twopi + wrap_to_2pi(chis + pi2) - wrap_to_2pi(theta + theta2)) \
            + R * wrap_to_2pi(twopi + wrap_to_2pi(chie - pi2) - wrap_to_2pi(theta + theta2 - math.pi))

    # ---- L4: LSL ----
    ll = float(np.linalg.norm(cls - cle))
    theta = math.atan((cre[1] - cls[1]) / (cre[0] - cls[0]))
    L4 = ll + R * wrap_to_2pi(twopi + wrap_to_2pi(chis + pi2) - wrap_to_2pi(theta + pi2)) \
            + R * wrap_to_2pi(twopi + wrap_to_2pi(theta + pi2) - wrap_to_2pi(chie + pi2))

    Ls = [L1, L2, L3, L4]
    idx = int(np.argmin(Ls))       # 并列取第一个（同 MATLAB min）
    L = Ls[idx]
    e1 = np.array([1.0, 0.0, 0.0])

    if idx == 0:      # RSR
        cs, lams, ce, lame = crs, 1.0, cre, 1.0
        q1 = (ce - cs) / np.linalg.norm(ce - cs)
        w1 = cs + R * (Rz(-pi2) @ q1)
        w2 = ce + R * (Rz(-pi2) @ q1)
    elif idx == 1:    # RSL
        cs, lams, ce, lame = crs, 1.0, cle, -1.0
        ll = float(np.linalg.norm(ce - cs))
        theta = math.atan2(ce[1] - cs[1], ce[0] - cs[0])
        theta2 = theta - pi2 + math.asin(2 * R / ll)
        q1 = Rz(theta2 + pi2) @ e1
        w1 = cs + R * (Rz(theta2) @ e1)
        w2 = ce + R * (Rz(theta2 + math.pi) @ e1)
    elif idx == 2:    # LSR
        cs, lams, ce, lame = cls, -1.0, cre, 1.0
        ll = float(np.linalg.norm(ce - cs))
        theta = math.atan2(ce[1] - cs[1], ce[0] - cs[0])
        theta2 = math.acos(2 * R / ll)
        q1 = Rz(theta + theta2 - pi2) @ e1
        w1 = cs + R * (Rz(theta + theta2) @ e1)
        w2 = ce + R * (Rz(theta + theta2 - math.pi) @ e1)
    else:             # LSL
        cs, lams, ce, lame = cls, -1.0, cle, -1.0
        q1 = (ce - cs) / np.linalg.norm(ce - cs)
        w1 = cs + R * (Rz(pi2) @ q1)
        w2 = ce + R * (Rz(pi2) @ q1)

    w3 = pe.copy()
    q3 = Rz(chie) @ e1

    return {"ps": ps, "chis": chis, "pe": pe, "chie": chie, "R": R, "L": L,
            "cs": cs, "lams": lams, "ce": ce, "lame": lame,
            "w1": w1, "q1": q1, "w2": w2, "w3": w3, "q3": q3}
