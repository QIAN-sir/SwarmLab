# -*- coding: utf-8 -*-
"""path_drawing —— 航点/路径 3D 绘制（对应 MATLAB graphics/graphics_map/
draw_waypoints.m / draw_path.m + points_along_dubins_path 子函数）。

坐标约定（与 DroneViewer/SwarmViewer 一致）：plot x=east, y=north, z=up（NED→ENU，z=-wd）。
draw_waypoints：fillet 航点(chi_d=-9999)直接连线；dubins 航点用 compute_dubins_param +
    points_along_dubins_path 生成密集点（圆弧+直线）。
draw_path：flag=1 直线段 / flag=2 圆弧 orbit。

⚠️ B-24（修复）：draw_path.m 的 c/rho/lambda 索引按 wing(30) 布局（c@8:11,rho@11,lam@12），
   对 quad(33) 布局（c@11:14,rho@14,lam@15）会错位读到 r_next。意图=画当前 path 段的
   orbit 圆心/半径，故按 len(path) 区分布局取正确索引。
"""
import math

import numpy as np

from ..guidance import compute_dubins_param

_TWO_PI = 2.0 * math.pi


def _arc_theta(th1, th2, lam, Del):
    """沿圆弧从 th1 到 th2（lam>0 逆时针/CCW 增，lam<0 顺时针减），步长 Del。"""
    if lam > 0:
        if th1 >= th2:
            return np.concatenate([np.arange(th1, _TWO_PI, Del),
                                   np.arange(0.0, th2 + Del, Del)])
        return np.arange(th1, th2 + Del, Del)
    else:
        if th1 <= th2:
            return np.concatenate([np.arange(th1, -Del, -Del),
                                   np.arange(_TWO_PI, th2 - Del, -Del)])
        return np.arange(th1, th2 - Del, -Del)


def points_along_dubins_path(dubinspath, Del=0.1):
    """Dubins 路径密集点（起点弧 + 直线 + 终点弧）。返回 (X, Y, Z)（NED 机体惯常系）。"""
    dp = dubinspath
    X, Y, Z = [], [], []

    # ---- 起点圆弧 ----
    th1 = math.atan2(dp["ps"][1] - dp["cs"][1], dp["ps"][0] - dp["cs"][0]) % _TWO_PI
    th2 = math.atan2(dp["w1"][1] - dp["cs"][1], dp["w1"][0] - dp["cs"][0]) % _TWO_PI
    for th in _arc_theta(th1, th2, dp["lams"], Del):
        X.append(dp["cs"][0] + dp["R"] * math.cos(th))
        Y.append(dp["cs"][1] + dp["R"] * math.sin(th))
        Z.append(dp["cs"][2])

    # ---- 直线 w1→w2 ----
    sig = 0.0
    while sig <= 1.0:
        X.append((1 - sig) * dp["w1"][0] + sig * dp["w2"][0])
        Y.append((1 - sig) * dp["w1"][1] + sig * dp["w2"][1])
        Z.append((1 - sig) * dp["w1"][2] + sig * dp["w2"][2])
        sig += Del

    # ---- 终点圆弧 ----
    th2 = math.atan2(dp["pe"][1] - dp["ce"][1], dp["pe"][0] - dp["ce"][0]) % _TWO_PI
    th1 = math.atan2(dp["w2"][1] - dp["ce"][1], dp["w2"][0] - dp["ce"][0]) % _TWO_PI
    for th in _arc_theta(th1, th2, dp["lame"], Del):
        X.append(dp["ce"][0] + dp["R"] * math.cos(th))
        Y.append(dp["ce"][1] + dp["R"] * math.sin(th))
        Z.append(dp["ce"][2])

    return np.array(X), np.array(Y), np.array(Z)


def _set_line(line, x, y, z):
    line.set_data(x, y)
    line.set_3d_properties(z)


def draw_waypoints(ax, wpt_list, radius, handle=None):
    """绘制航点连线（draw_waypoints.m）。wpt_list (N,5)=[wn,we,wd,chi_d,Va_d]。返回 Line3D。"""
    wpt = np.asarray(wpt_list, dtype=float).reshape(-1, 5)
    if wpt.shape[0] == 0:
        return handle
    if wpt[0, 3] == -9999:                       # fillet/直线航点：直接连
        XX, YY, ZZ = wpt[:, 0], wpt[:, 1], wpt[:, 2]
    else:                                       # dubins：相邻航点间密集点
        xs, ys, zs = [], [], []
        for i in range(1, wpt.shape[0]):
            dp = compute_dubins_param(np.concatenate([wpt[i - 1], [0.0, 0.0]]),
                                      np.concatenate([wpt[i], [0.0, 0.0]]), radius)
            if dp is None:
                continue
            tx, ty, tz = points_along_dubins_path(dp, 0.1)
            xs.append(tx); ys.append(ty); zs.append(tz)
        if not xs:
            return handle
        XX, YY, ZZ = np.concatenate(xs), np.concatenate(ys), np.concatenate(zs)

    if handle is None:
        handle, = ax.plot(YY, XX, -ZZ, "b", lw=1.0)   # x=east, y=north, z=up
    else:
        _set_line(handle, YY, XX, -ZZ)
    return handle


def draw_path(ax, path, map_width, handle=None):
    """绘制当前 path 段（draw_path.m）：flag=1 直线 / flag=2 orbit。返回 Line3D。"""
    path = np.asarray(path, dtype=float).ravel()
    if path.size < 13:
        return handle
    flag = path[0]
    r = path[2:5]
    q = path[5:8]
    if flag == 1:                                # 直线段
        XX = np.array([r[0], r[0] + map_width * q[0]])
        YY = np.array([r[1], r[1] + map_width * q[1]])
        ZZ = np.array([r[2], r[2] + map_width * q[2]])
    else:                                        # orbit 圆
        # B-24：按布局取 c/rho（quad=33 → c@11:14,rho@14；wing=30 → c@8:11,rho@11）
        if path.size >= 33:
            c, rho = path[11:14], path[14]
        else:
            c, rho = path[8:11], path[11]
        th = np.linspace(0.0, _TWO_PI, 101)
        XX = c[0] + rho * np.cos(th)
        YY = c[1] + rho * np.sin(th)
        ZZ = np.full_like(th, c[2])

    if handle is None:
        handle, = ax.plot(YY, XX, -ZZ, "r", lw=1.0)
    else:
        _set_line(handle, YY, XX, -ZZ)
    return handle
