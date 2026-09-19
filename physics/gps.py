# -*- coding: utf-8 -*-
"""gps —— GPS 测量仿真（对应 MATLAB physics/gps.m）。

输出 y(5) = [y_gps_pn; y_gps_pe; y_gps_ph; y_gps_Vg; y_gps_course]。
x 为 12 维状态（0-based：pn=0,pe=1,pd=2,vx=3,vy=4,vz=5,phi=6,theta=7,psi=8）。

MATLAB ``persistent mp_n/mp_e/mp_h``（一阶 Markov 过程，isempty→0）→ 状态字典 st
（计划 §10：persistent→对象/状态实例）。st 由 Drone 持有（self.gps_state）。
"""
import math

import numpy as np

from ..math_tools import Rb2i


def new_gps_state():
    """GPS Markov 过程初始状态（对应 MATLAB isempty(mp_*)→0）。"""
    return {"mp_n": 0.0, "mp_e": 0.0, "mp_h": 0.0}


def gps(x, dt, p_drone, st):
    """仿真 GPS 测量（gps.m）。st 为 new_gps_state() 返回的可变字典。"""
    x = np.asarray(x, dtype=float).ravel()

    pn = x[0]
    pe = x[1]
    pd = x[2]
    vx = x[3]
    vy = x[4]
    vz = x[5]
    phi = x[6]
    theta = x[7]
    psi = x[8]

    # Construct Markov Process（一阶，随 dt 衰减 + 高斯激励）
    decay = math.exp(-p_drone.k_gps * dt)
    st["mp_n"] = decay * st["mp_n"] + np.random.normal(0, p_drone.sd_gps_pn)
    st["mp_e"] = decay * st["mp_e"] + np.random.normal(0, p_drone.sd_gps_pe)
    st["mp_h"] = decay * st["mp_h"] + np.random.normal(0, p_drone.sd_gps_ph)

    # Construct North, East, and altitude GPS measurements
    y_gps_pn = pn + st["mp_n"]
    y_gps_pe = pe + st["mp_e"]
    y_gps_ph = -pd + st["mp_h"]

    Rbi = Rb2i(phi, theta, psi)
    vi = Rbi @ np.array([vx, vy, vz])
    vn = vi[0]
    ve = vi[1]

    y_gps_vn = vn + np.random.normal(0, p_drone.sd_gps_vn)
    y_gps_ve = ve + np.random.normal(0, p_drone.sd_gps_ve)

    # Construct groundspeed and course measurements
    y_gps_Vg = math.sqrt(y_gps_vn ** 2 + y_gps_ve ** 2)
    y_gps_course = math.atan2(y_gps_ve, y_gps_vn)

    return np.array([y_gps_pn, y_gps_pe, y_gps_ph, y_gps_Vg, y_gps_course])
