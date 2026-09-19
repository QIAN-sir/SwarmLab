# -*- coding: utf-8 -*-
"""autopilot_quad —— 四旋翼自动驾驶仪（对应 MATLAB control/control_drone/autopilot_quad.m）。

autopilot_version: 1=attitude, 2=velocity, 3=acceleration, 4=position。
输出 y = [delta(4); x_command(19)]。
z_hat 索引(0-based): pn0 pe1 h2 vx3 vy4 vz5 Va6 phi9 theta10 psi11 p13 q14 r15。
⚠️ B-17（记录）：MATLAB pe_ve_hold 签名缺 p_sim 却用 p_sim.dt（潜在未定义变量）；
   Python 补入 p_sim 参数使其可运行，语义不变。
"""
import math

import numpy as np

from ..math_tools import Rb2i
from .pid import pid_loop, reset_pid_param, new_pid_param, saturate_range

_INF = float("inf")


# ======================================================================
# Mixer / 饱和
# ======================================================================
def mixer(thrust, torque_phi, torque_theta, torque_psi, p_drone, p_sim):
    o1 = thrust / (4 * p_drone.C_prop) + torque_theta / (2 * p_drone.C_prop * p_drone.l_arm) - torque_psi / (4 * p_drone.CD)
    o2 = thrust / (4 * p_drone.C_prop) - torque_phi / (2 * p_drone.C_prop * p_drone.l_arm) + torque_psi / (4 * p_drone.CD)
    o3 = thrust / (4 * p_drone.C_prop) - torque_theta / (2 * p_drone.C_prop * p_drone.l_arm) - torque_psi / (4 * p_drone.CD)
    o4 = thrust / (4 * p_drone.C_prop) + torque_phi / (2 * p_drone.C_prop * p_drone.l_arm) + torque_psi / (4 * p_drone.CD)
    d1 = math.sqrt(o1) / p_drone.k_omega if o1 > 0 else 0.0
    d2 = math.sqrt(o2) / p_drone.k_omega if o2 > 0 else 0.0
    d3 = math.sqrt(o3) / p_drone.k_omega if o3 > 0 else 0.0
    d4 = math.sqrt(o4) / p_drone.k_omega if o4 > 0 else 0.0
    return d1, d2, d3, d4


def _sat_delta(d1, d2, d3, d4, p_drone):
    return (saturate_range(d1, p_drone.dt_max, 0), saturate_range(d2, p_drone.dt_max, 0),
            saturate_range(d3, p_drone.dt_max, 0), saturate_range(d4, p_drone.dt_max, 0))


def _alt_state_reinit(drone, h, t, p_drone):
    # MATLAB: 两分支均置 2（忠实保留）
    drone.altitude_state = 2 if h <= p_drone.altitude_take_off_zone else 2
    if t == 0 or drone.altitude_state != drone.altitude_state_prev:
        drone.initialize_integrator = 1
    else:
        drone.initialize_integrator = 0
    return drone.initialize_integrator


def _P(drone, name):
    P = getattr(drone, name)
    if P is None:
        P = new_pid_param()
        setattr(drone, name, P)
    return P


# ======================================================================
# Hold 函数（PID 外层回路）
# ======================================================================
def h_thrust_hold(h_c, h, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_h_thrust")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(h_c, h, _INF, p_drone.kp_h, p_drone.ki_h, p_drone.kd_h,
                      p_drone.thrust_max, p_sim.dt, 0.1, P)
    drone.P_h_thrust = P
    return out


def roll_torque_hold(phi_c, phi, p, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_roll_torque")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(phi_c, phi, p, p_drone.kp_phi, 0, p_drone.kd_phi,
                      p_drone.torque_max, p_sim.dt, 0.1, P)
    drone.P_roll_torque = P
    return out


def pitch_torque_hold(theta_c, theta, q, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_pitch_torque")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(theta_c, theta, q, p_drone.kp_theta, 0, p_drone.kd_theta,
                      p_drone.torque_max, p_sim.dt, 0.1, P)
    drone.P_pitch_torque = P
    return out


def yaw_torque_hold(psi_c, psi, r, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_psi_torque")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(psi_c, psi, r, p_drone.kp_psi, 0, p_drone.kd_psi,
                      p_drone.torque_max, p_sim.dt, 0.1, P)
    drone.P_psi_torque = P
    return out


def vd_thrust_hold(vd_c, vd, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_vd_thrust")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(vd_c, vd, _INF, p_drone.kp_vd, p_drone.ki_vd, 0,
                      p_drone.thrust_max, p_sim.dt, 0.1, P)
    drone.P_vd_thrust = P
    return out


def ve_roll_hold(ve_c, ve, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_ve_roll")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(ve_c, ve, _INF, p_drone.kp_ve, p_drone.ki_ve, 0,
                      p_drone.phi_max, p_sim.dt, 0.1, P)
    drone.P_ve_roll = P
    return out


def vn_pitch_hold(vn_c, vn, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_vn_pitch")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(vn_c, vn, _INF, p_drone.kp_vn, p_drone.ki_vn, 0,
                      p_drone.theta_max, p_sim.dt, 0.1, P)
    drone.P_vn_pitch = P
    return out


def ad_thrust_hold(ad_c, ad, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_ad_thrust")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(ad_c, ad, _INF, p_drone.kp_ad, p_drone.ki_ad, 0,
                      p_drone.thrust_max, p_sim.dt, 0.1, P)
    drone.P_ad_thrust = P
    return out


def ae_roll_hold(ae_c, ae, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_ae_roll")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(ae_c, ae, _INF, p_drone.kp_ae, p_drone.ki_ae, 0,
                      p_drone.phi_max, p_sim.dt, 0.1, P)
    drone.P_ae_roll = P
    return out


def an_pitch_hold(an_c, an, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_an_pitch")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(an_c, an, _INF, p_drone.kp_an, p_drone.ki_an, 0,
                      p_drone.theta_max, p_sim.dt, 0.1, P)
    drone.P_an_pitch = P
    return out


def pd_vd_hold(pd_c, pd, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_pd_thrust")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(pd_c, pd, _INF, p_drone.kp_vd, p_drone.ki_vd, p_drone.kd_vd,
                      p_drone.vd_max, p_sim.dt, 0.1, P)
    drone.P_pd_thrust = P
    return out


def pe_ve_hold(pe_c, pe, reinit, p_drone, p_sim, drone):   # B-17: 补 p_sim
    P = _P(drone, "P_pe_roll")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(pe_c, pe, _INF, p_drone.kp_ve, p_drone.ki_ve, p_drone.kd_ve,
                      p_drone.v_max, p_sim.dt, 0.1, P)
    drone.P_pe_roll = P
    return out


def pn_vn_hold(pn_c, pn, reinit, p_drone, p_sim, drone):
    P = _P(drone, "P_pn_pitch")
    if reinit:
        reset_pid_param(P)
    out, P = pid_loop(pn_c, pn, _INF, p_drone.kp_vn, p_drone.ki_vn, p_drone.kd_vn,
                      p_drone.v_max, p_sim.dt, 0.1, P)
    drone.P_pn_pitch = P
    return out


# ======================================================================
# Autopilot versions
# ======================================================================
def _finish(drone, d1, d2, d3, d4, x_command):
    drone.altitude_state_prev = drone.altitude_state
    return np.array([d1, d2, d3, d4]), np.array(x_command, dtype=float)


def autopilot_attitude(h_c, phi_c, theta_c, psi_c, h, phi, theta, psi,
                       p, q, r, t, p_drone, p_sim, drone):
    reinit = _alt_state_reinit(drone, h, t, p_drone)
    if drone.altitude_state == 1:
        d1 = d2 = d3 = d4 = 1.0
    else:
        thrust = h_thrust_hold(h_c, h, reinit, p_drone, p_sim, drone)
        t_phi = roll_torque_hold(phi_c, phi, p, reinit, p_drone, p_sim, drone)
        t_theta = pitch_torque_hold(theta_c, theta, q, reinit, p_drone, p_sim, drone)
        t_psi = yaw_torque_hold(psi_c, psi, r, reinit, p_drone, p_sim, drone)
        d1, d2, d3, d4 = mixer(thrust, t_phi, t_theta, t_psi, p_drone, p_sim)
        d1, d2, d3, d4 = _sat_delta(d1, d2, d3, d4, p_drone)
    xc = [0, 0, h_c, 0, 0, 0, 0, 0, 0, 0, 0, 0, phi_c, theta_c, psi_c, 0, 0, 0, 0]
    return _finish(drone, d1, d2, d3, d4, xc)


def autopilot_velocity(psi_c, vn_c, ve_c, vd_c, vx, vy, vz, phi, theta, psi,
                       p, q, r, h, t, p_drone, p_sim, drone):
    Rbi = Rb2i(phi, theta, psi)
    v_ned = Rbi @ np.array([vx, vy, vz])
    vn, ve, vd = v_ned[0], v_ned[1], v_ned[2]
    reinit = _alt_state_reinit(drone, h, t, p_drone)
    if drone.altitude_state == 1:
        d1 = d2 = d3 = d4 = 1.0
        phi_c = theta_c = 0.0
    else:
        thrust = vd_thrust_hold(vd_c, vd, reinit, p_drone, p_sim, drone)
        phi_c = ve_roll_hold(ve_c, ve, reinit, p_drone, p_sim, drone)
        t_phi = roll_torque_hold(phi_c, phi, p, reinit, p_drone, p_sim, drone)
        theta_c = vn_pitch_hold(vn_c, vn, reinit, p_drone, p_sim, drone)
        t_theta = pitch_torque_hold(theta_c, theta, q, reinit, p_drone, p_sim, drone)
        t_psi = yaw_torque_hold(psi_c, psi, r, reinit, p_drone, p_sim, drone)
        d1, d2, d3, d4 = mixer(thrust, t_phi, t_theta, t_psi, p_drone, p_sim)
        d1, d2, d3, d4 = _sat_delta(d1, d2, d3, d4, p_drone)
    xc = [0, 0, 0, vn_c, ve_c, vd_c, 0, 0, 0, 0, 0, 0, phi_c, theta_c, psi_c, 0, 0, 0, 0]
    return _finish(drone, d1, d2, d3, d4, xc)


def autopilot_acceleration(psi_c, an_c, ae_c, ad_c, an, ae, ad, phi, theta, psi,
                           p, q, r, h, t, p_drone, p_sim, drone):
    reinit = _alt_state_reinit(drone, h, t, p_drone)
    if drone.altitude_state == 1:
        d1 = d2 = d3 = d4 = 1.0
        phi_c = theta_c = 0.0
    else:
        thrust = ad_thrust_hold(ad_c, ad, reinit, p_drone, p_sim, drone)
        phi_c = ae_roll_hold(ae_c, ae, reinit, p_drone, p_sim, drone)
        t_phi = roll_torque_hold(phi_c, phi, p, reinit, p_drone, p_sim, drone)
        theta_c = an_pitch_hold(an_c, an, reinit, p_drone, p_sim, drone)
        t_theta = pitch_torque_hold(theta_c, theta, q, reinit, p_drone, p_sim, drone)
        t_psi = yaw_torque_hold(psi_c, psi, r, reinit, p_drone, p_sim, drone)
        d1, d2, d3, d4 = mixer(thrust, t_phi, t_theta, t_psi, p_drone, p_sim)
        d1, d2, d3, d4 = _sat_delta(d1, d2, d3, d4, p_drone)
    xc = [0, 0, 0, 0, 0, 0, an_c, ae_c, ad_c, 0, 0, 0, phi_c, theta_c, psi_c, 0, 0, 0, 0]
    return _finish(drone, d1, d2, d3, d4, xc)


def autopilot_position(psi_c, pn_c, pe_c, pd_c, vx, vy, vz, phi, theta, psi,
                       p, q, r, h, pn, pe, t, p_drone, p_sim, drone):
    Rbi = Rb2i(phi, theta, psi)
    v_ned = Rbi @ np.array([vx, vy, vz])
    vn, ve, vd = v_ned[0], v_ned[1], v_ned[2]
    reinit = _alt_state_reinit(drone, h, t, p_drone)
    if drone.altitude_state == 1:
        d1 = d2 = d3 = d4 = 1.0
        phi_c = theta_c = vn_c = ve_c = vd_c = 0.0
    else:
        vd_c = pd_vd_hold(pd_c, h, reinit, p_drone, p_sim, drone)
        thrust = vd_thrust_hold(vd_c, vd, reinit, p_drone, p_sim, drone)
        ve_c = pe_ve_hold(pe_c, pe, reinit, p_drone, p_sim, drone)
        phi_c = ve_roll_hold(ve_c, ve, reinit, p_drone, p_sim, drone)
        t_phi = roll_torque_hold(phi_c, phi, p, reinit, p_drone, p_sim, drone)
        vn_c = pn_vn_hold(pn_c, pn, reinit, p_drone, p_sim, drone)
        theta_c = vn_pitch_hold(vn_c, vn, reinit, p_drone, p_sim, drone)
        t_theta = pitch_torque_hold(theta_c, theta, q, reinit, p_drone, p_sim, drone)
        t_psi = yaw_torque_hold(psi_c, psi, r, reinit, p_drone, p_sim, drone)
        d1, d2, d3, d4 = mixer(thrust, t_phi, t_theta, t_psi, p_drone, p_sim)
        d1, d2, d3, d4 = _sat_delta(d1, d2, d3, d4, p_drone)
    xc = [pn_c, pe_c, pd_c, vn_c, ve_c, vd_c, 0, 0, 0, 0, 0, 0,
          phi_c, theta_c, psi_c, 0, 0, 0, 0]
    return _finish(drone, d1, d2, d3, d4, xc)


# ======================================================================
# 主入口
# ======================================================================
def autopilot_quad(drone, t):
    p_drone = drone.p_drone
    p_sim = drone.p_sim
    av = drone.autopilot_version
    z = drone.z_hat
    pn, pe, h = z[0], z[1], z[2]
    vx, vy, vz = z[3], z[4], z[5]
    phi, theta, psi = z[9], z[10], z[11]
    p, q, r = math.radians(z[13]), math.radians(z[14]), math.radians(z[15])

    cmd = drone.command
    if av == 1:
        h_c, phi_c, theta_c, psi_c = cmd[0], cmd[1], cmd[2], cmd[3]
        delta, xc = autopilot_attitude(h_c, phi_c, theta_c, psi_c, h, phi, theta, psi,
                                       p, q, r, t, p_drone, p_sim, drone)
    elif av == 2:
        psi_c, vn_c, ve_c, vd_c = cmd[0], cmd[1], cmd[2], cmd[3]
        delta, xc = autopilot_velocity(psi_c, vn_c, ve_c, vd_c, vx, vy, vz,
                                       phi, theta, psi, p, q, r, h, t, p_drone, p_sim, drone)
    elif av == 3:
        psi_c, an_c, ae_c, ad_c = cmd[0], cmd[1], cmd[2], cmd[3]
        F = drone.forces
        a_ned = Rb2i(phi, theta, psi) @ F / p_drone.mass
        delta, xc = autopilot_acceleration(psi_c, an_c, ae_c, ad_c,
                                           a_ned[0], a_ned[1], a_ned[2],
                                           phi, theta, psi, p, q, r, h, t, p_drone, p_sim, drone)
    else:
        psi_c, pn_c, pe_c, pd_c = cmd[0], cmd[1], cmd[2], cmd[3]
        delta, xc = autopilot_position(psi_c, pn_c, pe_c, pd_c, vx, vy, vz,
                                       phi, theta, psi, p, q, r, h, pn, pe, t,
                                       p_drone, p_sim, drone)
    return np.concatenate([delta, xc])
