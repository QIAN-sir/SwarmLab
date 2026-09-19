# -*- coding: utf-8 -*-
"""pid_loop / saturate（对应 MATLAB control/control_drone/pid_loop.m 与 autopilot 内 saturate）。

param 为 PID 状态字典 {int, diff, error_prev, y_c_prev}，存放在 Drone.P_* 属性上
（对应 MATLAB 以 struct 透传并持久化在 drone 对象上）。
"""
import math


def new_pid_param():
    return {"int": 0.0, "diff": 0.0, "error_prev": 0.0, "y_c_prev": 0.0}


def reset_pid_param(param):
    param["int"] = 0.0
    param["diff"] = 0.0
    param["error_prev"] = 0.0
    param["y_c_prev"] = 0.0
    return param


def saturate_sym(out_unsat, limit):
    """对称饱和 ±limit（pid_loop 内部）。"""
    if out_unsat > limit:
        return limit
    if out_unsat < -limit:
        return -limit
    return out_unsat


def saturate_range(in_, up_limit, low_limit):
    """非对称饱和 [low, up]（autopilot 对 delta 的饱和）。"""
    if in_ > up_limit:
        return up_limit
    if in_ < low_limit:
        return low_limit
    return in_


def pid_loop(y_c, y, dy, kp, ki, kd, limit, Ts, tau, param):
    """对应 pid_loop.m。dy=inf 表示无微分测量（用误差差分）。返回 (out, param)。"""
    if param is None:
        param = new_pid_param()
    integrator = param["int"]
    differentiator = param["diff"]
    error_prev = param["error_prev"]
    y_c_prev = param["y_c_prev"]

    error = y_c - y

    # Update integrator
    if math.isfinite(dy):
        integrator = integrator + Ts * (error + dy / 2)
    else:
        integrator = integrator + (Ts / 2) * (error + error_prev)

    # Update differentiator（kd==0 时 1/kd 非有限 → 跳过）
    if kd != 0:
        if math.isfinite(dy):
            dy_c = y_c - y_c_prev
            derror = dy_c - dy
            y_c_prev = y_c
        else:
            derror = error - error_prev
            error_prev = error
        differentiator = ((2 * tau - Ts) / (2 * tau + Ts) * differentiator
                          + 2 / (2 * tau + Ts) * derror)

    out_unsat = kp * error + ki * integrator + kd * differentiator
    out = saturate_sym(out_unsat, limit)

    # integrator anti-windup
    if ki != 0:
        integrator = integrator + Ts / ki * (out - out_unsat)

    param["int"] = integrator
    param["diff"] = differentiator
    param["error_prev"] = error_prev
    param["y_c_prev"] = y_c_prev
    return out, param
