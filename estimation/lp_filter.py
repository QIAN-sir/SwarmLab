# -*- coding: utf-8 -*-
"""lp_filter —— 一阶低通滤波（对应 MATLAB estimation/lp_filter.m）。

MATLAB 原函数签名 ``lp_filter(y_prev, u, a, dt)``（纯函数、无 persistent）:
    y = exp(-a*dt)*y_prev + (1 - exp(-a*dt))*u

⚠️ B-20（修复，计划 §2.3 用户授权）：MATLAB ``estimate_states`` 的调用为
   ``lp_filter(测量值, 0, a, dt)``——即把当前测量当 y_prev、把 0 当输入 u。根因是
   lp_filter 早期签名 ``(u, reset, a, dt)``（内含 persistent）被重构为 ``(y_prev, u, a, dt)``
   后调用方未同步，导致"低通滤波"退化成 ``exp(-a*dt)*测量值``（无记忆、近似直通）。
   意图=带记忆的一阶低通。修复：Estimator 各通道保存上一步输出，按正确顺序
   ``lp_filter(prev, u, a, dt)`` 调用（见 lp_step），首次调用以 u 初始化（对应 reset）。
"""
import math


def lp_filter(y_prev, u, a, dt):
    """一阶低通滤波公式（lp_filter.m，忠实）。y_prev=上一步输出，u=当前输入。"""
    e = math.exp(-a * dt)
    return e * y_prev + (1.0 - e) * u


def lp_step(lp_state, key, u, a, dt):
    """带各通道记忆的低通滤波（B-20 修复：实现 lp_filter 的意图）。

    lp_state : dict，各通道上一步输出（Estimator 持有，t=0 清空）。
    key      : 通道名（如 'p'/'q'/'h'/'Va'/'pn'/...）。
    首次（prev is None）以 u 初始化 → 输出 u（对应 MATLAB reset 分支 y_prev=u）。
    """
    prev = lp_state.get(key)
    y = u if prev is None else lp_filter(prev, u, a, dt)
    lp_state[key] = y
    return y
