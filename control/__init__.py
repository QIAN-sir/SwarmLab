# -*- coding: utf-8 -*-
"""SwarmLab_py.control —— 控制（对应 MATLAB control/control_drone/，Phase 9 / Stage 4）。

已迁移:
    true_states      伪真值状态估计（control_drone/true_states.m）
    pid_loop         PID 回路 + 饱和（control_drone/pid_loop.m）
    autopilot_quad   四旋翼自动驾驶仪 4 版本 + mixer（control_drone/autopilot_quad.m）
未迁移: autopilot_wing（固定翼，下一步）、compute_tf_model/compute_trim/Td2idx（配平/表，按需）。
"""
from .true_states import true_states
from .pid import pid_loop, saturate_sym, saturate_range, new_pid_param
from .autopilot_quad import autopilot_quad, mixer

__all__ = ["true_states", "pid_loop", "saturate_sym", "saturate_range",
           "new_pid_param", "autopilot_quad", "mixer"]
