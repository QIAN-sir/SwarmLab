"""SwarmLab_py.control —— 控制域（对应 MATLAB control/control_drone/）。

Phase 3 先引入 true_states（Drone.set_pos / estimate_states 依赖的伪状态估计）。
其余（autopilot_quad/wing、PID、mixer、saturate 等）在 Phase 9 迁移。
"""

from .true_states import true_states

__all__ = ["true_states"]
