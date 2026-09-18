"""SwarmLab_py.swarm —— 群体算法（对应 MATLAB @Swarm/compute_vel_*.m）。

按迁移计划 §15，本包存放两大群体速度指令算法：
    vasarhelyi.py     compute_vel_vasarhelyi   (Phase 6)
    olfati_saber.py   compute_vel_olfati_saber (Phase 7)

这些函数以 Swarm 实例为首参（对应 MATLAB @Swarm 外部方法的 self），
由 core/swarm.py 中 Swarm.update_command 分派调用。
"""

from .vasarhelyi import compute_vel_vasarhelyi, get_v_max
from .olfati_saber import compute_vel_olfati_saber

__all__ = ["compute_vel_vasarhelyi", "get_v_max", "compute_vel_olfati_saber"]
