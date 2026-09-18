"""SwarmLab_py.core —— 核心对象（对应 MATLAB @Drone / @Swarm）。

Phase 3: Drone（含 point_mass 故障状态机）。
Phase 4: Swarm（群管理 + 故障注入 set_dronestate + update_state(fad)）。
"""

from .drone import Drone
from .swarm import Swarm

__all__ = ["Drone", "Swarm"]
