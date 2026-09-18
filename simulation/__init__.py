"""SwarmLab_py.simulation —— 仿真主循环引擎。

对应 MATLAB examples/examples_swarm/*.m 的主循环部分（脱离 GUI/可视化）。
Stage 2（可视化）/ Stage 3（GUI）复用本引擎。
"""

from .runner import Simulation, FaultEvent

__all__ = ["Simulation", "FaultEvent"]
