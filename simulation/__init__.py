"""SwarmLab_py.simulation —— 仿真主循环引擎。

对应 MATLAB examples/examples_swarm/*.m 的主循环部分（脱离 GUI/可视化）。
Stage 2（可视化）/ Stage 3（GUI）复用本引擎。
result_dir：群体仿真结果目录管理（SwarmLab_py/result/<alg>_<ts>/，同 MATLAB 惯例）。
"""

from .runner import Simulation, FaultEvent
from .result_dir import new_result_dir, RESULT_ROOT

__all__ = ["Simulation", "FaultEvent", "new_result_dir", "RESULT_ROOT"]
