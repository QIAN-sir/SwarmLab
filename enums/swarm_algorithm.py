"""SwarmAlgorithm —— 群体算法枚举。

对应 MATLAB 源文件: enumerators/SwarmAlgorithm.m

MATLAB 原始注释（保留）:
    % This class (enumerator) is meant to manage the Swarm algorithms:
    % vasarhelyi, olfati_saber

迁移说明:
    - 采用 ``(str, Enum)`` 混入，保持 MATLAB 中 ``self.algorithm == "vasarhelyi"``
      这类字符串比较行为（见 Swarm.update_command）。
    - ⚠️ 该枚举的“值”还被用于动态脚本名拼接：param_swarm.m 末尾
      ``run(strcat("param_", SWARM_ALGORITHM))`` 会据此加载
      param_olfati_saber / param_vasarhelyi。迁移到 Python 后应改为
      显式的字典分派/工厂函数（Phase 2 处理），届时使用 ``.value``。
"""

from enum import Enum


class SwarmAlgorithm(str, Enum):
    """管理群体算法：vasarhelyi、olfati_saber。"""

    VASARHELYI = "vasarhelyi"
    OLFATI_SABER = "olfati_saber"
