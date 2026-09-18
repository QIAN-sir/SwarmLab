"""DroneType —— 无人机类型枚举。

对应 MATLAB 源文件: enumerators/DroneType.m

MATLAB 原始注释（保留）:
    % This class (enumerator) is meant to manage the Drone types:
    % quadcopter, fixed_wing and point_mass

迁移说明:
    - 采用 ``(str, Enum)`` 混入，使 ``DroneType.POINT_MASS == "point_mass"``
      成立，从而保持 MATLAB 中 ``drone_type == "point_mass"`` 这类
      “枚举与字符串直接比较”的行为（见 Drone.update_state 等）。
    - 成员“值”严格等于 MATLAB 枚举成员名（含大小写），成员“名”按 PEP8
      采用大写下划线风格。需要原始字符串时请使用 ``.value``。
"""

from enum import Enum


class DroneType(str, Enum):
    """管理无人机类型：quadcopter（四旋翼）、fixed_wing（固定翼）、point_mass（质点）。"""

    QUADCOPTER = "quadcopter"
    FIXED_WING = "fixed_wing"
    POINT_MASS = "point_mass"
