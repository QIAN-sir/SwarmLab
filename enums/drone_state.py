"""DroneState —— 无人机（故障）状态枚举。★二次开发新增★

对应 MATLAB 源文件: enumerators/DroneState.m

MATLAB 原始注释（保留）:
    %classdef DroneState：定义了一个名为 DroneState 的类，这是一个枚举类。
    % 代码定义了无人机的三种状态类型————“正常（Normal）”、“动力锁死（Stalled）”、
    % “失控漂移（Drifting）”
    enumeration	%enumeration：定义枚举值的开始。
        Normal, Stalled, Drifting

迁移说明:
    - 本枚举是本项目相对原版 EPFL SwarmLab 的核心增量，用于区分正常/故障状态，
      在 Drone.update_state 的 point_mass 分支、Swarm.set_dronestate、以及
      Olfati-Saber / Vasarhelyi 的故障邻居处理中被广泛使用。
    - 采用 ``(str, Enum)`` 混入，保持 MATLAB 中 ``drone_state == "Normal"``、
      ``self.drones(i).drone_state == "Normal"`` 这类字符串比较行为。
    - 成员值严格保留 MATLAB 大小写：Normal / Stalled / Drifting（首字母大写）。
"""

from enum import Enum


class DroneState(str, Enum):
    """无人机的三种状态：正常（Normal）、动力锁死（Stalled）、失控漂移（Drifting）。"""

    NORMAL = "Normal"      # 正常：跟随群体速度指令
    STALLED = "Stalled"    # 动力锁死：速度归零、位置保持（悬停/静止）
    DRIFTING = "Drifting"  # 失控漂移：记录故障前速度 drift_vel，按 fad 衰减
