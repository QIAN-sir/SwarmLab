"""BatteryParams —— 电池参数。

对应 MATLAB: parameters/param_battery.m（保留原始数值与单位注释）

被依赖方: Drone.update_battery / rpm2power / compute_Qt_hover。
"""

from dataclasses import dataclass


@dataclass
class BatteryParams:
    """电池模型参数。"""

    e0: float = 12.69      # [V]
    e1: float = -3.14      # [V]
    e2: float = 1.58       # [V]
    A: float = 1.53        # [V]
    B: float = 29.89       # [Ah^(-1)]
    V0: float = 12.59      # [V]
    Q0: float = 0.034      # [Ah]
    Qf: float = 1.20       # [Ah]
    R: float = 0.061       # [Ohm]
    tau: float = 1.95      # [s]

    # 电机转速→功率 多项式系数（rpm2power 使用）
    p0: float = 0.73
    p1: float = 1.31e-7
    p2: float = 4.03e-15
    p3: float = -1.22e-23
    p4: float = 1.65e-31

    power_board: float = 0  # 板载功耗 [W]
