"""SensorParamsMixin —— 传感器噪声/滤波/EKF 参数。

对应 MATLAB: parameters/param_drone/param_sensors.m
（param_quadcopter.m 与 param_fixed_wing.m 末尾都 ``run('param_sensors')``，
 故本 mixin 被 QuadcopterParams / FixedWingParams 共同混入；point_mass 不含。）

设计说明:
    - 采用 mixin（多继承）而非组合，以保持 MATLAB 的“扁平字段访问”
      （p_drone.sd_gyro 等直接可用），利于后续 Phase 8/9/10 忠实迁移。
    - Q/R 为派生矩阵，由 ``_init_sensor_matrices()`` 计算；混入本类的叶子类
      须在自己的 ``__post_init__`` 中调用它（避免多继承下 __post_init__ 的 MRO 问题）。
    - sd_accel = 0.0025 * gravity；gravity 在本项目恒为 9.81，故直接用默认值；
      如需其它 gravity，构造后重设 sd_accel 并再次调用 _init_sensor_matrices()。
"""

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class SensorParamsMixin:
    """传感器参数（GPS / 陀螺 / 加速度 / 气压 + EKF 噪声矩阵 + 滤波系数）。"""

    # ---- GPS ----
    sd_gps_pn: float = 0.21   # standard deviation (sd)
    sd_gps_pe: float = 0.21
    sd_gps_ph: float = 0.4
    sd_gps_vn: float = 0.05
    sd_gps_ve: float = 0.05
    sd_gps_vh: float = 0.01
    Ts_gps: float = 1.0       # [s]
    k_gps: float = 1.0 / 1100.0

    # ---- Gyroscope ----
    sd_gyro: float = math.radians(0.13)   # deg2rad(0.13) [rad/s]

    # ---- Acceleration ----
    sd_accel: float = 0.0025 * 9.81       # 0.0025 * p_physics.gravity [m/s^2]

    # ---- Pressure ----
    pres0: float = 101325.0    # static pressure at sea level [Pa]
    sd_static_pres: float = 10.0
    sd_diff_pres: float = 2.0

    # ---- Gyro bias ----
    # 注：MATLAB 中先赋 0.01，随后又覆盖为 0，最终生效值为 0
    bias_gyro_x: float = 0.0
    bias_gyro_y: float = 0.0
    bias_gyro_z: float = 0.0

    # ---- 低通滤波系数 (lp_filter 使用) ----
    a_gyro: float = 0.1
    a_static_pres: float = 0.1
    a_diff_pres: float = 0.5
    a_accel: float = 0.1
    a_gps_speed: float = 0.0
    a_gps_pos: float = 0.0

    # ---- EKF 过程/测量噪声矩阵（派生，见 _init_sensor_matrices）----
    # Q = [sd_gyro^2, 0; 0, sd_gyro^2]
    # R = ([sd_accel,0,0; 0,sd_accel,0; 0,0,sd_accel]).^2  (逐元素平方)
    Q: np.ndarray = field(default_factory=lambda: np.zeros((2, 2)))
    R: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))

    # ---- 其它 ----
    b_chidot: float = 1.2
    b_chi: float = 1.0
    b_hdot: float = 1.7
    b_h: float = 1.7
    b_Va: float = 8.0
    gamma_max: float = 10.0

    def _init_sensor_matrices(self) -> None:
        """计算 EKF 噪声矩阵 Q(2x2) 与 R(3x3)。

        对应 MATLAB:
            p_drone.Q = [ sd_gyro^2, 0; 0, sd_gyro^2 ];
            p_drone.R = ([ sd_accel,0,0; 0,sd_accel,0; 0,0,sd_accel ]).^2;
        """
        sg2 = self.sd_gyro ** 2
        self.Q = np.array([[sg2, 0.0],
                           [0.0, sg2]])
        sa2 = self.sd_accel ** 2
        self.R = np.array([[sa2, 0.0, 0.0],
                           [0.0, sa2, 0.0],
                           [0.0, 0.0, sa2]])
