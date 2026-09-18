"""DroneParams 及其子类 —— 无人机参数。

对应 MATLAB:
    parameters/param_drone/param_drone.m     （公共部分：初始条件/风/执行器上限/规划参数）
    parameters/param_drone/param_quadcopter.m（四旋翼专属 + run('param_sensors')）

设计（Phase 2，基类 + 继承子类）:
    - DroneParams        : 公共字段（各类型默认值 = param_drone.m 公共部分）
    - PointMassParams    : 仅覆盖 Va_T=5
    - QuadcopterParams   : 覆盖/新增四旋翼字段 + 混入 SensorParamsMixin
    - FixedWingParams    : 见 fixed_wing.py
    - 派生量（gamma1-8、PID 增益、thrust_max、传感器 Q/R）在子类 __post_init__ 计算；
      R_min/radius 依赖 gravity，由 build_drone_params 工厂计算。
    - KSI_OPT：param_drone.m 中的工作区常量(0.7)，非 p_drone 字段，此处作模块常量。
"""

import math
from dataclasses import dataclass, field

import numpy as np

from ..enums import DroneType
from .sensors import SensorParamsMixin

# param_drone.m: KSI_OPT = 0.7;  % 阻尼比（用于控制系统设计），工作区常量
KSI_OPT = 0.7


def compute_inertia_gammas(Jx, Jy, Jz, Jxz):
    """惯量派生量 gamma, gamma1..gamma8（quadcopter 与 fixed_wing 共用）。

    对应 param_quadcopter.m / param_fixed_wing.m 的 'Compute gammas' 段。
    返回 9 元组 (gamma, gamma1, ..., gamma8)。
    """
    g = Jx * Jz - Jxz ** 2
    return (
        g,
        Jxz * (Jx - Jy + Jz) / g,
        (Jz * (Jz - Jy) + Jxz ** 2) / g,
        Jz / g,
        Jxz / g,
        (Jz - Jx) / Jy,
        Jxz / Jy,
        ((Jx - Jy) * Jx + Jxz ** 2) / g,
        Jx / g,
    )


@dataclass
class DroneParams:
    """无人机通用参数（param_drone.m 公共部分）。"""

    # ---- 初始条件 Initial conditions ----
    pn_T: float = 0.0     # 初始北向位置 North [m]
    pe_T: float = 0.0     # 初始东向位置 East [m]
    pd_T: float = 0.0     # 初始下向位置 Down（负值表示高度）[m]
    vx_T: float = 0.0     # 机体系 x 速度 [m/s]
    vy_T: float = 0.0
    vz_T: float = 0.0
    phi_T: float = 0.0    # 初始滚转 [rad]
    theta_T: float = 0.0  # 初始俯仰 [rad]
    psi_T: float = 0.0    # 初始偏航 [rad]
    p_T: float = 0.0      # 初始滚转角速度 [rad/s]
    q_T: float = 0.0
    r_T: float = 0.0
    Va_T: float = 0.0     # 初始空速（子类覆盖：point_mass=5, quad=0, wing=35）

    # ---- 风参数 Parameters for the wind ----
    wn: float = 0.0       # 稳态风（NED）
    we: float = 0.0
    wd: float = 0.0
    Lx: float = 200.0     # Dryden 湍流尺度
    Ly: float = 200.0
    Lz: float = 50.0
    sigmax: float = 1.06  # 湍流强度
    sigmay: float = 1.06
    sigmaz: float = 0.7
    ws_ned0: np.ndarray = field(default_factory=lambda: np.array([10.0, 0.0, 0.0]))
    Va0: float = 10.0     # = norm(ws_ned0)；子类覆盖(quad=5, wing=35)

    # ---- 执行器上限 Maximum performance of actuators ----
    de_max: float = math.radians(20.0)   # 升降舵最大偏角
    de_teeth: int = 60
    da_max: float = math.radians(20.0)   # 副翼最大偏角
    da_teeth: int = 60
    dr_max: float = math.radians(25.0)   # 方向舵最大偏角
    dr_teeth: int = 60
    dt_max: float = 1.0                  # 油门（归一化）
    dt_freq: float = 50.0                # 控制频率 Hz
    phi_max: float = math.radians(60.0)  # 最大滚转角（子类可覆盖）
    theta_max: float = math.radians(45.0)  # 最大俯仰角（子类可覆盖）
    freq_autopilot: float = 200.0        # 控制器频率 Hz
    freq_motor: float = 50.0             # 电机更新频率 Hz

    # ---- 规划参数 Planning parameters ----
    k_orbit: float = 5.0     # 环绕控制增益（仅固定翼使用）
    k_path: float = 0.01     # 路径跟踪增益
    chi_inf: float = math.pi / 2.0   # 最大航向角误差
    size_wpt_array: int = 100
    size_waypoint_array: int = 100   # = size_wpt_array
    R_min: float = 0.0     # 派生: Va_T^2 / gravity / tan(phi_max)（工厂计算）
    radius: float = 0.0    # 派生: 1.3 * R_min（工厂计算）


@dataclass
class PointMassParams(DroneParams):
    """质点模型参数（param_drone.m 的 point_mass 分支：仅设 Va_T=5）。"""

    Va_T: float = 5.0


@dataclass
class QuadcopterParams(DroneParams, SensorParamsMixin):
    """四旋翼参数（param_quadcopter.m + param_sensors.m）。"""

    # ---- 物理/惯量 ----
    mass: float = 0.468 / math.sqrt(2.5)   # [kg]
    Jx: float = 0.004856     # [kg*m^2]
    Jy: float = 0.004856
    Jz: float = 0.008801
    Jxz: float = 0.0
    # gamma（派生）
    gamma: float = 0.0
    gamma1: float = 0.0
    gamma2: float = 0.0
    gamma3: float = 0.0
    gamma4: float = 0.0
    gamma5: float = 0.0
    gamma6: float = 0.0
    gamma7: float = 0.0
    gamma8: float = 0.0

    # ---- 气动/螺旋桨 ----
    l_arm: float = 0.225          # [m] 单臂长度
    J_prop: float = 3.357e-5      # [kg*m^2] 单个螺旋桨转动惯量
    k_omega: float = 2000.0 / 2.5  # [rad/s] 每个电机最大角速度
    C_prop: float = 2.98e-6       # 螺旋桨力比例系数
    Ax: float = 0.25              # [kg/s]
    Ay: float = 0.25
    Az: float = 0.25
    drag_area: float = 0.08       # [m^2]
    CD: float = 1.14e-7           # 阻力常数

    # ---- 覆盖基类的初始条件/参考值 ----
    Va_T: float = 0.0
    Va0: float = 5.0              # m/s（参考 DJI Phantom 3: 15m/s）
    altitude_take_off_zone: float = 5.0
    pd_T: float = -200.0          # 初始下向位置（负高度）
    alpha_T: float = 0.0
    beta_T: float = 0.0
    de_T: float = 0.0
    da_T: float = 0.0
    dr_T: float = 0.0
    dt_T: float = 0.0

    # ---- PID 增益 ----
    # 高度推力保持（直接赋值，非派生）
    kp_h: float = 1.5
    kd_h: float = 2.5
    ki_h: float = 0.5
    # 以下均为派生（见 _compute_pid_gains）
    kp_phi: float = 0.0
    kd_phi: float = 0.0
    kp_theta: float = 0.0
    kd_theta: float = 0.0
    kp_psi: float = 0.0
    kd_psi: float = 0.0
    kp_vd: float = 0.0
    ki_vd: float = 0.0
    kp_vn: float = 0.0
    ki_vn: float = 0.0
    kp_ve: float = 0.0
    ki_ve: float = 0.0
    kp_ad: float = 0.0
    ki_ad: float = 0.0
    kp_an: float = 0.0
    ki_an: float = 0.0
    kp_ae: float = 0.0
    ki_ae: float = 0.0
    kp_pd: float = 0.0
    ki_pd: float = 0.0
    kd_pd: float = 0.0
    kp_pe: float = 0.0
    ki_pe: float = 0.0
    kd_pe: float = 0.0
    kp_pn: float = 0.0
    ki_pn: float = 0.0
    kd_pn: float = 0.0

    # ---- 执行器上限（覆盖基类）----
    theta_max: float = math.radians(40.0)
    phi_max: float = math.radians(40.0)
    thrust_max: float = 0.0    # 派生 = k_omega [rad/s]
    torque_max: float = 100.0  # [N/m]
    vd_max: float = 10.0       # [m/s]
    v_max: float = 5.0         # [m/s]

    def __post_init__(self) -> None:
        self._compute_gammas()
        self._compute_pid_gains()
        self.thrust_max = self.k_omega      # p_drone.thrust_max = p_drone.k_omega
        self._init_sensor_matrices()        # 来自 SensorParamsMixin

    def _compute_gammas(self) -> None:
        """对应 param_quadcopter.m 的 'Compute gammas'。"""
        (self.gamma, self.gamma1, self.gamma2, self.gamma3, self.gamma4,
         self.gamma5, self.gamma6, self.gamma7, self.gamma8) = compute_inertia_gammas(
            self.Jx, self.Jy, self.Jz, self.Jxz)

    def _compute_pid_gains(self) -> None:
        """对应 param_quadcopter.m 的 'Tuning PID loops for quadrotor'。"""
        KSI = KSI_OPT
        mass = self.mass

        # Roll torque hold
        omega_n_phi = 4.0
        self.kp_phi = self.Jx * omega_n_phi ** 2
        self.kd_phi = 2 * KSI * omega_n_phi * self.Jx
        # Pitch torque hold
        omega_n_theta = 4.0
        self.kp_theta = self.Jy * omega_n_theta ** 2
        self.kd_theta = 2 * KSI * omega_n_theta * self.Jy
        # Yaw torque hold
        omega_n_psi = 3.0
        self.kp_psi = self.Jz * omega_n_psi ** 2
        self.kd_psi = 2 * KSI * omega_n_psi * self.Jz
        # vd thrust hold
        omega_n_vd = 5.0
        self.kp_vd = -2 * KSI * omega_n_vd * mass
        self.ki_vd = self.Az - omega_n_vd ** 2 * mass
        # vn pitch / ve roll hold（共享 uh）
        uh = self.C_prop * self.k_omega ** 2 * 4 * 0.31 ** 2
        omega_n_vn = 1.0
        self.kp_vn = (-2 * KSI * omega_n_vn * mass + self.Ax) / uh
        self.ki_vn = -(omega_n_vn ** 2 * mass) / uh
        omega_n_ve = 1.0
        self.kp_ve = (2 * KSI * omega_n_ve * mass - self.Ay) / uh
        self.ki_ve = (omega_n_ve ** 2 * mass) / uh
        # ad thrust hold
        tau_ad = 0.1
        self.kp_ad = 0.0
        self.ki_ad = self.Az - mass / tau_ad
        # an pitch hold
        tau_an = 0.2
        self.kp_an = 0.0
        self.ki_an = (tau_an * self.Ax - mass) / (tau_an * uh)
        # ae roll hold
        tau_ae = 0.2
        self.kp_ae = 0.0
        self.ki_ae = (tau_ae * self.Ay + mass) / (tau_ae * uh)
        # pd thrust / pe roll / pn pitch hold
        self.kp_pd = 0.0
        self.ki_pd = mass
        self.kd_pd = 0.0
        self.kp_pe = 0.0
        self.ki_pe = mass
        self.kd_pe = 0.0
        self.kp_pn = 0.0
        self.ki_pn = mass
        self.kd_pn = 0.0


def build_drone_params(drone_type, physics):
    """按 drone_type 构建 p_drone，并计算依赖 gravity 的派生量（R_min/radius）。

    复刻 MATLAB param_drone.m 的装配流程:
        公共字段 → 按类型 run(param_<type>) → 计算 R_min/radius。

    参数:
        drone_type : DroneType 或等价字符串（"point_mass"/"quadcopter"/"fixed_wing"）
        physics    : PhysicsParams（提供 gravity）
    """
    if drone_type == DroneType.POINT_MASS:
        p = PointMassParams()
    elif drone_type == DroneType.QUADCOPTER:
        p = QuadcopterParams()
    elif drone_type == DroneType.FIXED_WING:
        from .fixed_wing import FixedWingParams   # 延迟导入，避免循环依赖
        p = FixedWingParams(physics=physics)
    else:
        raise ValueError(f"未知 drone_type: {drone_type!r}")

    # param_drone.m: R_min = Va_T^2 / gravity / tan(phi_max); radius = 1.3*R_min
    p.R_min = p.Va_T ** 2 / physics.gravity / math.tan(p.phi_max)
    p.radius = 1.3 * p.R_min
    return p
