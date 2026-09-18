"""FixedWingParams —— 固定翼无人机参数。

对应 MATLAB:
    parameters/param_drone/param_fixed_wing.m  (+ run('param_sensors'))
    control/control_drone/Td2idx.m, compute_tf_model.m, param_table_T.m, table_T.mat

设计说明（table_T 处理 = 方案 b）:
    MATLAB 用 load('table_T.mat') + Td2idx + compute_tf_model 求配平状态并整定 PID。
    但对固定配平点 Td=[35, 0, Inf]，Td2idx 恒得索引 (1,1,1)，故全程只用到
    xyu_T = table_T(:,1,1,1) 这一个 19 维向量。此处用 scipy 一次性提取并以完整精度
    烘焙为常量 XYU_TRIM（见下），从而：无需在项目内携带二进制 .mat、无运行时
    scipy.io 依赖、数值透明可测。compute_tf_model 只用到其中的 ``a`` 结构（纯代数），
    其 tf() 传递函数（需 Control System Toolbox）在参数整定中未被使用，故不迁移。

    提取方式（可复现）:
        import scipy.io as sio
        T = sio.loadmat('.../control/control_drone/table_T.mat')['table_T']  # shape (19,1,1,2)
        xyu_T = T[:, 0, 0, 0]
"""

import math
from dataclasses import dataclass, InitVar

from .drone import DroneParams, KSI_OPT, compute_inertia_gammas
from .physics import PhysicsParams
from .sensors import SensorParamsMixin

# 配平状态向量 xyu_T = table_T(:,1,1,1)（完整精度，来源见模块 docstring）
# 顺序: [pn,pe,pd, vx,vy,vz, phi,theta,psi, p,q,r, Va,alpha,beta, de,da,dr,dt]
XYU_TRIM = (
    -1.2077591660658938e-22,   # 1  pn
    -1.3676831938330363e-21,   # 2  pe
    3.190986910446642e-23,     # 3  pd
    34.999795359147754,        # 4  vx
    -7.424102416830082e-26,    # 5  vy
    0.11968633079522861,       # 6  vz
    -2.4921351778094228e-21,   # 7  phi
    0.003419616115991571,      # 8  theta
    -3.488903661930597e-21,    # 9  psi
    1.2445046239518967e-22,    # 10 p
    -1.8639747421508803e-22,   # 11 q
    -3.4558943610290826e-22,   # 12 r
    35.0,                      # 13 Va
    0.003419616115991571,      # 14 alpha
    -2.121172119094309e-27,    # 15 beta
    -0.0493589082481536,       # 16 de
    -1.0884151086778801e-22,   # 17 da
    3.5592053701942707e-22,    # 18 dr
    0.4555434337688184,        # 19 dt
)


@dataclass
class FixedWingParams(DroneParams, SensorParamsMixin):
    """固定翼参数（Aerosonde UAV）+ 配平整定的 PID 增益 + 传感器参数。"""

    # 用于计算依赖 gravity/rho 的配平增益；InitVar 不作为字段存储
    physics: InitVar[object] = None

    # ---- 惯量（Aerosonde）----
    mass: float = 13.5
    Jx: float = 0.8244
    Jy: float = 1.135
    Jz: float = 1.759
    Jxz: float = 0.1204
    gamma: float = 0.0
    gamma1: float = 0.0
    gamma2: float = 0.0
    gamma3: float = 0.0
    gamma4: float = 0.0
    gamma5: float = 0.0
    gamma6: float = 0.0
    gamma7: float = 0.0
    gamma8: float = 0.0

    # ---- 气动几何/螺旋桨 ----
    S_wing: float = 0.55      # 机翼面积
    b: float = 2.8956         # 翼展
    c: float = 0.18994        # 平均气动弦长
    S_prop: float = 0.2027    # 螺旋桨面积
    k_motor: float = 80.0     # 电机常数
    k_TP: float = 0.0
    k_omega: float = 0.0
    e: float = 0.9            # Oswald 效率因子

    # ---- 纵向气动系数 ----
    CL0: float = 0.28
    CL_alpha: float = 3.45
    CL_q: float = 0.0
    CL_de: float = -0.36
    CD0: float = 0.03
    CD_alpha: float = 0.30
    CD_p: float = 0.0437
    CD_q: float = 0.0
    CD_de: float = 0.0
    Cm0: float = -0.02338
    Cm_alpha: float = -0.38
    Cm_q: float = -3.6
    Cm_de: float = -0.5

    # ---- 横向气动系数 ----
    CY0: float = 0.0
    CY_beta: float = -0.98
    CY_p: float = 0.0
    CY_r: float = 0.0
    CY_da: float = 0.0
    CY_dr: float = -0.17
    Cl0: float = 0.0
    Cl_beta: float = -0.12
    Cl_p: float = -0.26
    Cl_r: float = 0.14
    Cl_da: float = 0.08
    Cl_dr: float = 0.105
    Cn0: float = 0.0
    Cn_beta: float = 0.25
    Cn_p: float = 0.022
    Cn_r: float = -0.35
    Cn_da: float = 0.06
    Cn_dr: float = -0.032

    # ---- 螺旋桨/其它 ----
    C_prop: float = 1.0       # fx = S_prop*C_prop*delta_t
    M: float = 50.0
    epsilon: float = 0.1592
    alpha0: float = 0.4712

    # ---- Cp/Cr（派生，见 __post_init__）----
    Cp0: float = 0.0
    Cp_beta: float = 0.0
    Cp_p: float = 0.0
    Cp_r: float = 0.0
    Cp_da: float = 0.0
    Cp_dr: float = 0.0
    Cr0: float = 0.0
    Cr_b: float = 0.0
    Cr_p: float = 0.0
    Cr_r: float = 0.0
    Cr_da: float = 0.0
    Cr_dr: float = 0.0

    # ---- 参考值/状态机区域 ----
    Va0: float = 35.0                  # m/s (~85 mph)，覆盖基类
    altitude_take_off_zone: float = 40.0
    h_theta_hold_zone: float = 10.0
    tau_phi: float = 0.05              # [s] <=> 1 autopilot loop
    tau_theta: float = 0.1             # [s] <=> 10 autopilot loops

    # ---- 配平初始条件（由 XYU_TRIM 设置；alpha_T..dt_T 为固定翼专有）----
    alpha_T: float = 0.0
    beta_T: float = 0.0
    de_T: float = 0.0
    da_T: float = 0.0
    dr_T: float = 0.0
    dt_T: float = 0.0

    # ---- PID 增益（派生，见 __post_init__）----
    kp_phi: float = 0.0
    kd_phi: float = 0.0
    kp_chi: float = 0.0
    ki_chi: float = 0.0
    kp_theta: float = 0.0
    kd_theta: float = 0.0
    kp_h: float = 0.0
    ki_h: float = 0.0
    kp_va_dt: float = 0.0
    ki_va_dt: float = 0.0
    kp_va_theta: float = 0.0
    ki_va_theta: float = 0.0

    # ---- 匿名函数 p_drone.kb/kc（MATLAB: @(Va) 0.5*b/Va）----
    def kb(self, Va: float) -> float:
        return 0.5 * self.b / Va

    def kc(self, Va: float) -> float:
        return 0.5 * self.c / Va

    def __post_init__(self, physics) -> None:
        if physics is None:
            physics = PhysicsParams()
        rho = physics.rho
        gravity = physics.gravity

        # 1) gammas（与 quad 共用公式）
        (self.gamma, self.gamma1, self.gamma2, self.gamma3, self.gamma4,
         self.gamma5, self.gamma6, self.gamma7, self.gamma8) = compute_inertia_gammas(
            self.Jx, self.Jy, self.Jz, self.Jxz)

        # 2) Cp/Cr（param_fixed_wing lines 95-106）
        g3, g4, g8 = self.gamma3, self.gamma4, self.gamma8
        self.Cp0 = g3 * self.Cl0 + g4 * self.Cn0
        self.Cp_beta = g3 * self.Cl_beta + g4 * self.Cn_beta
        self.Cp_p = g3 * self.Cl_p + g4 * self.Cn_p
        self.Cp_r = g3 * self.Cl_r + g4 * self.Cn_r
        self.Cp_da = g3 * self.Cl_da + g4 * self.Cn_da
        self.Cp_dr = g3 * self.Cl_dr + g4 * self.Cn_dr
        self.Cr0 = g4 * self.Cl0 + g8 * self.Cn0
        self.Cr_b = g4 * self.Cl_beta + g8 * self.Cn_beta
        self.Cr_p = g4 * self.Cl_p + g8 * self.Cn_p
        self.Cr_r = g4 * self.Cl_r + g8 * self.Cn_r
        self.Cr_da = g4 * self.Cl_da + g8 * self.Cn_da
        self.Cr_dr = g4 * self.Cl_dr + g8 * self.Cn_dr

        # 3) 配平初始条件（param_fixed_wing lines 220-240；pd_T = xyu(3)-200）
        t = XYU_TRIM
        self.pn_T, self.pe_T, self.pd_T = t[0], t[1], t[2] - 200.0
        self.vx_T, self.vy_T, self.vz_T = t[3], t[4], t[5]
        self.phi_T, self.theta_T, self.psi_T = t[6], t[7], t[8]
        self.p_T, self.q_T, self.r_T = t[9], t[10], t[11]
        self.Va_T, self.alpha_T, self.beta_T = t[12], t[13], t[14]
        self.de_T, self.da_T, self.dr_T, self.dt_T = t[15], t[16], t[17], t[18]

        # 4) compute_tf_model 的 a 结构（仅需 a；tf() 未用于整定，不迁移）
        theta = t[7]
        de = t[15]
        dt = t[18]
        Va = t[12]
        alpha = t[13]
        p_dyn = 0.5 * rho * Va ** 2
        kb = self.kb(Va)
        kc = self.kc(Va)
        a_phi1 = -p_dyn * self.S_wing * self.b * self.Cp_p * kb
        a_phi2 = p_dyn * self.S_wing * self.b * self.Cp_da
        a_theta1 = -p_dyn * self.c * self.S_wing / self.Jy * self.Cm_q * kc
        a_theta2 = -p_dyn * self.c * self.S_wing / self.Jy * self.Cm_alpha
        a_theta3 = p_dyn * self.c * self.S_wing / self.Jy * self.Cm_de
        a_va1 = (rho * Va * self.S_wing / self.mass
                 * (self.CD0 + self.CD_alpha * alpha + self.CD_de * de)
                 + rho * self.S_prop / self.mass * self.C_prop * Va)
        a_va2 = rho * self.S_prop / self.mass * self.C_prop * self.k_motor ** 2 * dt

        # 5) PID 整定（param_fixed_wing lines 164-217）
        KSI = KSI_OPT
        Va_T = self.Va_T
        # Roll_aileron-hold
        ephi_max = math.radians(15)
        self.kp_phi = self.da_max / ephi_max
        wn_phi = math.sqrt(self.kp_phi * a_phi2)
        self.kd_phi = (2 * KSI * wn_phi - a_phi1) / a_phi2
        # Course_roll-hold
        W_chi_phi = 10.0
        wn_chi = wn_phi / W_chi_phi
        self.kp_chi = 2 * KSI * wn_chi * Va_T / gravity
        self.ki_chi = wn_chi ** 2 * Va_T / gravity
        # Pitch_elevator-hold
        etheta_max = math.radians(10)
        sgn_theta3 = (a_theta3 > 0) - (a_theta3 < 0)   # MATLAB sign()
        self.kp_theta = self.de_max / etheta_max * sgn_theta3
        wn_theta = math.sqrt(a_theta2 + self.kp_theta * a_theta3)
        if math.isfinite(a_theta3):
            self.kd_theta = (2 * KSI * wn_theta - a_theta1) / a_theta3
        else:
            self.kd_theta = 0.0
        # Altitude_pitch-hold
        K_theta_DC = self.kp_theta * a_theta3 / (a_theta2 + self.kp_theta * a_theta3)
        W_h_theta = 10.0
        wn_h = wn_theta / W_h_theta
        ksi_h = 1.0
        self.kp_h = 2 * ksi_h * wn_h / (K_theta_DC * Va_T)
        self.ki_h = wn_h ** 2 / (K_theta_DC * Va_T)
        # Airspeed_thrust-hold
        wn_va_dt = 10.0
        ksi_va_dt = 1.0
        self.kp_va_dt = (2 * ksi_va_dt * wn_va_dt - a_va1) / a_va2
        self.ki_va_dt = wn_va_dt ** 2 / a_va2
        # Airspeed_pitch-hold
        W_va_theta = 7.0
        wn_va_theta = wn_theta / W_va_theta
        ksi_va_theta = KSI
        self.kp_va_theta = (a_va1 - 2 * ksi_va_theta * wn_va_theta) / (K_theta_DC * gravity)
        self.ki_va_theta = -wn_va_theta ** 2 / (K_theta_DC * gravity)

        # 6) 传感器矩阵（来自 SensorParamsMixin）
        self._init_sensor_matrices()
