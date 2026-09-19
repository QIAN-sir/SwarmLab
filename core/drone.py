"""Drone —— 无人机核心状态载体（对应 MATLAB @Drone/Drone.m，故障二次开发版）。

MATLAB ``classdef Drone < handle`` → Python 普通 class（天然引用语义，符合计划 §4.1：
不得退化为 dict / ndarray）。

本文件覆盖 Phase 3 范围:
    ✅ 全部 properties（含二次开发的 drone_state / drift_vel / drift_initialized）
    ✅ 构造函数（7 参，含 drone_state）
    ✅ 状态/历史访问器：get_state / set_pos / set_vel / set_state / init_rand_pos
    ✅ update_state 的 point_mass 三态故障机（Normal / Stalled / Drifting + fad + drift_vel）
    ✅ estimate_states / update_sensor_measurements（use_estimation=False 路径，走 true_states）
    ✅ 电池：update_battery / rpm2power / compute_Qt_hover / update_actuators / get_delta_hover
    ⏳ 真实动力学分支（quadcopter/fixed_wing）：桩化，见 Phase 8/9
    ⏳ plan_path / manage_path / follow_path：桩化，见 Phase 11
    ⏳ plot_state：桩化，见 Phase 12

向量约定（对应计划 §13）:
    vect3(pos_ned/vel_xyz/attitude/rates/drift_vel/forces/moments) → np.ndarray shape (3,)
    command(4) / delta(4) → (4,)；z_hat(22) → (22,)；airdata(6) → (6,)；full_command(19) → (19,)
    history(pos_ned_history/vel_xyz_history) → (n, 3)，每行是一帧（对应 MATLAB 的 [h; v']）
    MATLAB 1-based 索引 → Python 0-based：command(2:4)→command[1:4]，command(1)→command[0]，
    attitude(3)→attitude[2]，vel_xyz_history(end,:)→vel_xyz_history[-1,:]
"""

import math

import numpy as np

from ..enums import DroneType, DroneState
from ..control.true_states import true_states
from scipy.integrate import solve_ivp

from ..physics import (forces_moments_quad, forces_moments_wing, kinematics_ode_fun)
from ..control.autopilot_quad import autopilot_quad


def _jet(n: int) -> np.ndarray:
    """MATLAB ``jet(n)`` 调色板近似（n×3）。

    ⚠️ 颜色仅用于可视化（Phase 12），非数值验证对象。端点与 MATLAB jet 一致
    （首=深蓝(0,0,0.5)，尾=深红(0.5,0,0)），中间为分段线性近似；Phase 12 可用
    matplotlib 的 'jet' 精确校准。
    """
    x = np.arange(n) / max(n - 1, 1)
    r = np.clip(1.5 - np.abs(x - 0.75) * 4, 0, 1)
    g = np.clip(1.5 - np.abs(x - 0.5) * 4, 0, 1)
    b = np.clip(1.5 - np.abs(x - 0.25) * 4, 0, 1)
    return np.stack([r, g, b], axis=1)


class Drone:
    """无人机对象：保存状态与参数，并计算状态更新。

    对应 MATLAB @Drone/Drone.m。每架无人机是 fixed_wing / quadcopter / point_mass 之一，
    并带有 drone_state（Normal / Stalled / Drifting，二次开发核心）。
    """

    def __init__(self, drone_type, drone_state, p_drone, p_battery, p_sim, p_physics, map_params):
        # ---- Parameters ----
        self.drone_type = drone_type          # DroneType（枚举或等价字符串）
        self.drone_state = drone_state        # DroneState（二次开发）
        # autopilot_version：MATLAB switch(drone_type)
        if drone_type == DroneType.QUADCOPTER:
            self.autopilot_version = 2
        elif drone_type == DroneType.FIXED_WING:
            self.autopilot_version = -1
        elif drone_type == DroneType.POINT_MASS:
            self.autopilot_version = 2
        else:
            self.autopilot_version = None
        self.p_drone = p_drone
        self.p_battery = p_battery
        self.p_sim = p_sim
        self.p_physics = p_physics
        self.map = map_params   # MATLAB: self.map  % FIXME: why does the drone need a map?

        # ---- State = [pos_ned; vel_xyz; attitude; rates]（均为 (3,) 列向量语义）----
        self.pos_ned = np.zeros(3)     # pn, pe, pd
        self.vel_xyz = np.zeros(3)     # vx, vy, vz
        self.drift_vel = np.zeros(3)   # 失控漂移前的速度（二次开发）
        self.attitude = np.zeros(3)    # phi, theta, psi
        self.rates = np.zeros(3)       # p, q, r

        # ---- State history（(n,3)）----
        self.pos_ned_history = np.zeros((0, 3))
        self.vel_xyz_history = np.zeros((0, 3))

        # ---- Auxiliary ----
        self.prev_state = None
        self.path_len = 0.0
        self.y = None                  # sensor measurements
        self.z_hat = np.zeros(22)      # estimated extended state
        self.airdata = np.zeros(6)

        # ---- Command ----
        self.command = np.zeros(4)
        self.prev_command = np.zeros(4)
        self.full_command = np.zeros(19)
        self.delta = np.zeros(4)

        # ---- Forces and moments ----
        self.forces = np.zeros(3)
        self.moments = np.zeros(3)
        self.wind = None               # MATLAB 构造函数未初始化 wind（保持空）

        # ---- Plot handles / 外观 ----
        self.body_handle = None
        self.path_handle = None
        self.waypoint_handle = None
        self.vertices = None
        self.faces = None
        self.face_colors = None
        cmap = _jet(16)
        # MATLAB: cmap(floor(rand*16)+1,:)'  → 随机取 jet(16) 一行（颜色仅用于可视化）
        self.color = cmap[int(math.floor(np.random.rand() * 16)), :]
        self.state_handle = None

        # ---- autopilot-quad 状态（MATLAB 构造函数将标量置 0，P_* 增益保持空）----
        self.altitude_state = 0
        self.altitude_state_prev = 0
        self.initialize_integrator = 0
        self.P_h_thrust = None
        self.P_roll_torque = None
        self.P_pitch_torque = None
        self.P_psi_torque = None
        self.P_ad_thrust = None
        self.P_vn_pitch = None
        self.P_ve_roll = None
        self.P_vd_thrust = None
        self.P_ae_roll = None
        self.P_an_pitch = None
        self.P_pd_thrust = None
        self.P_pe_roll = None
        self.P_pn_pitch = None

        # ---- autopilot-wing 状态 ----
        self.lat_state = 0
        self.lat_state_prev = 0
        self.lat_init_integrator = 0
        self.lat_counter_last_change = 0
        self.P_phi_da = None
        self.P_chi_phi = None
        self.P_theta_de = None
        self.P_h_theta = None
        self.P_va_dt = None
        self.P_va_theta = None

        # ---- Battery ----
        self.i = np.zeros(5)
        self.Q = p_battery.Q0
        self.Qt = 0.0                  # MATLAB 构造函数未显式初始化 Qt；置 0 以避免未定义
        self.V = p_battery.V0

        # ---- Path planner/manager ----
        self.path = np.zeros(26)
        self.nb_waypoints = None
        self.waypoints = None
        self.transition_time = None
        self.alpha = 0.0
        self.command_before_transition = None

        # ---- Flags ----
        self.use_estimation = False
        self.drift_initialized = False  # 用于判断是否已记录失控速度（二次开发）

    # ==================================================================
    # 历史累加辅助（对应 MATLAB 的 isempty 判断 + [history; v'] 追加）
    # ==================================================================
    def _append_pos_history(self) -> None:
        if self.pos_ned_history.shape[0] == 0:
            self.pos_ned_history = self.pos_ned.reshape(1, 3)
        else:
            self.pos_ned_history = np.vstack([self.pos_ned_history, self.pos_ned.reshape(1, 3)])

    def _append_vel_history(self) -> None:
        if self.vel_xyz_history.shape[0] == 0:
            self.vel_xyz_history = self.vel_xyz.reshape(1, 3)
        else:
            self.vel_xyz_history = np.vstack([self.vel_xyz_history, self.vel_xyz.reshape(1, 3)])

    # ==================================================================
    # 初始化 / 状态访问器
    # ==================================================================
    def init_rand_pos(self, map_size) -> None:
        """INIT_RAND_POS: 在地图上随机初始化位置。"""
        self.pos_ned = np.asarray(map_size, dtype=float).reshape(3) * np.random.rand(3)
        self.pos_ned[2] = -self.pos_ned[2]   # h = -pd
        self._append_pos_history()

    def set_pos(self, position_ned) -> None:
        """SET_POS: 设置 NED 位置并更新 pos_ned_history 与 z_hat。"""
        self.pos_ned = np.asarray(position_ned, dtype=float).reshape(3)
        self._append_pos_history()
        # MATLAB: self.z_hat = true_states([self.get_state(); self.airdata; NaN]);
        self.z_hat = true_states(np.concatenate([self.get_state(), self.airdata, [np.nan]]))

    def set_vel(self, velocity_xyz) -> None:
        """SET_VEL: 设置速度并更新 vel_xyz_history。"""
        self.vel_xyz = np.asarray(velocity_xyz, dtype=float).reshape(3)
        self._append_vel_history()

    def get_state(self) -> np.ndarray:
        """GET_STATE: 返回 12 维状态 [pos_ned; vel_xyz; attitude; rates]。"""
        return np.concatenate([self.pos_ned, self.vel_xyz, self.attitude, self.rates])

    def set_state(self, pos_ned, vel_xyz, attitude, rates) -> None:
        """SET_STATE: 设置完整状态。

        注：MATLAB 原版此处曾有 `self.set_pos(self, pos_ned)` 双 self 传参 bug；
        二次开发版已修正为 `self.set_pos(pos_ned)`，此处按修正后行为迁移。
        """
        self.set_pos(pos_ned)
        self.set_vel(vel_xyz)
        self.attitude = np.asarray(attitude, dtype=float).reshape(3)
        self.rates = np.asarray(rates, dtype=float).reshape(3)

    def change_param(self, autopilot_version, p_drone) -> None:
        """CHANGE_PARAM: GUI 修改参数时更新。"""
        self.autopilot_version = autopilot_version
        self.p_drone = p_drone

    # ==================================================================
    # 状态更新（核心）
    # ==================================================================
    def update_state(self, wind, time, fad) -> None:
        """UPDATE_STATE: 计算新的无人机状态。

        对应 MATLAB Drone.update_state(self, wind, time, fad)（fad 为二次开发新增）。
        """
        self.update_sensor_measurements()
        self.estimate_states(time)

        if self.drone_type != DroneType.POINT_MASS:
            # ---- 真实动力学分支（quadcopter / fixed_wing）----
            # MATLAB: temp3 = autopilot_quad(self,time) / autopilot_wing(self,0,time)
            #   self.delta = temp3(1:4); self.full_command = temp3(5:end)
            #   compute_dynamics → compute_kinematics → update_battery
            if self.drone_type == DroneType.QUADCOPTER:
                temp = autopilot_quad(self, time)
            else:
                raise NotImplementedError("fixed_wing 自动驾驶仪(autopilot_wing)待迁移")
            self.delta = temp[0:4].copy()
            self.full_command = temp[4:].copy()
            self.compute_dynamics(wind, time)
            self.compute_kinematics(time)
            self.update_battery()
        else:
            # ---- point_mass 故障状态机（二次开发核心）----
            # 先判断是否进入了失控漂移状态
            if self.drone_state != DroneState.DRIFTING:
                self.drift_initialized = False

            if self.drone_state == DroneState.NORMAL:
                self.vel_xyz = self.command[1:4].copy()          # command(2:4)
                self.pos_ned = self.pos_ned + self.vel_xyz * self.p_sim.dt
                self.attitude[2] = self.command[0]               # command(1) → psi（用于绘图）
            elif self.drone_state == DroneState.STALLED:
                self.vel_xyz = np.zeros(3)
                self.pos_ned = self.pos_ned + self.vel_xyz * self.p_sim.dt   # 位置不变
                self.attitude[2] = self.command[0]
            elif self.drone_state == DroneState.DRIFTING:
                # self.vel_xyz = fad * self.command(2:4);  # ← MATLAB 中被注释的备选方案
                # 记录失控漂移前的速度（仅首次进入 Drifting 时）
                if not self.drift_initialized:
                    self.drift_initialized = True
                    self.drift_vel = self.vel_xyz_history[-1, :].copy()   # vel_xyz_history(end,:)'
                # 根据衰减因子不断更新速度和位置
                self.vel_xyz = fad * self.drift_vel
                self.pos_ned = self.pos_ned + self.vel_xyz * self.p_sim.dt
                self.attitude[2] = self.command[0]

        # 更新历史
        self._append_pos_history()
        self._append_vel_history()

    def update_prev_state(self) -> None:
        """UPDATE_PREV_STATE: 用新状态替换旧状态。"""
        self.prev_state = self.get_state()

    def update_path_length(self) -> None:
        """UPDATE_PATH_LENGTH: 累加飞行路径长度。

        MATLAB: pdist([prev_state(1:3)'; pos_ned'], 'euclidean') = ||prev_pos - pos_ned||
        """
        self.path_len = self.path_len + float(
            np.linalg.norm(self.prev_state[0:3] - self.pos_ned))

    # ==================================================================
    # 电池
    # ==================================================================
    def update_battery(self) -> None:
        """UPDATE_BATTERY: 更新电池状态 (i, V, Q)。"""
        param = self.p_battery
        # // TODO drone.update_actuators();
        omega = (self.p_drone.k_omega * self.delta) * 30 / math.pi   # [rpm]
        pow_motors = (self.rpm2power(omega[0]) + self.rpm2power(omega[1])
                      + self.rpm2power(omega[2]) + self.rpm2power(omega[3]))
        pow_tot = param.power_board + pow_motors

        # 用电流新测量值更新 i（circshift(-1) 后覆盖末位）
        self.i = np.roll(self.i, -1)
        self.i[-1] = pow_tot / self.V

        # 电流滑动平均（MATLAB filter(b,a,i) 取末位 = 5 点均值）
        i_filt = float(np.mean(self.i))

        self.Qt = (i_filt * self.p_sim.dt) / 3600
        self.Q = self.Q + self.Qt   # [Ah]

        V_nominal = param.e0 + param.e1 * (self.Q / param.Qf) + param.e2 * (self.Q / param.Qf) ** 2
        V_final = param.A * math.exp(-param.B * (param.Qf - self.Q))
        self.V = V_nominal - param.R * i_filt - V_final

    def rpm2power(self, rpm) -> float:
        """由电机转速(rpm)计算所需功率 [W]。"""
        param = self.p_battery
        return param.p0 * (param.p1 * rpm ** 2 + param.p2 * rpm ** 4
                           + param.p3 * rpm ** 6 + param.p4 * rpm ** 8)

    def compute_Qt_hover(self) -> float:
        """计算悬停一个时间步消耗的电量 [Ah]。"""
        omega = self.get_delta_hover() * 30 / math.pi   # [rpm]
        pow_motors = 4 * self.rpm2power(omega)
        # ⚠️ Potential MATLAB bug（PROJECT_ARCHITECTURE §14 B-7）：原代码用 self.B.pow_board /
        #    self.B.V0，但 Drone 无属性 B（应为 p_battery）。此处忠实保留原行为：一旦被调用
        #    即抛 AttributeError（等价于 MATLAB 的未识别属性错误）。该方法当前不在主链路调用。
        pow_tot = self.B.pow_board + pow_motors
        intensity = pow_tot / self.B.V0
        return (intensity * self.p_sim.dt) / 3600

    def update_actuators(self) -> None:
        """UPDATE_ACTUATORS: 电机动力学模型。

        注：MATLAB 原代码末尾 `self.i = i_dot*dt + i_old` 会把 5 维的 i 赋成 4 维
        （i_dot 为 4 维），属可疑写法；此处忠实保留。该方法当前为死代码（未被调用）。
        """
        b = 0.0        # viscous constant
        L = 0.5e-3     # arm inductance
        R = 0.05       # arm resistance
        K = 4.2e-3     # EMF constant
        omega_old = self.delta * self.p_drone.k_omega
        i_old = self.i[4]   # MATLAB self.i(5)
        omega_dot = (-b * omega_old + K * i_old) / self.p_drone.J_prop
        i_dot = (-K * omega_old - R * i_old + self.V) / L
        self.delta = (omega_dot * self.p_sim.dt + omega_old) / self.p_drone.k_omega
        self.i = i_dot * self.p_sim.dt + i_old

    def get_delta_hover(self) -> float:
        return math.sqrt((self.p_drone.mass * self.p_physics.gravity)
                         / self.p_drone.C_prop) / (4 * self.p_drone.k_omega)

    # ==================================================================
    # 路径规划/管理/跟踪（桩化，Phase 11）
    # ==================================================================
    def plan_path(self, path_type, time) -> None:
        """PLAN_PATH: 生成航点（@Drone/plan_path.m）。见 Phase 11。"""
        raise NotImplementedError("plan_path 依赖 RRT/path planning，见 Phase 11")

    def manage_path(self, time) -> None:
        """MANAGE_PATH: 由航点提取路径（path_manager_wing/quad）。见 Phase 11。"""
        raise NotImplementedError("manage_path 依赖 path_manager_*，见 Phase 11")

    def follow_path(self) -> None:
        """FOLLOW_PATH: 由路径生成指令（path_follower_wing/quad）。见 Phase 11。"""
        raise NotImplementedError("follow_path 依赖 path_follower_*，见 Phase 11")

    # ==================================================================
    # 可视化（桩化，Phase 12）
    # ==================================================================
    def plot_state(self, time, output_rate) -> None:
        """绘制无人机状态曲线（@Drone/plot_state.m）。见 Phase 12。"""
        raise NotImplementedError("plot_state 依赖可视化，见 Phase 12")

    # ==================================================================
    # 私有：动力学 / 运动学 / 传感 / 估计
    # ==================================================================
    def compute_dynamics(self, wind, time) -> None:
        """COMPUTE_DYNAMICS: 由当前状态计算受力/力矩（forces_moments_*）。Phase 8。"""
        wind = np.asarray(wind, dtype=float)
        if self.drone_type == DroneType.QUADCOPTER:
            out = forces_moments_quad(self.get_state(), self.delta, wind,
                                      self.p_drone, self.p_sim, self.p_physics)
        elif self.drone_type == DroneType.FIXED_WING:
            out = forces_moments_wing(self.get_state(), self.delta, wind,
                                      self.p_drone, self.p_physics)
        else:
            raise NotImplementedError("point_mass 无真实动力学")
        self.forces = out[0:3].copy()
        self.moments = out[3:6].copy()
        self.airdata = out[6:12].copy()   # [Va, alpha, beta, wn, we, wd]

    def compute_kinematics(self, time) -> None:
        """COMPUTE_KINEMATICS: 由力/力矩积分出新状态（ode23≈RK23 / kinematics_ode_fun）。Phase 8。"""
        uu = np.concatenate([self.forces, self.moments])
        x0 = self.get_state()
        sol = solve_ivp(lambda t, x: kinematics_ode_fun(t, x, x0, uu, self.p_drone),
                        (0.0, self.p_sim.dt), x0, method="RK23",
                        rtol=1e-8, atol=1e-8)
        xnew = sol.y[:, -1]
        self.pos_ned = xnew[0:3].copy()
        self.vel_xyz = xnew[3:6].copy()
        self.attitude = xnew[6:9].copy()
        self.rates = xnew[9:12].copy()

    def update_sensor_measurements(self) -> None:
        """UPDATE_SENSOR_MEASUREMENTS: 更新传感器测量（仅 use_estimation=True 时）。"""
        if self.use_estimation:
            # MATLAB: y_imu_baro = sensors(...); y_gps = gps(...); self.y = [...]
            raise NotImplementedError("传感器仿真依赖 sensors/gps，见 Phase 10")
        # use_estimation=False：无操作（与 MATLAB 一致）

    def estimate_states(self, time) -> None:
        """估计状态 z_hat（use_estimation=False 时用 true_states）。"""
        if self.use_estimation:
            raise NotImplementedError("EKF 状态估计依赖 estimation，见 Phase 10")
        else:
            # MATLAB: self.z_hat = true_states([self.get_state(); self.airdata; time]);
            self.z_hat = true_states(np.concatenate([self.get_state(), self.airdata, [time]]))
