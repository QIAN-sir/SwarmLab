"""Swarm —— 无人机群管理对象（对应 MATLAB @Swarm/Swarm.m，故障二次开发版）。

MATLAB ``classdef Swarm < handle`` → Python 普通 class（引用语义）。
Swarm 持有 Drone 列表，负责：群体状态读写、故障状态注入(set_dronestate)、
速度指令分派(update_command)、状态传播(update_state)、虚拟质心(get_barycenter,仅健康机)。

本文件覆盖 Phase 4 范围:
    ✅ 全部 properties + 构造函数
    ✅ add_drone / init_rand_pos / set_pos / get_pos_ned / get_vel_ned / set_vel_commands
    ✅ set_dronestate（按 loss_radio 随机注入 Stalled/Drifting，二次开发核心）
    ✅ update_state(wind, time, fad)（逐机传播到 Drone 的 point_mass 故障机）
    ✅ get_barycenter（仅统计 Normal 健康机）/ getters / history
    ⏳ update_command 的算法调用（compute_vel_olfati_saber / vasarhelyi）→ 桩，见 Phase 6/7
    ⏳ plot_state / record_video → 桩，见 Phase 12

⚠️ 忠实保留的 MATLAB 潜在 bug（只记录不修，计划 §2.3；详见 PROJECT_ARCHITECTURE §14）:
    B-1 add_n_drones 以 1 参调用 add_drone（需 7 参）
    B-2 set_vel / set_state 使用未定义属性 self.nb_drones（应为 nb_agents）
    B-3 path_manager_individual 调用 drone.path_manager_wing（Drone 无此方法，应为 manage_path）
    B-4 path_follower_individual 调用 follow_path(time)（Drone.follow_path 无参）
    B-5 get_barycenter 未在开头清零 normal_drone（多次调用会累加）
    B-6 draw_agents_energy / record_state 在 MATLAB 中声明但无对应 .m 文件
"""

import math

import numpy as np

from ..enums import DroneState, SwarmAlgorithm
from ..math_tools import Rb2i
from ..swarm.vasarhelyi import compute_vel_vasarhelyi as _compute_vel_vasarhelyi
from ..swarm.olfati_saber import compute_vel_olfati_saber as _compute_vel_olfati_saber
from .drone import Drone


class Swarm:
    """无人机群（对应 MATLAB @Swarm/Swarm.m）。"""

    def __init__(self):
        self.drones = []                 # Drone 对象列表（MATLAB: 列向量）
        self.nb_agents = 0               # 无人机数量
        self.equivalent_drone = None     # 虚拟质心无人机（path planner 用）
        self.algorithm = None            # SwarmAlgorithm（外部赋值：swarm.algorithm = ...）

        self.swarm_state = []            # (:,1) DroneState：各无人机状态（二次开发）
        self.normal_drone = 0            # 健康无人机计数
        self.t_fault = 0.0               # 性能退化故障发生时间
        self.fad = 0.0                   # 性能退化衰退因子

        self.collisions_history = np.zeros((0, 3))   # 碰撞历史（MATLAB: []）

    # ==================================================================
    # 增删无人机
    # ==================================================================
    def add_drone(self, drone_type, drone_state, p_drone, p_battery, p_sim, p_physics, map_params):
        """添加一架无人机（7 参，含 drone_state）。"""
        self.nb_agents += 1
        drone = Drone(drone_type, drone_state, p_drone, p_battery, p_sim, p_physics, map_params)
        self.drones.append(drone)
        # 注：swarm_state 始终追加 Normal（与传入的 drone_state 无关），忠实照搬 MATLAB
        if self.nb_agents == 1:
            self.swarm_state = [DroneState.NORMAL]
        else:
            self.swarm_state.append(DroneState.NORMAL)

    def add_n_drones(self, drone, n):
        """添加 n 架无人机。

        ⚠️ B-1（忠实保留）：MATLAB 以 ``self.add_drone(drone)`` 单参调用，而 add_drone 需 7 参，
        故本方法一旦被调用即报错（与 MATLAB 行为一致）。
        """
        for _ in range(n):
            self.add_drone(drone)

    # ==================================================================
    # 初始化 / 状态读写
    # ==================================================================
    def init_rand_pos(self, map_size):
        for i in range(self.nb_agents):
            self.drones[i].init_rand_pos(map_size)

    def set_pos(self, pos):
        """设置各无人机位置；pos 为 3×N。"""
        for i in range(self.nb_agents):
            self.drones[i].set_pos(pos[:, i])

    def set_vel(self, vel):
        """设置各无人机速度；vel 为 3×N。

        ⚠️ B-2（忠实保留）：MATLAB 用 ``self.nb_drones``（未定义属性，应为 nb_agents），
        故本方法被调用即报 AttributeError（与 MATLAB 一致）。
        """
        for i in range(self.nb_drones):   # noqa: B-2 未定义属性（照搬 MATLAB）
            self.drones[i].set_vel(vel[:, i])

    def get_pos_ned(self):
        """返回 3×N 位置矩阵（各列为一个无人机的 pos_ned）。"""
        Pos_ned = np.zeros((3, self.nb_agents))
        for i in range(self.nb_agents):
            Pos_ned[:, i] = self.drones[i].pos_ned
        return Pos_ned

    def get_vel_ned(self):
        """返回 3×N 速度矩阵（NED），由机体速度经 Rb2i 旋转得到。"""
        Vel_ned = np.zeros((3, self.nb_agents))
        for i in range(self.nb_agents):
            drone = self.drones[i]
            phi = drone.attitude[0]
            theta = drone.attitude[1]
            psi = drone.attitude[2]
            Rbi = Rb2i(phi, theta, psi)
            Vel_ned[:, i] = Rbi @ drone.vel_xyz
        return Vel_ned

    def set_dronestate(self, DRONE_STATE, DRONE_STATE_CHANGED, loss_radio):
        """设置集群各无人机状态（故障注入，二次开发核心）。

        对应 MATLAB Swarm.set_dronestate：DRONE_STATE_CHANGED∈{1,2} 时按 loss_radio
        随机选取 ceil(loss_radio*nb_agents) 架置为 Stalled/Drifting，并同步到各 Drone。
        """
        if DRONE_STATE_CHANGED == 1 or DRONE_STATE_CHANGED == 2:
            # 先全部重置为 Normal
            self.swarm_state = [DroneState.NORMAL] * self.nb_agents
            if DRONE_STATE == "Normal":
                self.swarm_state = [DroneState.NORMAL] * self.nb_agents
            elif DRONE_STATE == "Stalled":
                k = math.ceil(loss_radio * self.nb_agents)     # 按比例随机选择
                idx = np.random.permutation(self.nb_agents)[:k]
                for j in idx:
                    self.swarm_state[j] = DroneState.STALLED
            else:  # Drifting
                k = math.ceil(loss_radio * self.nb_agents)
                idx = np.random.permutation(self.nb_agents)[:k]
                for j in idx:
                    self.swarm_state[j] = DroneState.DRIFTING
            # 同步到每架无人机
            for i in range(self.nb_agents):
                self.drones[i].drone_state = self.swarm_state[i]

    def set_state(self, state):
        """由 6N 向量设置位置/速度。

        ⚠️ B-2（忠实保留）：MATLAB 用 ``self.nb_drones``（未定义），调用即报错。
        """
        mask = np.tile(np.array([True, True, True, False, False, False]), self.nb_drones)
        Pos_ned = np.asarray(state)[mask].reshape(3, -1, order='F')
        Vel_xyz = np.asarray(state)[~mask].reshape(3, -1, order='F')
        self.set_pos(Pos_ned)
        self.set_vel(Vel_xyz)

    def get_state(self):
        """返回 6N 列向量 [Pos_ned; Vel_ned] 按列展平（MATLAB state(:)）。"""
        Pos_ned = self.get_pos_ned()
        Vel_ned = self.get_vel_ned()
        state = np.vstack([Pos_ned, Vel_ned])
        return state.ravel(order='F')

    def get_path_len(self):
        return np.array([self.drones[i].path_len for i in range(self.nb_agents)])

    def set_vel_commands(self, commands):
        """把 3×N 速度指令写入各无人机 command（command(1)=0, command(2:4)=vel）。"""
        for i in range(self.nb_agents):
            drone = self.drones[i]
            drone.command[0] = 0.0
            drone.command[1:4] = commands[:, i]

    def get_Qt(self):
        return np.array([self.drones[i].Qt for i in range(self.nb_agents)])

    def get_Q(self):
        return np.array([self.drones[i].Q for i in range(self.nb_agents)])

    def get_colors(self):
        colors = np.zeros((3, self.nb_agents))
        for i in range(self.nb_agents):
            colors[:, i] = self.drones[i].color
        return colors

    # ==================================================================
    # 状态传播 / 指令计算
    # ==================================================================
    def update_state(self, wind, time, fad):
        """逐机调用 Drone.update_state（含 fad，二次开发）。"""
        for i in range(self.nb_agents):
            self.drones[i].update_state(wind, time, fad)

    def update_command(self, p_swarm, r_coll, dt):
        """选择群体算法计算速度指令，累加碰撞历史，并写入各无人机 command。

        ⏳ compute_vel_vasarhelyi / compute_vel_olfati_saber 见 Phase 6/7（当前桩）。
        """
        if self.algorithm == "vasarhelyi":
            vel_commands, collisions = self.compute_vel_vasarhelyi(p_swarm, r_coll, dt)
        elif self.algorithm == "olfati_saber":
            vel_commands, collisions = self.compute_vel_olfati_saber(p_swarm, r_coll, dt)
        # 注：MATLAB 中若 algorithm 二者皆非，vel_commands/collisions 未定义 → 后续报错；此处一致

        if self.collisions_history.shape[0] == 0:
            self.collisions_history = np.asarray(collisions, dtype=float).reshape(1, -1)
        else:
            self.collisions_history = np.vstack(
                [self.collisions_history, np.asarray(collisions, dtype=float).reshape(1, -1)])

        self.set_vel_commands(vel_commands)
        return vel_commands, collisions

    # ---- 群体算法（外部方法，Phase 6/7 迁移）----
    def compute_vel_vasarhelyi(self, p_swarm, r_agent, dt):
        """Vasarhelyi 群体算法（@Swarm/compute_vel_vasarhelyi.m，Phase 6）。

        委托到 SwarmLab_py.swarm.vasarhelyi（对应 MATLAB @Swarm 外部方法，首参为 self）。
        """
        return _compute_vel_vasarhelyi(self, p_swarm, r_agent, dt)

    def compute_vel_olfati_saber(self, p_swarm, r_agent, dt):
        """Olfati-Saber 群体算法（@Swarm/compute_vel_olfati_saber.m，Phase 7）。

        委托到 SwarmLab_py.swarm.olfati_saber（对应 MATLAB @Swarm 外部方法，首参为 self）。
        """
        return _compute_vel_olfati_saber(self, p_swarm, r_agent, dt)

    # ==================================================================
    # 路径规划（群体/个体）
    # ==================================================================
    def path_planner_swarm(self, path_type, time):
        """为群体创建虚拟质心无人机并规划路径。"""
        self.equivalent_drone = self.get_barycenter()
        self.equivalent_drone.plan_path(path_type, time)

    def get_barycenter(self):
        """计算健康(Normal)无人机的质心，返回一个等效 Drone。

        ⚠️ B-5（忠实保留）：MATLAB 未在函数开头把 normal_drone 清零，多次调用会累加，
        导致第 2 次起质心计算偏小/除零风险。此处照搬（不清零）。
        """
        pos = np.zeros(3)
        vel = np.zeros(3)
        for i in range(self.nb_agents):
            if self.swarm_state[i] == "Normal":     # 只统计健康无人机
                pos = pos + self.drones[i].pos_ned
                vel = vel + self.drones[i].vel_xyz
                self.normal_drone = self.normal_drone + 1   # B-5：不清零
        pos = pos / self.normal_drone
        vel = vel / self.normal_drone
        d0 = self.drones[0]
        equivalent_drone = Drone(d0.drone_type, d0.drone_state, d0.p_drone,
                                 d0.p_battery, d0.p_sim, d0.p_physics, d0.map)
        equivalent_drone.set_pos(pos)
        equivalent_drone.set_vel(vel)
        return equivalent_drone

    def path_planner_individual(self, path_type, time):
        for i in range(self.nb_agents):
            self.drones[i].plan_path(path_type, time)

    def path_manager_individual(self, time):
        """⚠️ B-3（忠实保留）：MATLAB 调用 drone.path_manager_wing（无此方法）→ 报错。"""
        for i in range(self.nb_agents):
            self.drones[i].path_manager_wing(time)

    def path_follower_individual(self, time):
        """⚠️ B-4（忠实保留）：MATLAB 调用 follow_path(time)，而 Drone.follow_path 无参 → 报错。"""
        for i in range(self.nb_agents):
            self.drones[i].follow_path(time)

    # ==================================================================
    # 历史
    # ==================================================================
    def get_pos_ned_history(self):
        """横向拼接各无人机 pos_ned_history → (T, 3N)。"""
        return np.hstack([self.drones[i].pos_ned_history for i in range(self.nb_agents)])

    def get_vel_xyz_history(self):
        """横向拼接各无人机 vel_xyz_history → (T, 3N)。"""
        return np.hstack([self.drones[i].vel_xyz_history for i in range(self.nb_agents)])

    # ==================================================================
    # 可视化 / 记录（桩）
    # ==================================================================
    def plot_state(self, time, end_time, iter_, period, collisions, r_agent):
        """⏳ Phase 12：@Swarm/plot_state.m"""
        raise NotImplementedError("Swarm.plot_state 依赖可视化，见 Phase 12")

    def record_video(self, time, T, period, fig_handle, path):
        """⏳ Phase 12：@Swarm/record_video.m"""
        raise NotImplementedError("Swarm.record_video 依赖视频/可视化，见 Phase 12")

    def draw_agents_energy(self, time, period, fig_handle, axes_lim):
        """⚠️ B-6：MATLAB 声明此方法但无对应 .m 文件（死声明）。"""
        raise NotImplementedError("draw_agents_energy 在 MATLAB 中缺实现文件（B-6）")

    def record_state(self, time, T, period, is_active_cyl, collisions, map_params, dirname):
        """⚠️ B-6：MATLAB 声明此方法但无对应 .m 文件（死声明）。"""
        raise NotImplementedError("record_state 在 MATLAB 中缺实现文件（B-6）")
