"""compute_vel_vasarhelyi —— Vasarhelyi 群体算法（故障二次开发版）。

对应 MATLAB: @Swarm/compute_vel_vasarhelyi.m（424 行）。
Ref: Vasarhelyi, Science Robotics, 2018（+ 本项目新增的凝聚项与故障邻居处理）。

输入:
    swarm  : Swarm 实例（对应 MATLAB self）
    p_swarm: 群体参数（VasarhelyiParams）
    r_agent: 智能体安全半径
    dt     : 时间步长
输出:
    vel_command : (3, N) 各智能体指令速度
    collisions  : [nb_agent_collisions, nb_obs_collisions, min_dist_obs]

迁移要点（对应计划 §11/§13/§24）:
    - MATLAB 1-based → Python 0-based；sum(x,1)→np.sum(x,axis=0)；列向量 (3,1) → ravel 成 (3,)。
    - `isfield(p_swarm,'c_r')` 在 MATLAB 中恒为假（未定义）→ Python 用 `p_swarm.c_r is not None`。
    - `~isempty(p_swarm.max_v/max_a)` → `is not None`。
    - 故障机(Stalled/Drifting)：vel_command 先置 [0,0,0]，随后仍叠加 migration/goal/自推进项
      （忠实复刻 MATLAB 顺序；该项最终在 Drone.update_state 中被故障运动学覆盖）。
    - p_rel_u / v_rel_u 的自身列会出现 0/0=nan，但自身不在 neig_list 中、且等速邻居的
      v_rel_norm=0 不满足 `> v_fric_max`，故 nan 永不进入结果（与 MATLAB 行为一致）。
"""

import numpy as np

from .height_constraint import BUILDING_CLEARANCE, limit_downward_vel_z
from ..math_tools.obstacle_distance import uav_cyl_dist3d


def get_v_max(v_fric, r, a, p):
    """制动曲线允许的最大相对速度（对应 MATLAB 同文件末尾的 get_v_max）。

    if r < 0:                     v_fricmax = 0
    elif r*p > 0 and r*p < a/p:   v_fricmax = r*p
    else:                         v_fricmax = sqrt(2*a*r - a^2/p^2)
    最后保证 v_fricmax >= v_fric。
    """
    if r < 0:
        v_fricmax = 0.0
    elif r * p > 0 and r * p < a / p:
        v_fricmax = r * p
    else:
        # 2*a*r - a^2/p^2 可能为负（如 r=0）→ MATLAB sqrt 得复数/NaN；此处用 np.sqrt 得 nan
        with np.errstate(invalid="ignore"):
            v_fricmax = float(np.sqrt(2 * a * r - a ** 2 / p ** 2))
    if v_fricmax < v_fric:   # nan < v_fric 为 False（与 MATLAB 一致）
        v_fricmax = v_fric
    return v_fricmax


def compute_vel_vasarhelyi(swarm, p_swarm, r_agent, dt):
    """Vasarhelyi 群体速度指令（含障碍/墙壁避让与故障邻居处理）。"""

    # ---- Initialize variables ----
    pos = swarm.get_pos_ned()          # (3, N)
    vel = swarm.get_vel_ned()          # (3, N)

    nb_agents = swarm.nb_agents
    M = np.zeros((nb_agents, nb_agents))       # 邻接矩阵
    dist_mat = np.zeros((nb_agents, nb_agents))  # 距离矩阵
    vel_rep = np.zeros((3, nb_agents))         # 排斥/吸引速度
    vel_fric = np.zeros((3, nb_agents))        # 速度对齐项
    vel_wall = np.zeros((3, nb_agents))        # 场地排斥速度
    vel_obs = np.zeros((3, nb_agents))         # 障碍物排斥速度
    vel_command = np.zeros((3, nb_agents))     # 总指令速度

    nb_agent_collisions = 0
    nb_obs_collisions = 0
    min_dist_obs = float("inf")                  # Stage 14: 无接近时为 inf（原魔数20无物理意义）

    u_ref = np.asarray(p_swarm.u_ref).ravel()  # (3,) 迁移方向（列向量 ravel）

    # ---- Compute velocity commands ----
    for agent in range(nb_agents):
        # （MATLAB line 45 的 `if drone_state=="Normal"` 整体被注释 → 所有 agent 都计算）

        # == Find neighbors ==
        p_rel = pos - pos[:, agent][:, None]           # (3,N) 相对位置
        dist = np.sqrt(np.sum(p_rel ** 2, axis=0))     # (N,) 欧氏距离
        dist_mat[agent, :] = dist

        neig_list = np.arange(nb_agents)
        neig_list = neig_list[dist != 0]               # 排除自身（及同位置者）

        # 碰撞统计：距离 < 2*r_agent（含自身，故 -1）
        nb_agent_collisions = nb_agent_collisions + np.sum(dist < 2 * r_agent) - 1

        nb_neig = nb_agents - 1

        # 欧氏距离邻域约束（isfield(p_swarm,'r') 恒真）
        if hasattr(p_swarm, "r"):
            neig_list = neig_list[dist[neig_list] < p_swarm.r]
            nb_neig = len(neig_list)

        # 拓扑距离邻域约束（最多 max_neig 个最近邻）
        if hasattr(p_swarm, "max_neig"):
            if nb_neig > p_swarm.max_neig:
                order = np.argsort(dist[neig_list])
                neig_list = neig_list[order[: p_swarm.max_neig]]
                nb_neig = p_swarm.max_neig

        M[agent, neig_list] = 1                        # 邻接矩阵（有限视野下可能非对称）

        # == Compute different contributions ==
        if nb_neig != 0:
            v_rel = vel - vel[:, agent][:, None]       # (3,N) 速度差
            v_rel_norm = np.sqrt(np.sum(v_rel ** 2, axis=0))   # (N,)
            with np.errstate(divide="ignore", invalid="ignore"):
                p_rel_u = -p_rel / dist                # 排斥方向单位向量（自身列 nan，未用）
                v_rel_u = -v_rel / v_rel_norm          # 速度对齐方向单位向量

            for agent2 in neig_list:
                if swarm.drones[agent2].drone_state == "Normal":
                    # ---- 正常邻居：排斥/吸引 + 速度对齐 ----
                    if dist[agent2] < p_swarm.r0_rep:   # repulsion 排斥
                        vel_rep[:, agent] = vel_rep[:, agent] + \
                            p_swarm.p_rep * (p_swarm.r0_rep - dist[agent2]) * p_rel_u[:, agent2]
                    else:                               # attraction 吸引（凝聚项）
                        vel_rep[:, agent] = vel_rep[:, agent] + \
                            p_swarm.p_rep * (dist[agent2] - p_swarm.r0_rep) * -p_rel_u[:, agent2]

                    v_fric_max = get_v_max(p_swarm.v_fric, dist[agent2] - p_swarm.r0_fric,
                                           p_swarm.a_fric, p_swarm.p_fric)
                    if v_rel_norm[agent2] > v_fric_max:
                        vel_fric[:, agent] = vel_fric[:, agent] + \
                            p_swarm.C_fric * (v_rel_norm[agent2] - v_fric_max) * v_rel_u[:, agent2]
                else:
                    # ---- 故障邻居：视为危险静止障碍物（方案：增强排斥 + 硬安全区）----
                    p_fault = 3 * p_swarm.p_rep         # 显著增强排斥系数
                    r_safe = 3 * p_swarm.r0_rep         # 扩大排斥作用范围
                    if dist[agent2] < r_safe:
                        vel_rep[:, agent] = vel_rep[:, agent] + \
                            p_fault * (r_safe - dist[agent2]) * p_rel_u[:, agent2]

                    d_hard = 6 * r_agent                # 硬安全区（> 2*r_agent 碰撞判定）
                    if dist[agent2] < d_hard:
                        emergency = 300 * p_fault * (d_hard - dist[agent2]) ** 2 / d_hard
                        vel_rep[:, agent] = vel_rep[:, agent] + emergency * p_rel_u[:, agent2]

                    v_fric_max = get_v_max(p_swarm.v_fric, dist[agent2] - p_swarm.r0_fric,
                                           p_swarm.a_fric, p_swarm.p_fric)
                    if v_rel_norm[agent2] > v_fric_max:
                        vel_fric[:, agent] = vel_fric[:, agent] + \
                            p_swarm.C_fric * (v_rel_norm[agent2] - v_fric_max) * v_rel_u[:, agent2]

        # == Wall and obstacle avoidance ==
        # 竞技场（墙壁）排斥
        if p_swarm.is_active_arena:
            unit = np.eye(3)
            for axis in range(3):
                for dir_ in range(2):
                    dist_ab = abs(pos[axis, agent] - p_swarm.x_arena[axis, dir_])
                    v_wall_virtual = unit[:, axis] * p_swarm.v_shill
                    if dir_ == 1:                       # MATLAB dir==2 → 第二侧取反
                        v_wall_virtual = -v_wall_virtual
                    vel_ab = np.sqrt(np.sum((vel[:, agent] - v_wall_virtual) ** 2))
                    v_wall_max = get_v_max(0, dist_ab - p_swarm.r0_shill,
                                           p_swarm.a_shill, p_swarm.p_shill)
                    if vel_ab > v_wall_max:
                        vel_wall[:, agent] = vel_wall[:, agent] + \
                            (vel_ab - v_wall_max) * (v_wall_virtual - vel[:, agent]) / vel_ab

        # 球形障碍物
        if p_swarm.is_active_spheres:
            for obs in range(p_swarm.n_spheres):
                c_obs = p_swarm.spheres[0:3, obs]
                r_obs = p_swarm.spheres[3, obs]
                dist_ab = np.sqrt(np.sum((pos[:, agent] - c_obs) ** 2)) - r_obs
                nb_obs_collisions = nb_obs_collisions + int(dist_ab < r_agent)
                v_obs_virtual = (pos[:, agent] - c_obs) / (dist_ab + r_obs) * p_swarm.v_shill
                vel_ab = np.sqrt(np.sum((vel[:, agent] - v_obs_virtual) ** 2))
                if dist_ab < min_dist_obs:
                    min_dist_obs = dist_ab
                v_obs_max = get_v_max(0, dist_ab - p_swarm.r0_shill,
                                      p_swarm.a_shill, p_swarm.p_shill)
                if vel_ab > v_obs_max:
                    vel_obs[:, agent] = vel_obs[:, agent] + \
                        (vel_ab - v_obs_max) * (v_obs_virtual - vel[:, agent]) / vel_ab

        # 圆柱形障碍物（仅 x-y 平面）
        if p_swarm.is_active_cyl:
            h_uav = -pos[2, agent]                      # UAV 高度（NED down 正）
            cyl_h = getattr(p_swarm, "cyl_heights", None)
            h_aware = getattr(p_swarm, "height_aware_cyl", True)
            for obs in range(p_swarm.n_cyl):
                # Stage 13 逐建筑物高度感知：UAV 明显高于该建筑顶部 → 不构成障碍，跳过绕行
                if (h_aware and cyl_h is not None and obs < len(cyl_h)
                        and h_uav > cyl_h[obs] + BUILDING_CLEARANCE):
                    continue
                c_obs = p_swarm.cylinders[0:2, obs]
                r_obs = p_swarm.cylinders[2, obs]
                dist_ab = np.sqrt(np.sum((pos[0:2, agent] - c_obs) ** 2)) - r_obs
                # Stage 14: 碰撞统计/最小距离用三维表面距离（高空飞越不误判）；避障排斥仍用 dist_ab
                h_top = float(cyl_h[obs]) if (cyl_h is not None and obs < len(cyl_h)) else float("inf")
                dist3d = uav_cyl_dist3d(pos[:, agent], c_obs, r_obs, h_top)
                nb_obs_collisions = nb_obs_collisions + int(dist3d < r_agent)
                v_obs_virtual = (pos[0:2, agent] - c_obs) / (dist_ab + r_obs) * p_swarm.v_shill
                vel_ab = np.sqrt(np.sum((vel[0:2, agent] - v_obs_virtual) ** 2))
                if dist3d < min_dist_obs:
                    min_dist_obs = dist3d
                v_obs_max = get_v_max(0, dist_ab - p_swarm.r0_shill,
                                      p_swarm.a_shill, p_swarm.p_shill)
                if vel_ab > v_obs_max:
                    vel_obs[0:2, agent] = vel_obs[0:2, agent] + \
                        (vel_ab - v_obs_max) * (v_obs_virtual - vel[0:2, agent]) / vel_ab

        # == Sum agent-agent and obstacle contributions ==
        if swarm.drones[agent].drone_state == "Normal":
            vel_command[:, agent] = (vel_rep[:, agent] + vel_fric[:, agent]
                                     + vel_obs[:, agent] + vel_wall[:, agent])
        else:
            vel_command[:, agent] = np.zeros(3)         # 故障机指令置零

        # == Add self propulsion OR migration term（对所有 agent，含故障机）==
        v_norm = np.sqrt(np.sum(vel[:, agent] ** 2))
        if p_swarm.is_active_migration:                 # migration 迁移
            vel_command[:, agent] = vel_command[:, agent] + p_swarm.v_ref * u_ref
        elif p_swarm.is_active_goal:                    # goal 目标导向
            x_goal = np.asarray(p_swarm.x_goal).ravel()
            x_goal_rel = x_goal - pos[:, agent]
            u_goal = x_goal_rel / np.linalg.norm(x_goal_rel)
            vel_command[:, agent] = vel_command[:, agent] + p_swarm.v_ref * u_goal
        else:                                           # self-propulsion 自推进
            if v_norm > 0:
                vel_command[:, agent] = vel_command[:, agent] + p_swarm.v_ref * vel[:, agent] / v_norm

    # ---- Compute collisions and bound velocities and accelerations ----
    nb_agent_collisions = nb_agent_collisions / 2       # 每对碰撞被计两次 → /2
    collisions = np.array([nb_agent_collisions, nb_obs_collisions, min_dist_obs])

    # 随机速度扰动（MATLAB isfield(c_r) 恒假 → c_r 为 None 时跳过）
    if p_swarm.c_r is not None:
        vel_command = vel_command + p_swarm.c_r * np.random.randn(3, nb_agents)

    # 速度限幅（~isempty(max_v)）
    if p_swarm.max_v is not None:
        vel_cmd_norm = np.sqrt(np.sum(vel_command ** 2, axis=0))
        idx_to_bound = vel_cmd_norm > p_swarm.max_v
        if np.sum(idx_to_bound) > 0:
            vel_command[:, idx_to_bound] = p_swarm.max_v * \
                vel_command[:, idx_to_bound] / vel_cmd_norm[idx_to_bound]

    # 加速度限幅（~isempty(max_a)）
    if p_swarm.max_a is not None:
        accel_cmd = (vel_command - vel) / dt
        accel_cmd_norm = np.sqrt(np.sum(accel_cmd ** 2, axis=0))
        # 注：`| accel_cmd_norm < -max_a` 恒为 False（范数非负），忠实保留
        idx_to_bound = (accel_cmd_norm > p_swarm.max_a) | (accel_cmd_norm < -p_swarm.max_a)
        if np.sum(idx_to_bound) > 0:
            vel_command[:, idx_to_bound] = vel[:, idx_to_bound] + \
                dt * p_swarm.max_a * accel_cmd[:, idx_to_bound] / accel_cmd_norm[idx_to_bound]

    # Stage 13：高空 UAV 禁止向下进入建筑物高度区域（约束可行方向空间，非指定固定方向）；
    # 低空 UAV 不约束（原有低空避障基准）。仅 clip 向下分量，水平/向上仍由算法自行计算。
    for agent in range(nb_agents):
        vel_command[2, agent] = limit_downward_vel_z(pos[:, agent], vel_command[2, agent], p_swarm)

    return vel_command, collisions
