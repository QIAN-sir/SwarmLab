"""compute_vel_olfati_saber —— Olfati-Saber 群体算法（故障二次开发版）。

对应 MATLAB: @Swarm/compute_vel_olfati_saber.m（511 行）+ 子函数 rho_f/rho_der_f/repulsion_cubic_arena。

输入/输出同 Vasarhelyi：返回 (vel_command (3,N), collisions[3])。

与 Vasarhelyi 的关键差异:
    - Olfati 用 sigma-norm 势函数 phi 做位置匹配 + 速度匹配，**积分加速度**得速度指令
      （vel_command = vel + acc*dt）；Vasarhelyi 直接叠加速度。
    - 内部把 NED 的 z 翻成"上为正"，算完再翻回（line 41/45/416）。
    - 故障邻居：方案1（增强排斥 + 侧向滑移）+ 方案2（is_stuck 主动逃逸）。

迁移要点 / 忠实保留的可疑写法（计划 §2.3、§11、§13）:
    - **B-8**：`if isempty(max_v)` / `isempty(max_a)`（应为 ~isempty）→ Python `is None`；
      因 max_v/max_a 默认为数值（非 None），限幅分支**恒不执行**（死代码），与 MATLAB 一致。
      ⚠️ 这导致 Olfati 不做速度/加速度限幅（与 Vasarhelyi 不同），务必保留。
    - **B-14**：line 152 `acc_vm(:,agent) += c_vm*vel_rel(agent2)` 中 `vel_rel(agent2)` 是
      MATLAB **线性索引**（取 3×N 矩阵的第 agent2 个元素=标量），疑似应为 `vel_rel(:,agent2)`。
      忠实照搬：用列主序展平取值；仅在 `~is_active_migration` 时触发（默认 migration 开→不触发）。
    - `p_swarm.lambda` → `p_swarm.lambda_`（Python 关键字，KI-3）。
    - `mod(agent,2)`（agent 为 MATLAB 1-based）→ Python `(agent+1)%2`。
    - 方案2 的 `if ~rear_blocked` 在 MATLAB 中被注释 → 逃逸恒触发（rear_blocked 仅计算不使用）。
"""

import math

import numpy as np

from .height_constraint import BUILDING_CLEARANCE, limit_downward_vel_z
from ..math_tools.obstacle_distance import uav_cyl_dist3d


# ===========================================================================
# 子函数（对应 MATLAB 同文件末尾的 subfunctions）
# ===========================================================================
def rho_f(z, delta, r, k):
    """邻接系数（分段）：z<delta*r→1；delta*r<=z<r→余弦过渡；z>=r→0。"""
    if z < delta * r:
        return 1.0
    elif z < r:
        return (1 / 2 ** k) * (1 + math.cos(math.pi * (z / r - delta) / (1 - delta))) ** k
    else:
        return 0.0


def rho_der_f(z, delta, r, k):
    """邻接系数的导数。"""
    if z < delta * r:
        return 0.0
    elif z < r:
        arg = math.pi * (z / r - delta) / (1 - delta)
        return -math.pi / (1 - delta) * k / (2 ** k) * (1 + math.cos(arg)) ** (k - 1) * math.sin(arg)
    else:
        return 0.0


def repulsion_cubic_arena(pos_drone, pos_walls, wall_width, const_rep):
    """立方竞技场（墙壁）排斥加速度（对应 MATLAB repulsion_cubic_arena）。"""
    a_arena = np.zeros(3)
    for axis in range(3):
        x_drone = pos_drone[axis]
        # 左墙
        x_arena = pos_walls[axis, 0]
        if x_drone < x_arena:
            a_arena[axis] = a_arena[axis] + const_rep
        elif x_arena < x_drone < x_arena + wall_width:
            a_arena[axis] = a_arena[axis] + const_rep * 0.5 * (
                math.sin((math.pi / wall_width) * (x_drone + x_arena) + math.pi / 2) + 1)
        # 右墙
        x_arena = pos_walls[axis, 1]
        if x_arena - wall_width < x_drone < x_arena:
            a_arena[axis] = a_arena[axis] - const_rep * 0.5 * (
                math.sin((math.pi / wall_width) * (x_drone - x_arena - wall_width) - math.pi / 2) + 1)
        elif x_drone > x_arena:
            a_arena[axis] = a_arena[axis] - const_rep
    return a_arena


# ===========================================================================
# 主函数
# ===========================================================================
def compute_vel_olfati_saber(swarm, p_swarm, r_agent, dt):
    """Olfati-Saber 群体速度指令（含障碍/墙壁避让与故障邻居处理）。"""

    # ---- 有用函数（sigma-norm 势函数，闭包捕获 p_swarm）----
    a, b, c = p_swarm.a, p_swarm.b, p_swarm.c
    d_ref, r = p_swarm.d_ref, p_swarm.r
    delta, k = p_swarm.delta, p_swarm.k

    def psi(z):
        return ((a + b) * (math.sqrt(1 + (z - d_ref + c) ** 2) - math.sqrt(1 + c ** 2))
                + (a - b) * (z - d_ref)) / 2

    def psi_der(z):
        return (a + b) / 2 * (z - d_ref + c) / math.sqrt(1 + (z - d_ref + c) ** 2) + (a - b) / 2

    def phi(z):
        return 1 / r * rho_der_f(z, delta, r, k) * psi(z) + rho_f(z, delta, r, k) * psi_der(z)

    # ---- 获取位置/速度并翻转 z（NED down → up 为正）----
    pos = swarm.get_pos_ned()                       # (3,N)
    pos = np.vstack([pos[0:2, :], -pos[2:3, :]])
    vel = swarm.get_vel_ned()
    vel = np.vstack([vel[0:2, :], -vel[2:3, :]])

    u_ref = np.asarray(p_swarm.u_ref).ravel()       # (3,)

    # ---- 初始化 ----
    nb_agents = swarm.nb_agents
    M = np.zeros((nb_agents, nb_agents))
    D = np.zeros((nb_agents, nb_agents))
    acc_pm = np.zeros((3, nb_agents))
    acc_vm = np.zeros((3, nb_agents))
    acc_wall = np.zeros((3, nb_agents))
    acc_obs = np.zeros((3, nb_agents))
    acc_command = np.zeros((3, nb_agents))
    vel_command = np.zeros((3, nb_agents))

    nb_agent_collisions = 0
    nb_obs_collisions = 0
    min_dist_obs = float("inf")                  # Stage 14: 无接近时为 inf（原魔数20无物理意义）

    # ---- 逐个智能体 ----
    for agent in range(nb_agents):
        # == 查找邻居 ==
        p_rel = pos - pos[:, agent][:, None]
        dist = np.sqrt(np.sum(p_rel ** 2, axis=0))
        D[agent, :] = dist
        neig_lis = np.arange(nb_agents)
        neig_lis = neig_lis[dist != 0]
        nb_agent_collisions = nb_agent_collisions + np.sum(dist < 2 * r_agent) - 1
        nb_neig = nb_agents - 1
        if hasattr(p_swarm, "r"):
            neig_lis = neig_lis[dist[neig_lis] < p_swarm.r]
            nb_neig = len(neig_lis)
        if hasattr(p_swarm, "max_neig"):
            if nb_neig > p_swarm.max_neig:
                order = np.argsort(dist[neig_lis])
                neig_lis = neig_lis[order[: p_swarm.max_neig]]
                nb_neig = p_swarm.max_neig
        M[agent, neig_lis] = 1

        # == 计算各加速度贡献 ==
        if nb_neig != 0:
            vel_rel = vel - vel[:, agent][:, None]          # (3,N)
            with np.errstate(divide="ignore", invalid="ignore"):
                pos_rel_u = p_rel / dist                     # 无负号（与 Vasarhelyi 相反）
            vel_norm_current = np.linalg.norm(vel[:, agent])
            has_fault_neighbor = False
            min_dist_to_fault = math.inf
            fault_rear_count = 0
            fault_front_count = 0

            for agent2 in neig_lis:
                if swarm.drones[agent2].drone_state == "Normal":
                    # 位置匹配（势函数 phi）
                    acc_pm[:, agent] = acc_pm[:, agent] + phi(dist[agent2]) * pos_rel_u[:, agent2] * 2
                    # 速度匹配（仅非迁移模式）
                    if not p_swarm.is_active_migration:
                        # ⚠️ B-14：MATLAB vel_rel(agent2) 为线性索引（标量），忠实照搬
                        vel_rel_flat = vel_rel.ravel(order="F")
                        acc_vm[:, agent] = acc_vm[:, agent] + p_swarm.c_vm * vel_rel_flat[agent2]
                else:
                    # ===== 故障邻居处理（方案1 + 方案2）=====
                    has_fault_neighbor = True
                    min_dist_to_fault = min(min_dist_to_fault, dist[agent2])
                    # 统计故障机方位
                    if p_swarm.is_active_migration:
                        fault_dir = pos_rel_u[:, agent2]
                        cosang = float(np.dot(-fault_dir, u_ref))
                        cosang = max(-1.0, min(1.0, cosang))   # 数值安全（等价 MATLAB acos 定义域）
                        angle_to_migration = math.acos(cosang)
                        if angle_to_migration < math.pi / 3:
                            fault_front_count += 1
                        elif angle_to_migration > 2 * math.pi / 3:
                            fault_rear_count += 1
                    # ----- 方案1：增强排斥 + 侧向滑移 -----
                    r_fault_safe = 3 * d_ref
                    k_fault = 0.5
                    if dist[agent2] < r_fault_safe:
                        d_hard = 6 * r_agent
                        if dist[agent2] < d_hard:
                            emergency = 300 * (d_hard - dist[agent2]) ** 2 / d_hard
                            acc_pm[:, agent] = acc_pm[:, agent] + emergency * (-pos_rel_u[:, agent2])
                        repulsion_mag = k_fault * (r_fault_safe - dist[agent2])
                        lateral = np.cross(np.array([0.0, 0.0, 1.0]), pos_rel_u[:, agent2])
                        if np.linalg.norm(lateral) > 1e-6:
                            lateral = lateral / np.linalg.norm(lateral)
                            if p_swarm.is_active_migration:
                                lateral_sign = np.sign(np.dot(lateral, u_ref))
                                lateral = lateral_sign * lateral
                        else:
                            lateral = np.zeros(3)
                        escape_dir = 0.6 * (-pos_rel_u[:, agent2]) + 0.4 * lateral
                        escape_dir = escape_dir / (np.linalg.norm(escape_dir) + 1e-6)
                        acc_pm[:, agent] = acc_pm[:, agent] + repulsion_mag * escape_dir
                    # 注意：故障邻居不产生速度匹配项（acc_vm 不增加）

            # ===== 方案2：主动逃逸（前后夹击时后退绕行）=====
            is_stuck = (vel_norm_current < 0.3) and has_fault_neighbor and \
                       (min_dist_to_fault < 2 * d_ref)
            if is_stuck and p_swarm.is_active_migration:
                rear_blocked = (fault_rear_count > 0)   # 计算但 MATLAB 中 `if ~rear_blocked` 被注释
                escape_dir = -u_ref.copy()
                lateral_offset = np.cross(np.array([0.0, 0.0, 1.0]), u_ref)
                if (agent + 1) % 2 == 0:                # MATLAB: mod(agent_1based, 2)==0
                    lateral_offset = -lateral_offset
                escape_dir = escape_dir + 5 * lateral_offset
                escape_dir = escape_dir / np.linalg.norm(escape_dir)
                escape_strength = 3.0 * p_swarm.c_vm * p_swarm.v_ref * \
                    (1 + 0.5 * (2 - min_dist_to_fault / d_ref))
                acc_pm[:, agent] = acc_pm[:, agent] + escape_strength * escape_dir

        # == 迁移效应（覆盖 acc_vm；在邻居块之外，对所有 agent 生效）==
        if p_swarm.is_active_migration:
            acc_vm[:, agent] = p_swarm.c_vm * (u_ref * p_swarm.v_ref - vel[:, agent])

        # == 墙壁和障碍物避障 ==
        if p_swarm.is_active_arena:
            acc_wall[:, agent] = repulsion_cubic_arena(pos[:, agent], p_swarm.x_arena,
                                                       p_swarm.d_arena, p_swarm.c_arena)

        if p_swarm.is_active_spheres:
            for obs in range(p_swarm.n_spheres):
                c_obs = p_swarm.spheres[0:3, obs]
                r_obs = p_swarm.spheres[3, obs]
                pos_agent = pos[:, agent]
                vel_agent = vel[:, agent]
                dist_ab = np.sqrt(np.sum((pos[:, agent] - c_obs) ** 2)) - r_obs
                nb_obs_collisions = nb_obs_collisions + int(dist_ab < r_agent)
                if dist_ab < min_dist_obs:
                    min_dist_obs = dist_ab
                if dist_ab < p_swarm.r0:
                    s = r_obs / (r_obs + dist_ab)
                    pos_obs = s * pos_agent + (1 - s) * c_obs
                    s_dot = r_obs * (np.dot(vel_agent, (pos_obs - pos_agent)) / dist_ab) / (r_obs + dist_ab) ** 2
                    vel_obs = s * vel_agent - r_obs * (s_dot / s) * (pos_obs - pos_agent) / dist_ab
                    pos_gamma = c_obs + p_swarm.lambda_ * u_ref
                    d_ag = np.linalg.norm(pos_gamma - pos[:, agent])
                    acc_obs[:, agent] = acc_obs[:, agent] + p_swarm.c_pm_obs * \
                        rho_f(dist_ab / p_swarm.r0, delta, r, k) * \
                        (phi(dist_ab - d_ref) * (pos_obs - pos_agent) / dist_ab
                         + phi(d_ag - d_ref) * (pos_gamma - pos_agent) / np.linalg.norm(pos_gamma - pos[:, agent])) \
                        + p_swarm.c_vm_obs * (vel_obs - vel_agent)

        if p_swarm.is_active_cyl:
            h_uav = pos[2, agent]                       # UAV 高度（olfati 内部 z 已 flip 向上正）
            cyl_h = getattr(p_swarm, "cyl_heights", None)
            h_aware = getattr(p_swarm, "height_aware_cyl", True)
            for obs in range(p_swarm.n_cyl):
                # Stage 13 逐建筑物高度感知：UAV 明显高于该建筑顶部 → 不构成障碍，跳过绕行
                if (h_aware and cyl_h is not None and obs < len(cyl_h)
                        and h_uav > cyl_h[obs] + BUILDING_CLEARANCE):
                    continue
                c_obs = p_swarm.cylinders[0:2, obs]
                r_obs = p_swarm.cylinders[2, obs]
                pos_agent = pos[0:2, agent]
                vel_agent = vel[0:2, agent]
                dist_ab = np.sqrt(np.sum((pos[0:2, agent] - c_obs) ** 2)) - r_obs
                # Stage 14: 碰撞统计/最小距离用三维表面距离（高空飞越不误判）；避障排斥仍用 dist_ab
                h_top = float(cyl_h[obs]) if (cyl_h is not None and obs < len(cyl_h)) else float("inf")
                pos_ned_agent = np.array([pos[0, agent], pos[1, agent], -pos[2, agent]])
                dist3d = uav_cyl_dist3d(pos_ned_agent, c_obs, r_obs, h_top)
                nb_obs_collisions = nb_obs_collisions + int(dist3d < r_agent)
                if dist3d < min_dist_obs:
                    min_dist_obs = dist3d
                if dist_ab < p_swarm.r0:
                    s = r_obs / (r_obs + dist_ab)
                    pos_obs = s * pos_agent + (1 - s) * c_obs
                    s_dot = r_obs * (np.dot(vel_agent, (pos_obs - pos_agent)) / dist_ab) / (r_obs + dist_ab) ** 2
                    vel_obs = s * vel_agent - r_obs * (s_dot / s) * (pos_obs - pos_agent) / dist_ab
                    pos_gamma = c_obs + p_swarm.lambda_ * u_ref[0:2]
                    d_ag = np.linalg.norm(pos_gamma - pos[0:2, agent])
                    acc_obs[0:2, agent] = acc_obs[0:2, agent] + p_swarm.c_pm_obs * \
                        rho_f(dist_ab / p_swarm.r0, delta, r, k) * \
                        (phi(dist_ab - d_ref) * (pos_obs - pos_agent) / dist_ab
                         + phi(d_ag - d_ref) * (pos_gamma - pos_agent) / np.linalg.norm(pos_gamma - pos[0:2, agent])) \
                        + p_swarm.c_vm_obs * (vel_obs - vel_agent)

        # == 汇总加速度并积分得速度指令 ==
        acc_command[:, agent] = acc_pm[:, agent] + acc_vm[:, agent] + acc_wall[:, agent] + acc_obs[:, agent]
        vel_command[:, agent] = vel[:, agent] + acc_command[:, agent] * dt

    # ---- 碰撞统计与限幅 ----
    nb_agent_collisions = nb_agent_collisions / 2
    collisions = np.array([nb_agent_collisions, nb_obs_collisions, min_dist_obs])

    if p_swarm.c_r is not None:
        vel_command = vel_command + p_swarm.c_r * np.random.randn(3, nb_agents)

    # ⚠️ B-8：MATLAB 为 `if isempty(max_v)`（应为 ~isempty）→ 恒不执行（死代码），忠实保留
    if p_swarm.max_v is None:
        v_norm = np.sqrt(np.sum(vel_command ** 2, axis=0))
        idx_to_bound = v_norm > p_swarm.max_v
        if np.sum(idx_to_bound) > 0:
            vel_command[:, idx_to_bound] = p_swarm.max_v * vel_command[:, idx_to_bound] / v_norm[idx_to_bound]

    # ⚠️ B-8：同上，`if isempty(max_a)` 恒不执行
    if p_swarm.max_a is None:
        accel_cmd = (vel_command - vel) / dt
        accel_cmd_norm = np.sqrt(np.sum(accel_cmd ** 2, axis=0))
        idx_to_bound = (accel_cmd_norm > p_swarm.max_a) | (accel_cmd_norm < -p_swarm.max_a)
        if np.sum(idx_to_bound) > 0:
            vel_command[:, idx_to_bound] = vel[:, idx_to_bound] + \
                dt * p_swarm.max_a * accel_cmd[:, idx_to_bound] / accel_cmd_norm[idx_to_bound]

    # ---- 翻回 NED（z 取反）----
    vel_command = np.vstack([vel_command[0:2, :], -vel_command[2:3, :]])

    # Stage 13：高空 UAV 禁止向下进入建筑物高度区域（约束可行方向空间，非指定固定方向）；
    # 低空 UAV 不约束（原有低空避障基准）。仅 clip 向下分量，水平/向上仍由算法自行计算。
    # （olfati 内部 pos z 向上正 → NED pd = -pos[2]）
    for agent in range(nb_agents):
        pos_ned = np.array([pos[0, agent], pos[1, agent], -pos[2, agent]])
        vel_command[2, agent] = limit_downward_vel_z(pos_ned, vel_command[2, agent], p_swarm)

    return vel_command, collisions
