# SwarmLab 项目架构分析（Phase 0）

> 本文件为 `SwarmLab_MATLAB_to_Python_Migration_Plan.md` 中 **Phase 0（项目分析）** 的交付物。
> Phase 0 全程 **未修改任何 MATLAB 源代码**，仅做只读分析。

---

## 0. 分析范围与方法

### 0.1 分析对象（权威版本）

| 项 | 内容 |
|---|---|
| **二次开发版（分析主体）** | `大创文件——单群避障版/swarmlab-master`（用户提供的 zip，已解压至 `phase0_fault/swarmlab-master`） |
| **原版基线（对照）** | `大创车辆路径规划/swarmlab-master.zip`（EPFL 原版，已解压至 `phase0_matlab/swarmlab-master`） |
| **一致性验证** | 用户 zip 与工作区 `单群避障版` 文件夹 **103 个 `.m` 文件逐文件 MD5 全一致**（IDENTICAL=103, DIFFERENT=0） |
| **代码规模** | 103 个 `.m` 文件；其中相对原版 **1 个新增、24 个修改、78 个未改动** |

### 0.2 关键结论（先读）

1. **本项目 = EPFL SwarmLab 原版 + "无人机群成员失效/故障"二次开发**。二次开发高度集中，未改动 control/estimation/physics 等底层模块（78 个文件与原版逐字节相同）。
2. **唯一被维护、可运行的入口是群体故障仿真**：`example_olfati_saber.m` 与 `example_vasarhelyi.m`（`point_mass` + 群体算法 + 故障注入）。
3. **单机示例与 tests 已与故障版类接口脱节**（构造函数/`update_state` 参数数量不匹配，运行会报错）——详见 §16。
4. 迁移核心难点集中在：`drone_state` 故障状态机、`fad` 衰减、Olfati-Saber/Vasarhelyi 的故障邻居处理、`persistent` 状态、以及大量 `vectN` 打包向量的索引。

---

## 1. 项目总览

### 1.1 定位

- **原始项目**：EPFL LIS 实验室开源无人机/无人机群仿真框架 SwarmLab（https://github.com/lis-epfl/swarmlab）。
- **当前项目**：在其基础上加入 **无人机群成员失效/故障状态机制**（正常 / 动力锁死 / 失控漂移），并研究健康无人机对故障邻居的避让、逃逸与群体迁移行为。
- **仿真主体**：`point_mass`（质点）模型 + 群体速度指令算法；也保留了原版的四旋翼/固定翼真实动力学链路（当前未随二次开发维护）。

### 1.2 分层架构

```text
                         Example / GUI (.mlapp)
                                  │
                                  ▼
                    Parameter Setup (param_* 脚本)
                                  │
                                  ▼
                              Swarm (handle)
                    ┌─────────────┴──────────────┐
                    │                            │
              Drone[] (handle)            Swarm Algorithm
              (含 drone_state)          ┌────────┴────────┐
                    │                   │                 │
                    │            Olfati-Saber        Vasarhelyi
                    │            (故障邻居/逃逸)     (故障邻居/迁移)
                    │                   └────────┬────────┘
                    │                            ▼
                    │                    Velocity Command
                    └────────────────────────────┤
                                                 ▼
                                   Drone.update_state(wind,time,fad)
                          ┌──────────────────────┴───────────────────────┐
                          │                                              │
                     point_mass                                   Real Dynamics
              ┌───────────┼───────────┐                    ┌─────────────┴────────────┐
              │           │           │                    │                          │
           Normal      Stalled     Drifting           Quadcopter                 Fixed Wing
        (正常积分)   (速度置零)  (drift_vel*fad)    autopilot_quad           autopilot_wing
                                                          │                          │
                                                   forces_moments_*          forces_moments_wing
                                                          │                          │
                                                   kinematics_ode_fun (ode23)         │
                                                          └─────────────┬──────────────┘
                                                                        ▼
                                                          History (pos/vel _history)
                                                                        │
                                                          ┌─────────────┴─────────────┐
                                                          ▼                           ▼
                                                  Visualization                 Analysis
                                        (SwarmViewer/plot_state)      (compute_swarm_performance)
```

---

## 2. 模块与文件清单

### 2.1 目录结构（模块职责）

| 目录 | 职责 | 是否被二次开发改动 |
|---|---|---|
| `@Drone/` | `Drone` 类（状态载体）+ `plan_path` + `plot_state` | ✅ 改动（Drone.m, plot_state.m） |
| `@Swarm/` | `Swarm` 类（群管理）+ 两大算法 + `plot_state`/`record_video` | ✅ 改动（Swarm.m, 两算法, plot_state.m, record_video.m） |
| `enumerators/` | 5 个枚举类 | ✅ 新增 DroneState.m |
| `parameters/` | 参数脚本（`param_*`） | ✅ 改动（swarm/vasarhelyi/drone/map） |
| `examples/` | 主入口脚本 + GUI(.mlapp) | ✅ 改动（两个 swarm 示例, main_GUI） |
| `graphics/` | 可视化（在线/离线 viewer, 绘图） | ✅ 改动（SwarmViewer 等多个） |
| `math_tools/` | 数学/图论/性能工具 | ✅ 改动（compute_swarm_performance） |
| `guidance/` | 路径规划/管理/跟踪 | ⚠️ 少量改动（path_manager_quad, path_follower_quad） |
| `control/` | 自动驾驶仪（quad/wing, PID, mixer） | ❌ 未改动（原版） |
| `estimation/` | 状态估计（EKF, LP filter） | ❌ 未改动（原版） |
| `physics/` | 动力学/传感器/风（forces, kinematics, sensors, gps, wind） | ❌ 未改动（原版） |
| `turbulent_wind_generator/` | 湍流风场生成 | ⚠️ generate_one_bts 微改 |
| `tests/` | 测试/对比脚本 | ⚠️ example_co_mn 改动（但接口已过时） |
| `docs/` | 文档与图片 | — |

### 2.2 相对原版的改动清单（哈希 diff 权威结果）

**新增（1）**
- `enumerators/DroneState.m` —— 故障状态枚举 `Normal / Stalled / Drifting`

**修改（24）** —— 按重要性分组：

| 组 | 文件 | 原版→故障版行数 |
|---|---|---|
| 核心类 | `@Drone/Drone.m` | 536→567 |
| 核心类 | `@Swarm/Swarm.m` | 251→310 |
| 算法 | `@Swarm/compute_vel_olfati_saber.m` | 270→371 |
| 算法 | `@Swarm/compute_vel_vasarhelyi.m` | 236→310 |
| 主循环 | `examples/examples_swarm/example_olfati_saber.m` | 173→216 |
| 主循环 | `examples/examples_swarm/example_vasarhelyi.m` | 175→220 |
| 主循环 | `examples/main_GUI.m` | 3→3 |
| 参数 | `parameters/param_swarm/param_swarm.m` | 117→122 |
| 参数 | `parameters/param_swarm/param_vasarhelyi.m` | 29→26 |
| 参数 | `parameters/param_drone/param_drone.m` | 122→102 |
| 参数 | `parameters/param_map.m` | 30→37 |
| 可视化 | `@Swarm/plot_state.m` | 255→183 |
| 可视化 | `graphics/graphics_swarm/SwarmViewer.m` | 215→306 |
| 可视化 | `graphics/graphics_swarm/SwarmViewerOffline.m` | 191→215 |
| 可视化 | `graphics/graphics_swarm/plot_swarm.m` | 131→149 |
| 可视化 | `graphics/graphics_swarm/plot_swarm_performance.m` | 30→77 |
| 可视化 | `graphics/graphics_swarm/plot_state_offline.m` | 57→72 |
| 可视化 | `graphics/graphics_swarm/plot_trajectories_offline.m` | 35→54 |
| 可视化 | `graphics/VideoWriterWithRate.m` | 46→59 |
| 制导 | `guidance/path_following/path_follower_quad.m` | 74→50 |
| 制导 | `guidance/path_manager/path_manager_quad.m` | 167→176 |
| 工具 | `math_tools/compute_swarm_performance.m` | 140→151 |
| 测试 | `tests/example_co_mn.m` | 130→170 |
| 风场 | `turbulent_wind_generator/generate_one_bts.m` | 40→42 |

**未改动（78）**：`control/*`、`estimation/*`、`physics/*`（除个别）、`math_tools/*`（多数）、`guidance/path_planner` 与多数 `path_manager`、`examples/examples_drone/*`、`turbulent_wind_generator/*`（多数）等——即原版 SwarmLab 逻辑。

> ⚠️ 迁移含义：**故障/群体行为逻辑 = 二次开发（需重点对照迁移）**；**真实动力学/控制/估计 = 原版（可参照上游 SwarmLab 迁移经验）**。

---

## 3. classdef 全清单与继承关系

**共 11 个 `classdef`**：5 个枚举类 + 6 个 handle 类。

### 3.1 枚举类（`enumeration`，无父类）

| 类 | 值 | 用途 | 备注 |
|---|---|---|---|
| `DroneType` | `quadcopter, fixed_wing, point_mass` | 无人机类型 | 原版 |
| `DroneState` | `Normal, Stalled, Drifting` | **故障状态** | 🆕 二次开发新增 |
| `PathType` | `fillet, dubins, straight_rrt` | 路径类型 | 原版；⚠️ 与 `plan_path` 的 switch 分支不一致（§16） |
| `SwarmAlgorithm` | `vasarhelyi, olfati_saber` | 群体算法 | 原版 |
| `SwarmViewerType` | `drone, agent, agent_with_energy` | 可视化模式 | 原版 |

### 3.2 handle 类（`< handle`，引用语义）

| 类 | 文件 | 职责 |
|---|---|---|
| `Drone` | `@Drone/Drone.m` | 核心状态载体：状态/参数/传感器/估计/自动驾驶/动力学/路径/电池/历史/**故障状态** |
| `Swarm` | `@Swarm/Swarm.m` | 群管理：持有 `Drone[]`、算法选择、速度指令、碰撞统计、**故障状态注入/传播** |
| `DroneViewer` | `graphics/graphics_drone/DroneViewer.m` | 单机 3D 可视化 |
| `SwarmViewer` | `graphics/graphics_swarm/SwarmViewer.m` | 群体在线可视化（含故障状态着色/标记） |
| `SwarmViewerOffline` | `graphics/graphics_swarm/SwarmViewerOffline.m` | 群体离线回放 |
| `VideoWriterWithRate` | `graphics/VideoWriterWithRate.m` | 按帧率录制视频 |

> **继承关系**：所有 6 个 handle 类都直接继承 MATLAB `handle`，**无自定义类间继承**（不存在子类化）。枚举类为独立 `enumeration`。
> **Python 映射**：`handle` 引用语义 → 普通 Python class（天然引用语义）；`enumeration` → `enum.Enum`。

### 3.3 组合关系（Composition / 持有）

```text
Swarm (handle)
 ├── drones            : Drone[] 列向量（handle 对象数组）
 ├── equivalent_drone  : Drone（get_barycenter 生成的"虚拟质心无人机"）
 ├── swarm_state       : DroneState 列向量（:,1），与 drones 一一对应
 ├── algorithm         : SwarmAlgorithm（枚举标量）
 └── collisions_history: 矩阵（逐步累加）

Drone (handle)
 ├── drone_type        : DroneType（枚举）
 ├── drone_state       : DroneState（枚举）🆕
 ├── p_drone/p_battery/p_sim/p_physics : 参数 struct
 ├── map               : 地图 struct
 ├── pos_ned/vel_xyz/drift_vel/attitude/rates : vect3 列向量
 ├── command/prev_command : vect4；full_command : vect19；z_hat : vect22
 ├── pos_ned_history/vel_xyz_history : n×3 矩阵（逐行累加）
 └── body_handle/path_handle/... : 图形句柄

SwarmViewer / SwarmViewerOffline / DroneViewer
 └── 持有 figure/scatter 句柄；update(time, swarm, map) 读取 Swarm/Drone 状态
```

> **注意**：`Drone[]` 是 handle 对象数组，`Swarm.drones(i)` 返回引用；对其属性修改会影响原对象。Python 迁移须保持引用语义（不要退化为 dict / ndarray）。

---

## 4. 核心调用链

### 4.1 主仿真循环（以 `example_olfati_saber.m` 为准）

```text
run(param_sim/battery/physics/drone/map/swarm)   % 参数装配（有顺序依赖）
      │
swarm = Swarm(); swarm.algorithm = "olfati_saber"
      │
for i=1:nb_agents:  swarm.add_drone(DRONE_TYPE, DRONE_STATE, p_drone,...,map)
      │
swarm.set_pos(p_swarm.Pos0)
      │
swarm_viewer = SwarmViewer(...); video = VideoWriterWithRate(...)
      │
┌─────▼──────────── for time = start_time : dt : end_time ─────────────────┐
│  if exist('app'):  从 GUI 读取 orientation/loss_radio/drone_state         │
│      ├─ 若 drone_state 变化: set_dronestate(DRONE_STATE, CHANGED, loss_radio) │
│      └─ Drifting 时记录 swarm.t_fault = time                              │
│  [~,collisions] = swarm.update_command(p_swarm, r_coll, dt)  % 算速度指令 │
│      └─ compute_vel_olfati_saber / compute_vel_vasarhelyi                │
│      └─ set_vel_commands(vel_commands) → 写入每个 drone.command(2:4)     │
│  计算 swarm.fad（Drifting: 1/(1+k*(t-t_fault)), 截断到[0,1], <0.1→0）    │
│  swarm.update_state(wind, time, swarm.fad)                  % 更新状态   │
│      └─ 每个 drone.update_state(wind,time,fad)                           │
│  if DEBUG: swarm.plot_state(...)                                          │
│  if VIDEO: swarm_viewer.update(time,swarm,map); video.update(...)        │
└──────────────────────────────────────────────────────────────────────────┘
      │
离线分析：SwarmViewerOffline + get_pos/vel_history + compute_swarm_performance + plot
```

> `wind` 在群体示例中恒为 `zeros(6,1)`（未调用 `get_wind`）。`get_wind`/湍流风场仅用于单机 `example_controller`。

### 4.2 `Swarm.update_command()` → 算法

```text
update_command(p_swarm, r_coll, dt)
   ├── algorithm=="vasarhelyi"    → compute_vel_vasarhelyi(...)
   ├── algorithm=="olfati_saber"  → compute_vel_olfati_saber(...)
   ├── collisions_history 累加
   └── set_vel_commands(vel_commands)  % drone.command(1)=0; command(2:4)=vel
```

### 4.3 `Drone.update_state(wind, time, fad)`

```text
update_state(wind, time, fad)
   ├── update_sensor_measurements()   % 仅 use_estimation=true 时有效
   ├── estimate_states(time)          % use_estimation ? EKF : true_states()
   │
   ├── [drone_type ~= point_mass]  真实动力学：
   │      ├── autopilot_wing(self,0,time)  或  autopilot_quad(self,time)
   │      ├── delta=temp3(1:4); full_command=temp3(5:end)
   │      ├── compute_dynamics(wind,time)  → forces_moments_quad/wing
   │      ├── compute_kinematics(time)     → ode23(kinematics_ode_fun)
   │      └── update_battery()
   │
   └── [drone_type == point_mass]  故障状态机（二次开发核心）：
          ├── if drone_state ~= Drifting:  drift_initialized=false
          ├── Normal   : vel_xyz=command(2:4); pos+=vel*dt; attitude(3)=command(1)
          ├── Stalled  : vel_xyz=[0;0;0];      pos+=vel*dt; attitude(3)=command(1)
          └── Drifting : 首次记录 drift_vel=vel_xyz_history(end,:)';
                         vel_xyz = fad * drift_vel;  pos+=vel*dt; attitude(3)=command(1)
          （之后统一累加 pos_ned_history / vel_xyz_history）
```

---

## 5. 参数系统

### 5.1 参数文件与装配顺序（有依赖！）

群体示例的装配顺序：`param_sim → param_battery → param_physics → param_drone → param_map → param_swarm`。

| 文件 | 产出 struct | 关键内容 | 依赖 |
|---|---|---|---|
| `param_sim.m` | `p_sim` | `dt=0.01`, `dt_plot=0.5`, `dt_video=0.1`, `start_time=0`, `end_time=120`(默认) | — |
| `param_battery.m` | `p_battery` | `Q0,V0,e0,e1,e2,A,B,R,Qf,p0..p4,power_board` | — |
| `param_physics.m` | `p_physics` | `gravity, rho`（2 行） | — |
| `param_drone/param_drone.m` | `p_drone` | 初始状态、风参、执行器上限、规划参数；按类型 `run(param_<type>)` | 需 `DRONE_TYPE`, `p_physics.gravity` |
| `param_drone/param_quadcopter.m` | `p_drone`(续) | 四旋翼物理/控制增益 | 由 param_drone 动态调用 |
| `param_drone/param_fixed_wing.m` | `p_drone`(续) | 固定翼气动/控制参数 | 由 param_drone 动态调用 |
| `param_drone/param_sensors.m` | `p_drone`(续) | 传感器噪声/偏置 | — |
| `param_map.m` | `map` | `width=300`(quad/pm), `max_height=100`, 建筑布局 | 需 `DRONE_TYPE`, `ACTIVE_ENVIRONMENT` |
| `param_swarm/param_swarm.m` | `p_swarm` | 见 §5.2 | 需 `map`, `ACTIVE_ENVIRONMENT`, `SWARM_ALGORITHM` |
| `param_swarm/param_olfati_saber.m` | `p_swarm`(续) | `c_vm,a,b,c,delta,k,r0,lambda,c_pm_obs,c_vm_obs` | 由 param_swarm 动态调用 |
| `param_swarm/param_vasarhelyi.m` | `p_swarm`(续) | `r0_rep,p_rep,r0_fric,C_fric,v_fric,p_fric,a_fric,r0_shill,v_shill,p_shill,a_shill` | 由 param_swarm 动态调用 |

> **动态脚本调用（迁移注意）**：
> - `param_drone.m`：`run(strcat("param_", DRONE_TYPE))`（point_mass 时跳过，仅设 `Va_T=5`）
> - `param_swarm.m`：`run(strcat("param_", SWARM_ALGORITHM))`
> 这类"按字符串拼接脚本名再 run"的隐式依赖，Python 中应改为显式函数/字典分派。

### 5.2 `p_swarm` 关键字段（含二次开发新增）

| 字段 | 值 | 说明 |
|---|---|---|
| `is_active_migration` | `true` | 🆕 是否启用群体迁移 |
| `is_active_goal` | `false` | 目标点吸引 |
| `is_active_arena` | `false` | 边界排斥 |
| `is_active_spheres` | `false` | 球形障碍 |
| `is_active_cyl` | `true` | 圆柱障碍（由 map 建筑生成） |
| `loss_radio` | `0.2` | 🆕 故障比例（set_dronestate 用） |
| `nb_agents` | `5`(默认) | 无人机数量 |
| `r` | `150` | 最大感知半径 |
| `max_neig` | `10` | 最大邻居数（拓扑约束） |
| `r_coll` | `0.5` | 碰撞半径（距离<2*r_coll 记碰撞） |
| `d_ref` | `25` | 期望间距 |
| `u_ref` | `[1;0;0]` | 迁移方向（GUI 可改） |
| `v_ref` | `6` | 期望速度 |
| `max_a` / `max_v` | `10` / `7` | 加速度/速度上限 |
| `Pos0` | `P0 + P*rand(3,N)` | 初始位置（`rng('shuffle')`） |
| `cylinders/n_cyl` | 由 map 生成 | 圆柱障碍 `[x;y;r]` |

> **随机性**：`param_swarm.m` 用 `rng('shuffle')`（非固定种子）。迁移做数值对照时须改为固定种子（计划 §18）。

---

## 6. 故障机制（二次开发核心）★

这是本项目相对原版 SwarmLab 的核心增量，也是迁移的最高优先级与最高风险区。

### 6.1 状态定义与存储

- `DroneState` 枚举：`Normal`（正常）/ `Stalled`（动力锁死）/ `Drifting`（失控漂移）。
- **双份状态存储**：
  - `Drone.drone_state`（每架无人机自身，`update_state` 用它分支）
  - `Swarm.swarm_state`（`(:,1) DroneState` 列向量，群体级；算法用 `self.drones(i).drone_state` 判定邻居）
- 辅助：`Drone.drift_vel`（vect3，故障前速度）、`Drone.drift_initialized`（bool）、`Swarm.normal_drone`（健康机计数）、`Swarm.t_fault`（故障发生时刻）、`Swarm.fad`（衰减因子）。

### 6.2 故障注入：`Swarm.set_dronestate(DRONE_STATE, DRONE_STATE_CHANGED, loss_radio)`

```text
if DRONE_STATE_CHANGED == 1 或 2:
    swarm_state 全部重置为 Normal
    ├── DRONE_STATE=="Normal"  : 全部 Normal
    ├── DRONE_STATE=="Stalled" : idx=randperm(N, ceil(loss_radio*N)); swarm_state(idx)=Stalled
    └── else (Drifting)        : idx=randperm(N, ceil(loss_radio*N)); swarm_state(idx)=Drifting
    逐架同步：drones(i).drone_state = swarm_state(i)
```

- 由主循环在检测到 GUI `app.drone_state` 变化时触发；`Drifting` 同时记录 `swarm.t_fault = time`。
- `DRONE_STATE_CHANGED`：1=Normal/Stalled，2=Drifting（两者都会触发注入）。

### 6.3 故障运动学：`Drone.update_state` 的 point_mass 分支

| 状态 | 速度 | 位置 | 说明 |
|---|---|---|---|
| `Normal` | `vel = command(2:4)` | `pos += vel*dt` | 正常跟随群体指令 |
| `Stalled` | `vel = [0;0;0]` | `pos += 0`（不动） | 动力锁死，悬停/静止 |
| `Drifting` | `vel = fad * drift_vel` | `pos += vel*dt` | 首次进入时 `drift_vel = vel_xyz_history(end,:)'`；此后按 `fad` 衰减 |

**衰减因子 `fad`（在主循环计算，非 Drone 内部）**：

```text
Normal/Stalled : fad = 1
Drifting       : k = 0.2
                 fad = 1 / (1 + k*(time - t_fault))     % 双曲衰减
                 fad = min(max(fad,0),1)                % 截断
                 if fad < 0.1: fad = 0                  % 最终停止
```

> ⚠️ 迁移要点（对应计划 §6.3）：Drifting **不能**简单写成 `vel=0`；必须保留"记录故障前速度 + 按 fad 衰减 + 持续更新位置 + 最终趋停"的完整语义。`drift_vel` 只在首次进入 Drifting 时记录一次（`drift_initialized` 门控），`fad` 每步乘以固定的 `drift_vel`（衰减来自主循环随时间减小的 fad）。

### 6.4 故障在群体算法中的传播

```text
Drone.drone_state
     │（算法遍历邻居时读取 self.drones(agent2).drone_state）
     ▼
Swarm Algorithm
     ├── Normal 邻居  → 正常交互（位置匹配/速度匹配 或 排斥吸引/速度对齐）
     └── Fault 邻居   → 危险静止障碍物处理
              ├── 增强排斥（Olfati: r_fault_safe=3*d_ref, k_fault=0.5；Vasarhelyi: p_fault=3*p_rep, r_safe=3*r0_rep）
              ├── 硬安全区紧急排斥（d_hard=6*r_agent, emergency∝(d_hard-dist)^2）
              ├── Olfati 侧向滑移（cross([0;0;1],pos_rel_u)，按迁移方向选绕行侧）
              ├── Olfati 主动逃逸（is_stuck && migration：rear_blocked/escape_strength，按 agent 奇偶左右分流）
              └── 故障机自身：Vasarhelyi 直接 vel_command=[0;0;0]；不产生速度匹配项
     │
     ▼
get_barycenter（虚拟质心无人机）：仅统计 Normal 健康机（normal_drone 计数）
```

**受保护的关键变量/机制**（计划 §6.4 明列，务必逐一迁移）：
`has_fault_neighbor`、`min_dist_to_fault`、`fault_rear_count`、`fault_front_count`、`is_stuck`、`rear_blocked`、`escape_strength`、`drift_vel`、`fad`、`t_fault`、`loss_radio`、`is_active_migration`、`normal_drone`。

---

## 7. 算法模块

### 7.1 Olfati-Saber（`compute_vel_olfati_saber.m`，故障版）

- **数学内核（原版保留）**：sigma-norm 势函数 `psi`/`psi_der`、邻接函数 `rho_f`/`rho_der_f`、组合 `phi`；位置匹配 `acc_pm` + 速度匹配 `acc_vm` + 墙 `acc_wall` + 障碍 `acc_obs`；`vel_command = vel + acc*dt`。
- **邻居搜索**：欧氏半径 `p_swarm.r` + 拓扑上限 `max_neig`（取最近 N 个）。
- **🆕 故障处理**（`if nb_neig~=0` 内）：对每个邻居判 `drone_state`；Normal→标准交互；Fault→方案1（增强排斥+侧向滑移）+方案2（`is_stuck` 主动逃逸）。逃逸方向含 `-u_ref` + 侧向偏移（`mod(agent,2)` 左右分流），`escape_strength` 与被困程度成正比。
- **迁移项**：`is_active_migration` 时 `acc_vm = c_vm*(u_ref*v_ref - vel)`。
- **障碍**：球形/圆柱/竞技场（`repulsion_cubic_arena` 子函数）。
- **子函数**：`rho_f`, `rho_der_f`, `repulsion_cubic_arena`（同文件内）。
- ⚠️ **潜在 Bug**：末尾限速/限加速度用 `if isempty(p_swarm.max_v)` / `isempty(p_swarm.max_a)`（应为 `~isempty`），导致限幅为**死代码**（见 §16）。

### 7.2 Vasarhelyi（`compute_vel_vasarhelyi.m`，故障版）

- **数学内核（原版保留）**：排斥/吸引 `vel_rep`（`r0_rep` 分界）、速度对齐 `vel_fric`（`get_v_max` 制动曲线）、墙 `vel_wall`（shill agent）、障碍 `vel_obs`；迁移/目标/自推进三选一叠加。
- **🆕 故障处理**：Normal 邻居→标准排斥吸引+对齐；Fault 邻居→视为危险静止障碍（`p_fault=3*p_rep`, `r_safe=3*r0_rep` 线性排斥 + `d_hard=6*r_agent` 紧急排斥）。**故障机自身** `vel_command=[0;0;0]`。
- **限幅**：`if ~isempty(p_swarm.max_v)` / `~isempty(max_a)`（**正确**，与 Olfati 相反）。
- **子函数**：`get_v_max(v_fric,r,a,p)`（同文件内）。

> 两算法输出统一为 `[vel_command(3×N), collisions=[nb_agent_coll, nb_obs_coll, min_dist_obs]]`，坐标系在 Olfati 内部临时翻转为"上为正"再翻回 NED。


---

## 8. 路径规划 / 管理 / 跟踪（guidance）

> ⚠️ 该链路服务于 **单机航点导航**（`examples_drone/*`），**群体故障仿真主循环不调用它**（群体用算法直接算速度指令）。但它属于计划 §12 要求分析的调用链，且含关键 `persistent`。

### 8.1 调用链

```text
Drone.plan_path(path_type, time)          % @Drone/plan_path.m
      └── switch path_type:
            ├── "fillet"/"dubins"         → 硬编码航点表
            ├── "city_dubins"             → plan_RRT_dubins(...)   ❌ 文件缺失
            ├── "straight_line_rrt"       → plan_RRT(...)          ✅ guidance/path_planner/plan_RRT.m
            └── 5 (数值)                  → plan_cover_RRT_dubins(...) ❌ 文件缺失
      → 写 self.nb_waypoints, self.waypoints(reshape 5*size,1)

Drone.manage_path(time)                   % Drone.m 内
      ├── fixed_wing → path_manager_wing([nb_wpt;waypoints;z_hat;time], p_drone, p_sim)
      └── quad/point_mass → path_manager_quad(...)
                                  └── path_manager_fillet_quad(in,p_drone,start_of_simulation)

Drone.follow_path()                       % Drone.m 内（无 time 参数）
      ├── fixed_wing → path_follower_wing(path, p_drone)
      └── quad/point_mass → path_follower_quad(path, p_drone) → command=[psi_c;vn_c;ve_c;vd_c]
```

### 8.2 `plan_RRT.m`（RRT  rapid-exploring random tree）

- 主函数 + 同文件子函数：`generate_random_node`, `collision`, `points_along_path`, `down_at_NE`, `extend_tree`, `find_minimum_path`, `smooth_path`, `plot_map`。
- 节点格式 `[pn,pe,pd,chi,cost,parent_idx,flag_connect_to_goal]`；`collision` 用 `down_at_NE` 读 `map.buildings_*` 判障。
- ⚠️ 用 `&`（非短路）而非 `&&`；`while nb_paths<3` 扩展树。

### 8.3 `path_manager_quad.m` 的 persistent 状态机（迁移重点，对应计划 §10）

- 主函数 persistent：`start_of_simulation`
- 子函数 `path_manager_fillet_quad` persistent：`wpt_list_prev`、`idx`（航点指针）、`flag_transition`（1=直线段, 2=圆弧过渡）、`flag_need_new_wpts`
- 这是"跨调用持久化的路径管理器状态"，Python 迁移应封装为 `PathManager` 类的实例状态（计划 §10 的示例正是此意）。
- 输入/输出均为 **打包长向量**（`in=[nb_wpt; waypoints(5*size); z_hat(22); time]`，用 `NN` 偏移解析），迁移须精确复刻索引。

---

## 9. Control / Estimation / Physics（真实动力学链路，均为原版未改动）

> 仅当 `drone_type ~= point_mass` 时进入。群体故障仿真用 `point_mass`，**不触发**本链路；但迁移计划 Phase 8/9/10 覆盖它。

### 9.1 Control（`control/control_drone/`）

```text
autopilot_quad(drone, t)  → y=[delta(4); x_command(19)]
   ├── 按 autopilot_version 分派：
   │     1 attitude / 2 velocity / 3 acceleration / 4 position
   ├── 级联 PID：h_thrust_hold, vd_thrust_hold, ve_roll_hold, vn_pitch_hold,
   │            roll/pitch/yaw_torque_hold, ... → 均调用 pid_loop(...)
   ├── mixer(thrust, torque_*) → d1..d4
   └── saturate(d, dt_max, 0)
autopilot_wing(drone, _, t)  → 固定翼（TECS/uavbook 类保持控制器）
compute_pid_controller / pid_loop / compute_tf_model / compute_trim /
generate_command / param_table_T / Td2idx / true_states
```

- **状态承载方式**：autopilot 的积分器/前值状态（`P_*` struct：int/diff/error_prev/y_c_prev）**存放在 Drone 对象属性上**并透传（`drone.P_h_thrust` 等），**未使用 persistent**（原版 `autopilot_wing.m` 里的 persistent 全部被注释掉）。→ 迁移友好：直接映射为 Drone 实例属性。

### 9.2 Estimation（`estimation/`）

```text
Drone.update_sensor_measurements() → sensors(...) + gps(...)  → self.y
Drone.estimate_states(time):
   ├── use_estimation==true  → estimate_states(y,time,dt,p_drone,p_physics)  → z_hat(22)
   └── use_estimation==false → true_states([get_state(); airdata; time])     → z_hat(22)
```

- `estimate_states.m` 含 **persistent `kf1`, `x`, `kf_P`**（EKF 状态跨调用持久化）+ LP 滤波（`lp_filter`）+ `ekf_phi_theta`/`ext_kalman_filter`（当前 EKF 主体部分被注释，主要走 LP + 简化 KF）。
- ⚠️ **`z_hat` 布局不一致**：`true_states` 与 `estimate_states` 输出的 22 维语义在第 4–6、12–13、19 位不同（见 §16）。默认 `use_estimation=false`，走 `true_states`。

### 9.3 Physics（`physics/`）

```text
compute_dynamics → forces_moments_quad(x,delta,wind,~,p_drone,p_sim,p_physics) → out(12)
                   forces_moments_wing(...)                                     → out(12)
                   out=[F(3); M(3); Va; alpha; beta; wn; we; wd]
compute_kinematics → ode23(@(t,x) kinematics_ode_fun(time,x,x_old,uu,p_drone)) → 新 state(12)
sensors(x,forces,airdata,p_drone,p_physics) → y(8)（陀螺/加速度/气压 + 噪声）
gps(x,dt,p_drone) → y_gps(5)
get_wind(time,map,...) → total_wind(6)（persistent wind_struct 缓存湍流场）
```

- `kinematics_ode_fun`：刚体 12 维状态导数（`Rb2i`、欧拉率矩阵 `Si_b`、`gamma1..8` 惯量参数）。
- ⚠️ `get_wind.m` 含 **persistent `wind_struct`**（首次调用生成整个湍流风场并缓存）。

---

## 10. 可视化 / GUI

### 10.1 可视化（graphics）

| 组件 | 类型 | 说明 |
|---|---|---|
| `SwarmViewer` | handle 类 | 群体在线可视化；🆕 按 `drone_state` 用 ○(Normal)/□(Stalled)/△(Drifting) 三种 marker 分别绘制（`scatter_handle_normal/stalled/drifting`）；读 `[swarm1.drones.drone_state]`（逗号列表展开） |
| `SwarmViewerOffline` | handle 类 | 离线回放 |
| `DroneViewer` | handle 类 | 单机 3D（`define_body`→`rotate`/`translate`→NED→ENU） |
| `@Swarm/plot_state.m` | 方法 | 实时曲线（距离/速度/碰撞），**persistent 图形句柄**，uitab 分组 |
| `@Drone/plot_state.m` | 方法 | 单机状态曲线，**persistent 句柄** |
| `plot_swarm*.m`, `plot_*_offline.m` | 函数 | 群体/离线绘图（多数被二次开发改动） |
| `graphics_map/*` | 函数 | 建筑/圆柱/球/路径/航点绘制 |
| `VideoWriterWithRate` | handle 类 | 按帧率录制（**persistent video_handle** 在 `record_video.m`） |
| `mseb.m` | 函数 |  shaded error bar（第三方） |

### 10.2 GUI（App Designer `.mlapp`）

- `examples/GUI_swarm.mlapp`（480 行）：群体仿真 GUI，即主循环中 `app` 变量来源。提供 `app.drone_type / drone_state / drone_state_prev / drone_state_changed / loss_radio / nb_agents / d_ref / v_ref / orientation / wind* / debug_plot / sim_time / Switch.Value` 等。
- `examples/GUI_drone.mlapp`（162 行）：单机 GUI；`main_GUI.m` → `run('GUI_drone')`。
- `.mlapp` 为 App Designer 专有格式（内含 MATLAB 代码 + 组件布局）。**对应计划 Phase 13（最后迁移）**；核心仿真已能脱离 GUI 运行（`if exist('app','var')` 分支）。

---

## 11. `persistent` 全清单（迁移必须转对象/专用状态）

| 文件 | persistent 变量 | 语义 | 迁移建议 |
|---|---|---|---|
| `estimation/estimate_states.m` | `kf1, x, kf_P` | EKF 状态跨步持久 | 封装为 Estimator 类实例状态 |
| `physics/get_wind.m` | `wind_struct` | 缓存整场湍流风 | WindField 类（一次生成，多次查询） |
| `guidance/path_manager/path_manager_quad.m` | `start_of_simulation` | 仿真起始标记 | PathManager 实例状态 |
| └ 子函数 `path_manager_fillet_quad` | `wpt_list_prev, idx, flag_transition, flag_need_new_wpts` | 航点状态机 | PathManager 实例状态（计划 §10 示例） |
| `@Swarm/plot_state.m` | `fig_handle, d_mm_handle, ... tg, tabs` | 图形句柄 | Viewer/Plotter 实例状态（time==0 重置） |
| `@Drone/plot_state.m` | `fig_handle, pn_handle, ...` | 图形句柄 | 同上 |
| `graphics/graphics_swarm/plot_swarm.m` | `swarm_fig, scatter_handle, colors, prediction_handle` | 图形句柄 | 同上 |
| `graphics/graphics_map/draw_spheres.m` | `scatter_handle` | 图形句柄 | 同上 |
| `@Swarm/record_video.m` | `video_handle` | 视频写入句柄 | VideoWriter 实例状态 |

> `control/control_drone/autopilot_wing.m` 中所有 `persistent` **均已被注释**，改用 `drone.*` 对象属性承载 → 无需特殊处理。

---

## 12. `global` 与 workspace 隐式依赖

- **`global`：全项目 0 处**（✅ 无全局变量）。
- **workspace 隐式依赖（脚本共享变量）**：这是本项目主要的隐式耦合来源：
  - `param_*.m` 均为 **脚本**（非函数），直接在调用者 workspace 产出 `p_sim/p_drone/p_battery/p_physics/map/p_swarm` 等变量。
  - 依赖顺序：`param_map` 需 `DRONE_TYPE`/`ACTIVE_ENVIRONMENT`；`param_swarm` 需 `map`/`ACTIVE_ENVIRONMENT`/`SWARM_ALGORITHM`；`param_drone` 需 `DRONE_TYPE`/`p_physics`。
  - `param_swarm` 末尾 `run(strcat("param_", SWARM_ALGORITHM))`、`param_drone` 中 `run(strcat("param_", DRONE_TYPE))`：**按变量值动态选择脚本**。
  - 示例脚本顶部 `addpath(genpath(project_root))` 依赖 MATLAB 路径机制。
- **迁移建议**：Python 中用显式配置对象 / `dataclass` / 依赖注入替代 workspace 共享（计划 §8）；动态 `run` 改为字典分派或工厂函数。

---

## 13. MATLAB 特有数据结构（迁移映射，对应计划 §9）

| 结构 | 出现位置 | 说明 | Python 映射 |
|---|---|---|---|
| `struct`（参数） | `p_sim/p_drone/p_battery/p_physics/map/p_swarm` | 字段动态增删（`isfield` 判定） | `dataclass` 或带默认值的配置类 |
| **打包列向量 `vectN`** | `command(4)`, `full_command(19)`, `z_hat(22)`, `path(26)`, `airdata(6)`, `state(12)`, `waypoints(5*size)` | 用固定偏移索引解析（`NN` 偏移） | `np.ndarray` + 明确索引常量/命名切片 |
| 矩阵 | `pos_ned_history(n×3)`, `Pos_ned(3×N)`, 邻接矩阵 `M`, 距离矩阵 `D` | 行列语义需分清 | `np.ndarray`（注意 3×N vs N×3） |
| `cell` | `network_components.m` 的 `members` | 连通分量成员列表 | `list[list[int]]` |
| `enumeration` | 5 个枚举类 | 字符串比较 `== "Normal"` | `enum.Enum`（注意与字符串比较语义） |
| handle 对象数组 | `Swarm.drones` | 引用语义、`[swarm1.drones.drone_state]` 逗号列表展开 | `list[Drone]` + 列表推导 |
| 图形句柄 | `scatter3/patch/figure` handle | 增量 set Xdata/Ydata | matplotlib artist 对象 |
| 匿名函数 | `psi=@(z)...`, `phi=@(z)...`, `rho=@rho_f` | 算法内核 | Python lambda / 函数引用 |
| `ode23` | `compute_kinematics` | 变步长 ODE 积分 | `scipy.integrate.solve_ivp`（须核对步长/容差一致性） |
| Statistics Toolbox | `pdist/squareform/pdist2/normrnd` | 距离/随机 | `scipy.spatial.distance` / `numpy` |

---

## 14. 潜在接口不一致 / potential bug（只记录，不修改 —— 计划 §2.3）

> 以下均为 **Potential MATLAB inconsistency / potential bug**，迁移时默认**保持原行为**，除非明确要求修复。

### 14.1 二次开发引入的接口脱节（最重要）

| 编号 | 位置 | 现象 | 影响 | 迁移处理 |
|---|---|---|---|---|
| I-1 | `examples_drone/example_controller.m:48` 等 4 个单机示例 | `Drone(...)` 用 **6 参**（缺 `drone_state`），但故障版构造函数需 **7 参** | 单机示例运行即报错 | Python 版 Drone 构造须兼容/显式；注明这些示例已过时 |
| I-2 | 同上 `:100/104/106` | `drone.update_state(wind,time)` 用 **2 参**，故障版需 **3 参**（`fad`） | 同上 | `fad` 设为可选默认参数或显式传 1 |
| I-3 | `tests/example_co_mn.m:60`, `example_ct.m:44` | `swarm.add_drone(...)` 用 **6 参**（缺 `drone_state`） | tests 报错 | 同上 |
| I-4 | `tests/example_co_mn.m:134`, `example_ct.m:88` | `swarm.update_state(wind,time)` 用 **2 参** | tests 报错 | 同上 |

### 14.2 原版遗留 / 逻辑可疑

| 编号 | 位置 | 现象 | 迁移处理 |
|---|---|---|---|
| B-1 | `Swarm.m:64` `add_n_drones` | 调 `self.add_drone(drone)` 仅 1 参，签名需 7 参 | 保持记录；Python 版修正签名或标注不可用 |
| B-2 | `Swarm.m:89,188,193` | 用 `self.nb_drones`，但属性名是 `nb_agents`（未定义属性） | `set_vel`/`set_state` 潜在报错；记录 |
| B-3 | `Swarm.m:367` `path_manager_individual` | 调 `drones(i).path_manager_wing(time)`，Drone 无此方法（应为 `manage_path`） | 记录 |
| B-4 | `Swarm.m:377` `path_follower_individual` | 调 `follow_path(time)`，但 `Drone.follow_path()` 无参 | 记录 |
| B-5 | `Swarm.m:333` `get_barycenter` | `self.normal_drone` 累加前**未清零**；多次调用会无限增长；若全故障则除零 | 记录（迁移时每次调用前重置更安全，但需确认是否改变行为） |
| B-6 | `Swarm.m:406,412` | 声明外部方法 `draw_agents_energy`/`record_state`，但 `@Swarm/` **无对应 .m 文件** | 死声明（示例未调用）；记录 |
| B-7 | `Drone.m:531,533` `compute_Qt_hover` | 用 `self.B.pow_board`/`self.B.V0`，但无属性 `B`（应为 `p_battery`） | 记录（该方法未被主链路调用） |
| B-8 | `compute_vel_olfati_saber.m:389,401` | 限速/限加速度用 `if isempty(max_v/max_a)`（应为 `~isempty`），导致限幅**死代码** | 记录；Vasarhelyi 处为正确的 `~isempty`（两算法不一致） |
| B-9 | `plan_path.m:74` | `waypoints(i,-63)` **负索引**（疑似 `1` 笔误） | 记录（仅 city_dubins 分支） |
| B-10 | `plan_path.m` vs `PathType.m` | switch 分支 `"city_dubins"/"straight_line_rrt"/5` 与枚举 `{fillet,dubins,straight_rrt}` 不匹配；且调 `plan_RRT_dubins`/`plan_cover_RRT_dubins`（**文件缺失**） | 记录 |
| B-11 | `true_states.m` vs `estimate_states.m` | `z_hat(22)` 布局语义不一致（第 4–6、12–13、19 位） | 记录；迁移须固定一套布局并注明 |
| B-12 | `Drone.m:573`（原版亦有） | `if self.pos_ned_history == [NaN,NaN,NaN]` 对空/矩阵比较语义可疑 | 记录 |
| B-13 | `example_olfati_saber.m` vs `example_vasarhelyi.m` | 后者在状态变化时 `collisions_sum=0` 清零、建 `shared_fig`；前者无 | 行为差异，记录 |

---

## 15. 原始 SwarmLab vs 二次开发（差异对照总结）

| 维度 | 原版 SwarmLab | 二次开发（单群避障版） |
|---|---|---|
| 无人机状态 | 仅 `drone_type` | 增 `drone_state`(Normal/Stalled/Drifting) + `DroneState` 枚举 |
| Drone 构造 | `Drone(type,p_drone,...,map)` 6 参 | 增 `drone_state`，7 参 |
| `update_state` | `(wind,time)` 2 参 | `(wind,time,fad)` 3 参 |
| point_mass 更新 | `vel=command; pos+=vel*dt` 单一 | Normal/Stalled/Drifting 三分支 + `drift_vel`/`fad` 衰减 |
| Swarm | `drones/nb_agents/algorithm/...` | 增 `swarm_state/normal_drone/t_fault/fad` + `set_dronestate` |
| get_barycenter | 全部无人机平均 | 仅健康(Normal)机平均 |
| Olfati/Vasarhelyi | 标准群体算法 | 增故障邻居处理、增强排斥、逃逸、迁移逻辑 |
| 参数 | 无故障相关 | 增 `is_active_migration/loss_radio` |
| 可视化 | 单一 scatter | 按状态分 3 类 marker；性能图扩展 |
| 单机示例/tests | 与类接口一致 | **未同步更新 → 与故障版类脱节**（§14.1） |
| control/estimation/physics | — | **完全未改动**（78 文件逐字节相同） |

---

## 16. Python 迁移映射建议（对齐计划 Phase 1–13）

| 计划 Phase | MATLAB 源 | 建议 Python 结构 | 关键注意 |
|---|---|---|---|
| P1 枚举/常量 | `enumerators/*.m`(5) | `enums/*.py`（`Enum`） | `DroneState` 新增；字符串比较语义 |
| P2 参数 | `parameters/**`(11) | `parameters/*.py`（`dataclass`） | 装配顺序依赖；动态 `run` → 分派；`rng('shuffle')`→固定种子 |
| P3 Drone | `@Drone/Drone.m` | `core/drone.py` | 保留 handle 引用语义；`vectN` 打包向量索引；`update_state(wind,time,fad)` |
| P4 Swarm | `@Swarm/Swarm.m` | `core/swarm.py` | `drones` list；`swarm_state`；`set_dronestate`；`get_barycenter`(normal_drone) |
| P5 point_mass | `Drone.update_state` pm 分支 | `core/drone.py` | ★Normal/Stalled/Drifting + drift_vel + fad（勿简化为 vel=0） |
| P6 Vasarhelyi | `compute_vel_vasarhelyi.m` | `swarm/vasarhelyi.py` | 故障邻居=危险障碍；`get_v_max`；`~isempty` 限幅 |
| P7 Olfati-Saber | `compute_vel_olfati_saber.m` | `swarm/olfati_saber.py` | ★势函数/邻接/逃逸/迁移；保留 `isempty` 限幅原行为(B-8) |
| P8 Physics | `physics/*` | `physics/*` | `ode23`→`solve_ivp`（核对一致性）；forces/kinematics |
| P9 Control | `control/*` | `control/*` | autopilot 级联 PID；状态存 Drone 属性（无 persistent） |
| P10 Estimation | `estimation/*` | `estimation/*` | `persistent kf1/x/kf_P`→Estimator 实例状态；z_hat 布局(B-11) |
| P11 Guidance | `guidance/*` | `guidance/*` | `path_manager` persistent 状态机→PathManager 类；缺失文件(B-10) |
| P12 Visualization | `graphics/*` | `visualization/*` | persistent 句柄→Plotter 状态；按 drone_state 分色/标记 |
| P13 GUI | `GUI_*.mlapp` | `gui/*`（最后） | 核心仿真须先脱离 GUI 独立运行 |

---

## 17. 迁移风险与数值验证要点（对齐计划 §17/§18）

1. **最高风险 = 故障机制**：`drone_state` 状态机、`fad` 双曲衰减、`drift_vel` 一次性记录、故障邻居排斥/逃逸/迁移。任何简化都会改变行为。
2. **打包向量索引**：`z_hat(22)/path(26)/command(4)/full_command(19)/waypoints`，1-based→0-based 必须逐处核对（计划 §11）。
3. **坐标系翻转**：Olfati 内部 NED→"上为正"再翻回；`get_vel_ned` 用 `Rb2i` 旋转；可视化 NED→ENU。迁移须保持一致。
4. **限幅死代码(B-8)**：Olfati 的 `max_v/max_a` 限幅因 `isempty` 实际未生效——若要"行为一致"，Python 需**保留该无效行为**（或明确记录为待修复）。
5. **随机性**：`rng('shuffle')`、`randperm`（故障机选择）、`randn`（c_r/传感器）、`rand`（初始色/位置）。数值对照须固定种子并复刻调用顺序。
6. **ODE 求解器**：`ode23`（变步长）→ Python `solve_ivp`；真实动力学分支需核对积分容差，point_mass 分支为显式欧拉（简单）。
7. **对照测试锚点**：以 `example_olfati_saber`/`example_vasarhelyi` 为唯一可运行入口，比较 `pos(t)/vel(t)/command(t)/drone_state(t)/collisions(t)` 与性能指标 `safety/order/union/alg_conn`。

---

## 18. 建议的下一步（Phase 0 → Phase 1）

1. 确认本架构文档无误（尤其 §14 接口不一致、§16 迁移映射）。
2. 决定 Python 项目骨架（计划 §15 的目标结构）与 Git 分支（`matlab-original` / `python-migration`，计划 §19）。
3. 进入 **Phase 1**：迁移 5 个枚举（含新增 `DroneState`）为 Python `Enum`，并建立 MATLAB/Python 对照测试脚手架。

---

## 附录 A：已读文件清单（Phase 0 直接精读）

`@Drone/Drone.m`、`@Drone/plan_path.m`、`@Swarm/Swarm.m`、`@Swarm/compute_vel_olfati_saber.m`、`@Swarm/compute_vel_vasarhelyi.m`、`@Swarm/plot_state.m`、`enumerators/*.m`(5)、`parameters/param_sim.m`、`param_map.m`、`param_drone/param_drone.m`、`param_swarm/param_swarm.m`、`param_olfati_saber.m`、`param_vasarhelyi.m`、`examples/examples_swarm/example_olfati_saber.m`、`example_vasarhelyi.m`、`examples/examples_drone/example_controller.m`、`examples/main_GUI.m`、`estimation/estimate_states.m`、`control/control_drone/true_states.m`、`autopilot_quad.m`、`physics/forces_moments_quad.m`、`kinematics_ode_fun.m`、`sensors.m`、`get_wind.m`、`guidance/path_planner/plan_RRT.m`、`path_manager/path_manager_quad.m`、`path_following/path_follower_quad.m`、`math_tools/compute_swarm_performance.m`、`compute_neighborhood.m`、`network_components.m`、`pos2dist.m`、`Rb2i.m`、`graphics/graphics_swarm/SwarmViewer.m`。

> 其余文件（`control/*` 其余、`estimation/*` 其余、`physics/*` 其余、`guidance/*` wing/dubins 系列、`graphics/*` 其余、`turbulent_wind_generator/*`、`tests/*`）已通过调用链、`persistent`/`global` 全局搜索与哈希 diff 完成结构级定位；它们相对原版**未改动或仅微调**，可在对应 Phase（8–13）迁移时逐文件精读。

## 附录 B：分析产物位置

- 原版基线解压：`c:\SwarmLab\phase0_matlab\swarmlab-master`
- 故障版解压（ASCII 路径副本）：`c:\SwarmLab\phase0_fault\swarmlab-master`
- 版本 diff 脚本：`c:\SwarmLab\_phase0_diff.ps1`（可复跑验证一致性）
