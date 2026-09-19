# SwarmLab_py 迁移日志（MIGRATION LOG）

> 依据迁移计划 §2.4「修改必须可追踪」：每阶段记录 修改/新增/删除文件、原因、
> MATLAB↔Python 对应、测试结果、已知问题。
> 权威 MATLAB 源码：`大创车辆路径规划/大创文件——单群避障版/swarmlab-master`
> （= 用户提供的 zip，103 个 `.m` 文件哈希全一致）。架构分析见 `PROJECT_ARCHITECTURE.md`。

---

## 目录组织约定

- 所有迁移的 Python 代码与生成的文件夹统一置于 `SwarmLab_py/`。
- **按功能分类归入同一文件夹**，文件夹命名参考 MATLAB 开发文件存放规则：

| Python 文件夹 | 对应 MATLAB 目录 | 引入阶段 |
|---|---|---|
| `enums/` | `enumerators/` | Phase 1 ✅ |
| `tests/` | （新增，对照测试） | Phase 1 ✅ |
| `parameters/` | `parameters/` | Phase 2（待） |
| `core/` | `@Drone/`, `@Swarm/` | Phase 3–4（待） |
| `swarm/` | `@Swarm/compute_vel_*` | Phase 6–7（待） |
| `physics/` `control/` `estimation/` `guidance/` `visualization/` | 同名目录 | Phase 8–13（待） |

---

## Phase 1 —— 枚举 / 常量

**日期**：2026-09-17　**状态**：✅ 完成（代码已写 + 运行验证通过）

### 新增文件（8）

| Python 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `SwarmLab_py/__init__.py` | — | 包根，项目说明 |
| `SwarmLab_py/enums/__init__.py` | `enumerators/`（目录） | 统一导出 5 个枚举 |
| `SwarmLab_py/enums/drone_type.py` | `enumerators/DroneType.m` | quadcopter/fixed_wing/point_mass |
| `SwarmLab_py/enums/drone_state.py` | `enumerators/DroneState.m` | ★Normal/Stalled/Drifting（二次开发新增） |
| `SwarmLab_py/enums/path_type.py` | `enumerators/PathType.m` | fillet/dubins/straight_rrt |
| `SwarmLab_py/enums/swarm_algorithm.py` | `enumerators/SwarmAlgorithm.m` | vasarhelyi/olfati_saber |
| `SwarmLab_py/enums/swarm_viewer_type.py` | `enumerators/SwarmViewerType.m` | drone/agent/agent_with_energy |
| `SwarmLab_py/tests/test_enums.py` | — | 对照测试（值/数量/字符串比较行为） |

### 修改 / 删除文件

- 无（Phase 1 为纯新增，未改动任何 MATLAB 源码，符合 Phase 0/迁移铁律）。

### 迁移原因

- 枚举是全项目的类型基础：`Drone.drone_type/drone_state`、`Swarm.algorithm`、
  `SwarmViewer.viewer_type`、`PathType` 均依赖它们。按计划 Phase 1 先行迁移。

### 关键设计决策

1. **`(str, Enum)` 混入**（而非纯 `Enum` 或 3.11+ 的 `StrEnum`）：
   - 原因：MATLAB 源码大量使用「枚举 == 字符串字面量」比较（如
     `self.drone_type == "point_mass"`、`drone_state == "Normal"`、
     `self.algorithm == "olfati_saber"`）。`(str, Enum)` 使
     `DroneType.POINT_MASS == "point_mass"` 成立，**行为与 MATLAB 一致**，
     且兼容 Python 3.4+（不绑定 3.11）。
2. **成员值严格等于 MATLAB 枚举名**（含大小写：`Normal/Stalled/Drifting` 首字母大写，
   其余小写）；**成员名按 PEP8 用大写下划线**（`POINT_MASS` 等）。
   需要原始字符串处使用 `.value`。
3. **保留 MATLAB 原注释**（计划 §2.2）：每个文件 docstring 内保留对应 `.m` 的原始注释。

### 测试结果

- ✅ **已运行通过**（解释器：`C:\Python\python.exe`，Python 3.14.7）：
  ```
  $ python SwarmLab_py/tests/test_enums.py
  [PASS] test_values_match_matlab
  [PASS] test_member_count
  [PASS] test_string_comparison_behavior
  [PASS] test_specific_matlab_comparisons
  --------------------------------------------------
  Total: 4  Passed: 4  Failed: 0   (ExitCode 0)
  ```
- 结论：5 个枚举的成员值/数量/顺序与 MATLAB `enumerators/*.m` 完全一致；
  `(str, Enum)` 的“枚举 == 字符串”比较行为与 MATLAB 一致（Python 3.14 下实测通过）。

### 已知问题 / 待办（记录，不在 Phase 1 修改）

- **KI-1**（承 `PROJECT_ARCHITECTURE.md` §14 B-10）：`PathType` 枚举值
  `{fillet,dubins,straight_rrt}` 与 `@Drone/plan_path.m` 的 switch 分支
  `{fillet,dubins,city_dubins,straight_line_rrt,5}` 不一致。Phase 1 保持与
  **枚举定义（原版）** 一致，差异留待 Phase 11（Guidance）处理并记录。
- **KI-2**：`SwarmAlgorithm.value` 在 MATLAB 中用于 `run(strcat("param_",算法))`
  的动态脚本加载；Phase 2 迁移参数系统时须改为显式字典分派/工厂函数。


---

## Phase 2 —— 参数系统

**日期**：2026-09-18　**状态**：✅ 完成（代码 + 运行验证通过，9/9 + 枚举回归 4/4）
**方案**：B（基类 + 继承子类）；全部迁移（含 point_mass / quadcopter / fixed_wing）

### 新增文件（10）

| Python 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `parameters/__init__.py` | （装配入口） | 导出 + `ParamsBundle` + `build_params()` |
| `parameters/physics.py` | `param_physics.m` | `PhysicsParams`(gravity, rho) |
| `parameters/sim.py` | `param_sim.m` | `SimParams`(dt=0.01…) |
| `parameters/battery.py` | `param_battery.m` | `BatteryParams`(16 字段) |
| `parameters/sensors.py` | `param_drone/param_sensors.m` | `SensorParamsMixin`(+Q/R 派生) |
| `parameters/drone.py` | `param_drone.m`, `param_quadcopter.m` | `DroneParams`/`PointMassParams`/`QuadcopterParams` + `build_drone_params` + `compute_inertia_gammas` |
| `parameters/fixed_wing.py` | `param_fixed_wing.m`(+`Td2idx`/`compute_tf_model`/`table_T.mat`) | `FixedWingParams` + `XYU_TRIM` 常量 |
| `parameters/map_params.py` | `param_map.m`, `create_shifted_buildings.m` | `MapParams` + `create_shifted_buildings` + `build_map_params` |
| `parameters/swarm.py` | `param_swarm.m`, `param_olfati_saber.m`, `param_vasarhelyi.m` | `SwarmParams`/`OlfatiSaberParams`/`VasarhelyiParams` + `build_swarm_params` |
| `tests/test_parameters.py` | — | 9 项对照测试（独立手算期望值） |

### 修改 / 删除文件

- 无 MATLAB 改动。`parameters/drone.py` 内部重构：抽出 `compute_inertia_gammas()` 供
  quad/wing 共用（消除 gamma 公式重复）。

### 关键设计决策

1. **基类 + 继承子类**（方案 B）：`DroneParams`←`PointMass/Quadcopter/FixedWing`；
   `SwarmParams`←`OlfatiSaber/Vasarhelyi`；传感器用 `SensorParamsMixin` 多继承混入，
   **保持 MATLAB 扁平字段访问**（`p_drone.mass`、`p_drone.sd_gyro` 不变）。
2. **派生量在 `__post_init__`/工厂计算**：gamma1-8、~30 个 quad PID 增益、wing 配平 PID、
   `c=(b-a)/2√(ab)`、`c_vm_obs`、`r0_rep=d_ref`、传感器 Q/R、`center_arena`；
   `R_min/radius`（依赖 gravity）在 `build_drone_params` 工厂计算。
3. **动态 `run(strcat("param_",X))` → 工厂 + 分派**（KI-2 已落实）。
4. **`isfield` 条件默认 → dataclass 默认值 + 工厂 `**overrides`**（对应 GUI 先设后 run）。
5. **table_T.mat 处理 = 方案 b**：`Td2idx([35,0,Inf])` 恒得索引 (1,1,1)，故只用到
   `xyu_T=table_T(:,1,1,1)`；已用 scipy 以**完整精度**提取 19 值烘焙为 `XYU_TRIM` 常量，
   并忠实迁移 `compute_tf_model` 的 `a` 结构（`tf()` 传递函数未用于整定，故不迁移，
   **无需 Control System Toolbox**）。
6. **随机初始化**：`build_swarm_params(seed=...)`；`seed=None` 复刻 `rng('shuffle')` 不可复现。

### ⚠️ 命名/表示差异（迁移须知）

- **KI-3**：MATLAB `p_swarm.lambda` 是 Python 关键字 → 重命名为 **`p_swarm.lambda_`**。
  Phase 7 迁移 Olfati-Saber 避障时，凡 `p_swarm.lambda` 一律对应 `lambda_`。
- **KI-4**：`SwarmParams.x_goal` 在 MATLAB 中当 `is_active_goal=false` 时未定义；Python
  加了默认 `zeros(3,1)` 以防 AttributeError（行为等价，因该分支默认不触发）。
- **KI-5**：无环境时 MATLAB `p_swarm.cylinders=0`（标量）；Python 用 `np.zeros((3,0))`
  + `n_cyl=0`（算法 `for obs=1:n_cyl` 不执行，行为等价）。
- **KI-6**：`create_buildings.m`（随机高度版）在 `param_map.m` 中被注释、未启用，故未迁移。
- 传感器 `bias_gyro_*`：MATLAB 先赋 0.01 后覆盖为 0，Python 直接取最终值 0（已在注释标注）。

### 测试结果（`C:\Python\python.exe`，Python 3.14.7 / numpy 2.5.3 / scipy 1.18.1）

```
$ python SwarmLab_py/tests/test_parameters.py
[PASS] test_physics_sim_battery   [PASS] test_point_mass   [PASS] test_quadcopter
[PASS] test_fixed_wing            [PASS] test_map          [PASS] test_swarm_olfati
[PASS] test_swarm_vasarhelyi      [PASS] test_seed_reproducibility
[PASS] test_build_params_and_overrides
Total: 9  Passed: 9  Failed: 0
$ python SwarmLab_py/tests/test_enums.py   → 4/4 PASS（Phase 1 回归）
```

对照基准为“按 MATLAB 公式独立重算的期望值”，抽样核对通过，例如：
`R_min(pm)=1.47133`、`quad.mass=0.295989`、`quad.kp_phi=0.077696`、`quad.gamma3=205.9308`、
`wing.kp_phi=4/3`、`wing.kp_theta=-2`、`wing.R_min=72.0952`、`olfati.c=0.894427`、
`cyl_r=15.9099`、`n_cyl=14`、建筑 north/east 数组逐一吻合。

### 待办

- **KI-7（建议）**：本机无 MATLAB，暂未做“MATLAB 导出参数快照 ↔ Python 逐元素 diff”。
  若需严格 §18 数值验证，可在 MATLAB 中 `save` 一份 `p_drone/p_swarm/map` 快照，
  我再加一个 `scipy.io.loadmat` 对照测试。wing 的 `kd_phi/kp_chi/.../ki_va_theta`
  目前仅验证“有限且量级合理”，建议纳入该快照 diff。


---

## Phase 3 —— Drone 类（point_mass 主线优先）

**日期**：2026-09-18　**状态**：✅ 完成（代码 + 运行验证通过，drone 7/7，全量 20/20）
**范围**：先跑通 point_mass 主线（含故障状态机）；真实动力学(quad/wing)桩化，后续 Phase 8/9 补。

### 新增文件（5）

| Python 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `core/__init__.py` | `@Drone/`（目录） | 导出 `Drone` |
| `core/drone.py` | `@Drone/Drone.m` | Drone 类（全属性 + 构造 + point_mass 故障机 + 电池；真实动力学/路径/绘图桩化） |
| `control/__init__.py` | `control/control_drone/`（目录） | 导出 `true_states` |
| `control/true_states.py` | `control/control_drone/true_states.m` | 伪状态估计（Drone.set_pos / estimate_states 依赖） |
| `tests/test_drone.py` | — | 7 项对照测试（构造/访问器/三态/drift 衰减/桩） |

### 修改 / 删除文件

- 无 MATLAB 改动。

### 关键设计决策

1. **Drone = 普通 Python class**（MATLAB `< handle` → 引用语义天然对应；符合计划 §4.1，
   未退化为 dict/ndarray）。
2. **向量约定**：vect3(pos_ned/vel_xyz/attitude/rates/drift_vel/forces/moments) → `np.ndarray (3,)`；
   command/delta → `(4,)`；z_hat → `(22,)`；airdata → `(6,)`；full_command → `(19,)`；
   history → `(n,3)`（每行一帧，对应 MATLAB `[h; v']`）。
3. **索引 1-based→0-based**（计划 §11）：`command(2:4)→command[1:4]`、`command(1)→command[0]`、
   `attitude(3)→attitude[2]`、`vel_xyz_history(end,:)→vel_xyz_history[-1,:]`。
4. **point_mass 故障状态机完整实现**（二次开发核心，等价覆盖计划 Phase 5 逻辑）：
   - `Normal`：`vel=command[1:4]`，`pos+=vel*dt`，`attitude[2]=command[0]`
   - `Stalled`：`vel=0`，位置不变
   - `Drifting`：首次记录 `drift_vel=vel_xyz_history[-1]`（`drift_initialized` 门控），
     此后 `vel=fad*drift_vel`；非 Drifting 时 `drift_initialized` 复位
   - `fad` 由主循环传入（Phase 4/示例迁移时复刻 `1/(1+0.2Δt)` 衰减）
5. **true_states 一并迁移**：`set_pos`/`estimate_states`(use_estimation=False) 依赖它，
   纯状态映射、无动力学依赖；其 z_hat 布局与 estimate_states 的差异（§14 B-11）各自忠实保留。
6. **桩化并标注 Phase**：真实动力学分支、`compute_dynamics`/`compute_kinematics`（Phase 8）、
   `autopilot_*`（Phase 9）、`sensors`/`gps`/`estimate_states` 的 use_estimation=True 路径（Phase 10）、
   `plan_path`/`manage_path`/`follow_path`（Phase 11）、`plot_state`（Phase 12）→ 抛 `NotImplementedError`。
7. **忠实保留 MATLAB 可疑写法**（只记录不修，计划 §2.3）：
   - `compute_Qt_hover` 的 `self.B.*`（B-7，无 B 属性 → 调用即 AttributeError，死代码）
   - `update_actuators` 末尾把 5 维 `i` 赋成 4 维（死代码）
   - `set_state`：二次开发版已修正原版的双 self bug，按修正后行为迁移
8. **color**：构造函数用 `_jet(16)` 近似调色板随机取一行（仅可视化用，Phase 12 用 matplotlib 'jet' 校准）。

### 测试结果（`C:\Python\python.exe`，Python 3.14.7）

```
$ python SwarmLab_py/tests/test_drone.py
[PASS] test_construction                 [PASS] test_set_pos_vel_get_state
[PASS] test_normal_update                [PASS] test_stalled_update
[PASS] test_drifting_update_and_decay    [PASS] test_drift_initialized_reset_on_normal
[PASS] test_real_dynamics_stub
Total: 7  Passed: 7  Failed: 0
全量回归：enums 4/4 + parameters 9/9 + drone 7/7 = 20/20 PASS
```

手算对照抽样：Normal 两步后 `pos=[0.04,0,0]`；切 Drifting(fad=0.5) `drift_vel=[2,0,0]`、
`vel=[1,0,0]`、`pos=[0.05,0,0]`；再 fad=0.25 → `vel=[0.5,0,0]`、`pos=[0.055,0,0]`；fad=0 → 趋停。

### 已知问题 / 待办

- **KI-8**：`Drifting` 首步需 `vel_xyz_history` 非空（否则 `[-1,:]` 越界）——与 MATLAB
  `vel_xyz_history(end,:)` 对空矩阵报错行为一致；实际流程 Normal 先跑若干步，故不触发。
- **KI-7（沿用）**：暂无 MATLAB 数值快照做逐元素 diff；point_mass 主线已用手算期望值充分验证。
- 待 Phase 4：`Swarm` 类（`add_drone`/`set_dronestate`/`update_command`/`update_state(fad)`/
  `get_barycenter` 仅健康机）+ 主循环 `fad` 衰减与故障注入的端到端联调。


---

## Phase 4 —— Swarm 类

**日期**：2026-09-18　**状态**：✅ 完成（代码 + 运行验证通过，swarm 15/15，全量 35/35）
**范围**：Swarm 管理 + 故障注入 + 状态传播；`update_command` 的算法调用桩化（Phase 6/7）。

### 新增文件（4）/ 修改（1）

| Python 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `core/swarm.py`（新增） | `@Swarm/Swarm.m` | Swarm 类（管理 + set_dronestate + update_state(fad) + get_barycenter + getters；算法/绘图桩） |
| `math_tools/rb2i.py`（新增） | `math_tools/Rb2i.m` | 机体→NED 旋转矩阵（get_vel_ned 依赖） |
| `math_tools/__init__.py`（新增） | `math_tools/`（目录） | 导出 Rb2i |
| `tests/test_swarm.py`（新增） | — | 15 项对照测试 |
| `core/__init__.py`（修改） | — | 增导出 `Swarm` |

- 无 MATLAB 改动。

### 关键设计决策

1. **Swarm = 普通 class**；`drones` 用 Python `list`（保持 Drone 引用语义）；
   `swarm_state` 用 `list[DroneState]`（对应 MATLAB `(:,1) DroneState` 列向量）。
2. **set_dronestate 故障注入**：`randperm(N,ceil(loss_radio*N))` → `np.random.permutation(N)[:k]`；
   先全置 Normal，再按 Stalled/Drifting 覆盖，最后同步 `drones[i].drone_state`。
3. **update_state(wind,time,fad)** 逐机传播到 Drone 的 point_mass 故障机（Phase 3 已实现）。
4. **get_barycenter 仅统计 Normal 健康机**，`equivalent_drone` 用 `drones[0]` 的类型/参数构造。
5. **update_command** 保留分派结构与 `collisions_history` 累加，算法体桩化（Phase 6/7）。
6. **Rb2i 忠实照搬矩阵**（含原文件"body/inertial"注释与实际相反之处，不改动，§2.3）。

### 忠实保留的 MATLAB bug（只记录 + 测试锁定行为，不修）

- **B-1** `add_n_drones` 单参调用 `add_drone`（需 7 参）→ TypeError（test_bug_B1 锁定）
- **B-2** `set_vel`/`set_state` 用未定义 `self.nb_drones` → AttributeError（test_bug_B2 锁定）
- **B-3** `path_manager_individual` 调 `drone.path_manager_wing`（无此方法）
- **B-4** `path_follower_individual` 调 `follow_path(time)`（Drone.follow_path 无参）
- **B-5** `get_barycenter` 不清零 `normal_drone`，多次调用累加（test_..._accumulates_B5 锁定：2 次调用后 normal_drone=4，质心被错误除以 4）
- **B-6** `draw_agents_energy`/`record_state` MATLAB 声明但无 .m 文件 → 桩 NotImplementedError

### 测试结果（`C:\Python\python.exe`，Python 3.14.7）

```
$ python SwarmLab_py/tests/test_swarm.py   → 15/15 PASS
  construction / add_drone / set_pos·get_pos_ned / get_vel_ned /
  set_dronestate(Stalled 计数=2, Drifting 计数=3, Normal 复位) /
  update_state 传播 + 故障机不动 / get_barycenter 仅健康机 / B-5 累加 /
  getters 形状 / update_command 桩 / B-2 / B-1
全量回归：enums 4 + parameters 9 + drone 7 + swarm 15 = 35/35 PASS
```

抽样手算对照：N=10、loss_radio=0.2 → `ceil(0.2*10)=2` 架 Stalled；0.3 → 3 架 Drifting；
2 机（d0@[0,0,0] Normal、d1@[10,0,0] Stalled）→ 质心=[0,0,0]（仅 d0）。

### 已知问题 / 待办

- **KI-9**：`update_command` 需 Phase 6/7 的 `compute_vel_vasarhelyi`/`compute_vel_olfati_saber`
  才能真正产出速度指令与碰撞统计；届时补端到端联调。
- **KI-10**：主循环里的 `fad = 1/(1+0.2*(t-t_fault))`（含 <0.1→0 截断）与故障触发/GUI 交互
  属于 example/仿真 runner 层，待算法迁移后（Phase 6/7 之后）作为“仿真入口”一并复刻。
- **进度提示**：原计划 **Phase 5（point_mass: normal/stalled/drifting/drone_state/fad）已在
  Phase 3 内完成并由 test_drone 充分验证**，故下一步直接进入 **Phase 6（Vasarhelyi）**。


---

## Phase 6 —— Vasarhelyi 算法

**日期**：2026-09-18　**状态**：✅ 完成（代码 + 运行验证通过，vasarhelyi 9/9，全量 44/44）

> 说明：原计划 Phase 5（point_mass）已在 Phase 3 内完成，故本阶段为原计划 **Phase 6**。

### 新增文件（3）/ 修改（1）

| Python 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `swarm/__init__.py`（新增） | `@Swarm/`（算法部分） | 群体算法包，导出 compute_vel_vasarhelyi/get_v_max |
| `swarm/vasarhelyi.py`（新增） | `@Swarm/compute_vel_vasarhelyi.m`（424 行） | Vasarhelyi 算法 + get_v_max 制动曲线 |
| `tests/test_vasarhelyi.py`（新增） | — | 9 项对照测试 |
| `core/swarm.py`（修改） | — | `Swarm.compute_vel_vasarhelyi` 由桩改为委托到 `swarm/vasarhelyi.py` |

- 无 MATLAB 改动。

### 关键设计决策

1. **`swarm/` 独立包**（计划 §15）：算法实现为**以 Swarm 实例为首参的模块函数**
   （`compute_vel_vasarhelyi(swarm, p_swarm, r_agent, dt)`），精确对应 MATLAB `@Swarm` 外部
   方法的 `self`；`core/swarm.py` 的同名方法仅做委托。
2. **列/行向量适配（§13）**：`u_ref/x_goal` 等 (3,1) 列向量在算法内 `ravel()` 成 (3,)，
   避免 numpy 广播成 (3,3)。
3. **`isfield`/`isempty` 语义映射**：`isfield(p_swarm,'c_r')`(MATLAB 恒假) → `c_r is not None`；
   `~isempty(max_v/max_a)` → `is not None`（None 表示 MATLAB 的 `[]`）。
4. **故障机指令顺序忠实复刻**：`vel_command[:,agent]=[0,0,0]` 后**仍叠加** migration/goal/自推进项
   （MATLAB line 314-344 顺序）；该项最终在 `Drone.update_state` 被故障运动学覆盖。
5. **故障邻居处理**（二次开发核心）：`p_fault=3*p_rep`、`r_safe=3*r0_rep` 线性排斥 +
   `d_hard=6*r_agent` 硬安全区 `emergency=300*p_fault*(d_hard-d)^2/d_hard`。
6. **NaN 行为一致**：`p_rel_u/v_rel_u` 自身列为 0/0=nan，但自身不在 neig_list、等速邻居
   `v_rel_norm=0` 不满足 `>v_fric_max`，故 nan 永不进入结果（用 `np.errstate` 抑制告警）。
   `get_v_max` 的 `sqrt(负)` → nan（`np.sqrt` + errstate），与 MATLAB 一致。
7. **忠实保留可疑写法**：加速度限幅的 `| accel_cmd_norm < -max_a`（范数非负，恒 False）照搬。

### 测试结果（`C:\Python\python.exe`，Python 3.14.7）

```
$ python SwarmLab_py/tests/test_vasarhelyi.py   → 9/9 PASS
  get_v_max(4 例) / migration=[6,0,0] / 正常排斥=[-0.45,0,0] /
  故障邻居增强排斥=[-5.85,0,0](>正常) / 故障机指令(migration关=[0,0,0], 开=[6,0,0]) /
  碰撞计数=1 / 速度限幅→[3,0,0] / 加速度限幅→[0.1,0,0] / update_command 写入+history 累加
全量回归：enums 4 + parameters 9 + drone 7 + swarm 15 + vasarhelyi 9 = 44/44 PASS
```

手算对照抽样：正常邻居 `p_rep*(r0_rep-d)=0.03*(25-10)=0.45`；故障邻居
`3*p_rep*(3*r0_rep-d)=0.09*(75-10)=5.85`（13× 增强）；加速度限幅 `dt*max_a=0.01*10=0.1`。

### 缺口状态（经确认延后）

- **缺口 A（单元测试覆盖不全）**、**缺口 B（§17 MATLAB/Python 数值快照 diff）**：
  用户已确认**暂时搁置**，作为后续“测试补强”统一回补。当前验证 = 忠实转录 + 独立手算期望值 + 行为断言。

### 已知问题 / 待办

- **KI-11**：`compute_vel_olfati_saber` 仍为桩（Phase 7）；`test_swarm.test_update_command_stub`
  以 olfati_saber 触发桩、仍有效。
- 下一步 **Phase 7 —— Olfati-Saber**：sigma-norm 势函数 `psi/phi`、邻接 `rho_f/rho_der_f`、
  位置/速度匹配、故障邻居（增强排斥 + 侧向滑移 + `is_stuck` 主动逃逸 `rear_blocked/escape_strength`）、
  迁移、障碍/墙、`repulsion_cubic_arena`；⚠️ 注意 §14 B-8（`isempty(max_v/max_a)` 限幅死代码，须忠实保留）。


---

## Phase 7 —— Olfati-Saber 算法

**日期**：2026-09-18　**状态**：✅ 完成（代码 + 运行验证通过，olfati 8/8，全量 52/52）

### 新增文件（2）/ 修改（3）

| Python 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `swarm/olfati_saber.py`（新增） | `@Swarm/compute_vel_olfati_saber.m`（511 行） | Olfati-Saber + `rho_f`/`rho_der_f`/`repulsion_cubic_arena` |
| `tests/test_olfati_saber.py`（新增） | — | 8 项对照测试 |
| `swarm/__init__.py`（修改） | — | 增导出 compute_vel_olfati_saber |
| `core/swarm.py`（修改） | — | `Swarm.compute_vel_olfati_saber` 由桩改为委托 |
| `tests/test_swarm.py`（修改） | — | `test_update_command_stub` → `test_update_command_dispatch`（两算法均可跑通） |

- 无 MATLAB 改动。

### 关键设计决策

1. **势函数为闭包**：`psi/psi_der/phi` 定义为捕获 `p_swarm` 的嵌套函数（对应 MATLAB 匿名函数
   `@(z)...`）；`rho_f/rho_der_f/repulsion_cubic_arena` 为模块级函数（对应 MATLAB subfunctions）。
2. **加速度积分**：`vel_command = vel + acc_command*dt`（区别于 Vasarhelyi 的速度叠加）。
3. **坐标翻转**：入口 `pos/vel` 的 z 取反（NED down→up），出口 `vel_command` z 再取反回 NED。
4. **迁移项覆盖 acc_vm**：`acc_vm[:,agent] = c_vm*(u_ref*v_ref - vel[:,agent])`（是 `=` 覆盖，
   且在邻居块之外，对所有 agent 生效）。
5. **`lambda`→`lambda_`**（KI-3）：障碍物虚拟目标点 `pos_gamma = c_obs + lambda_*u_ref`。
6. **`mod(agent,2)` 1-based → `(agent+1)%2`**（方案2 逃逸的左右分流，计划 §11）。
7. **`acos` 数值安全**：`dot(-fault_dir,u_ref)` 理论 ∈[-1,1]，对浮点溢出做 clip（等价 MATLAB
   acos 定义域，非逻辑改动）。

### 忠实保留的 MATLAB bug / 可疑写法（只记录 + 测试锁定）

- **B-8**：`if isempty(max_v)`/`isempty(max_a)`（应为 `~isempty`）→ Python `is None`；因默认
  为数值故限幅分支**恒不执行**（死代码）。**Olfati 因此不做速度/加速度限幅**（与 Vasarhelyi 不同）。
  `test_no_bounding_B8` 锁定：即便 max_v=0.01，指令仍为 0.18（未被限幅）。
- **B-14（新发现）**：line 152 `acc_vm(:,agent) += c_vm*vel_rel(agent2)`，`vel_rel(agent2)` 是
  MATLAB **线性索引**（取 3×N 矩阵第 agent2 个元素=标量），疑似应为 `vel_rel(:,agent2)`。忠实照搬
  （列主序展平取值）；仅在 `~is_active_migration` 时触发（默认 migration 开→不触发）。
- 方案2 的 `if ~rear_blocked` 在 MATLAB 中被注释 → **逃逸恒触发**（`rear_blocked` 仅计算不使用），照搬。

### 测试结果（`C:\Python\python.exe`，Python 3.14.7）

```
$ python SwarmLab_py/tests/test_olfati_saber.py   → 8/8 PASS
  rho_f / rho_der_f（分段精确值）/ migration 加速度积分=[0.18,0,0] / B-8 不限幅 /
  正常位置匹配=psi_der(10)*2*dt（独立重算）/ 故障邻居增强排斥+侧向滑移(+y) /
  is_stuck 主动逃逸(stuck→后退 vel_x<0；not-stuck→迁移 vel_x>0)/ 碰撞计数=1 + 端到端
全量回归：enums 4 + parameters 9 + drone 7 + swarm 15 + vasarhelyi 9 + olfati 8 = 52/52 PASS
```

手算对照抽样：`rho_f(90)=0.25`；migration `c_vm*v_ref*dt=3*6*0.01=0.18`；正常位置匹配
`phi(10)=psi_der(10)≈-4.9925`（rho=1,rho_der=0）；故障方案1 `repulsion_mag=0.5*(75-10)=32.5`。

### 里程碑

- ✅ **两大群体算法（Vasarhelyi + Olfati-Saber）+ Drone/Swarm + point_mass 故障机 全部迁移完毕**。
  故障研究的**核心组件层已齐全**；尚缺"仿真主循环 runner"把组件串成端到端可运行仿真（KI-10）。

### 已知问题 / 待办

- **KI-10（下一步建议）**：迁移 `example_olfati_saber.m`/`example_vasarhelyi.m` 的**主循环**
  （脱离 GUI 独立运行，计划 §14）：装配参数 → 建群 → set_pos → 循环{update_command → 计算
  `fad=1/(1+0.2*(t-t_fault))`(含<0.1→0) → update_state(fad)} → 历史/性能。这是让**主线端到端跑通**的最后一环。
- 之后按原计划 Phase 8（Physics）→ 9（Control）→ 10（Estimation）→ 11（Path）→ 12（Viz）→ 13（GUI）补真实动力学与外设。


---

## Stage 1 —— 主线仿真 runner（原 KI-10）

**日期**：2026-09-18　**状态**：✅ 完成（simulation 10/10，全量 62/62，示例端到端跑通）

> **路线调整（经用户确认）**：采用 5 阶段路线 —— Stage1 runner → Stage2 可视化 → Stage3 GUI →
> Stage4 真实动力学（后期重点，补 MATLAB 数值对照=缺口B）→ Stage5 路径规划。
> 先做主线产品（point_mass+故障+GUI），真实动力学后期精心完善。

### 新增文件（4）

| Python 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `simulation/runner.py` | `example_olfati_saber.m`/`example_vasarhelyi.m` 主循环 | `Simulation` 引擎（step/run 双模式 + 故障注入 + fad） |
| `simulation/__init__.py` | — | 导出 `Simulation`/`FaultEvent` |
| `examples/run_swarm_fault.py` | `example_*.m` | headless 可运行示例（命令行） |
| `tests/test_simulation.py` | — | 10 项端到端测试 |

### 关键设计决策

1. **`Simulation` 类 step()/run() 双模式**：headless 批量运行 + GUI 逐步驱动；`run(on_step,should_stop)`
   预留可视化/GUI 钩子（Stage 2/3 复用）。
2. **`FaultEvent` 时间表驱动故障注入**（替代 GUI 下拉框）：到点触发 `set_dronestate`，Drifting 记 `t_fault`。
3. **fad 忠实复刻**：`1/(1+0.2*(t-t_fault))`，截断 [0,1]，<0.1→0；`t_fault==0` 时不更新（保留原值）。
4. **每步顺序**：故障注入 → `update_command` → 算 fad → `update_state(fad)`（与 MATLAB 主循环一致）。
5. **时间网格** `start+i*dt` 匹配 MATLAB `start:dt:end`；`wind=zeros(6)`（群体示例恒 0）。

### 测试 + 示例结果

- `test_simulation` **10/10**：迁移运动 / Stalled 冻结 / Drifting fad 公式(1/1.2,1/1.4) / 速度衰减 /
  fad 归零(t>45s) / 状态历史 / step 模式 / 可复现 / 双算法 / 历史形状。
- 示例 `run_swarm_fault.py`（10 机 / 80s / 14 圆柱障碍 / 迁移）：**碰撞=0**；
  fad 抽样 t=45→0.5000、t=60→0.2000（=1/(1+0.2Δt) 精确吻合）；t=70 恢复 Normal→10/10 健康。

### 已知问题 / 差异

- **KI-12**：`example_olfati_saber.m` 无 GUI 时 `DRONE_STATE` 未定义（MATLAB 两示例不一致：
  vasarhelyi 有 `DRONE_STATE="Normal"` 默认，olfati 没有）；runner 用显式初始 Normal 规避。
- 故障"选哪几架"用 `np.random.permutation`（全局 RNG，不可复现），与 MATLAB `randperm` 一致；
  初始位置 `Pos0` 由 `seed` 控制、可复现。
- **KI-10 已解决**（runner 完成，主线端到端可运行）。

### 里程碑

- ✅ **主线端到端跑通**：参数 → 建群 → 故障注入 → 双算法速度指令 → point_mass 三态运动学 →
  fad 衰减 → 历史/碰撞，全程 headless 可运行、可复现、可测试。

### 下一步（Stage 2 可视化）

- matplotlib 迁移 `SwarmViewer`（在线）+ 离线轨迹/性能图 + **故障状态 ○/□/△ 着色**；
  `math_tools/compute_swarm_performance`（safety/order/union/alg_conn/safety_obs/min_d_obs）。
- 需安装 matplotlib（届时提醒）。


---

## Stage 2 —— 可视化 + 性能指标（matplotlib 3.11.2）

**日期**：2026-09-18　**状态**：✅ 完成（performance 9/9 + visualization 4/4，全量 75/75；demo 出图 12 张）

### 新增文件

| Python 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `math_tools/pos2dist.py` | `pos2dist.m` | 位置→两两距离（pdist/squareform） |
| `math_tools/compute_neighborhood.py` | `compute_neighborhood.m` | 距离→邻接矩阵（含 B-16） |
| `math_tools/network_components.py` | `network_components.m` | 连通分量（Larremore） |
| `math_tools/compute_swarm_performance.py` | `compute_swarm_performance.m` | 6 指标（pdist2→cdist、eig→eigvalsh） |
| `visualization/map_drawing.py` | `draw_buildings.m`/`draw_cylinders.m` | 长方体/棱柱障碍（Poly3DCollection） |
| `visualization/swarm_viewer.py` | `SwarmViewer.m` | 在线 3D + 故障 ○/□/△ + 环境首帧绘制 |
| `visualization/offline_plots.py` | `plot_trajectories_offline.m`/`plot_swarm_performance.m` | 轨迹图 + 性能/碰撞/障碍距离图 |
| `examples/run_swarm_viz.py` | example 出图部分 | headless demo，输出 12 张 PNG |
| `tests/test_performance.py` / `test_visualization.py` | — | 9 + 4 项测试 |

### 关键设计决策

1. **坐标 NED→ENU**：plot x=east(pe)、y=north(pn)、z=up(-pd)；轨迹图俯视（MATLAB `view(2)`）。
2. **故障状态 marker**：Normal='o'、Stalled='s'、Drifting='^'（对应 SwarmViewer.m 二次开发）。
3. **环境首帧绘制一次**，此后每帧仅更新散点（对应 `plot_initialized` 节流逻辑）。
4. **headless**：Agg 后端 + `save_frame()`；Stage 3 GUI 复用 `SwarmViewer`。
5. 映射：`pdist2→cdist`、`eig→numpy.linalg.eigvalsh`（对称拉普拉斯升序，取 [1]=Fiedler）。

### 忠实保留的 bug

- **B-15**：球形障碍 `min_d_obs` 行 2D-pos 对 3D-球心维度不匹配（球形默认关闭=死代码，照搬）。
- **B-16**：`compute_neighborhood` 拓扑裁剪用循环索引 `agent` 而非 `to_cut(agent)`（仅 N>max_neig+1 触发）。

### 测试 + demo

- `test_performance` 9/9（6 指标手算对照）；`test_visualization` 4/4（Agg 存图）。全量 75/75。
- demo `run_swarm_viz.py`（10 机/60s/14 障碍，Stalled@15/Drifting@30/Normal@50）：**碰撞=0**，
  输出 `SwarmLab_py/examples/output/`：3 张故障状态帧 + trajectories + performance(+5 单项) +
  collision_history + obstacle_distance。

### 下一步（Stage 3 GUI）

- 桌面应用（推荐 PySide6 + matplotlib/pyqtgraph 画布）：控制面板（drone_type/state、loss_radio、
  nb_agents、d_ref/v_ref、orientation、wind、sim_time、启停）+ 实时视图，驱动 `Simulation.step()` + `SwarmViewer`。
- 需先定 GUI 框架（PySide6 vs Tkinter vs Web）。

> **更新（2026-09-18）**：GUI 框架经用户确认选 **Web（Streamlit + Plotly）**（非 PySide6）。见下方 Stage-3 记录。

---

## Stage 3（GUI）尝试与回滚 —— 记录（计划 §2.4 可追踪）

**日期**：2026-09-18　**状态**：↩️ 已回滚；代码库恢复到 Stage-2 基线（9 个测试文件 = 75/75 全绿）

- **经过**：开始 Stage-3 Streamlit GUI，新建 `gui/app.py` + `tests/test_gui.py`。AppTest 报
  `attempted relative import beyond top-level package`，我误判为“导入约定问题”，运行一次性脚本
  `fix_imports.py` 把 7 个源文件的相对导入 `from ..X` 批量改成裸绝对 `from X`，与本工程约定冲突，
  导致整包 import 失败（`No module named 'enums'`）。
- **根因（违反计划 §22/§26）**：改代码前未核实真实文件结构与导入约定，凭“有损对话摘要”臆断；
  且一度误判为“外部工具删除文件”。实为：`physics/`、`turbulent_wind_generator/`、
  `math_tools/{wrap_to_pi,sigma_norm,limit_speed,limit_acceleration,determine_sensing_area}.py`
  **从未存在**——相关 helper（sigma_norm/rho_f/wrap_to_pi/limit_* 等）**内联**在两个算法文件里，
  障碍距离计算**内联**在 `compute_swarm_performance` 里。
- **处置**：行首锚定正则把 7 文件 13 处导入改回 `from ..X`；删除 `gui/`、`tests/test_gui.py`；
  重跑 9 个测试文件 = **75/75**。无任何源文件丢失。
- **现状**：代码库 = Stage-2 干净基线；Stage-3 GUI 已按**正确约定**（附录 A.0）重做完成，见下一节。

---

## Stage 3 —— GUI 重做完成（最终框架：PySide6 桌面）

**日期**：2026-09-18　**状态**：✅ 完成（`test_gui` PySide6 offscreen 1/1；全量 **76/76**）
**框架演进**：先试 Web（Streamlit+Plotly）→ 用户提供**原版 GUI_swarm 面板截图**与**运行中实时控制**需求后，
改为 **PySide6 桌面**（`gui/desktop_app.py`）；Streamlit 版 `gui/app.py` 已删除（git `3a229b2` 可找回）。
严格对照附录 A 的接口与导入约定。

### 新增/修改文件

| Python 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `gui/desktop_app.py`（新增） | `examples/GUI_swarm.mlapp` | PySide6 桌面 GUI：复刻原版面板 + 内嵌 matplotlib 3D + 运行中实时控制 |
| `gui/__init__.py`（改） | — | 包说明 + 启动命令（desktop_app） |
| `tests/test_gui.py`（改） | — | PySide6 offscreen headless 测试 |
| `gui/app.py`（删除） | — | Streamlit 版（舍弃） |

### 面板复刻（对照原版截图）

- 左列「类型选择」：无人机类型(质点型) / 算法(Vasarhelyi|Olfati-Saber) / 无人机状态(正常|动力锁死|失控漂移) /
  失控比例滑杆(0-60) / 显示地图障碍 / 绘制调试视图。
- 右列「集群参数设置」：数量(spin) / 间距滑杆[m](5-15) / 速度滑杆[m/s](2-8) / 仿真时长 /
  **前进方向罗盘(180=N，自绘可拖拽旋钮)**。
- 底部「开始仿真」**拨动开关**（自绘 pill，关闭/开启）。

### 实时控制（用户要求；不预设故障/恢复时间）

- 运行中**每步**读取：罗盘(u_ref)、间距(d_ref，含 Vasarhelyi `r0_rep`)、速度(v_ref)、失控比例、无人机状态
  （`set_dronestate` 即时生效；切回 Normal 全体恢复）。
- 仅 数量/算法/地图/时长 为开局设定（运行中禁用这些控件）。
- 引擎 `SimController` 逐步复刻 runner 主循环（update_command→fad→update_state），参数来自实时控件。

### 3D 视图（对照毕设演示视频抽帧对齐）

- 内嵌 matplotlib QtAgg；复用 `map_drawing.draw_buildings/draw_cylinders/set_map_axes`。
- 每机 jet 一色 + 状态 marker ○/□/△（与 SwarmViewer 一致）；**细彩线轨迹尾迹**（对齐视频观感）。
- 标题 `Simulation time: X seconds`（对齐视频）；坐标 NED→ENU。

### 测试结果

- `test_gui`（PySide6 offscreen）**1/1**：建窗 + 30 步 + Stalled 注入(2/4) + 切回恢复 +
  `_update_view`(尾迹)/`_update_debug` 渲染。
- 全量回归 **76/76**。
- 启动：`C:\Python\python.exe -m SwarmLab_py.gui.desktop_app`（依赖 `pip install PySide6`）。

### 已知点 / 待办

- GUI 仅覆盖 point_mass 主线；真实动力学（Stage 4 / Phase 8）完成后需增 drone_type 选择与相应视图。
- MATLAB/Python 数值对照（§17 / 缺口 B）仍待 Stage 4 统一回补。

---

## Stage 4 —— 真实动力学（Phase 8 物理 + Phase 9 控制 + 接通 + §17 对照）【完成】

**日期**：2026-09-19　**状态**：✅ 完成（quad+wing 真实动力学接通；§17 数值对照机器精度匹配，缺口B 关闭）

### Phase 8 —— physics（新增）

| 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `physics/__init__.py` | — | 导出 |
| `physics/forces_moments.py` | forces_moments_quad.m / _wing.m | 机体系力/力矩，out(12)=[F;M;Va;alpha;beta;wn;we;wd] |
| `physics/kinematics.py` | kinematics_ode_fun.m | 刚体 12 维导数（orientation 冻结=步初，供 solve_ivp/RK23≈ode23） |
| `tests/test_physics.py` | — | 13 项解析对照（悬停平衡 F=M=0、力矩符号、翼升力、刚体导数 vs Rb2i） |

### Phase 9 —— control（四旋翼，新增）

| 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `control/pid.py` | pid_loop.m | PID 回路 + 对称/非对称饱和；param 字典存 Drone.P_* |
| `control/autopilot_quad.py` | autopilot_quad.m | 4 版本 + hold 函数 + mixer；y=[delta(4);x_command(19)] |
| `control/autopilot_wing.py` | autopilot_wing.m | 固定翼（uavbook 主路径 + tuning/TECS）；y=[delta(4);x_command(19)] |
| `control/__init__.py`（改） | — | 导出 pid_loop/autopilot_quad/autopilot_wing/mixer |
| `tests/test_real_dynamics.py` | — | 5 项闭环（悬停保持/北向响应/delta 限幅/fixed_wing 可运行） |

### 接通 Drone 真实动力学（core/drone.py 改）

- `update_state` 真实分支：autopilot_quad → delta/full_command → compute_dynamics →
  compute_kinematics(RK23) → update_battery；point_mass 分支逻辑不变（改为 else）。
- `compute_dynamics`：forces_moments_quad/wing → forces/moments/airdata。
- `compute_kinematics`：solve_ivp(RK23, rtol=atol=1e-8) 积分 kinematics_ode_fun（orientation 冻结）。
- fixed_wing 接通 `autopilot_wing(self, 0, time)`；B-18 下坠忠实复现（2s: 50→43.3m），不修复。
- `tests/test_drone.py`：`test_real_dynamics_stub` → `test_real_dynamics_quad_active`。

### 记录的新潜在 bug

- **B-17**：MATLAB `pe_ve_hold` 签名缺 p_sim 却用 p_sim.dt（潜在未定义变量）；Python 补入 p_sim 参数使其可运行，语义不变。
- **B-18**：原版 fixed_wing 仿真最初几秒急速下坠并消失（原始团队 fixed_wing 代码不完善/严重 bug）；Python 忠实复现（2s: 50→43.3m），不修复。

### §17/§18 数值对照（已完成，缺口B 关闭）

- 参考数据：`matlab_ref/export_ref_swarm.m`（MATLAB 导出：无故障、rng(0)、导出 Pos0）。
- 对照：`tests/test_matlab_ref.py` 用同一 Pos0 跑 Python 无故障仿真，逐帧比 pos/vel（§18 max/mean 绝对误差）。
- 结果（3000 步、N=5、有障碍）：
  - olfati_saber：pos max|err| = 0.0，vel max|err| = 8.9e-16（机器精度）
  - vasarhelyi：pos max|err| = 0.0，vel max|err| = 0.0
- 结论：point_mass 主线两算法与 MATLAB **机器精度一致**。

### 测试

- 全量 **96/96**（含 physics 13、real_dynamics 5、matlab_ref 2、gui 1）。

### 待办

- （Stage 4 完成。后续：Stage 5 路径规划 / Phase 10 估计 / Phase 11 制导 / Phase 12 真实动力学可视化，按需。）

---

## Stage 5 —— 路径规划（Phase 11：RRT + path_manager + path_follower + 接通）【完成】

**日期**：2026-09-19　**状态**：✅ 完成（quad/point_mass 路径链路接通；wing 路径管理/跟踪按需待补）

### 新增文件

| 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `guidance/__init__.py` | — | 导出 |
| `guidance/path_planner.py` | plan_RRT.m | RRT + 子函数（generate_random_node/collision/points_along_path/down_at_NE/extend_tree/find_minimum_path/smooth_path）；plot_map 未迁移（Phase 12） |
| `guidance/path_manager.py` | path_manager_quad.m | path_manager_quad + fillet 圆角过渡；persistent→状态字典（Drone.pm_state，计划 §10） |
| `guidance/path_follower.py` | path_follower_quad.m | 路径跟踪→速度指令 [psi_c;vn_c;ve_c;vd_c] |
| `tests/test_guidance.py` | — | 14 项（RRT 无碰撞/起终点、manager nb=0/fillet、follower、Drone 链路） |

### 接通 Drone（core/drone.py 改）

- `plan_path`：fillet/dubins 硬编码航点；straight_line_rrt→plan_RRT；city_dubins / 5 → NotImplementedError（plan_RRT_dubins/plan_cover_RRT_dubins 缺失，B-10）。
- `manage_path`：quad/point_mass→path_manager_quad（pm_state 持久状态）；wing 待迁移。
- `follow_path`：quad/point_mass→path_follower_quad；wing 待迁移。
- 新增 `Drone.pm_state`（path_manager persistent 状态）。

### 记录的新潜在 bug / 忠实保留

- **B-19**：find_minimum_path 回溯跳过第一级父节点（MATLAB 原逻辑），忠实保留。
- path_follower_quad 的 `psi_c = chi_c`（MATLAB 注释“不准确”，忠实保留）。

### 测试

- 全量 **110/110**（含 guidance 14）。

### 待办

- wing 路径管理/跟踪（path_manager_wing / path_follower_wing）按需。

---

## Stage 6 —— 状态估计（Phase 10：sensors + gps + lp_filter + estimate_states + 接通）【完成】

**日期**：2026-09-19　**状态**：✅ 完成（estimation 32/32；全量 **142/142**）
**授权**：用户明确要求——对破坏预期功能的重大 MATLAB bug，定位根因 + 理解意图 + 修复 + 记录（非忠实复现）。见记忆《SwarmLab_py 重大功能性 MATLAB bug 的修复授权与流程》。

### 新增文件

| 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `physics/sensors.py` | physics/sensors.m | IMU+气压计测量 y(8)（normrnd→np.random.normal）；含 B-21 修复 |
| `physics/gps.py` | physics/gps.m | GPS 测量 y(5)；persistent Markov `mp_n/e/h`→状态字典（`Drone.gps_state`） |
| `estimation/lp_filter.py` | estimation/lp_filter.m | 一阶低通公式 + `lp_step`（带各通道记忆，B-20 修复） |
| `estimation/estimate_states.py` | estimation/estimate_states.m | LP + 2 状态(phi/theta) EKF；persistent `x/kf_P`→`Drone.est_state`；B-11/B-22 修复 |
| `estimation/__init__.py` | — | 导出 lp_filter/lp_step/estimate_states/new_est_state |
| `tests/test_estimation.py` | — | 32 项（零噪声手算 + 4 bug 修复行为 + Drone 接线 + EKF 稳定性） |

### 接通 Drone（core/drone.py 改）

- `update_sensor_measurements`：`use_estimation=True` → `y=[sensors();gps()]`（forces/airdata 用上一步值，一步滞后，同 MATLAB）。
- `estimate_states`：`use_estimation=True` → `estimate_states(y,...)`；`False` → `true_states`（不变）。
- 新增 `Drone.gps_state` / `Drone.est_state`（persistent→状态字典，计划 §10）。

### 修复的重大 bug（计划 §2.3 用户授权：根因 / 意图 / 修复 / 影响）

- **B-11（修复）**：`estimate_states` 的 z_hat 布局与 `true_states`/自驾仪索引不一致（原把 chi 放 idx11、p/q/r 放 12-14、psi 放 18）。根因=两套排序约定漂移；意图=与 `true_states` 统一规范布局。修复：按 `[pn,pe,h,vx,vy,vz,Va,alpha,beta,phi,theta,psi,chi,p,q,r,Vg,wn,we,bx,by,bz]` 输出。影响：`use_estimation=True` 时自驾仪读到正确槽位（否则速度=0、角速率/航向错位）。
- **B-22（修复）**：EKF 更新门限 `norm(z-gravity)<0.5`（把标量 g 从每个加速度分量减）几乎永不触发→phi/theta 只预测不更新、姿态估计开环漂移。意图=加速度模≈g 的低动态判据。修复：`abs(norm(z)-gravity)<0.5`。影响：量测更新正常触发、姿态收敛。
- **B-20（修复）**：`lp_filter(y_prev,u,a,dt)` 无记忆，但调用传 `(测量,0,a,dt)`→退化成 `exp(-a·dt)·测量`。根因=签名重构后调用方未同步。意图=带记忆一阶低通。修复：Estimator 各通道存 prev，按 `(prev,u,a,dt)` 调用（`lp_step`），首次以 u 初始化。
- **B-21（修复）**：`sensors` 的 `y_accel_z` 误用 `fx/mass`（应 `fz/mass`），上游 EPFL 原版遗留（fault 版与原版逐字节相同）。意图=z 轴加速度计测 z 向比力。修复：改用 `fz`。影响：动态下 phi/theta EKF 量测正确。

### 估计器适用边界（重要，忠实记录）

- `estimate_states` 本质是**固定翼风格**估计器：估计 `{pn,pe,h,Va,phi,theta,chi,p,q,r,Vg}`；**不估计机体速度 vx/vy/vz 与偏航 psi**（原版即置 0；传感器套件无磁力计，yaw 不可观测）。
- **fixed_wing**：闭环验证 z_hat 全有限、h/Va/chi 估计正确（50 步 `h_est≈50.0`、`Va_est≈13.9`、`chi_est≈0`）。
- **quadcopter**：姿态/高度可用，但 (1) `vx/vy/vz=0`、`psi=0`（速度环/偏航无估计反馈）；(2) 近零空速时动压被噪声压负→`Vahat=NaN`（idx6，忠实对应 MATLAB `sqrt(负)`=复数）；**quad 自驾仪不读 idx6**，NaN 不入控制回路（20 步闭环 `h_est≈50.7` vs `h_true≈49.8`，可运行）。→ quad 全状态高保真需另做**增强估计器**（速度可由 GPS vn/ve 重构、yaw 需磁力计），属新开发，未纳入本次迁移。

### 测试

- `test_estimation` **32/32**；全量 **142/142**（原 110 + 估计 32）。
- 零噪声手算：sensors `accel_z`(B-21)、gps 直通 `[pn,pe,-pd,Vg,course]`、`h`=静压反解=50、`Va`=动压反解=15、`p/q/r`@13/14/15；`estimate_states`↔`true_states` 槽位一致(B-11)；B-22 触发时 `trace(kf_P)` 收缩、跳过时=Q；EKF 水平飞行 500 步有界。

### 待办

- Stage 7（wing 路径 `path_manager_wing`/`path_follower_wing` + `path_manager_dubins` + **B-18 固定翼下坠修复**）、Stage 8（Phase 12 真实动力学可视化 DroneViewer/plot_state）。

---

## Stage 7 —— 固定翼路径 + B-18/B-11 修复（Phase 11-wing）【完成】

**日期**：2026-09-19　**状态**：✅ 完成（path_wing 27/27 + real_dynamics 9/9；全量 **173/173**）
**授权**：承 Stage 6，对重大功能性 bug 定位根因+理解意图+修复+记录。

### 新增文件

| 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `guidance/dubins.py` | compute_dubins_param.m | Dubins 参数（RSR/RSL/LSR/LSL 取最短）+ Rz + wrapTo2Pi；返回 dict |
| `guidance/path_manager_wing.py` | path_manager_wing.m + path_manager_fillet.m + path_manager_dubins.m | 固定翼路径管理（nb0 直线 / fillet / dubins 6状态机）；out(30)；persistent→状态字典 |
| `tests/test_path_wing.py` | — | 27 项（dubins 几何 / 三分支 / follower 直线·orbit / B-11 chi@12 / wing 闭环不下坠） |

### 修改文件

| 文件 | 改动 |
|---|---|
| `guidance/path_follower.py` | 新增 `path_follower_wing`（直线/orbit→[Va_c,h_c,chi_c,phi_ff]）；quad chi 索引 @8→@12（B-11） |
| `guidance/path_manager.py` | quad nb0 分支 chi @8→@12（B-11） |
| `guidance/__init__.py` | 导出 wing 组件 + compute_dubins_param |
| `core/drone.py` | **B-18**：构造时 fixed_wing 调 `_init_fixed_wing_trim()`；manage_path/follow_path 接通 fixed_wing 分支 |
| `tests/test_real_dynamics.py` | `test_fixed_wing_runs`→`test_fixed_wing_trim_flight`（断言平飞不下坠） |

### 修复的重大 bug（根因 / 意图 / 修复 / 影响）

- **B-18（修复）固定翼下坠消失**：根因=trim 初始条件（xyu_T）虽已算入 `p_drone.*_T`，但原版从未应用到 drone 状态；`example_controller` 仅 `set_pos`（vel=0/delta=0），`generate_command` 给 wing 命令 `Va_c=0`（B-23）→ 零空速/零油门起步、升力≈0 → 急速下坠。意图=以配平空速 `Va_T=35`、配平姿态、配平舵偏（`de_T=-0.049`/`dt_T=0.4555`）平飞。修复：Drone 构造时 fixed_wing 置 vel/attitude/rates/delta 到 trim。验证：3s `h` 稳在 49.8（掉<0.4m）、`Va→35.0`、`delta` 自动收敛到 `[de_T,dt_T]`（autopilot PID 积分器自然收敛，**无需 trim 前馈**）。
- **B-11（修复·完整）z_hat 布局统一**：Stage 6 已修 `estimate_states`；本阶段发现 **guidance 亦用 Beard-McLain 布局**（chi@state[8]、Va@3），而 `true_states`/autopilot 用 canonical（chi@12、Va@6）→ path_manager/path_follower（quad+wing）读 chi 错位（读到 beta）。修复：统一 guidance 到 canonical，chi 索引 @8→@12（pn/pe/h@0/1/2 两布局一致，无需改）；含回修 Stage 5 已迁移的 quad guidance（test_guidance 14/14 无回归）。验证：`z_hat[12]=π/2 → path q=[0,1,0]`。

### 记录（example 层，未迁移）

- **B-23**：`generate_command.m` 给 fixed_wing 的命令 `[0 100 0.26 0]` → `Va_c=0`（应≈Va_T）。属 example 层（未迁移）；Python wing 命令由 `path_follower_wing` 产出（`Va_c=Va_d` 合理），故不影响。记录备查。

### 关键实现说明

- wing path 向量长 **30**（`[flag,Va_d,r(3),q(3),c(3),rho,lambda,state(16),flag_new]`），与 quad 的 **33**（多 r_next）不同——两套 MATLAB 文件布局本就不同，各自忠实。
- `path_manager_dubins` 用 `size_waypoint_array` 解析、`start_of_simulation` 块用 `size_wpt_array`（两者=100，忠实照搬）；`idx_a/idx_b/fillet.idx` 存 0-based。
- dubins 忠实保留原逻辑（含 case4 `flag_first_time_in_state=0` 先行导致 `==1` 分支恒不触发）。

### 测试

- `test_path_wing` **27/27**；`test_real_dynamics` **9/9**（含 B-18 平飞断言）；全量 **173/173**。

### 里程碑

- ✅ **quad + fixed_wing 两型真实动力学全链路可用**：物理→控制→估计(use_estimation)→路径(规划/管理/跟踪)。B-18 固定翼平飞、B-11 z_hat 全模块（estimation+guidance）统一到 canonical。

### 待办

- Stage 8（Phase 12 真实动力学可视化：DroneViewer quad/wing 机体 3D + plot_state 曲线）。

---

## Stage 8 —— 真实动力学可视化（Phase 12：DroneViewer + plot_state）【完成】

**日期**：2026-09-19　**状态**：✅ 完成（drone_viz 23/23；全量 **196/196**）

### 新增文件

| 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `visualization/drone_body.py` | define_quad_body.m / define_wing_body.m / rotate.m / translate.m | 机体几何（quad 21顶点9面 / wing 10顶点3面，0-based 面索引）+ 刚体 rotate/translate |
| `visualization/drone_viewer.py` | graphics_drone/DroneViewer.m | 单机在线 3D：rotate→translate→NED→ENU→Poly3DCollection；point_mass 散点；center_view 跟随；view(32,47) |
| `visualization/drone_state_plot.py` | @Drone/plot_state.m | DroneStatePlotter：7 组（Position/Velocity/Acceleration/Air data/Attitude/Angle rates/Actuators）real/est/cmd 时序曲线；uitab→GridSpec；persistent 句柄→Line2D |
| `examples/run_drone_viz.py` | example_controller.m 可视化部分 | demo：quad 悬停 + wing 配平平飞出图（4 张 PNG） |
| `tests/test_drone_viz.py` | — | 23 项（几何/变换/viewer/plotter headless 存图） |

### 修改文件

| 文件 | 改动 |
|---|---|
| `visualization/__init__.py` | 导出 DroneViewer/DroneStatePlotter/define_body/rotate/translate |

### 关键实现说明

- NED→ENU 统一：plot x=east, y=north, z=up（与 SwarmViewer/map_drawing 一致）；DroneViewer 用 `R=[0,1,0;1,0,0;0,0,-1]`。
- DroneStatePlotter 的 z_hat 用 canonical 布局（B-11 修复后）；full_command(19)/delta 按类型（wing de/da/dr/dt，quad d1-d4）。
- MATLAB uitab（7 标签页）→ matplotlib 单 figure GridSpec（每组一行，ncols=4）；headless Agg 存图。
- 未迁移（按需）：draw_waypoints/draw_path（graphics_map）、SwarmViewerOffline（回放）、VideoWriterWithRate（视频）。

### 测试 + demo

- `test_drone_viz` **23/23**；全量 **196/196**。
- demo `run_drone_viz.py`：quad 悬停 3s h=49.2m；wing 配平平飞 3s h=49.8m/Va=35（B-18 修复后不下坠）；出 4 张 PNG 到 `examples/output/`。

### 里程碑

- ✅ **迁移主线全部阶段完成**（Phase 0-12 + Stage 1-8）：枚举/参数/Drone/Swarm/point_mass 故障/双算法/runner/群体可视化/GUI/真实动力学(物理+控制+估计)/路径(quad+wing)/真实动力学可视化。两型真实动力学全链路 + 可视化齐备。

### 待办（可选收尾）

- SwarmViewerOffline 回放、draw_waypoints/draw_path、视频录制（VideoWriterWithRate）按需。

---

## Stage 9 —— 路径可视化 + 视频录制（供汇报/回顾）【完成】

**日期**：2026-09-19　**状态**：✅ 完成（path_video 18/18；全量 **214/214**）
**范围**：用户明确需要路径可视化 + 视频录制（汇报/回顾）；SwarmViewerOffline 回放**不需要**（跳过）。

### 新增文件

| 文件 | 对应 MATLAB | 说明 |
|---|---|---|
| `visualization/path_drawing.py` | draw_waypoints.m / draw_path.m + points_along_dubins_path | 航点连线（fillet 直连 / dubins 密集点）+ 当前 path 段（直线/orbit）；含 B-24 修复 |
| `visualization/video_writer.py` | VideoWriterWithRate.m | figure 按 output_rate 节流录 mp4（matplotlib FFMpegWriter，自动探测 ffmpeg） |
| `examples/run_drone_video.py` | example_path_manager 可视化+录制 | demo：wing 沿 Dubins 路径飞 + 全局路径视图 + 录 mp4 |
| `tests/test_path_video.py` | — | 18 项（waypoints/path/dubins 点/B-24/路径层/mp4） |

### 修改文件

| 文件 | 改动 |
|---|---|
| `visualization/drone_viewer.py` | 叠加路径层：每帧 draw_waypoints + draw_path（`wpt_handle`/`path_handle`，`draw_path_layer` 开关） |
| `visualization/__init__.py` | 导出 draw_waypoints/draw_path/points_along_dubins_path/VideoWriterWithRate |

### 修复的 bug

- **B-24（修复）**：`draw_path.m` 的 `c/rho/lambda` 索引按 wing(30) 布局（c@8:11,rho@11），对 quad(33) 布局（c@11:14,rho@14）会错位读到 r_next。意图=画当前 path 段 orbit 圆心/半径。修复：按 `len(path)` 区分布局取正确索引。验证：quad33 设干扰 r_next=999 时圆心仍取真 c@11:14。

### 视频后端

- 无 imageio；用 matplotlib `FFMpegWriter`（mp4/h264）+ 自动探测 ffmpeg（`C:\APP\ffmpeg\ffmpeg_full\bin\ffmpeg.exe` 或 PATH）。

### 测试 + demo

- `test_path_video` **18/18**；全量 **214/214**。
- demo `run_drone_video.py`：wing 沿 Dubins 10s 飞至 (350,-10)/h=70/Va=35；出 `drone_path_flight.mp4`（99 帧）+ `.png`（建筑+蓝航点线+红 path 段）。

### 里程碑

- ✅ 汇报/回顾素材齐备：群体仿真图/视频（Stage 2/3）、单机真实动力学机体 3D + 状态曲线（Stage 8）、路径飞行视频（Stage 9）。

---

## Stage 10 —— 结果目录 result/（群体仿真视频 + debug 参数图集中存放）【完成】

**日期**：2026-09-19　**状态**：✅ 完成（result_dir 9/9；全量 **223/223**）
**需求**：用户要求在 `SwarmLab_py/result/` 下按 MATLAB 惯例（`results/results_swarm/<alg>_<ts>/`）
集中存放群体仿真视频 + debug 参数图，便于后续挑选/对比多次运行。

### 新增文件

| 文件 | 说明 |
|---|---|
| `simulation/result_dir.py` | `RESULT_ROOT`=SwarmLab_py/result；`new_result_dir(algorithm, base, tag)` 建 `<alg>_<yyyy_mm_dd_HH_MM_SS>[_tag]/` |
| `result/README.md` | result/ 用途 + 子目录命名 + 每次运行产物清单 |
| `examples/run_swarm_result.py` | demo：群体故障仿真 → result/<alg>_<ts>/ 存 swarm_simulation.mp4 + 三态帧 + 轨迹 + 6 性能图 + 碰撞/障碍图 + performance.npz |
| `tests/test_result_dir.py` | 9 项（RESULT_ROOT 位置 / 命名含算法+时间戳+tag / 唯一性） |

### 修改文件

| 文件 | 改动 |
|---|---|
| `simulation/__init__.py` | 导出 `new_result_dir` / `RESULT_ROOT` |

### 验证

- `test_result_dir` **9/9**；全量 **223/223**。
- demo 实跑：`result/olfati_saber_2026_09_19_21_07_27_fault30/` 生成 14 文件
  （swarm_simulation.mp4 + frame_1/2/3 + trajectories + performance*×7 + collision_history +
  obstacle_distance + performance.npz），碰撞=0。

---

## Stage 11 —— GUI 每次仿真自动记录到 result/（含交互视角视频）【完成】

**日期**：2026-09-19　**状态**：✅ 完成（gui_result 9/9；全量 **232/232**）
**需求**：仿真在 GUI 进行；**每次仿真自动**记录到 result/<alg>_<ts>/；视频**完整记录用户对仿真
画面的视角旋转/缩放/平移**（录用户实际看到的 3D 交互画面，非固定视角渲染）。

### 新增/修改

| 文件 | 改动 |
|---|---|
| `visualization/video_writer.py` | 新增 `FrameVideoWriter`：numpy 帧(H,W,3)→ffmpeg rawvideo 管道→mp4（录 GUI 像素帧） |
| `gui/desktop_app.py` | 开始仿真→`new_result_dir`+启动 10fps 抓帧定时器；`_capture_frame` 抓 3D 视图 widget（`QWidget.grab()`，pyvista/matplotlib 通用，含用户相机操作）；`_finalize_run` 停止时关视频+`_save_result_plots` 存轨迹/6性能图/碰撞/障碍/npz；`SimController.vel_history` |
| `visualization/__init__.py` | 导出 `FrameVideoWriter` |
| `tests/test_gui_result.py` | 9 项（FrameVideoWriter mp4 + GUI 自动记录全链路） |
| `tests/test_gui.py` | 测试后清理其 result 目录（防回归污染） |

### 修复的实现 bug

- ffmpeg 参数 `-c v libx264` → `-c:v libx264`（原写法 ffmpeg 立即退出、帧写不进）。
- PySide6 `QImage.bits()` 返回 memoryview（无 `setsize`）→ 改用 `bytesPerLine` 处理 padding 后 `np.frombuffer`。

### 验证

- `test_gui_result` **9/9**：开始→建 result 目录+启动录制；抓帧>0；停止→mp4 非空 + trajectories/performance/npz 齐全。
- 全量 **232/232**。

---

## Stage 12 —— GUI 集群中心高度滑杆 + 方案1防撞（高度锁H+纯水平避让）【完成】

**日期**：2026-09-19　**状态**：✅ 完成（test_gui 含高度保持/无人车断言 1/1；全量 **232/232**）
**需求**：(1) 竖直滑杆设定集群中心目标高度 H（刻度 0~障碍最高+50m），高度保持自动收敛、
无需频繁操作；H=0 → 贴地行驶 = **无人车模式**。(2) 低空/近楼高时避让故障机可能撞地/撞楼
（算法垂直方向无地面/建筑约束）→ 用户拍板**方案1**：高度锁H + 避让纯水平化。

### 修改

| 文件 | 改动 |
|---|---|
| `gui/desktop_app.py` | 竖直滑杆「集群中心高度[m]（0=地面/无人车）」0~(max_height+50)，开局同步初始中心高；`SimController.h_target`；`step` 方案C：避让中(近故障机)算法z自由(三维避障)、避让后中心保持拉回H+个体分散(鸟群)、个体下限安全兜底；`_tick` 实时读滑杆 |
| `tests/test_gui.py` | 高度断言：中心收敛(±3m) + 个体分散 std>0.5(非扁平) + **避让后自动恢复高度** + h_target=0 贴地(无人车) |

### 高度控制 + 防撞机制（方案C：条件式高度恢复，迭代后）

- **避让中**（健康机近故障机 < r_avoid=3·d_ref）：算法 z **完全自由** → 保留**三维避障**（各方向绕开，含上下）。
- **避让后/无故障邻居**：中心高度保持 c=clamp(0.5*(H-h_center),±3) 把集群中心拉回滑杆高度 H
  → **自动解决“避让后停低空不恢复”**（无需手动拉滑杆）；个体 z = (算法z_i - 非避让均值) - c，
  保留个体相对高低差异（鸟群感，非扁平）。
- **个体下限安全兜底（防撞地/楼顶）**：个体低于安全高度（飞行时地面2m；水平在建筑 footprint 内则
  楼顶+2m）时向上推（覆盖）。H=0（无人车）不启用下限（贴地即地面，水平靠圆柱避障防楼）。
- 迭代记录：① 误做“垂直速度滑杆”→纠正为“目标高度滑杆”；② 锁每架个体到 H→扁平化→纠正为“只控中心+保留分散”；
  ③ 扣全体均值把群体共同避让 z 也扣除→失去三维避障→纠正为**方案C（避让中自由/避让后中心保持）**。

### 验证

- `test_gui` 1/1（中心收敛 + 个体分散 std>0.5 + **避让后自动恢复高度** + 无人车贴地 + 故障注入）；全量 **232/232**。

---

## 附录 A —— Python 接口基线（权威参照）

> 目的：避免后续开发（Stage-3 GUI、Stage-4 真实动力学等）接口不对齐。以下为**逐文件核实**的
> 已迁移代码接口快照（非摘要臆断）。任何阶段开发前先对照本节。

### A.0 导入约定（★ 上次出错点）

- `SwarmLab_py` 是**根包**（含 `__init__.py`）；入口把其**父目录** `c:\SwarmLab`（`_REPO_ROOT`）加入 `sys.path`。
- **源码内部**：相对导入 `from ..enums import ...` / `from .drone import ...`。
- **tests / examples /（将来）gui**：绝对导入 `from SwarmLab_py.simulation import Simulation`。
- ⚠️ 源码里**不要**用裸绝对 `from enums import ...`（根包约定下解析不到 → `ModuleNotFoundError`）。

### A.1 模块完成度

| 模块 | 状态 | 说明 |
|---|---|---|
| `enums/` | ✅ | 5 个 `(str,Enum)` |
| `parameters/` | ✅ | dataclass + 工厂 + `ParamsBundle` |
| `core/drone.py` | ✅ point_mass | 真实动力学/路径/绘图方法为桩 |
| `core/swarm.py` | ✅ | 管理 + 故障注入 + 指令分派 + 历史 |
| `swarm/{olfati_saber,vasarhelyi}.py` | ✅ | 含内联子函数 |
| `math_tools/` | ✅ | Rb2i/pos2dist/compute_neighborhood/network_components/compute_swarm_performance |
| `control/true_states.py` | ✅ | 伪真值估计（z_hat 22） |
| `simulation/runner.py` | ✅ | `Simulation` + `FaultEvent`（主线引擎） |
| `simulation/result_dir.py` | ✅ | `new_result_dir`/`RESULT_ROOT`（result/<alg>_<ts>/，Stage 10） |
| `visualization/` | ✅ | `SwarmViewer` + `map_drawing` + `offline_plots` + `DroneViewer`/`DroneStatePlotter`/`drone_body`（Stage 2/8）+ `path_drawing`/`video_writer`(含 FrameVideoWriter)（Stage 9/11） |
| `gui/desktop_app.py` | ✅ | PySide6 桌面 GUI（Stage 3）+ 每次仿真自动记录 result/ + 交互视角视频（Stage 11）+ 集群中心高度滑杆+方案1防撞（Stage 12） |
| `physics/` | ✅ | forces_moments_quad/wing + kinematics + sensors + gps（Stage 4/6） |
| `control/` | ✅ | true_states + pid + autopilot_quad/wing（Stage 4） |
| `guidance/` | ✅ | plan_RRT + path_manager_quad/wing + path_follower_quad/wing + dubins（Stage 5/7） |
| `estimation/` | ✅ | lp_filter + estimate_states（LP+2状态EKF，Stage 6；B-11/20/21/22 已修） |
| 桩（`NotImplementedError`） | ⏳ | plot_state·record_video（P12）；path_manager_wing/path_follower_wing |

### A.2 关键接口签名（核实版）

```python
# enums（均 (str,Enum)，可直接 == "Normal"）
DroneState{Normal,Stalled,Drifting}  DroneType{quadcopter,fixed_wing,point_mass}
SwarmAlgorithm{vasarhelyi,olfati_saber}  PathType{fillet,dubins,straight_rrt}
SwarmViewerType{drone,agent,agent_with_energy}

# parameters
build_params(drone_type, algorithm=None, *, active_environment=True, end_time=None,
             nb_agents=None, d_ref=None, v_ref=None, loss_radio=None, u_ref=None,
             seed=None) -> ParamsBundle
ParamsBundle(p_sim, p_battery, p_physics, p_drone, map: MapParams, p_swarm=None)
#  SwarmParams 关键字段: nb_agents=5,r=150,loss_radio=0.2,max_neig=10,r_coll=0.5,
#    d_ref=25,v_ref=6,u_ref(3,1),max_a=10,max_v=7,is_active_{migration,goal,arena,spheres,cyl},
#    cylinders(3,n),n_cyl,Pos0(3,N),x_arena,spheres
#  OlfatiSaberParams: c_vm,a,b,c(派生),delta,k,r0,lambda_,c_pm_obs,c_vm_obs(派生)
#  VasarhelyiParams : r0_rep(派生=d_ref),p_rep,r0_fric,C_fric,v_fric,p_fric,a_fric,*_shill
#  MapParams: width=300(fixed_wing=4000),max_height=100,ACTIVE_ENVIRONMENT,building_shape,
#             building_width,buildings_north/east/heights (N,1)   ← 建筑数据在此

# core.Drone（7 参构造；无 get_buildings）
Drone(drone_type, drone_state, p_drone, p_battery, p_sim, p_physics, map_params)
#  属性: pos_ned(3),vel_xyz(3),drift_vel(3),attitude(3),rates(3),pos_ned_history(n,3),
#        vel_xyz_history(n,3),command(4),z_hat(22),color(3),drone_state,map,drift_initialized
update_state(wind, time, fad)   # point_mass 三态: Normal/Stalled/Drifting(drift_vel*fad)
set_pos(pos_ned) / set_vel(vel) / get_state()->12 / set_state(pos,vel,att,rates) / estimate_states(time)

# core.Swarm（无参构造，随后 swarm.algorithm = "olfati_saber"）
Swarm()
#  属性: drones[],nb_agents,algorithm,swarm_state[](DroneState),normal_drone,t_fault,fad,
#        collisions_history(T,3),equivalent_drone
add_drone(drone_type, drone_state, p_drone, p_battery, p_sim, p_physics, map_params)
set_pos(pos 3×N) / get_pos_ned()->3×N / get_vel_ned()->3×N / get_colors()->3×N
set_dronestate(DRONE_STATE, DRONE_STATE_CHANGED, loss_radio)   # 1/2 触发注入
update_command(p_swarm, r_coll, dt) -> (vel_cmd 3×N, collisions[3])
update_state(wind, time, fad) / get_pos_ned_history()->(T,3N) / get_vel_xyz_history()->(T,3N)
get_barycenter()->Drone   # 仅 Normal 健康机

# swarm 算法（首参为 swarm；collisions=[机-机, 机-障, min_dist_obs]）
compute_vel_olfati_saber(swarm, p_swarm, r_agent, dt) -> (vel_command 3×N, collisions[3])
compute_vel_vasarhelyi (swarm, p_swarm, r_agent, dt) -> (vel_command 3×N, collisions[3])

# math_tools
Rb2i(phi, theta, psi) -> 3×3
pos2dist(positions) -> 距离矩阵
compute_neighborhood(distance_matrix, r_comm, max_neig) -> 邻接矩阵 M(N,N)
network_components(A) -> (n_comp, ...)
compute_swarm_performance(pos_history(T,3N), vel_history(T,3N), p_swarm, dirname=None)
    -> (safety, order, union, alg_conn, safety_obs, min_d_obs)   # 前5个(T,), min_d_obs(T,N)

# control
true_states(x) -> z_hat(22)   # x = [state(12); airdata(6); time]

# simulation（主线入口）
FaultEvent(time, state, loss_radio=None)   # state: "Normal"/"Stalled"/"Drifting"
Simulation(drone_type="point_mass", algorithm="olfati_saber", *, nb_agents, d_ref, v_ref,
           loss_radio, u_ref, orientation_deg, active_environment=True, end_time, seed,
           wind, fault_schedule)
#  属性: params(ParamsBundle),swarm,p_sim,p_swarm,dt,time,done,current_state(str),
#        collisions_sum(3),time_history[],fad_history[],state_history[](List[str])
step() -> bool
run(until=None, on_step=None, should_stop=None) -> Simulation
get_pos_history()->(T,3N) / get_vel_history()->(T,3N)

# visualization（from SwarmLab_py.visualization import ...）
SwarmViewer(output_rate=0.5, center_view_on_swarm=False, viewer_type="agent", figsize=(8,8), marker_size=60)
    .update(time, swarm, map_params) / .save_frame(path, dpi=100) / .close()
plot_trajectories(pos_history(T,3N), nb_agents, colors=None, map_params=None, save_path=None) -> Figure
plot_performance(time_history, safety, order, union, alg_conn, safety_obs, min_d_obs,
                 collisions_history=None, dirname=None) -> dict{name: Figure}
draw_buildings(ax, map_params, ...) / draw_cylinders(ax, map_params, ...) / set_map_axes(ax, map_params)
```

### A.3 易错点更正（相对上一版“有损摘要”的臆断）

| 错误假设 | 真实接口 |
|---|---|
| `Drone/Swarm.get_buildings()` | 无此方法；建筑在 `sim.params.map`（`MapParams.buildings_*`） |
| `compute_swarm_performance(pos_list, swarm, buildings=, env_shape=)` | `(pos_history, vel_history, p_swarm, dirname=None) -> 6 元组` |
| `sim.pos_history` / `sim.min_dist_overall` / `state_history[i]["per_drone"]` | `sim.get_pos_history()`；`state_history` 是 `List[str]`；碰撞在 `swarm.collisions_history(T,3)` / `sim.collisions_sum(3)` |
| `compute_neighborhood(agents, ...)` | `compute_neighborhood(distance_matrix, r_comm, max_neig)` |
| 源码用裸绝对 `from enums` | 源码用相对 `from ..enums`；入口用 `from SwarmLab_py.xxx` |

### A.4 忠实保留的 MATLAB bug（不改，只记录）

忠实保留（不改，只记录）：B-1…B-8、B-14、B-15、B-16、B-17（补 p_sim 使其可运行）、B-19（详见各 Phase 小节与 `PROJECT_ARCHITECTURE.md` §14）。

用户授权修复（重大功能性 bug，均记根因/意图/改动于对应 Stage 小节）：**B-11**（z_hat 布局统一到 canonical：Stage6 修 estimate_states、Stage7 修 guidance 的 chi@8→@12）、**B-18**（固定翼下坠→trim 初始化平飞，Stage7）、**B-20**（lp_filter 退化→带记忆真滤波）、**B-21**（accel_z 用 fz）、**B-22**（EKF 门限 `abs(‖z‖-g)`）。example 层未迁移但记录：**B-23**（generate_command 给 wing 的 Va_c=0）。
