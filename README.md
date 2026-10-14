# SwarmLab-Py：无人机群故障容错仿真平台（MATLAB → Python）

SwarmLab-Py 是将 **EPFL SwarmLab** 开源无人机/无人机群仿真框架的**二次开发版**（加入"无人机群成员失效/故障"机制）完整迁移到 Python 的工程。它既能运行**质点模型（point_mass）的群体故障容错仿真**（研究健康无人机如何避让/逃逸失效邻居），也保留了原版的**四旋翼 / 固定翼真实动力学全链路**（物理 → 控制 → 估计 → 路径规划/管理/跟踪），并提供 **PySide6 桌面 GUI、3D 可视化、视频录制与结果管理**。

> 迁移以"行为与算法可验证的等价实现"为目标：与 MATLAB 原版做了**机器精度数值对照**；对原版中破坏功能的重大 bug 做了**根因修复并全程记录**（见下文）。

---

## 一、这个项目是干什么的？

无人机群（swarm）在飞行中可能出现个体失效（动力锁死、失控漂移）。失效个体会变成"危险障碍物"，健康无人机需要**感知故障邻居、增强排斥、侧向滑移、主动逃逸**，同时保持群体凝聚与迁移。本项目提供一个**可运行、可可视化、可复现**的仿真平台来研究这一过程：

- **群体层**：Vásárhelyi 与 Olfati-Saber 两大群体算法 + 故障三态状态机 + 故障邻居处理。
- **个体层**：point_mass 质点模型（主线），以及 quad / fixed_wing 真实动力学（物理+控制+估计+路径）。
- **交互层**：桌面 GUI 实时调参/注入故障/调节集群高度/旋转缩放视角，并**自动记录每次仿真**（视频+参数图）。
- **分析层**：6 项群体性能指标、碰撞统计、轨迹/性能曲线、3D 机体渲染、mp4 视频。

---

## 二、核心功能

### 1. 群体故障仿真（point_mass 主线）
- **双群体算法**：`Vasarhelyi`（排斥/吸引 + 速度对齐 + 限幅）与 `Olfati-Saber`（sigma-norm 势函数 + 位置/速度匹配）。
- **故障三态**（二次开发核心）：`Normal`（正常）/ `Stalled`（动力锁死，速度归零）/ `Drifting`（失控漂移，记录故障前速度并按 `fad=1/(1+0.2·Δt)` 双曲衰减趋停）。
- **故障邻居处理**：增强排斥、硬安全区紧急排斥、Olfati 侧向滑移、`is_stuck` 主动逃逸（前后夹击时后退绕行）。
- **headless 仿真引擎**：`Simulation`（`step()`/`run()` 双模式）+ `FaultEvent` 故障注入时间表，可脱离 GUI 批量复现。

### 2. 真实动力学（quadcopter + fixed_wing）
- **物理**：`forces_moments_quad/wing`（机体系力/力矩/空气数据）+ `kinematics_ode_fun`（刚体 12 维，RK23 积分）。
- **控制**：`autopilot_quad`（attitude/velocity/acceleration/position 四版本 + 级联 PID + mixer + 饱和）、`autopilot_wing`（uavbook 主路径 + tuning/TECS）。
- **估计**：`sensors`（IMU+气压）、`gps`（Markov 噪声）、`estimate_states`（低通 + 2 状态 φ/θ EKF）；`use_estimation=True` 时自驾仪消费估计状态。
- **路径**：`plan_RRT`（RRT 规划）、`path_manager_quad/wing`（fillet 圆角 / Dubins 状态机）、`path_follower_quad/wing`（直线/orbit 跟踪）。
- **固定翼可平飞**：修复了原版"固定翼最初几秒急速下坠消失"的重大 bug（按配平 trim 初始化）。

### 3. 三维高度感知避障
- 无人机**高于建筑物顶部**时不再按低空逻辑绕行，可**直接飞越**（逐建筑物、基于实际几何判断，非集群统一高度）。
- 高空避让失效无人机时**禁止向下进入建筑物高度区域**（约束可行方向空间，水平/向上方向仍由算法自行计算，**不硬编码固定方向**）。
- **低空**保持原有避障逻辑不变（基准）。

### 4. 可视化与视频
- **群体**：`SwarmViewer`（matplotlib 3D，故障三态 marker ○/□/△）+ 轨迹图 + 6 项性能指标图 + 碰撞/障碍距离图。
- **单机**：`DroneViewer`（quad/wing 机体 3D 刚体渲染，NED→ENU）+ `DroneStatePlotter`（7 组状态曲线 real/estimated/command）。
- **路径**：`draw_waypoints`（fillet 直连 / Dubins 密集点）+ `draw_path`（直线/orbit 段）。
- **视频**：`VideoWriterWithRate` / `FrameVideoWriter`（ffmpeg 录 mp4；GUI 视频**完整记录用户视角旋转/缩放**）。

### 5. PySide6 桌面 GUI
- 复刻原版面板：无人机类型/算法/状态/失控比例/地图/数量/间距/速度/时长/前进方向罗盘/开始开关。
- **运行中实时控制**：方向、间距、速度、失控比例、故障状态即时生效。
- **3D 视图**：pyvista GPU（terrain 交互、预设视角、相机跟随/聚焦、滚轮缩放）或 matplotlib 回退。
- **集群中心高度滑杆**：平滑调节集群高度（自然鸟群聚散不被破坏）；`H=0` 切换**无人车模式**（贴地行驶）。
- **每次仿真自动记录**到 `result/<算法>_<时间戳>/`（交互视角视频 + 全部参数图）。

### 6. 结果管理 `result/`
- 沿用 MATLAB `results/results_swarm/` 惯例：每次运行一个带时间戳子目录，集中存放视频、关键帧、轨迹、性能图、`performance.npz`，便于挑选/对比多次运行。

---

## 三、快速开始

### 环境要求
- **Python ≥ 3.9**（开发/验证于 3.14）
- 依赖：
  ```bash
  pip install numpy scipy matplotlib PySide6 pyvista pyvistaqt
  ```
- **视频录制**需要 `ffmpeg`（自动探测 `C:\APP\ffmpeg\...\ffmpeg.exe` 或系统 PATH；无 ffmpeg 时视频功能自动降级、其余功能不受影响）。
- GPU 3D 视图依赖 pyvista/pyvistaqt；缺失时 GUI 自动回退 matplotlib 3D。

### 克隆与运行
```bash
git clone <your-repo-url> SwarmLab
cd SwarmLab
```
所有入口脚本会自动把仓库根加入 `sys.path`，**无需安装为包**即可运行。

### 运行示例（headless / 出图 / 视频）
```bash
# 1) 群体故障仿真（headless，10 机 / 80s / 故障注入时间表）
python SwarmLab_py/examples/run_swarm_fault.py

# 2) 群体仿真出图（关键帧 + 轨迹 + 性能指标）→ examples/output/
python SwarmLab_py/examples/run_swarm_viz.py [olfati_saber|vasarhelyi]

# 3) 群体仿真 → result/<算法>_<时间戳>/（交互视频 + 全部参数图）
python SwarmLab_py/examples/run_swarm_result.py [olfati_saber|vasarhelyi] [tag]

# 4) 单机真实动力学可视化（quad 悬停 + wing 平飞：机体3D + 状态曲线）
python SwarmLab_py/examples/run_drone_viz.py

# 5) 固定翼沿 Dubins 路径飞行 + 录 mp4（路径可视化）
python SwarmLab_py/examples/run_drone_video.py
```

### 启动桌面 GUI
```bash
python SwarmLab_py/gui/desktop_app.py
```
在 GUI 中：选择算法/状态/失控比例 → 拨动"开始仿真" → 运行中拖罗盘/滑杆实时控制 → 拖拽旋转、滚轮缩放 3D 视图 → 停止后结果自动存入 `SwarmLab_py/result/`。

### Python API 示例
```python
from SwarmLab_py.simulation import Simulation, FaultEvent

# headless 群体故障仿真
sim = Simulation(drone_type="point_mass", algorithm="olfati_saber",
                 nb_agents=10, active_environment=True, end_time=60.0, seed=0,
                 fault_schedule=[FaultEvent(15.0, "Stalled", 0.3),
                                 FaultEvent(30.0, "Drifting", 0.3),
                                 FaultEvent(50.0, "Normal")])
sim.run()
pos = sim.get_pos_history()          # (T, 3N)
vel = sim.get_vel_history()
print("累计碰撞 [机-机, 机-障, min_dist累计] =", sim.collisions_sum)

# 性能指标
from SwarmLab_py.math_tools import compute_swarm_performance
safety, order, union, alg_conn, safety_obs, min_d_obs = \
    compute_swarm_performance(pos, vel, sim.p_swarm, dirname="my_result_dir")
```

```python
# 单机真实动力学（固定翼 trim 平飞 + 路径跟踪）
from SwarmLab_py.core import Drone
from SwarmLab_py.enums import DroneType, DroneState
from SwarmLab_py.parameters import build_params

b = build_params("fixed_wing")
d = Drone(DroneType.FIXED_WING, DroneState.NORMAL, b.p_drone, b.p_battery,
          b.p_sim, b.p_physics, b.map)          # 构造时已按 trim 初始化（可平飞）
d.set_pos([0, 0, -50])
d.plan_path("dubins", 0.0)                       # 或 "fillet" / "straight_line_rrt"
for i in range(1000):
    t = i * b.p_sim.dt
    d.manage_path(t); d.follow_path()
    d.update_state([0]*6, t, 1.0)
```

---

## 四、目录结构

```
SwarmLab_py/
├── core/            Drone / Swarm 核心状态载体（故障三态、历史、电池）
├── enums/           5 个枚举（DroneType/DroneState/PathType/SwarmAlgorithm/SwarmViewerType）
├── parameters/      参数系统（dataclass + 工厂；sim/battery/physics/drone/fixed_wing/map/swarm/sensors）
├── swarm/           群体算法（vasarhelyi / olfati_saber）+ 三维高度感知（height_constraint）
├── physics/         真实动力学（forces_moments / kinematics / sensors / gps）
├── control/         控制（autopilot_quad / autopilot_wing / pid / true_states）
├── estimation/      估计（lp_filter / estimate_states）
├── guidance/        路径（path_planner RRT / path_manager quad+wing / path_follower / dubins）
├── math_tools/      数学/图论/性能指标（Rb2i / pos2dist / neighborhood / network / performance）
├── simulation/      主循环引擎（runner: Simulation/FaultEvent）+ 结果目录（result_dir）
├── visualization/   可视化（SwarmViewer / DroneViewer / DroneStatePlotter / map_drawing /
│                    path_drawing / offline_plots / video_writer）
├── gui/             PySide6 桌面 GUI（desktop_app）
├── examples/        可运行示例（run_swarm_fault / run_swarm_viz / run_swarm_result /
│                    run_drone_viz / run_drone_video）+ output/
├── result/          每次仿真的自动记录（<算法>_<时间戳>/：视频+参数图）
├── tests/           249 项对照/回归测试
└── MIGRATION_LOG.md 逐阶段迁移日志（修改/原因/对应/测试/已知问题）
```

---

## 五、测试

共 **249 项**测试，覆盖枚举/参数/Drone/Swarm/双算法/仿真引擎/性能指标/物理/控制/估计/路径/可视化/GUI/高度感知等：

```bash
# 单个测试
python SwarmLab_py/tests/test_swarm.py
# 全量（Windows PowerShell 示例）
Get-ChildItem SwarmLab_py/tests/test_*.py | ForEach-Object { python $_.FullName }
```

- **MATLAB 数值对照**：`tests/test_matlab_ref.py` 加载 `matlab_ref/*.mat`（由 MATLAB 导出），同初始位置/无故障逐帧对比 pos/vel，达到**机器精度一致**（验证迁移忠实性）。
- GUI/视频测试为 headless（offscreen + Agg），无需显示器。

---

## 六、与 MATLAB 原版的关系 & Bug 修复

- **忠实迁移原则**：优先保持 MATLAB 原始逻辑/注释/行为；1-based→0-based、persistent→状态字典、struct→dataclass 等按映射规范转换。
- **机器精度对照**：point_mass 主线两算法与 MATLAB 输出一致（`test_matlab_ref`）。
- **修复的重大功能性 bug**（根因+意图+记录，见 `MIGRATION_LOG.md`）：
  - `B-11` 估计/制导的 z_hat 布局与自驾仪索引不一致 → 统一到 canonical 布局。
  - `B-18` 固定翼最初几秒急速下坠消失 → 按配平 trim 初始化，可平飞。
  - `B-20` 低通滤波退化为直通 → 恢复带记忆一阶低通。
  - `B-21` 加速度计 z 轴误用 fx → 改用 fz。
  - `B-22` EKF 更新门限永不触发 → 改为加速度模判据。
  - `B-24` draw_path 的 orbit 索引不区分 quad/wing 布局 → 按长度区分。
- **保留记录、未修复**的原版 quirk：B-1…B-8、B-14…B-17、B-19 等（详见 `MIGRATION_LOG.md` 附录 A.4）。
- **行为开关**：`p_swarm.height_aware_cyl`（默认 `True` 启用三维高度感知；设 `False` 还原 MATLAB 原 2D 避障行为，用于对照测试）。

---

## 七、可能的扩展方向

以下模块当前**未迁移/未完成**，可作为后续扩展：

- `SwarmViewerOffline` 群体离线回放。
- 湍流风场 `get_wind` / `turbulent_wind_generator`（persistent 风场缓存）。
- 完整 EKF（当前为低通 + 2 状态 φ/θ EKF，固定翼风格；quad 全状态估计需增强）。
- 固定翼 TECS 纵向控制、`autopilot_wing` tuning 模式完善。
- 城市 Dubins / 覆盖路径规划（原版 MATLAB 缺失 `plan_RRT_dubins`/`plan_cover_RRT_dubins`）。
- 真实部署接口（ROS / PX4 / SITL）——若从仿真走向真机。
- 更多汇报素材自动化（批量多配置对比、自动出报告）。

---

## 八、引用与致谢

- 原始框架：**EPFL SwarmLab** — https://github.com/lis-epfl/swarmlab （LIS, EPFL）
- 本项目为其**加入无人机群失效/故障机制的二次开发版**的 Python 迁移。
- 群体算法参考：Vásárhelyi et al., *Science Robotics* 2018；Olfati-Saber 势函数方法；固定翼/控制参考 Beard & McLain, *Small Unmanned Aircraft*。
- 逐阶段迁移分析与日志见 `PROJECT_ARCHITECTURE.md`、`SwarmLab_MATLAB_to_Python_Migration_Plan.md`、`SwarmLab_py/MIGRATION_LOG.md`。

## License

本迁移代码供**学术研究/学习**使用；原始 SwarmLab 的许可条款见其仓库 LICENSE。如需商用或再分发，请遵循原版许可并保留出处。

---

**欢迎 issue / PR。** 若你在仿真中观察到异常行为，请先对照 `MIGRATION_LOG.md` 的"已记录 bug"列表，确认是否为原版已知 quirk 的忠实复现。
