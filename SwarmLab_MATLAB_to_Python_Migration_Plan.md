# SwarmLab MATLAB → Python 项目迁移计划

## 1. 项目目标

将当前基于 EPFL SwarmLab 开源项目二次开发的大型 MATLAB 仿真项目，逐步迁移为 Python 项目。

原始项目：
- GitHub：https://github.com/lis-epfl/swarmlab.git
- 定位：无人机/无人机群仿真框架
- 当前项目：在 SwarmLab 基础上进行了二次开发，加入了面向无人机群成员失效/故障状态的相关机制。

核心目标不是机械地进行 MATLAB → Python 语法替换，而是：

> 在保持原有软件架构、类之间调用关系、算法数学逻辑和仿真行为基本一致的前提下，将整个 MATLAB 项目逐步迁移为结构清晰、可维护、可验证的 Python 项目。

---

## 2. 总体工作原则

### 2.1 优先保证逻辑一致性

不要为了“Python 风格”擅自改变 MATLAB 算法。

优先级：

1. MATLAB 原始逻辑
2. 当前项目实际行为
3. Python 的合理实现
4. Python 风格优化

如果 Python 风格与 MATLAB 行为冲突，优先保证行为一致。

### 2.2 保留源代码注释

MATLAB `% comment` 转为 Python `# comment`。

原有注释应尽可能保留，包括：
- 注释内容
- 注释语义
- 注释所在代码附近的位置

不得因为重构而随意删除原注释。

### 2.3 不擅自修复 MATLAB Bug

如果发现：
- 属性名称不一致
- 参数数量不一致
- 方法调用接口不一致
- 未定义变量
- 疑似逻辑错误
- MATLAB 旧版本遗留代码

不得直接修改其语义。

应记录为 `Potential MATLAB inconsistency / potential bug`，注明文件、函数、当前行为、可能问题以及 Python 迁移时的处理方式。

只有明确要求时才进行修复。

### 2.4 修改必须可追踪

每个阶段记录：
- 修改/新增/删除文件
- 修改原因
- MATLAB 对应文件
- Python 对应文件
- 测试结果
- 已知问题

---

# 3. 项目整体架构

核心关系可抽象为：

```text
                         Example / GUI
                              │
                              ▼
                       Parameter Setup
                              │
                              ▼
                            Swarm
                              │
                    ┌─────────┴─────────┐
                    │                   │
                  Drone[]         Swarm Algorithm
                    │             ┌─────┴─────┐
                    │             │           │
                    │       Olfati-Saber  Vasarhelyi
                    │             │           │
                    └─────────────┴───────────┘
                                  │
                           Velocity Command
                                  │
                                  ▼
                         Drone.update_state()
                                  │
                    ┌─────────────┴─────────────┐
                    │                           │
                point_mass                 Real Dynamics
                    │                    ┌──────┴──────┐
                    │                    │             │
              Fault Model            Quadcopter    Fixed Wing
                    │                    │             │
                    │                 Control       Control
                    │                    │             │
                    │                 Physics       Physics
                    │                    │             │
                    └────────────────────┴─────────────┘
                                         │
                                         ▼
                                      History
                                         │
                               ┌─────────┴─────────┐
                               ▼                   ▼
                         Visualization          Analysis
```

---

# 4. 核心对象模型

## 4.1 Drone

`Drone` 是核心状态载体，负责：
- 保存无人机状态与参数
- 传感器
- 状态估计
- 自动驾驶仪
- 动力学
- 路径规划
- 路径跟踪
- 电池状态
- 状态历史
- 故障状态

Python 应继续保持：

```python
class Drone:
    ...
```

不要把 Drone 完全转换成 dictionary 或单纯 NumPy 数组。

## 4.2 Swarm

`Swarm` 是无人机群管理对象：

```text
Swarm
 ├── Drone 1
 ├── Drone 2
 ├── Drone 3
 └── Drone N
```

Python 建议保持：

```python
class Swarm:
    self.drones = [...]
```

NumPy 用于批量数值计算，不应完全取代 Drone 对象体系。

## 4.3 Handle Class

MATLAB 核心类使用 `< handle` 时，Python 普通 class 通常可以自然对应其引用语义。

仍需检查所有对象复制、赋值和状态修改行为。

---

# 5. 核心调用链

## 5.1 主仿真流程

```text
Parameter Setup
      │
      ▼
Create Swarm
      │
      ▼
Create Drone[]
      │
      ▼
Swarm.update_command()
      │
      ├── Vasarhelyi
      └── Olfati-Saber
      │
      ▼
Drone command
      │
      ▼
Swarm.update_state()
      │
      ▼
Drone.update_state()
      │
      ▼
State History
      │
      ▼
Visualization / Analysis
```

## 5.2 Drone.update_state()

真实动力学无人机：

```text
update_state()
   │
   ├── update_sensor_measurements()
   ├── estimate_states()
   ├── autopilot_quad() / autopilot_wing()
   ├── compute_dynamics()
   ├── compute_kinematics()
   └── update_battery()
```

`point_mass`：

```text
update_state()
   │
   ├── Normal
   │      └── 正常速度与位置更新
   ├── Stalled
   │      └── 速度归零/保持
   └── Drifting
          ├── 保存故障前速度
          ├── 根据 fad 衰减
          └── 更新位置
```

---

# 6. 当前项目的重要二次开发内容

## 6.1 drone_state

每架无人机通过 `drone_state` 区分正常和故障状态：

```text
drone_state
 ├── Normal
 ├── Stalled / 动力锁死
 └── Drifting / 失控漂移
```

必须保留。

## 6.2 Stalled

动力锁死时通常表现为速度归零/状态停止，具体行为必须以 MATLAB 源代码为准。

## 6.3 Drifting

失控漂移应：
1. 记录故障发生前速度
2. 保存 `drift_vel`
3. 根据 `fad` 等参数进行速度衰减
4. 持续更新位置
5. 最终趋近停止

不得简单替换为 `velocity = 0`。

## 6.4 Fault Neighbor

Swarm 算法会读取 Drone 的 `drone_state`：

```text
Drone State
     │
     ▼
Swarm Algorithm
     │
     ├── Normal Neighbor
     └── Fault Neighbor
              ├── Repulsion
              ├── Escape
              └── Blocking Detection
```

重点保护变量/机制：
- `has_fault_neighbor`
- `min_dist_to_fault`
- `fault_rear_count`
- `fault_front_count`
- `is_stuck`
- `rear_blocked`
- `escape_strength`

---

# 7. 算法模块

## 7.1 Olfati-Saber

必须保持：
- sigma-norm 等数学计算
- 邻居搜索
- 势函数
- 速度/加速度计算
- 故障邻居处理
- escape / fault handling 机制

## 7.2 Vasarhelyi

必须保持：
- 速度控制
- 加速度限制
- 邻居交互
- 避障逻辑
- MATLAB 参数定义

---

# 8. 参数系统

重点参数：
```text
p_sim
p_battery
p_physics
p_drone
map
p_swarm
```

Python 不应简单全部变成全局变量。

推荐逐步整理为：

```text
parameters/
├── simulation.py
├── battery.py
├── physics.py
├── drone.py
├── map.py
└── swarm.py
```

必要时使用 `dataclass`。

但必须先确认 MATLAB 参数之间的依赖关系。

---

# 9. MATLAB 特有结构转换

| MATLAB | Python 推荐 |
|---|---|
| `classdef` | `class` |
| `properties` | instance attributes / `@property` |
| `properties (Constant)` | class attributes |
| `properties (SetAccess=private)` | property/setter 或封装 |
| `methods (Static)` | `@staticmethod` |
| `enumeration` | `Enum` |
| `struct` | `dict` / `dataclass` |
| `table` | `pandas.DataFrame` |
| `cell` | `list` / tuple / object array，按实际用途 |
| matrix | `numpy.ndarray` |
| `persistent` | object state / dedicated state class |
| `global` | 显式配置对象或依赖注入 |
| MATLAB function | Python function / method |

---

# 10. persistent

发现：

```matlab
persistent variable
```

时不得简单写成局部变量。

应分析其持久化含义。优先考虑将其转换为对象状态，例如：

```python
class PathManager:
    def __init__(self):
        self.previous_waypoints = ...
        self.index = ...
        self.transition_flag = ...
```

---

# 11. MATLAB 索引

必须重点检查 1-based → 0-based：

```matlab
A(1)       → A[0]
A(:,1)     → A[:,0]
A(2:5)     → A[1:5]
```

所有 `1:n`、`end`、`:`、逻辑索引、矩阵行列索引都必须检查。

---

# 12. 路径规划调用链

```text
Drone.plan_path()
      │
      ▼
RRT
      │
      ├── generate_random_node()
      ├── collision()
      ├── extend_tree()
      ├── find_minimum_path()
      ├── smooth_path()
      └── points_along_path()
```

之后：

```text
waypoints
    │
    ▼
manage_path()
    │
    ├── path_manager_wing
    └── path_manager_quad
              └── path_manager_fillet_quad
```

最后：

```text
follow_path()
    ├── path_follower_wing
    └── path_follower_quad
```

---

# 13. Control / Physics / Estimation

## Control

四旋翼：
```text
autopilot_quad
 ├── attitude
 ├── velocity
 ├── acceleration
 ├── position
 ├── PID
 ├── mixer
 └── saturation
```

固定翼：
```text
autopilot_wing
 ├── autopilot_tuning
 ├── autopilot_uavbook
 ├── autopilot_TECS
 └── various hold controllers
```

## Estimation

```text
update_sensor_measurements()
        │
        ▼
sensors / GPS
        │
        ▼
estimate_states()
        │
        ├── EKF
        ├── EKF phi/theta
        └── LP filter
        │
        ▼
z_hat
```

## Physics

包括：
- quadcopter
- fixed wing
- point mass
- kinematics
- forces
- moments
- wind

---

# 14. Visualization / GUI

核心仿真必须先脱离 GUI 独立运行。

推荐架构：

```text
Core Simulation
      │
      ▼
History / State
      │
      ├── Visualization
      └── GUI
```

GUI 最后迁移。

---

# 15. 推荐 Python 项目结构

目标结构可以向以下方向演化：

```text
swarmlab_python/
│
├── core/
│   ├── drone.py
│   ├── swarm.py
│   └── state.py
│
├── enums/
│   ├── drone_state.py
│   ├── drone_type.py
│   ├── path_type.py
│   └── swarm_algorithm.py
│
├── parameters/
│   ├── simulation.py
│   ├── battery.py
│   ├── physics.py
│   ├── drone.py
│   ├── map.py
│   └── swarm.py
│
├── control/
├── estimation/
├── physics/
├── swarm/
│   ├── olfati_saber.py
│   └── vasarhelyi.py
│
├── guidance/
│   ├── path_planning/
│   ├── path_manager/
│   └── path_following/
│
├── math_tools/
├── visualization/
├── simulation/
└── examples/
```

这是目标方向，不要求第一阶段一次性完成。

---

# 16. 推荐迁移阶段

## Phase 0 — 项目分析

禁止修改代码。

任务：
1. 分析所有 MATLAB 文件
2. 找出所有 `classdef`
3. 分析继承关系
4. 分析类之间组合关系
5. 分析方法调用关系
6. 分析主仿真循环
7. 分析参数依赖
8. 分析 `persistent`
9. 分析 global/workspace 依赖
10. 分析 MATLAB 特有数据结构
11. 找出潜在接口不一致
12. 区分原始 SwarmLab 与当前二次开发
13. 输出 `PROJECT_ARCHITECTURE.md`

## Phase 1 — Enums / Constants

迁移：
- 枚举
- 常量
- 基础配置

## Phase 2 — Parameters

迁移：
- simulation
- physics
- battery
- drone
- map
- swarm

## Phase 3 — Drone

迁移：
- `Drone` class
- properties
- constructor
- state
- history
- `update_state`

先确保：

```python
drone = Drone(...)
```

可以正确创建。

## Phase 4 — Swarm

迁移：
- `Swarm`
- `Drone[]`
- `update_command`
- `update_state`
- neighbor logic
- collision

## Phase 5 — Point Mass

优先迁移：
- normal
- stalled
- drifting
- `drone_state`
- `fad`

## Phase 6 — Vasarhelyi

独立迁移和测试。

## Phase 7 — Olfati-Saber

独立迁移和测试。

重点验证：
- normal neighbor
- fault neighbor
- repulsion
- escape
- blocking
- velocity command

## Phase 8 — Physics

迁移：
- quadcopter
- fixed wing
- forces
- moments
- kinematics
- dynamics
- wind

## Phase 9 — Control

迁移：
- autopilot
- PID
- mixer
- saturation
- controllers

## Phase 10 — Estimation

迁移：
- sensors
- GPS
- EKF
- filters

## Phase 11 — Path Planning / Guidance

迁移：
- RRT
- path manager
- path follower

## Phase 12 — Visualization

迁移：
- online viewer
- offline viewer
- trajectory
- performance plots

## Phase 13 — GUI

最后迁移 GUI。

---

# 17. 测试策略

不能以“Python 程序没有报错”作为完成标准。

必须进行 MATLAB / Python 对照测试。

使用相同：
- initial position
- initial velocity
- initial state
- parameters
- time step
- simulation duration
- random seed（如适用）

比较：
- position(t)
- velocity(t)
- acceleration(t)
- command(t)
- drone_state(t)
- collision(t)
- swarm state

---

# 18. 数值验证

计算：

```text
error = Python result - MATLAB result
```

检查：
- max absolute error
- mean absolute error
- relative error

例如：

```text
max |position_matlab - position_python|
max |velocity_matlab - velocity_python|
max |command_matlab - command_python|
```

随机算法应尽量固定随机种子。

---

# 19. Git 工作原则

不要直接在唯一工作版本上进行大规模 Agent 修改。

建议：

```text
main
│
├── matlab-original
│
└── python-migration
```

每个阶段单独 commit，例如：

```text
Phase 1: Convert enums
Phase 2: Convert parameters
Phase 3: Convert Drone
Phase 4: Convert Swarm
...
```

每次 Agent 修改后检查 Diff。

---

# 20. Qoder + VS Code + ChatGPT 分工

## VS Code

主要开发环境：
- 查看代码
- Git
- Diff
- 运行 MATLAB
- 运行 Python
- 调试
- 测试

## Qoder

主要承担：
- 项目级搜索
- 依赖分析
- 多文件修改
- 批量转换
- 创建 Python 文件
- 执行测试
- 根据错误迭代修复

但不得自行改变核心架构或数学逻辑。

## ChatGPT

主要承担：
- 项目架构设计
- MATLAB → Python 映射方案
- 类设计
- 算法逻辑检查
- 数学公式一致性检查
- Qoder 工作计划设计
- Agent 输出审查
- MATLAB / Python 差异分析
- 关键代码复核

---

# 21. Qoder 第一阶段推荐 Prompt

不要立即转换代码。

```text
不要修改任何 MATLAB 源代码。

请完整分析当前 MATLAB 项目。

1. 找出所有 classdef。
2. 找出所有继承关系。
3. 找出所有类之间的组合关系。
4. 分析 Drone 与 Swarm 的调用关系。
5. 分析主仿真循环。
6. 分析 Olfati-Saber 和 Vasarhelyi 的调用链。
7. 分析 point_mass 的状态更新过程。
8. 分析故障状态 drone_state 的传播路径。
9. 分析路径规划、路径管理和路径跟踪。
10. 分析 control / estimation / physics。
11. 找出所有 persistent。
12. 找出所有 global 和 workspace 隐式依赖。
13. 找出 MATLAB 特有数据结构。
14. 找出可能存在的接口不一致。
15. 区分原始 SwarmLab 逻辑与当前项目二次开发逻辑。
16. 输出 PROJECT_ARCHITECTURE.md。

禁止修改 MATLAB 源文件。
```

---

# 22. Agent 行为准则

修改代码前必须：

1. 阅读相关源文件
2. 阅读调用方
3. 阅读被调用方
4. 确认数据类型
5. 确认输入输出
6. 确认 MATLAB 索引
7. 确认对象生命周期
8. 确认状态是否持久化
9. 确认是否涉及故障状态
10. 再进行修改

禁止只读取单个函数后进行跨项目重构。

---

# 23. 每次迁移任务标准流程

```text
读取 MATLAB 源码
      ↓
分析调用关系
      ↓
确定 Python 对应结构
      ↓
编写迁移计划
      ↓
修改代码
      ↓
静态检查
      ↓
运行 Python
      ↓
运行 MATLAB 对照实验
      ↓
数值比较
      ↓
记录结果
      ↓
Git commit
```

---

# 24. 特别需要关注的 MATLAB/Python 差异

重点检查：

- 1-based vs 0-based indexing
- matrix multiplication `*`
- element-wise multiplication `.*`
- element-wise division `./`
- power `.^`
- column vector / row vector
- implicit broadcasting
- `end`
- `:` slicing
- logical indexing
- `isempty`
- `size`
- `length`
- `numel`
- `find`
- `zeros`
- `ones`
- `reshape`
- `squeeze`
- `permute`
- `persistent`
- `global`
- `struct`
- `cell`
- `table`
- handle class
- value class
- enumeration
- nested functions
- anonymous functions
- MATLAB path/workspace behavior

---

# 25. 迁移成功标准

## 架构层面
- 类关系清晰
- Drone / Swarm 对象关系保持
- 模块职责清晰
- 不存在不必要的循环依赖

## 算法层面
- Olfati-Saber 逻辑保持
- Vasarhelyi 逻辑保持
- 路径规划逻辑保持
- point_mass 逻辑保持
- fault / drone_state 逻辑保持

## 数值层面
- 相同输入下 MATLAB 与 Python 输出基本一致
- 对误差进行定量验证

## 工程层面
- Python 可以独立运行
- 核心仿真不依赖 GUI
- 有基本测试
- 有 Git 历史
- 每个迁移阶段可追踪
- 源代码注释得到保留

---

# 26. 最终原则

本项目不是：

> “把 MATLAB 翻译成 Python。”

而是：

> **在理解原 SwarmLab 架构和当前二次开发逻辑的基础上，建立一个行为与算法可验证的 Python 等价实现。**

任何 Agent 如果无法确定某段 MATLAB 代码的真实含义，应优先：
1. 搜索调用关系；
2. 阅读上下游代码；
3. 标记不确定点；
4. 请求人工确认；

而不是自行猜测。

特别是涉及以下模块时必须谨慎：

```text
Drone
Swarm
Olfati-Saber
Vasarhelyi
drone_state
fault model
point_mass
path planning
```
