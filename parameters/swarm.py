"""SwarmParams 及其子类 —— 群体算法参数。

对应 MATLAB:
    parameters/param_swarm/param_swarm.m         （公共字段 + cylinders/Pos0 生成）
    parameters/param_swarm/param_olfati_saber.m  （Olfati-Saber 专属）
    parameters/param_swarm/param_vasarhelyi.m     （Vasarhelyi 专属）

设计（基类 + 继承子类）:
    - SwarmParams        : 公共字段（激活开关/nb_agents/r/max_neig/r_coll/d_ref/u_ref/v_ref/
                           max_a/max_v/loss_radio/arena/spheres/cylinders/Pos0/Vel0 …）
    - OlfatiSaberParams  : a,b,c,delta,k,c_vm,r0,lambda_,c_pm_obs,c_vm_obs
    - VasarhelyiParams   : p_rep,r0_rep,r0_fric,C_fric,v_fric,p_fric,a_fric,*_shill
    - 派生量（c、c_vm_obs、r0_rep、center_arena）在 __post_init__ 计算；
      cylinders（来自 map）与 Pos0/Vel0（随机）在 build_swarm_params 工厂计算。

⚠️ 命名差异（务必注意）:
    MATLAB 字段 ``p_swarm.lambda`` 是 Python 关键字，无法作标识符，
    此处重命名为 ``lambda_``。迁移 Olfati-Saber（Phase 7）时，凡 MATLAB 的
    ``p_swarm.lambda`` 一律对应 Python 的 ``p_swarm.lambda_``。

随机性说明（对应计划 §18）:
    MATLAB 用 ``rng('shuffle')`` + ``rand(3,N)`` 生成 Pos0，不可复现；且 MATLAB 与
    NumPy 的随机数算法不同，即使同种子 Pos0 也不一致。因此 MATLAB↔Python 数值对照时，
    初始位置必须显式传递（导出 MATLAB 的 Pos0 喂给 Python，或反之）。build_swarm_params
    的 seed 仅用于 Python 侧可复现。
"""

import math
from dataclasses import dataclass, field

import numpy as np

from ..enums import SwarmAlgorithm


@dataclass
class SwarmParams:
    """群体公共参数（param_swarm.m）。"""

    # ---- 激活开关 ----
    is_active_migration: bool = True    # 群体迁移（整体移动）
    is_active_goal: bool = False        # 目标点吸引
    is_active_arena: bool = False       # 竞技场边界排斥
    is_active_spheres: bool = False     # 球形障碍物
    is_active_cyl: bool = True          # 圆柱形障碍物

    # ---- 基本量 ----
    nb_agents: int = 5
    r: float = 150.0                    # 最大感知半径（公制距离）
    loss_radio: float = 0.2             # 无人机群失控比例（二次开发）
    max_neig: int = 10                  # 最大邻居数（拓扑距离）
    r_coll: float = 0.5                 # 碰撞半径（距离 < 2*r_coll 记碰撞）

    # ---- 竞技场（cubic arena）----
    x_arena: np.ndarray = field(default_factory=lambda: np.array(
        [[-100.0, 100.0], [-100.0, 100.0], [-100.0, 100.0]]))   # (3,2)
    center_arena: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))  # 派生
    d_arena: float = 1.5                # 边界排斥力作用范围
    c_arena: float = 10.0               # 边界排斥力系数

    # ---- 球形障碍物 ----
    spheres: np.ndarray = field(default_factory=lambda: np.array(
        [[-70.0, 70.0], [50.0, 50.0], [5.0, 5.0], [50.0, 50.0]]))  # (4,2)
    n_spheres: int = 2

    # ---- 圆柱障碍物（由 map 建筑生成，见工厂）----
    cylinders: np.ndarray = field(default_factory=lambda: np.zeros((3, 0)))
    n_cyl: int = 0
    # 每圆柱对应建筑物高度 [m]（逐建筑物高度感知，Stage 13；索引与 cylinders 一致）
    cyl_heights: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # Stage 13 三维高度感知开关：True=高空不绕行+禁向下进入建筑高度（新行为）；
    # False=还原 MATLAB 原 2D 避障行为（供 §17 MATLAB 对照测试）
    height_aware_cyl: bool = True

    # ---- 参考值 ----
    d_ref: float = 25.0                 # 无人机间期望距离
    u_ref: np.ndarray = field(default_factory=lambda: np.array([[1.0], [0.0], [0.0]]))  # (3,1) 迁移方向
    v_ref: float = 6.0                  # 期望速度
    x_goal: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))  # is_active_goal 时用（MATLAB 默认未定义）

    # ---- 速度/加速度上限 ----
    max_a: float = 10.0
    max_v: float = 7.0

    # ---- 初始位置/速度 ----
    P0: np.ndarray = field(default_factory=lambda: np.array([[-20.0], [150.0], [-50.0]]))  # (3,1)
    P: float = 20.0                     # 立方体棱长
    V0: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))  # (3,1)
    V: float = 0.0
    Pos0: np.ndarray = field(default_factory=lambda: np.zeros((3, 0)))  # 派生(工厂): P0 + P*rand(3,N)
    Vel0: np.ndarray = field(default_factory=lambda: np.zeros((3, 0)))  # 派生(工厂): V0 + V*rand(3,N)

    # ---- 可选随机速度系数（MATLAB 未定义时 isfield 为假 → 跳过）----
    c_r: float = None

    def __post_init__(self) -> None:
        # center_arena = sum(x_arena, 2)/2  → (3,1)
        self.center_arena = self.x_arena.sum(axis=1, keepdims=True) / 2.0


@dataclass
class OlfatiSaberParams(SwarmParams):
    """Olfati-Saber 专属参数（param_olfati_saber.m）。"""

    c_vm: float = 3.0                   # 速度匹配系数
    a: float = 1.0
    b: float = 5.0
    c: float = 0.0                      # 派生: (b-a)/(2*sqrt(a*b))
    delta: float = 0.2
    k: float = 2.0

    # 障碍物参数
    r0: float = 10.0
    lambda_: float = 1.0                # ⚠️ MATLAB p_swarm.lambda（Python 关键字，故加下划线）(0,1]
    c_pm_obs: float = 5.0
    c_vm_obs: float = 0.0               # 派生: = c_vm

    def __post_init__(self) -> None:
        super().__post_init__()
        # p_swarm.c = (b-a)/(2*sqrt(a*b))
        self.c = (self.b - self.a) / (2 * math.sqrt(self.a * self.b))
        # p_swarm.c_vm_obs = p_swarm.c_vm
        self.c_vm_obs = self.c_vm


@dataclass
class VasarhelyiParams(SwarmParams):
    """Vasarhelyi 专属参数（param_vasarhelyi.m）。"""

    # 排斥 Repulsion
    r0_rep: float = 25.0                # 派生: = d_ref（排斥半径）
    p_rep: float = 0.03                 # 排斥增益

    # 摩擦/速度对齐 Friction
    r0_fric: float = 85.3               # 对齐停止点偏移
    C_fric: float = 0.05                # 速度对齐系数
    v_fric: float = 0.63                # 对齐速度松弛项
    p_fric: float = 3.2                 # 制动曲线增益
    a_fric: float = 4.16                # 制动曲线加速度

    # 墙/障碍 Walls and obstacles
    r0_shill: float = 0.3               # 墙壁停止点偏移
    v_shill: float = 13.6               # 虚拟引导体速度
    p_shill: float = 3.55               # 墙壁制动曲线增益
    a_shill: float = 3.02               # 墙壁制动曲线加速度

    def __post_init__(self) -> None:
        super().__post_init__()
        # param_vasarhelyi.m: p_swarm.r0_rep = p_swarm.d_ref（随 d_ref 覆盖而更新）
        self.r0_rep = self.d_ref


def build_swarm_params(algorithm, map_params=None, seed=None, **overrides):
    """按算法构建 p_swarm，并生成 cylinders（来自 map）与 Pos0/Vel0（随机）。

    复刻 MATLAB param_swarm.m 流程:
        公共字段（含 isfield 默认）→ 由 map 生成 cylinders → rng + Pos0/Vel0 → run(param_<算法>)

    参数:
        algorithm  : SwarmAlgorithm 或等价字符串（"olfati_saber" / "vasarhelyi"）
        map_params : MapParams（提供建筑→圆柱障碍）；None 或环境未激活时无圆柱
        seed       : NumPy 随机种子（None=不可复现，复刻 rng('shuffle')；整数=可复现）
        **overrides: 覆盖字段，如 nb_agents=10, d_ref=30, v_ref=8, loss_radio=0.3,
                     u_ref=np.array([...]), is_active_cyl=False 等（对应 GUI 先设后 run）
    """
    if algorithm == SwarmAlgorithm.OLFATI_SABER:
        cls = OlfatiSaberParams
    elif algorithm == SwarmAlgorithm.VASARHELYI:
        cls = VasarhelyiParams
    else:
        raise ValueError(f"未知 algorithm: {algorithm!r}")

    p = cls(**overrides)

    # ---- 圆柱障碍物（param_swarm.m: 由 map 建筑转换）----
    if (map_params is not None and map_params.ACTIVE_ENVIRONMENT
            and map_params.buildings_east.size > 0):
        bn = map_params.buildings_north.ravel()   # (nb,)
        be = map_params.buildings_east.ravel()    # (nb,)
        nb_obstacles = be.shape[0]
        if map_params.building_shape == "cylinder":
            cylinder_radius = map_params.building_width / 2.0
        else:
            cylinder_radius = map_params.building_width / 2.0 * math.sqrt(2.0)
        p.cylinders = np.vstack([bn, be, np.full(nb_obstacles, cylinder_radius)])  # (3, nb)
        p.n_cyl = nb_obstacles
        # 逐圆柱对应建筑物高度（bn/be 由 buildings_north/east ravel，索引与 heights 一致）
        p.cyl_heights = np.asarray(map_params.buildings_heights, dtype=float).ravel()
    else:
        # MATLAB: p_swarm.cylinders = 0; p_swarm.n_cyl = 0;（无环境）
        p.cylinders = np.zeros((3, 0))
        p.n_cyl = 0
        p.cyl_heights = np.zeros(0)

    # ---- 初始位置/速度（param_swarm.m: rng('shuffle'); Pos0=P0+P*rand(3,N)）----
    rng = np.random.default_rng(seed)
    p.Pos0 = p.P0 + p.P * rng.random((3, p.nb_agents))
    p.Vel0 = p.V0 + p.V * rng.random((3, p.nb_agents))
    return p
