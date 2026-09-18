"""SwarmLab_py.parameters —— 参数系统（对应 MATLAB parameters/ 目录）。

设计（Phase 2，采用“基类 + 继承子类”方案）:
    - 每组参数一个 ``@dataclass``，字段默认值 = MATLAB 中赋的值；
    - MATLAB 里 ``if ~isfield(p,'x'); p.x=默认; end`` 的条件默认 → dataclass 默认值，
      可通过构造参数覆盖（对应 GUI/example 先设字段再 run 脚本的行为）；
    - MATLAB 的 ``run(strcat("param_", 类型/算法))`` 动态脚本调用 → 显式工厂函数
      （build_drone_params / build_map_params / build_swarm_params）+ 分派；
    - 派生量（gamma、PID 增益、c=(b-a)/2√(ab)、传感器 Q/R、R_min、center_arena 等）
      在 ``__post_init__`` 或工厂中计算，逐一与 MATLAB 数值对照；
    - ``build_params(...)`` 复刻示例脚本的装配顺序（sim→battery→physics→drone→map→swarm），
      用显式 ``ParamsBundle`` 取代 MATLAB 的 workspace 隐式共享变量。

子模块:
    physics      PhysicsParams        (param_physics.m)
    sim          SimParams            (param_sim.m)
    battery      BatteryParams        (param_battery.m)
    sensors      SensorParamsMixin    (param_drone/param_sensors.m)
    drone        DroneParams + PointMassParams/QuadcopterParams (param_drone.m, param_quadcopter.m)
    fixed_wing   FixedWingParams      (param_drone/param_fixed_wing.m)
    map_params   MapParams            (param_map.m, create_shifted_buildings.m)
    swarm        SwarmParams + OlfatiSaberParams/VasarhelyiParams (param_swarm*.m)
"""

from dataclasses import dataclass
from typing import Optional

from .physics import PhysicsParams
from .sim import SimParams
from .battery import BatteryParams
from .sensors import SensorParamsMixin
from .drone import (
    DroneParams,
    PointMassParams,
    QuadcopterParams,
    build_drone_params,
    compute_inertia_gammas,
    KSI_OPT,
)
from .fixed_wing import FixedWingParams, XYU_TRIM
from .map_params import MapParams, create_shifted_buildings, build_map_params
from .swarm import (
    SwarmParams,
    OlfatiSaberParams,
    VasarhelyiParams,
    build_swarm_params,
)


@dataclass
class ParamsBundle:
    """一次仿真所需的全部参数集合（取代 MATLAB 的 workspace 隐式共享变量）。

    字段名与 MATLAB 变量名一致：p_sim / p_battery / p_physics / p_drone / map / p_swarm。
    """

    p_sim: SimParams
    p_battery: BatteryParams
    p_physics: PhysicsParams
    p_drone: DroneParams
    map: MapParams
    p_swarm: Optional[SwarmParams] = None   # 单机示例可为 None


def build_params(drone_type, algorithm=None, *, active_environment=True,
                 end_time=None, nb_agents=None, d_ref=None, v_ref=None,
                 loss_radio=None, u_ref=None, seed=None) -> ParamsBundle:
    """按 MATLAB 示例的装配顺序构建全部参数。

    复刻 example_olfati_saber.m / example_vasarhelyi.m 顶部:
        run(param_sim); run(param_battery); run(param_physics);
        run(param_drone); run(param_map); run(param_swarm);

    参数:
        drone_type         : DroneType 或字符串（"point_mass"/"quadcopter"/"fixed_wing"）
        algorithm          : SwarmAlgorithm 或字符串；None 表示不构建群体参数（单机场景）
        active_environment : 对应 ACTIVE_ENVIRONMENT
        end_time           : 覆盖 p_sim.end_time（对应 GUI app.sim_time）
        nb_agents/d_ref/v_ref/loss_radio/u_ref : 覆盖 p_swarm 字段（对应 GUI 先设后 run）
        seed               : Pos0 随机种子（None=不可复现，复刻 rng('shuffle')）
    """
    p_sim = SimParams()
    if end_time is not None:
        p_sim.end_time = end_time

    p_battery = BatteryParams()
    p_physics = PhysicsParams()

    p_drone = build_drone_params(drone_type, p_physics)
    map_params = build_map_params(drone_type, active_environment)

    p_swarm = None
    if algorithm is not None:
        overrides = {}
        if nb_agents is not None:
            overrides["nb_agents"] = nb_agents
        if d_ref is not None:
            overrides["d_ref"] = d_ref
        if v_ref is not None:
            overrides["v_ref"] = v_ref
        if loss_radio is not None:
            overrides["loss_radio"] = loss_radio
        if u_ref is not None:
            overrides["u_ref"] = u_ref
        p_swarm = build_swarm_params(algorithm, map_params, seed=seed, **overrides)

    return ParamsBundle(
        p_sim=p_sim,
        p_battery=p_battery,
        p_physics=p_physics,
        p_drone=p_drone,
        map=map_params,
        p_swarm=p_swarm,
    )


__all__ = [
    # 基础
    "PhysicsParams", "SimParams", "BatteryParams", "SensorParamsMixin",
    # drone
    "DroneParams", "PointMassParams", "QuadcopterParams", "FixedWingParams",
    "build_drone_params", "compute_inertia_gammas", "KSI_OPT", "XYU_TRIM",
    # map
    "MapParams", "create_shifted_buildings", "build_map_params",
    # swarm
    "SwarmParams", "OlfatiSaberParams", "VasarhelyiParams", "build_swarm_params",
    # 顶层
    "ParamsBundle", "build_params",
]
