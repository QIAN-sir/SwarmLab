"""MapParams —— 地图/障碍物参数 + 城市建筑布局生成。

对应 MATLAB:
    parameters/param_map.m
    graphics/graphics_map/create_shifted_buildings.m
    （create_buildings.m 为随机高度版本，param_map 默认未启用，故不迁移；如需要另补）

设计说明:
    - MATLAB param_map.m 是脚本：先按 DRONE_TYPE 设 width，再在 ACTIVE_ENVIRONMENT
      为真时生成建筑。此处用 build_map_params(drone_type, active_environment) 工厂复刻，
      并显式传入依赖（不再依赖 workspace 隐式变量）。
    - buildings_north / buildings_east / buildings_heights 保持 MATLAB 的“列向量(N,1)”形状，
      以便 p_swarm.cylinders 组装时对它们做转置（见 swarm.py）。
    - create_shifted_buildings 是确定性的（高度固定为 max_height），无随机性。
"""

from dataclasses import dataclass, field

import numpy as np

from ..enums import DroneType


@dataclass
class MapParams:
    """地图参数（param_map.m）。"""

    width: float = 300.0          # 地图宽度（正方形 width×width）；fixed_wing=4000
    max_height: float = 100.0     # 建筑最大高度
    ACTIVE_ENVIRONMENT: bool = False

    # 以下仅在 ACTIVE_ENVIRONMENT 为真时由 build_map_params 填充
    bl_corner_north: float = 0.0  # 地图左下角基准点（北向）
    bl_corner_east: float = 0.0   # 地图左下角基准点（东向）
    nb_blocks: int = 4            # 每行/列建筑块数量
    street_width_perc: float = 0.7  # 街道宽度占每个 block 的比例
    building_shape: str = "rectangle"  # 'rectangle' 或 'cylinder'

    building_width: float = 0.0   # 派生: width/nb_blocks*(1-street_width_perc)
    street_width: float = 0.0     # 派生: width/nb_blocks*street_width_perc

    buildings_north: np.ndarray = field(default_factory=lambda: np.zeros((0, 1)))
    buildings_east: np.ndarray = field(default_factory=lambda: np.zeros((0, 1)))
    buildings_heights: np.ndarray = field(default_factory=lambda: np.zeros((0, 1)))


def create_shifted_buildings(map_params: MapParams) -> None:
    """生成错位网格城市建筑（就地修改 map_params）。

    对应 MATLAB: graphics/graphics_map/create_shifted_buildings.m
        for i=1:nb_blocks: buildings_north(i)=0.5*width/nb_blocks*(2*(i-1)+1)
        奇数行 i：north 追加 nb_blocks 个相同值；east 追加整列 buildings_north
        偶数行 i：north 追加 nb_blocks-1 个相同值；east 追加 buildings_north(1:end-1)+offset
        其中 offset=(building_width+street_width)/2；高度恒为 max_height。
    """
    nb = map_params.nb_blocks
    width = map_params.width
    offset = (map_params.building_width + map_params.street_width) / 2.0

    # 每行的北向坐标（列向量语义，此处用 1-D 便于拼接）
    row_north = np.array([0.5 * width / nb * (2 * (i - 1) + 1) for i in range(1, nb + 1)])

    bn = []  # buildings_north
    be = []  # buildings_east
    for i in range(1, nb + 1):
        if i % 2 == 1:  # 奇数
            bn.extend([row_north[i - 1]] * nb)
            be.extend(row_north.tolist())
        else:           # 偶数
            bn.extend([row_north[i - 1]] * (nb - 1))
            be.extend((row_north[: nb - 1] + offset).tolist())

    map_params.buildings_north = np.array(bn, dtype=float).reshape(-1, 1)
    map_params.buildings_east = np.array(be, dtype=float).reshape(-1, 1)
    nb_buildings = len(be)
    map_params.buildings_heights = map_params.max_height * np.ones((nb_buildings, 1))


def build_map_params(drone_type, active_environment: bool = True) -> MapParams:
    """按 drone_type 与环境开关构建 map 参数（复刻 param_map.m 流程）。

    参数:
        drone_type         : DroneType 或等价字符串
        active_environment : 对应 MATLAB 工作区变量 ACTIVE_ENVIRONMENT
    """
    if drone_type == DroneType.FIXED_WING:
        width = 4000.0   # 固定翼飞行范围更大
    else:                # quadcopter / point_mass
        width = 300.0

    m = MapParams(width=width, max_height=100.0, ACTIVE_ENVIRONMENT=bool(active_environment))

    # MATLAB: 若环境未激活则提前 return（仅保留 width/max_height/ACTIVE_ENVIRONMENT）
    if not active_environment:
        return m

    m.building_width = m.width / m.nb_blocks * (1.0 - m.street_width_perc)
    m.street_width = m.width / m.nb_blocks * m.street_width_perc
    # building_shape 默认 'rectangle'（param_map.m 中 cylinder 被注释）
    create_shifted_buildings(m)
    return m
