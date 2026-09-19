# -*- coding: utf-8 -*-
"""result_dir —— 群体仿真结果目录管理（对应 MATLAB results/results_swarm/<alg>_<timestamp>/）。

在 ``SwarmLab_py/result/`` 下按 ``<algorithm>_<yyyy_mm_dd_HH_MM_SS>`` 为每次运行建子目录，
集中存放群体仿真视频 + debug 参数图（性能/碰撞/障碍距离/轨迹/关键帧），便于后续挑选与
对比多次运行（同 MATLAB 惯例）。

用法::
    from SwarmLab_py.simulation import new_result_dir
    rdir = new_result_dir("olfati_saber")     # → SwarmLab_py/result/olfati_saber_2026_09_19_.../
    ... 把视频/图存到 rdir ...
"""
import os
from datetime import datetime

# SwarmLab_py 包根 → result/
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULT_ROOT = os.path.join(_PKG_ROOT, "result")


def new_result_dir(algorithm, base=None, tag=None):
    """创建并返回一次运行的结果目录 ``<base>/<algorithm>_<timestamp>[_tag]``。

    algorithm : 算法名（olfati_saber / vasarhelyi），用于子目录前缀
    base      : 结果根目录，默认 SwarmLab_py/result
    tag       : 可选附加标签（如故障配置说明），便于区分同算法多次运行
    """
    ts = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    name = f"{algorithm}_{ts}" + (f"_{tag}" if tag else "")
    d = os.path.join(base or RESULT_ROOT, name)
    os.makedirs(d, exist_ok=True)
    return d
