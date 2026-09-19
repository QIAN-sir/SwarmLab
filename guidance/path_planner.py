# -*- coding: utf-8 -*-
"""plan_RRT —— 直线 RRT 路径规划（对应 MATLAB guidance/path_planner/plan_RRT.m）。

节点格式 [pn, pe, pd, chi, cost, parent_idx, flag_connect_to_goal]（parent_idx 存 0-based）。
plot_map 未迁移（可视化，Phase 12）。
⚠️ B-19（记录，不修复）：find_minimum_path 回溯时跳过第一级父节点（MATLAB 原逻辑
   `parent_node=tree(parent_node,6)` 后再 prepend），忠实保留。
"""
import numpy as np


def generate_random_node(map_params, pd, chi):
    pn = map_params.width * np.random.rand()
    pe = map_params.width * np.random.rand()
    return np.array([pn, pe, pd, chi, 0.0, 0.0, 0.0])


def points_along_path(start_node, end_node, step_length):
    X = [start_node[0]]; Y = [start_node[1]]; Z = [start_node[2]]
    q = end_node[0:3] - start_node[0:3]
    L = float(np.linalg.norm(q))
    if L == 0:
        return np.array(X), np.array(Y), np.array(Z)
    q = q / L
    w = start_node[0:3].copy()
    for _ in range(2, int(np.floor(L / step_length)) + 1):
        w = w + step_length * q
        X.append(w[0]); Y.append(w[1]); Z.append(w[2])
    return np.array(X), np.array(Y), np.array(Z)


def down_at_NE(map_params, n, e):
    bn = np.asarray(map_params.buildings_north).ravel()
    be = np.asarray(map_params.buildings_east).ravel()
    bh = np.asarray(map_params.buildings_heights).ravel()
    if bn.size == 0 or be.size == 0:
        return 0.0
    d_n = float(np.min(np.abs(n - bn)))
    d_e = float(np.min(np.abs(e - be)))
    idx_e = int(np.argmin(np.abs(e - be)))
    if d_n <= map_params.building_width and d_e <= map_params.building_width:
        return -bh[idx_e]
    return 0.0


def collision(start_node, end_node, map_params):
    X, Y, Z = points_along_path(start_node, end_node, 0.1)
    for i in range(len(X)):
        if Z[i] >= down_at_NE(map_params, X[i], Y[i]):
            return 1
    return 0


def extend_tree(tree, end_node, segment_length, map_params, pd, chi):
    flag1 = 0
    new_node = None
    while flag1 == 0:
        random_node = generate_random_node(map_params, pd, chi)
        tmp = tree[:, 0:3] - random_node[0:3]
        d2 = np.diag(tmp @ tmp.T)
        idx = int(np.argmin(d2))
        L = min(float(np.sqrt(d2[idx])), segment_length)
        cost = tree[idx, 4] + L
        tmp = random_node[0:3] - tree[idx, 0:3]
        nrm = float(np.linalg.norm(tmp))
        new_point = tree[idx, 0:3] + L * (tmp / nrm) if nrm > 0 else tree[idx, 0:3]
        new_node = np.concatenate([new_point, [chi, cost, float(idx), 0.0]])
        if collision(tree[idx], new_node, map_params) == 0:
            tree = np.vstack([tree, new_node])
            flag1 = 1
    if (np.linalg.norm(new_node[0:3] - end_node[0:3]) < segment_length) and \
            collision(new_node, end_node, map_params) == 0:
        flag = 1
        tree[-1, 6] = 1
    else:
        flag = 0
    return tree, flag


def find_minimum_path(tree, end_node):
    connecting = tree[tree[:, 6] == 1]
    if connecting.size == 0:
        return np.vstack([tree[0], end_node])
    idx = int(np.argmin(connecting[:, 4]))
    path = np.vstack([connecting[idx], end_node])
    parent = int(connecting[idx, 5])
    while parent > 0:                      # B-19 忠实：跳过第一级父节点
        parent = int(tree[parent, 5])
        path = np.vstack([tree[parent], path])
    return path


def smooth_path(path, map_params):
    new_path = path[0:1, :]
    ptr = 1
    while ptr <= path.shape[0] - 2:
        if collision(new_path[-1], path[ptr + 1], map_params) != 0:
            new_path = np.vstack([new_path, path[ptr]])
        ptr += 1
    new_path = np.vstack([new_path, path[-1]])
    return new_path


def plan_RRT(wpt_start, wpt_end, map_params):
    segment_length = 100.0
    pd = wpt_end[2]
    chi = -9999.0
    start_node = np.array([wpt_start[0], wpt_start[1], pd, chi, 0, 0, 0], dtype=float)
    end_node = np.array([wpt_end[0], wpt_end[1], pd, chi, 0, 0, 0], dtype=float)
    tree = start_node.reshape(1, -1)
    if (np.linalg.norm(start_node[0:3] - end_node[0:3]) < segment_length) and \
            collision(start_node, end_node, map_params) == 0:
        path = np.vstack([start_node, end_node])
    else:
        nb_paths = 0
        while nb_paths < 3:
            tree, flag = extend_tree(tree, end_node, segment_length, map_params, pd, chi)
            nb_paths += flag
    path = find_minimum_path(tree, end_node)
    return smooth_path(path, map_params)
