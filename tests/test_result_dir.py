# -*- coding: utf-8 -*-
"""Stage 10 —— 结果目录管理测试（result/<alg>_<ts>[_tag]/）。"""
import os
import re
import sys
import tempfile

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from SwarmLab_py.simulation import new_result_dir, RESULT_ROOT

PASS = []


def _check(name, cond):
    PASS.append((name, bool(cond)))
    print(("[PASS] " if cond else "[FAIL] ") + name)


def test_result_root_location():
    _check("RESULT_ROOT  basename=result", os.path.basename(RESULT_ROOT) == "result")
    _check("RESULT_ROOT 父目录=SwarmLab_py",
           os.path.basename(os.path.dirname(RESULT_ROOT)) == "SwarmLab_py")


def test_new_result_dir_naming():
    base = tempfile.mkdtemp(prefix="swarmlab_res_")
    d = new_result_dir("olfati_saber", base=base, tag="fault30")
    name = os.path.basename(d)
    _check("目录已创建", os.path.isdir(d))
    _check("名前缀=算法名", name.startswith("olfati_saber_"))
    _check("名后缀=tag", name.endswith("_fault30"))
    _check("时间戳格式 yyyy_mm_dd_HH_MM_SS",
           re.match(r"olfati_saber_\d{4}_\d{2}_\d{2}_\d{2}_\d{2}_\d{2}_fault30$", name) is not None)
    _check("位于 base 下", os.path.dirname(d) == base)


def test_new_result_dir_no_tag():
    base = tempfile.mkdtemp(prefix="swarmlab_res_")
    d = new_result_dir("vasarhelyi", base=base)
    name = os.path.basename(d)
    _check("无 tag 时名字=算法_时间戳",
           re.match(r"vasarhelyi_\d{4}_\d{2}_\d{2}_\d{2}_\d{2}_\d{2}$", name) is not None)


def test_new_result_dir_unique_per_call():
    base = tempfile.mkdtemp(prefix="swarmlab_res_")
    import time as _t
    d1 = new_result_dir("olfati_saber", base=base)
    _t.sleep(1.1)                      # 时间戳精度到秒， sleep 保证不同
    d2 = new_result_dir("olfati_saber", base=base)
    _check("两次调用目录不同", d1 != d2 and os.path.isdir(d1) and os.path.isdir(d2))


if __name__ == "__main__":
    test_result_root_location()
    test_new_result_dir_naming()
    test_new_result_dir_no_tag()
    test_new_result_dir_unique_per_call()
    n = len(PASS); ok = sum(1 for _, c in PASS if c)
    print("-" * 50)
    print(f"Total: {n}  Passed: {ok}  Failed: {n - ok}")
    sys.exit(0 if ok == n else 1)
