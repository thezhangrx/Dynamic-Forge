#!/usr/bin/env python3
"""PS 侧 `VisionParams` 的默认值必须与 C 参考模型**逐项相同**。

## 为什么必须有这一条（踩过的真事）

板上"真图进 PL"那条路第一次跑通时，`--verify-both` 报：**同一张图，PL 与 PS
黄金模型给出的几何帧从第 84 字节起就不一样**（障碍个数、缺口数、尺寸全对，
只有坐标不同）。查下去发现 `vision_driver.hpp` 的 `VisionParams` 默认值里有
**13 项**与参考模型不同：

    band_row_pct 12 vs 5、band_join_rows 2 vs 4、band_min_rows 4 vs 5、
    field_col_min 8 vs 5、wall_col_pct 50 vs 25、wall_col_min 3 vs 2、
    edge_min 20 vs 12、edge_min_cols 2 vs 1、min_gap_px 12 vs 8、
    player_min_area 120 vs 60、player_max_area 6000 vs 1500、
    player_min_fill 30 vs 45、player_max_aspect 250 vs 160

于是 PS 写进 PL 的阈值与**整条验证链**（RTL 复位值 / C 参考模型 / Python 真值）
用的那一套不同 —— 表现就是"个数对、坐标不对"，**完全不报错**。

## 为什么以前没被发现

* 协同仿真的 `tb_axi` 用的是 **C 参考模型**那套参数写给 RTL，两边天生一致，
  所以从没验过 C++ 这份；
* 板上之前的跑法要么走 TPG（期望 CRC 出自 RTL 复位值，而那次也没比 CRC），
  要么走 PS 参考模型（用 C 的默认值），**都没有真正用到 `VisionParams` 的默认值**；
* `tests/test_vision_driver.cpp` 只验了"结构体字段 → 寄存器下标"的映射
  （抽查 5 项），**没验默认值本身**。

这条测试把 `vision_driver.hpp` 的默认值直接和 `vision_ref.c` 的
`VR_DEFAULT_PARAMS` 对起来（C↔Python 那一层已由
`core/fpga/tests/test_ref_vs_python.py` 守着，所以三者一致是传递的）。
"""

from __future__ import annotations

import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
HPP = ROOT / "core" / "board" / "vision_driver.hpp"
REF_C = ROOT / "core" / "fpga" / "ref" / "vision_ref.c"

#: `VisionParams` 的字段名 → C 参考模型 `VR_DEFAULT_PARAMS` 的字段名。
#: **必须逐个列全** —— 少一项就等于少守一个阈值（这次的 bug 正是漏了 13 项）。
PAIRS = {
    "wall_rb":           "wall_rb",
    "wall_min_r":        "wall_min_r",
    "wall_min_g":        "wall_min_g",
    "bright_min":        "bright_min",
    "bright_spread":     "bright_spread",
    "green_gm":          "green_gm",
    "green_min":         "green_min",
    "band_row_pct":      "band_row_pct",
    "band_join_rows":    "band_join_rows",
    "band_min_rows":     "band_min_rows",
    "field_col_min":     "field_col_min",
    "wall_col_pct":      "wall_col_pct",
    "wall_col_min":      "wall_col_min",
    "edge_min":          "edge_min",
    "edge_min_cols":     "edge_min_cols",
    "min_seg_px":        "min_seg_px",
    "min_gap_px":        "min_gap_px",
    "player_min_area":   "player_min_area",
    "player_max_area":   "player_max_area",
    "player_min_fill":   "player_min_fill_pct",
    "player_max_aspect": "player_max_aspect_pct",
}


def _hpp_defaults() -> dict[str, int]:
    """`vision_driver.hpp` 里 ``int name = value;`` 的默认值（只取 VisionParams 段）。"""
    src = HPP.read_text(encoding="utf-8")
    m = re.search(r"struct VisionParams\s*\{(.*?)\n\};", src, re.S)
    assert m, "vision_driver.hpp 里找不到 struct VisionParams"
    return {name: int(val) for name, val in
            re.findall(r"int\s+(\w+)\s*=\s*(-?\d+)\s*;", m.group(1))}


def _ref_defaults() -> dict[str, int]:
    """`vision_ref.c` 里 **`VR_DEFAULT_REF_PARAMS`** 的 ``.field = value``。

    ⚠ 必须指名 `VR_DEFAULT_REF_PARAMS`（完整的 VrRefParams，21 项）。
    文件里还有一个 `VR_DEFAULT_PARAMS`（WallWithGapParamsC，只有 17 项，
    不含像素分类的 wall_rb/wall_min_*/bright_*/green_*）。
    一开始图省事写了个能匹配到后者的正则，于是有 7 项"没找到"而误报失败 ——
    解析器抓错结构时，失败信息会指向被解析的一方，很容易带偏方向。
    """
    src = REF_C.read_text(encoding="utf-8")
    m = re.search(r"VR_DEFAULT_REF_PARAMS\s*=\s*\{(.*?)\n\};", src, re.S)
    assert m, "vision_ref.c 里找不到 VR_DEFAULT_REF_PARAMS"
    return {name: int(val) for name, val in
            re.findall(r"\.(\w+)\s*=\s*(-?\d+)\b", m.group(1))}


def test_both_sources_parse() -> None:
    hpp, ref = _hpp_defaults(), _ref_defaults()
    assert len(hpp) >= len(PAIRS), (
        f"VisionParams 只解析出 {len(hpp)} 个默认值，少于对照表的 {len(PAIRS)} 项"
        "（正则需要更新，或者结构体被改过）")
    assert ref, "VR_DEFAULT_PARAMS 一个字段都没解析出来（正则需要更新）"


@pytest.mark.parametrize("hpp_name,ref_name", sorted(PAIRS.items()))
def test_default_matches_reference(hpp_name: str, ref_name: str) -> None:
    hpp, ref = _hpp_defaults(), _ref_defaults()
    assert hpp_name in hpp, f"vision_driver.hpp 的 VisionParams 里没有 {hpp_name}"
    assert ref_name in ref, f"vision_ref.c 的 VR_DEFAULT_PARAMS 里没有 {ref_name}"
    assert hpp[hpp_name] == ref[ref_name], (
        f"默认值不一致：vision_driver.hpp 的 {hpp_name} = {hpp[hpp_name]}，"
        f"而参考模型（= Python 语义真值）的 {ref_name} = {ref[ref_name]}。\n"
        "PS 写进 PL 的阈值会与整条验证链用的那一套不同 —— "
        "表现是几何帧'个数对、坐标不对'，且完全不报错")


def test_pair_table_covers_every_field() -> None:
    """对照表必须覆盖 VisionParams 的每一个字段（防止以后新加字段没人守）。"""
    hpp = _hpp_defaults()
    missing = sorted(n for n in hpp if n not in PAIRS)
    assert not missing, (
        f"VisionParams 里这些字段没纳入对照表：{missing}；"
        "新加字段就要补上，否则它的默认值没人守")
