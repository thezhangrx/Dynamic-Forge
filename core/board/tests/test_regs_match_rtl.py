#!/usr/bin/env python3
"""PS 侧的寄存器契约不能和 PL 的 RTL 漂移。

**为什么这条测试必须有**：`core/board/vision_regs.hpp` 是 PS 侧唯一"记住"
PL 长什么样的地方，而 PS 和 PL 是两份独立维护的代码。地址错一个字节，
读回来的就是**别的寄存器**——不会报错，只会静默给错数。

所以这里**直接解析两边的源文件**、逐项核对，而不是靠人眼：

    core/fpga/rtl/vision_axi.v   ← localparam [11:0] ADDR_*
    core/board/vision_regs.hpp   ← constexpr uint32_t kOff*

只比"两边都写了的"量。RTL 里没有对应 localparam 的量（比如 cfg 字段序号）
不在这里管——那些由 core/fpga/tests/test_rtl_contract.py 管。
"""

from __future__ import annotations

import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
AXI = ROOT / "core" / "fpga" / "rtl" / "vision_axi.v"
REGS = ROOT / "core" / "board" / "vision_regs.hpp"


def _axi_addrs() -> dict[str, int]:
    """从 vision_axi.v 抠出 ``ADDR_XXX = 12'hNNN``。"""
    src = AXI.read_text(encoding="utf-8")
    out: dict[str, int] = {}
    for name, raw in re.findall(
            r"localparam\s*\[11:0\]\s*(ADDR_\w+)\s*=\s*12'h([0-9A-Fa-f]+)", src):
        out[name] = int(raw, 16)
    return out


def _hpp_offsets() -> dict[str, int]:
    """从 vision_regs.hpp 抠出 ``kOffXXX = 0xNNN``。"""
    src = REGS.read_text(encoding="utf-8")
    out: dict[str, int] = {}
    for name, raw in re.findall(
            r"constexpr\s+uint32_t\s+(kOff\w+)\s*=\s*0x([0-9A-Fa-f]+)u", src):
        out[name] = int(raw, 16)
    return out


# RTL 的 localparam 名 → C++ 的常量名。**必须逐个列全**，
# 少列一项就等于少守一个地址。
PAIRS = {
    "ADDR_CONFIG": "kOffConfig",
    "ADDR_DBG":    "kOffDbgSel",
    "ADDR_DBGDAT": "kOffDbgData",
    "ADDR_CTRL":   "kOffCtrl",
    "ADDR_SEQ":    "kOffSeq",
    "ADDR_STAMP0": "kOffStamp0",
    "ADDR_STAMP1": "kOffStamp1",
    "ADDR_FLEN":   "kOffFLen",
    "ADDR_FRAME":  "kOffFrame",
    # PS 喂像素（真摄像头）：见 rtl/vision_pixfeed.v
    "ADDR_PIXPUSH": "kOffPixPush",
    "ADDR_PIXSTAT": "kOffPixStat",
    "ADDR_PIXCTL":  "kOffPixCtl",
}


def test_both_files_exist() -> None:
    assert AXI.exists(), f"找不到 RTL：{AXI}"
    assert REGS.exists(), f"找不到寄存器契约：{REGS}"


@pytest.mark.parametrize("rtl_name,cpp_name", sorted(PAIRS.items()))
def test_register_offset_matches_rtl(rtl_name: str, cpp_name: str) -> None:
    axi = _axi_addrs()
    hpp = _hpp_offsets()
    assert rtl_name in axi, f"vision_axi.v 里找不到 {rtl_name}"
    assert cpp_name in hpp, f"vision_regs.hpp 里找不到 {cpp_name}"
    assert hpp[cpp_name] == axi[rtl_name], (
        f"寄存器偏移漂移：vision_axi.v 的 {rtl_name} = 0x{axi[rtl_name]:03X}，"
        f"而 vision_regs.hpp 的 {cpp_name} = 0x{hpp[cpp_name]:03X}。"
        "PS 侧读到的会是别的寄存器，且不会报错。")


def test_pair_table_covers_every_addressed_localparam() -> None:
    """RTL 里每一个 ADDR_* 都要在对照表里出现（除了计算出来的 END）。

    防止"新加了个寄存器，但忘了同步 PS 侧"。
    """
    axi = _axi_addrs()
    missing = sorted(n for n in axi if n not in PAIRS and not n.endswith("_END"))
    assert not missing, (
        f"vision_axi.v 里有未纳入 PS 契约的地址常量 {missing}；"
        "要么在 vision_regs.hpp 里补上，要么显式说明它不该给 PS 用")


def test_cfg_block_does_not_overrun_its_window() -> None:
    """21 个配置寄存器必须落在 0x000..0x07F 的配置块里。

    vision_axi.v 里 `rd_conf = rd_cycle && (ar_a < ADDR_CONFIG + 12'h080)` ——
    一旦 cfg_count 超过 32，配置写就会撞进调试口。
    """
    hpp = REGS.read_text(encoding="utf-8")
    m = re.search(r"kCfgCount\s*=\s*(\d+)", hpp)
    assert m, "vision_regs.hpp 里找不到 kCfgCount"
    n = int(m.group(1))
    assert 0 < n <= 32, (
        f"配置寄存器有 {n} 个；配置块只有 0x000..0x07F（32 个字），"
        "超过就会撞进 0x080 的调试口")


def test_frame_window_is_within_the_address_window() -> None:
    """几何帧窗口必须落在 4 KB 地址窗口内。"""
    hpp = REGS.read_text(encoding="utf-8")
    off = _hpp_offsets()["kOffFrame"]
    m = re.search(r"kFrameWindowBytes\s*=\s*(\d+)u", hpp)
    assert m, "vision_regs.hpp 里找不到 kFrameWindowBytes"
    end = off + int(m.group(1))
    assert end <= 0x1000, (
        f"帧窗口 {off:#x}..{end:#x} 超出 4 KB 地址窗口（ADDR_W=12）；"
        "vision_axi.v 的地址是 12 位，超出的位会被截断")


# ---------------------------------------------------------------------------
# 基址：**跨三处**的契约
#
# 为什么必须有这一条：基址是 PS 侧**独立维护**的常量（`vision_regs.hpp` 的
# kRegBase、`tools/*.c` 的 BASE），而块设计那边原来只靠 `assign_bd_address`
# **自动分配**。自动分配的结果会随 BD 里元件的增减而变，一旦漂移，PS 侧会
# **静默**读写到别的窗口 —— 不会报错，只会给错数。
#
# 2026-10-15 实测就抓到了：`vision_regs.hpp` 写的是 **0x400C0000**（那是 NPU 的
# 窗口），而块设计把 `vision_0/s/reg0` 分配在 **0x43C00000** —— 板上工具用
# 0x43C00000 读得通，说明 PS 侧那份契约是错的（写 0x400C0000 更糟：会碰 NPU）。
# 所以块设计那边现在就**显式钉死**，并由这条测试把三处钉在一起。
# ---------------------------------------------------------------------------
BD_TCL = ROOT / "core" / "fpga" / "synth" / "bd_vision.tcl"
TOOLS_DIR = ROOT / "core" / "board" / "tools"


def _bd_pinned_base() -> int:
    """块设计里显式钉死的基址（``set_property offset 0x… $_seg``）。"""
    src = BD_TCL.read_text(encoding="utf-8")
    m = re.search(r"set_property\s+offset\s+(0x[0-9A-Fa-f]+)\s+\$_seg", src)
    assert m, (
        "bd_vision.tcl 里找不到显式钉基址的 `set_property offset 0x… $_seg`。"
        "只靠 assign_bd_address 自动分配是不行的 —— 基址是 PS 侧的契约，"
        "自动分配会随元件增减而漂移")
    return int(m.group(1), 16)


def test_register_base_matches_block_design() -> None:
    """vision_regs.hpp 的 kRegBase 必须等于块设计钉死的基址。"""
    base = _bd_pinned_base()
    m = re.search(r"kRegBase\s*=\s*0x([0-9A-Fa-f]+)u", REGS.read_text(encoding="utf-8"))
    assert m, "vision_regs.hpp 里找不到 kRegBase"
    got = int(m.group(1), 16)
    assert got == base, (
        f"基址漂移：vision_regs.hpp 的 kRegBase = {got:#010x}，"
        f"而块设计钉死的是 {base:#010x}。PS 侧会静默读写到别的窗口，且不会报错。")


def test_tools_base_matches_block_design() -> None:
    """板上 C 工具的 BASE 也必须等于同一个基址。"""
    base = _bd_pinned_base()
    seen: dict[str, int] = {}
    for c in sorted(TOOLS_DIR.glob("*.c")):
        m = re.search(r"#define\s+BASE\s+(0x[0-9A-Fa-f]+)", c.read_text(encoding="utf-8"))
        if m:
            seen[c.name] = int(m.group(1), 16)
    assert seen, "core/board/tools/*.c 里一个 BASE 都没解析到"
    bad = {k: v for k, v in seen.items() if v != base}
    assert not bad, (
        f"这些工具的 BASE 和块设计钉死的基址 {base:#010x} 不一致："
        + "，".join(f"{k}={v:#010x}" for k, v in bad.items()))
