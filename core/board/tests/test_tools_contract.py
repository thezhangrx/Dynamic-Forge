#!/usr/bin/env python3
"""板上 C 工具两件必须守住的事：**偏移量不许漂**、**必须编得过**。

## 为什么这条测试必须有

`core/board/tools/*.c` 是在板子上直接 gcc 编的小工具，它们各自**硬编码**了
AXI 偏移量；`axi_soak.c` 里还**复制了一份**配置寄存器表（偏移 + 位宽 + 名字）。
这些都是"PS 侧独立维护的一份 PL 认知"，地址错一个字节不会报错，只会静默读写
**别的寄存器**。

`test_regs_match_rtl.py` 已经守住了 `vision_regs.hpp`（C++ 那份），但**这批 C
工具没人守** —— 所以这里补上，做法一样：直接解析两边的源文件逐项核对，不靠人眼。

    core/fpga/rtl/vision_axi.v   ← localparam [11:0] ADDR_*（偏移）
    core/fpga/rtl/vision_top.v   ← localparam [5:0] CFG_*（配置寄存器序号）
                                   reg [N:0] r_xxx（位宽）
    core/board/tools/*.c         ← #define OFF_* + axi_soak.c 的 CFG[] 表

## 另外：编译守卫

这些工具平时不参与构建（只在板子上手编），所以**没人会发现它们编不过了**。
这里用 `-std=c11 -Wall -Wextra -Werror` 编一遍，把"手编才暴露"的问题挪到
`pytest` 里。`-Werror` 是刻意的：告警就是漂移的前兆。

（2026-10-15 这条思路当场见效：`dbg_probe.c` 里三个 `usleep` 在 `-std=c11` 下
是隐式声明 —— 它以前只用默认 gnu11 编过，所以一直没暴露。）
"""

from __future__ import annotations

import pathlib
import re
import shutil
import subprocess

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
TOOLS = ROOT / "core" / "board" / "tools"
AXI = ROOT / "core" / "fpga" / "rtl" / "vision_axi.v"
TOP = ROOT / "core" / "fpga" / "rtl" / "vision_top.v"

C_SOURCES = sorted(TOOLS.glob("*.c"))
SH_SCRIPTS = sorted(TOOLS.glob("*.sh"))


# ---------------------------------------------------------------------------
# 解析：从 RTL 里抠事实
# ---------------------------------------------------------------------------
def _axi_addrs() -> dict[str, int]:
    """vision_axi.v 的 ``localparam [11:0] ADDR_XXX = 12'hNNN``。"""
    src = AXI.read_text(encoding="utf-8")
    return {n: int(v, 16) for n, v in re.findall(
        r"localparam\s*\[11:0\]\s*(ADDR_\w+)\s*=\s*12'h([0-9A-Fa-f]+)", src)}


def _cfg_indices() -> dict[str, int]:
    """vision_top.v 的 ``CFG_XXX = 6'dN``（一条 localparam 里逗号分隔多项）。"""
    src = TOP.read_text(encoding="utf-8")
    return {n: int(v) for n, v in re.findall(r"(CFG_\w+)\s*=\s*6'd(\d+)", src)}


def _cfg_reg_widths() -> dict[str, int]:
    """配置寄存器**目标变量**的位宽：``reg [7:0] r_wall_rb, r_wall_min_r;``。

    返回 ``{"r_wall_rb": 8, ...}``。注意一条 reg 声明可以带多个名字，
    它们的位宽相同 —— 所以先把声明拆成 (位宽, 名字列表)。
    """
    src = TOP.read_text(encoding="utf-8")
    out: dict[str, int] = {}
    for width, names in re.findall(
            r"reg\s*\[(\d+):0\]\s*([^;]+);", src):
        for name in names.split(","):
            name = name.strip()
            if name.startswith("r_"):
                out[name] = int(width) + 1
    return out


# CFG_XXX（RTL）→ 目标寄存器变量名。**名字对不上，必须逐个列全**：
# 少列一项就等于少守一个位宽。
CFG_TO_REG = {
    "CFG_WALL_RB":     "r_wall_rb",
    "CFG_WALL_MIN_R":  "r_wall_min_r",
    "CFG_WALL_MIN_G":  "r_wall_min_g",
    "CFG_BRIGHT_MIN":  "r_bright_min",
    "CFG_BRIGHT_SPRD": "r_bright_spread",
    "CFG_GREEN_GM":    "r_green_gm",
    "CFG_GREEN_MIN":   "r_green_min",
    "CFG_BAND_PCT":    "r_band_row_pct",
    "CFG_BAND_JOIN":   "r_band_join_rows",
    "CFG_BAND_MINR":   "r_band_min_rows",
    "CFG_FIELD_COL":   "r_field_col_min",
    "CFG_WALL_PCT":    "r_wall_col_pct",
    "CFG_WALL_COLMIN": "r_wall_col_min",
    "CFG_EDGE_MIN":    "r_edge_min",
    "CFG_EDGE_COLS":   "r_edge_min_cols",
    "CFG_MIN_SEG":     "r_min_seg_px",
    "CFG_MIN_GAP":     "r_min_gap_px",
    "CFG_P_MIN_AREA":  "r_player_min_area",
    "CFG_P_MAX_AREA":  "r_player_max_area",
    "CFG_P_FILL":      "r_player_min_fill_pct",
    "CFG_P_ASPECT":    "r_player_max_aspect_pct",
}


# ---------------------------------------------------------------------------
# 解析：从 C 工具里抠事实
# ---------------------------------------------------------------------------
def _c_defines(path: pathlib.Path) -> dict[str, int]:
    """C 文件里的 ``#define NAME 0xNNN`` / ``#define NAME NNN``。"""
    src = path.read_text(encoding="utf-8")
    out: dict[str, int] = {}
    for name, raw in re.findall(
            r"^#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+)\s*$", src, re.M):
        out[name] = int(raw, 0)
    return out


def _soak_cfg_table() -> list[tuple[int, int, str]]:
    """axi_soak.c 的 CFG[] 表：``{ 0x000,  8, "WALL_RB" },``。"""
    src = (TOOLS / "axi_soak.c").read_text(encoding="utf-8")
    body = src.split("static const struct", 1)[1].split("};", 1)[0]
    return [(int(o, 16), int(b), n) for o, b, n in re.findall(
        r'\{\s*(0x[0-9A-Fa-f]+)\s*,\s*(\d+)\s*,\s*"([A-Z_]+)"\s*\}', body)]


# ---------------------------------------------------------------------------
# 1) 偏移量契约
# ---------------------------------------------------------------------------
# C 工具里的宏名 → vision_axi.v 的 localparam 名
OFF_PAIRS = {
    "OFF_CTRL":   "ADDR_CTRL",
    "OFF_FLEN":   "ADDR_FLEN",
    "OFF_FRAME":  "ADDR_FRAME",
    "OFF_SEQ":    "ADDR_SEQ",
    "OFF_STAMP0": "ADDR_STAMP0",
    "OFF_STAMP1": "ADDR_STAMP1",
}


@pytest.mark.parametrize("c_path", C_SOURCES, ids=lambda p: p.name)
def test_tool_offsets_match_rtl(c_path: pathlib.Path) -> None:
    """C 工具里出现的每个已知 OFF_* 都必须等于 RTL 的 ADDR_*。"""
    axi = _axi_addrs()
    defines = _c_defines(c_path)
    checked = 0
    for macro, rtl_name in OFF_PAIRS.items():
        if macro not in defines:
            continue
        assert rtl_name in axi, f"vision_axi.v 里找不到 {rtl_name}"
        assert defines[macro] == axi[rtl_name], (
            f"{c_path.name} 的 {macro} = {defines[macro]:#05x}，"
            f"而 vision_axi.v 的 {rtl_name} = {axi[rtl_name]:#05x} —— "
            "偏移漂了，板上会读写到别的寄存器，且不会报错")
        checked += 1
    assert checked > 0, f"{c_path.name} 里一个已知的 OFF_* 都没解析到，正则该更新了"


def test_axi_soak_cfg_table_matches_vision_top() -> None:
    """axi_soak.c 的 CFG[] 表：名字、偏移、位宽三项都要和 vision_top.v 对上。"""
    idx = _cfg_indices()
    widths = _cfg_reg_widths()
    table = _soak_cfg_table()
    assert table, "axi_soak.c 的 CFG[] 表没解析出来，正则该更新了"

    for off, bits, cname in table:
        rtl_name = "CFG_" + cname
        assert rtl_name in idx, (
            f"axi_soak.c 有配置寄存器 {cname}，但 vision_top.v 里没有 "
            f"{rtl_name} —— 名字拼错了，或者 RTL 那边改名了")
        assert off == idx[rtl_name] * 4, (
            f"{cname} 的偏移错了：axi_soak.c 写 {off:#05x}，"
            f"vision_top.v 的 {rtl_name} = {idx[rtl_name]} → 应为 "
            f"{idx[rtl_name] * 4:#05x}（配置寄存器按字索引，偏移 = 序号×4）")
        assert rtl_name in CFG_TO_REG, (
            f"{rtl_name} 没在 CFG_TO_REG 里登记 —— 新加了配置寄存器就要补上，"
            "否则位宽没人守")
        reg = CFG_TO_REG[rtl_name]
        assert reg in widths, f"vision_top.v 里找不到 {reg} 的位宽声明"
        assert bits == widths[reg], (
            f"{cname} 的位宽错了：axi_soak.c 写 {bits} 位，而 vision_top.v 的 "
            f"{reg} 是 {widths[reg]} 位。压测会按错的位宽做掩码，"
            "读回比对就会假报不一致（或漏掉高位的错）")


def test_soak_cfg_table_covers_every_writable_register() -> None:
    """RTL 里每个可写的 CFG_* 都要出现在 axi_soak.c 的表里（CFG_STATUS 除外）。

    防止"新加了个配置寄存器，但板上压测没跟上"——那样新寄存器永远没人压。
    """
    idx = _cfg_indices()
    have = {"CFG_" + n for _, _, n in _soak_cfg_table()}
    missing = sorted(n for n in idx if n not in have and n != "CFG_STATUS")
    assert not missing, (
        f"vision_top.v 里这些配置寄存器没进 axi_soak.c 的 CFG[] 表：{missing}；"
        "它们永远不会被板上压测覆盖（CFG_STATUS 是只读的，可以不在表里）")


# ---------------------------------------------------------------------------
# 2) 编译守卫
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("c_path", C_SOURCES, ids=lambda p: p.name)
def test_tool_compiles_warning_free(c_path: pathlib.Path, tmp_path: pathlib.Path) -> None:
    """``-std=c11 -Wall -Wextra -Werror`` 下必须零告警编过。

    用严格 c11 而不是默认 gnu11 是刻意的：POSIX 的 usleep/sigaction 在
    c11 下不声明，只有把 `_POSIX_C_SOURCE` 写进源文件才稳 —— 依赖编译选项
    的写法在板子上换个命令就编不过了，而板子上编译是唯一的验证途径。
    """
    gcc = shutil.which("gcc") or shutil.which("cc")
    if gcc is None:
        pytest.skip("没有 gcc")
    r = subprocess.run(
        [gcc, "-O2", "-Wall", "-Wextra", "-std=c11", "-Werror",
         "-c", str(c_path), "-o", str(tmp_path / (c_path.stem + ".o"))],
        capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, (
        f"{c_path.name} 在 -Wall -Wextra -Werror 下有告警/错误：\n"
        f"{r.stdout}\n{r.stderr}")


@pytest.mark.parametrize("sh_path", SH_SCRIPTS, ids=lambda p: p.name)
def test_shell_script_syntax(sh_path: pathlib.Path) -> None:
    """``bash -n`` 过一遍：脚本平时只在板上跑，语法错了没人知道。"""
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("没有 bash")
    r = subprocess.run([bash, "-n", str(sh_path)],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, f"{sh_path.name} 语法错误：\n{r.stderr}"


def test_no_bare_usleep_in_tools() -> None:
    """工具里不许出现裸 ``usleep``。

    POSIX.1-2008 已经把它摘掉了，`-std=c11` 下是隐式声明 —— 而隐式声明在
    64 位下会静默截断参数。统一走各自的 ``sleep_ms()``。
    """
    bad: list[str] = []
    for c_path in C_SOURCES:
        for i, line in enumerate(c_path.read_text(encoding="utf-8").splitlines(), 1):
            code = line.split("/*")[0]
            if re.search(r"(?<![\w.])usleep\s*\(", code):
                bad.append(f"{c_path.name}:{i}")
    assert not bad, f"这些地方还在用 usleep，改用 sleep_ms()：{bad}"
