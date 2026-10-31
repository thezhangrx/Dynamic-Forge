"""RTL 与 Python 的**跨语言契约核对**。

为什么需要它
------------
`core/fpga/rtl/vision_pkg.vh` 里的 magic / version / 定标 / 记录字节数，
必须和 `core/standard/vision/vision_frame.py: describe_vision_protocol()`
**逐个相等**。两边一旦漂移，症状是"PS 解出来的字段整体错位"或
"CRC 永远不过" —— 而这类错误在板上极难定位（帧看起来有数据，只是全错）。

这种核对**不需要 Verilator**：直接从 `.vh` 里解析宏，和 Python 常量比。
所以它是当前环境（无仿真工具）下**唯一能真正跑起来**的 RTL 检查，
价值比"写一堆没人编译过的 Verilog"高。

它同时也是"接口已冻结"的证据：这个测试绿 = HDL 与 Python 说的是同一件事。
"""

from __future__ import annotations

import pathlib
import re

import pytest

from standard.vision.vision_frame import (
    GAP_BYTES,
    HEADER_BYTES,
    OBSTACLE_BYTES,
    PLAYER_BYTES,
    TARGET_BYTES,
    TRAILER_BYTES,
    VISION_MAGIC,
    VISION_PROTOCOL_VERSION,
    VISION_Q,
    describe_vision_protocol,
    vision_frame_size,
)

_VH = pathlib.Path(__file__).resolve().parents[1] / "rtl" / "vision_pkg.vh"

#: 从 ``\`define NAME value`` 里取值。忽略尾注释。
#: Verilog 的定宽字面量有 ``32'h31564842`` / ``6'd64`` / ``8'b1010`` 三种写法，
#: 宽度前缀必须一起吃掉 —— 只认纯十进制会把一半宏漏掉（实测漏了 magic 与 Q）。
_DEFINE = re.compile(
    r"^`define\s+(\w+)\s+([0-9]+'[hHbBdDoO][0-9A-Fa-f]+|[0-9]+)\s*(?://.*)?$",
    re.MULTILINE)


def _macros() -> dict[str, int]:
    if not _VH.exists():
        pytest.fail(f"找不到 RTL 头文件 {_VH}；契约核对无从谈起")
    text = _VH.read_text(encoding="utf-8", errors="replace")
    out: dict[str, int] = {}
    for name, value in _DEFINE.findall(text):
        # 支持 32'h... / 16'd... / 6'd6 这几种写法
        v = value
        if "'" in v:
            base, digits = v.split("'", 1)
            radix = {"h": 16, "d": 10, "b": 2, "o": 8}[digits[0].lower()]
            out[name] = int(digits[1:], radix)
        else:
            out[name] = int(v, 10)
    return out


def test_rtl_magic_and_version_match_python() -> None:
    m = _macros()
    assert m["VISION_MAGIC"] == VISION_MAGIC, (
        f"magic 不一致：RTL 0x{m['VISION_MAGIC']:08X} vs Python 0x{VISION_MAGIC:08X}")
    assert m["VISION_VERSION"] == VISION_PROTOCOL_VERSION


def test_rtl_quantisation_scale_matches_python() -> None:
    """定标必须一致：RTL 用"右移 log2(Q)"，Python 用 Q。

    这条最容易错 —— 一旦 Q 不同，所有几何量整体差一个倍数，
    而帧结构、CRC 全都正常，只有数值全错。
    """
    m = _macros()
    assert m["VISION_Q_DIV"] == VISION_Q, (
        f"定标不一致：RTL {m['VISION_Q_DIV']} vs Python {VISION_Q}")
    assert (1 << m["VISION_Q"]) == VISION_Q, (
        f"RTL 的 log2 定标与 Q 不符：1<<{m['VISION_Q']} != {VISION_Q}")


@pytest.mark.parametrize("name,expected", [
    ("VISION_HEADER_BYTES", HEADER_BYTES),
    ("VISION_PLAYER_BYTES", PLAYER_BYTES),
    ("VISION_TARGET_BYTES", TARGET_BYTES),
    ("VISION_OBSTACLE_BYTES", OBSTACLE_BYTES),
    ("VISION_GAP_BYTES", GAP_BYTES),
    ("VISION_TRAILER_BYTES", TRAILER_BYTES),
])
def test_rtl_record_sizes_match_python(name: str, expected: int) -> None:
    assert _macros()[name] == expected, f"{name} 与 Python 不一致"


def test_rtl_fixed_bytes_and_size_formula_agree() -> None:
    """定长部分与"总长公式"两边都要对得上。"""
    m = _macros()
    assert m["VISION_FIXED_BYTES"] == HEADER_BYTES + PLAYER_BYTES + TARGET_BYTES + TRAILER_BYTES
    # 用真实编码器验证公式：76 + 36*n_obs + 36*n_gaps
    for n_obs, n_gaps in ((0, 0), (1, 0), (0, 1), (3, 2)):
        want = m["VISION_FIXED_BYTES"] + OBSTACLE_BYTES * n_obs + GAP_BYTES * n_gaps
        assert vision_frame_size(n_obs, n_gaps) == want, (n_obs, n_gaps)


def test_rtl_flag_bit_positions_match_python() -> None:
    """flags 的**位号**必须一致（位号错 = 语义错，CRC 照样过）。"""
    m = _macros()
    assert m["VF_UNITS_M"] == 0
    assert m["VF_PLAYER_VALID"] == 1
    assert m["VF_TARGET_VALID"] == 2
    assert m["VAF_VALID"] == 0 and m["VAF_OCCLUDED"] == 1
    assert m["VGF_RELIABLE"] == 0 and m["VGF_OCCLUDED"] == 1


def test_rtl_shape_and_type_codes_match_python() -> None:
    """shape/type 编码表两边必须一致（只追加不重排）。"""
    d = describe_vision_protocol()
    m = _macros()
    assert m["VSHAPE_CIRCLE"] == d["shape_codes"]["circle"]
    assert m["VSHAPE_RECT"] == d["shape_codes"]["rect"]
    assert m["VTYPE_NONE"] == d["type_codes"]["None"]
    assert m["VTYPE_MOVING_BLOCK"] == d["type_codes"]["ObstacleType.MOVING_BLOCK"]
    assert m["VTYPE_WALL_WITH_GAP"] == d["type_codes"]["ObstacleType.WALL_WITH_GAP"]
    assert m["VTYPE_SMALL_OBSTACLES"] == d["type_codes"]["ObstacleType.SMALL_OBSTACLES"]


def test_rtl_has_no_velocity_fields() -> None:
    """速度**不在**线协议里 —— RTL 侧也不许偷偷加。

    理由见 ``vision_frame.py`` 顶部：PL 不知道 speed（速度上限），
    带上 vx/vy 会让 PS 解出的 PlayerState 自校验失败。
    """
    text = _VH.read_text(encoding="utf-8", errors="replace")
    for bad in ("vx", "vy", "velocity", "VELOCITY"):
        assert bad not in text, f"RTL 头里出现了速度字段 {bad!r} —— 它不该在线协议里"


def test_rtl_frame_limits_are_declared() -> None:
    """帧内记录数上限必须显式声明（影响 BRAM 与时延预算）。"""
    m = _macros()
    assert m["VISION_MAX_OBSTACLES"] > 0 and m["VISION_MAX_GAPS"] > 0
    # 上限不能是"看起来够用"就随手写的数：一条墙=2 段，实测一屏最多 ~8 堵
    assert m["VISION_MAX_OBSTACLES"] >= 16, "障碍上限太小，放不下一屏的墙段"


# ---------------------------------------------------------------------------
# vision_ops.vh：定点算子的表与 C 参考模型必须**逐项相等**
#
# 这几张表是"逐位一致"里最容易悄悄漂移的地方：改一个数不会让任何东西报错，
# 只会让 RTL 和 C 在最后一位上分叉，而 cosim 要跑 300 帧才看得出来。
# ---------------------------------------------------------------------------
_OPS_VH = pathlib.Path(__file__).resolve().parents[1] / "rtl" / "vision_ops.vh"
_VISION_TOP = pathlib.Path(__file__).resolve().parents[1] / "rtl" / "vision_top.v"
_REF_C = pathlib.Path(__file__).resolve().parents[1] / "ref" / "vision_ref.c"

_AT_ENTRY = re.compile(r"4'd(\d+):\s*vo_at\s*=\s*16'd(\d+);")
_AT_DEFAULT = re.compile(r"default:\s*vo_at\s*=\s*16'd(\d+);")
_C_AT = re.compile(r"AT\[9\]\s*=\s*\{([^}]*)\}")


def _rtl_at_table() -> list[int]:
    text = _OPS_VH.read_text(encoding="utf-8", errors="replace")
    out = {int(i): int(v) for i, v in _AT_ENTRY.findall(text)}
    defaults = _AT_DEFAULT.findall(text)
    assert len(defaults) == 1, "vo_at() 必须正好有一个 default 分支"
    out[8] = int(defaults[0])          # default 对应 i = 8
    return [out[i] for i in range(9)]


def _c_at_table() -> list[int]:
    text = _REF_C.read_text(encoding="utf-8", errors="replace")
    m = _C_AT.search(text)
    assert m, "在 vision_ref.c 里找不到 AT[9] 表"
    return [int(x) for x in m.group(1).replace(" ", "").split(",")]


def test_atan_table_matches_between_rtl_and_c() -> None:
    """arctan 表两边必须一模一样 —— 差一项就是最后一位上的分叉。"""
    assert _rtl_at_table() == _c_at_table(), (
        f"arctan 表不一致：RTL {_rtl_at_table()} vs C {_c_at_table()}")


def test_atan_table_is_actually_arctan() -> None:
    """表值必须真的是 ``round(atan(i/8) * 256)``，不是随手填的数。

    只查"两边相等"是不够的 —— 两边一起写错也是相等。这里对真值做一次核对。
    """
    import math
    for i, v in enumerate(_rtl_at_table()):
        want = math.atan(i / 8.0) * 256.0
        assert abs(v - want) <= 0.51, (
            f"AT[{i}] = {v}，但 atan({i}/8)*256 = {want:.3f}")


def test_edge_lag_is_two_because_rtl_hardcodes_two_stages() -> None:
    """``EDGE_AVG`` 必须等于 2。

    RTL 的"持续台阶"是**两级移位寄存器**（列内向下扫时比较 ``gray[y]`` 与
    ``gray[y+2]``）。把参数改成别的值不会报错，会静默算错 —— 所以在这里钉住。
    改这个数就必须同步改 ``vision_top.v`` 的 S_COL_SCAN。
    """
    rtl = _VISION_TOP.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"localparam integer EDGE_AVG\s*=\s*(\d+);", rtl)
    assert m, "在 vision_top.v 里找不到 EDGE_AVG"
    assert int(m.group(1)) == 2, "EDGE_AVG 改了就必须同步改两级移位寄存器"

    c = _REF_C.read_text(encoding="utf-8", errors="replace")
    mc = re.search(r"\.edge_avg\s*=\s*(\d+)", c)
    assert mc, "在 vision_ref.c 的 VR_DEFAULT_PARAMS 里找不到 edge_avg"
    assert int(mc.group(1)) == 2, "C 参考模型的 edge_avg 也必须等于 2"


# ---------------------------------------------------------------------------
# 配置寄存器的**地址映射**三方一致
#
# RTL（vision_top.v 的 CFG_*）、协同仿真 tb（tb_vision.cpp 的 enum）、
# C 参考模型（ref_api.h 的 VrRefParams 注释）各写了一遍。地址错位不会让
# 任何东西编译失败，只会让"改阈值"改到别的寄存器上 —— 而那看起来就是
# "阈值不管用"，在板上极难定位。
# ---------------------------------------------------------------------------
_CFG_RTL = re.compile(r"CFG_(\w+)\s*=\s*6'd(\d+)")
_CFG_TB = re.compile(r"CFG_(\w+)\s*=\s*(\d+)", )


def _cfg_rtl() -> dict[str, int]:
    text = _VISION_TOP.read_text(encoding="utf-8", errors="replace")
    block = re.search(r"localparam \[5:0\] CFG_WALL_RB.*?CFG_STATUS\s*=\s*6'd\d+;",
                      text, re.S)
    assert block, "在 vision_top.v 里找不到 CFG_* 地址表"
    return {m.group(1): int(m.group(2)) for m in _CFG_RTL.finditer(block.group(0))}


def _cfg_tb() -> dict[str, int]:
    text = (_VISION_TOP.parents[1] / "cosim" / "tb_vision.cpp").read_text(
        encoding="utf-8", errors="replace")
    block = re.search(r"enum \{.*?CFG_STATUS\s*=\s*\d+\s*\n\};", text, re.S)
    assert block, "在 tb_vision.cpp 里找不到 CFG_* enum"
    return {m.group(1): int(m.group(2)) for m in _CFG_TB.finditer(block.group(0))}


def test_config_register_map_matches_between_rtl_and_cosim() -> None:
    rtl, tb = _cfg_rtl(), _cfg_tb()
    assert rtl == tb, f"配置寄存器地址表不一致：RTL {rtl} vs cosim {tb}"
    # 顺序也要对：address = VrRefParams 里的字段顺序（ref_api.h 有注释）
    order = ["WALL_RB", "WALL_MIN_R", "WALL_MIN_G", "BRIGHT_MIN", "BRIGHT_SPRD",
             "GREEN_GM", "GREEN_MIN",
             "BAND_PCT", "BAND_JOIN", "BAND_MINR", "FIELD_COL",
             "WALL_PCT", "WALL_COLMIN", "EDGE_MIN", "EDGE_COLS",
             "MIN_SEG", "MIN_GAP",
             "P_MIN_AREA", "P_MAX_AREA", "P_FILL", "P_ASPECT"]
    assert [rtl[n] for n in order] == list(range(21)), "配置寄存器地址必须是 0..20 连续"
    assert rtl["STATUS"] == 31, "状态寄存器固定在 31"


# ---------------------------------------------------------------------------
# AXI 包装层（vision_axi.v）的**地址映射**与协同仿真 tb 一致
#
# 地址译码错位的症状和协议常量漂移一样：编译全过、板上"读回来全是 0"，
# 极难定位。tb_axi.cpp 里那套 A_* 枚举是"PS 会怎么访问"的书面化，
# 它必须和 RTL 的 ADDR_* 逐项相等。
# ---------------------------------------------------------------------------
_AXI_V = pathlib.Path(__file__).resolve().parents[1] / "rtl" / "vision_axi.v"
_TB_AXI = pathlib.Path(__file__).resolve().parents[1] / "cosim" / "tb_axi.cpp"

_ADDR_RTL = re.compile(r"ADDR_(\w+)\s*=\s*12'h([0-9A-Fa-f]+)")
_ADDR_TB = re.compile(r"A_(\w+)\s*=\s*(0x[0-9A-Fa-f]+)")


def _addr_rtl() -> dict[str, int]:
    text = _AXI_V.read_text(encoding="utf-8", errors="replace")
    return {m.group(1): int(m.group(2), 16) for m in _ADDR_RTL.finditer(text)}


def _addr_tb() -> dict[str, int]:
    text = _TB_AXI.read_text(encoding="utf-8", errors="replace")
    return {m.group(1): int(m.group(2), 16) for m in _ADDR_TB.finditer(text)}


def test_axi_address_map_matches_between_rtl_and_cosim() -> None:
    rtl, tb = _addr_rtl(), _addr_tb()
    # 两边命名不完全一样：RTL 的 ADDR_DBGDAT 在 tb 里叫 A_DBG_DATA。
    alias = {"DBG": "DBG_SEL", "DBGDAT": "DBG_DATA"}
    for name, val in rtl.items():
        if name in ("CONFIG", "FRAME_END"):
            continue
        key = alias.get(name, name)
        assert key in tb, f"协同仿真 tb 里没有 A_{key}（RTL 侧 ADDR_{name} = {val:#x}）"
        assert tb[key] == val, f"A_{key} = {tb[key]:#x}，RTL ADDR_{name} = {val:#x}"
    # 窗口必须落在 0x100..0xFFF（帧窗口下标就是按这段地址取的）
    assert rtl["FRAME"] == 0x100
    assert rtl["CONFIG"] == 0x000


# ---------------------------------------------------------------------------
# **参数默认值三方一致**：Python（语义真值） ↔ C 参考模型 ↔ RTL 寄存器复位值
#
# 这道守卫是补上的，而且它本该早点存在：
# 曾经 C 模型与 RTL 把 wall 的 G 下限写成 110（Python 是 60）、bright 少了
# `max-min < bright_spread` 这条、green 写成 50/90（Python 是 10/90）。
# 三处都偏了，而 **"RTL == C 模型" 全程全绿** —— 因为两边错得一模一样。
# 症状是 P1 对账里"缺口条数 Python 160 / C 159、墙段 320 / C 318"，
# 当时被当成"连通域/边沿语义的正常差异"，其实判据就不一样。
# 修完之后 P1 变成 160/160、320/320，缺口中心误差 max 0.16 → 0.00 px。
#
# 所以：**任何一层的默认参数都必须与 Python 的 WallWithGapParams 逐项相等**。
# ---------------------------------------------------------------------------
_REF_C = pathlib.Path(__file__).resolve().parents[1] / "ref" / "vision_ref.c"

#: Python 字段名 → (C/RTL 字段名, 缩放)。缩放用于把 0.05 / 0.25 这类比例
#: 换成 C/RTL 用的百分数整数（避免 (int)(0.05*w) 在个别宽度上差一）。
_PARAM_MAP: dict[str, tuple[str, int]] = {
    "wall_diff":         ("wall_rb", 1),
    "wall_min_r":        ("wall_min_r", 1),
    "wall_min_g":        ("wall_min_g", 1),
    "bright_min":        ("bright_min", 1),
    "bright_spread":     ("bright_spread", 1),
    "green_score":       ("green_gm", 1),
    "green_min_g":       ("green_min", 1),
    "band_row_frac":     ("band_row_pct", 100),
    "band_join_rows":    ("band_join_rows", 1),
    "band_min_rows":     ("band_min_rows", 1),
    "field_col_min":     ("field_col_min", 1),
    "wall_col_frac":     ("wall_col_pct", 100),
    "wall_col_min":      ("wall_col_min", 1),
    "edge_min":          ("edge_min", 1),
    "edge_min_cols":     ("edge_min_cols", 1),
    "min_seg_px":        ("min_seg_px", 1),
    "min_gap_px":        ("min_gap_px", 1),
    "player_min_area":   ("player_min_area", 1),
    "player_max_area":   ("player_max_area", 1),
    "player_min_fill":   ("player_min_fill_pct", 100),
    "player_max_aspect": ("player_max_aspect_pct", 100),
}

_RTL_RST = re.compile(r"r_(\w+)\s*<=\s*\d+'d(\d+);")
_C_DEFAULT = re.compile(r"\.(\w+)\s*=\s*(\d+)")


def _python_params() -> dict[str, float]:
    from vision.wall_with_gap import WallWithGapParams
    p = WallWithGapParams()
    return {name: getattr(p, name) for name in _PARAM_MAP}


def _c_params() -> dict[str, int]:
    text = _REF_C.read_text(encoding="utf-8", errors="replace")
    blk = re.search(r"VR_DEFAULT_REF_PARAMS\s*=\s*\{(.*?)\};", text, re.S)
    assert blk, "在 vision_ref.c 里找不到 VR_DEFAULT_REF_PARAMS"
    return {m.group(1): int(m.group(2)) for m in _C_DEFAULT.finditer(blk.group(1))}


def _rtl_params() -> dict[str, int]:
    text = _VISION_TOP.read_text(encoding="utf-8", errors="replace")
    # 复位块：从第一个寄存器到 `end else if (cfg_we)` 之前
    blk = re.search(r"if \(!rst_n\) begin\s*\n\s*r_wall_rb.*?\n\s*end else if \(cfg_we\)",
                    text, re.S)
    assert blk, "在 vision_top.v 里找不到配置寄存器复位块"
    return {m.group(1): int(m.group(2)) for m in _RTL_RST.finditer(blk.group(0))}


def test_default_params_match_python_truth_in_c_and_rtl() -> None:
    py = _python_params()
    c, rtl = _c_params(), _rtl_params()

    for py_name, (hw_name, scale) in _PARAM_MAP.items():
        want = int(round(py[py_name] * scale))
        assert hw_name in c, f"vision_ref.c 的默认参数里缺 {hw_name}"
        assert hw_name in rtl, f"vision_top.v 的寄存器复位值里缺 r_{hw_name}"
        assert c[hw_name] == want, (
            f"C 模型 {hw_name} = {c[hw_name]}，但 Python 的 {py_name} = "
            f"{py[py_name]}（应折成 {want}）")
        assert rtl[hw_name] == want, (
            f"RTL {hw_name} 复位值 = {rtl[hw_name]}，但 Python 的 {py_name} = "
            f"{py[py_name]}（应折成 {want}）")
