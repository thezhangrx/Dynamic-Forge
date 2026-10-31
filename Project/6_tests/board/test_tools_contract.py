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


def _soak_default_table() -> list[int]:
    """axi_soak.c 的 DEF[] 表（复位默认值，顺序与 CFG[] 一致）。"""
    src = (TOOLS / "axi_soak.c").read_text(encoding="utf-8")
    m = re.search(r"static const uint32_t DEF\[NCFG\]\s*=\s*\{(.*?)\};", src, re.S)
    assert m, "axi_soak.c 里找不到 DEF[NCFG] 表"
    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)   # 去掉行内注释
    return [int(x) for x in re.findall(r"\d+", body)]


def _vision_top_reset_defaults() -> dict[str, int]:
    """vision_top.v 配置寄存器复位块里的 ``r_xxx <= N'dV;``。

    只取 ``配置寄存器`` 那个 always 块（到 ``end else if (cfg_we)`` 为止），
    免得把别处的复位值也扫进来。
    """
    src = TOP.read_text(encoding="utf-8")
    assert "配置寄存器" in src, "vision_top.v 里找不到配置寄存器复位块"
    block = src.split("配置寄存器", 1)[1].split("end else if (cfg_we)", 1)[0]
    return {n: int(v) for n, _w, v in re.findall(
        r"(r_\w+)\s*<=\s*(\d+)'d(\d+)", block)}


# ---------------------------------------------------------------------------
# 1) 偏移量契约
# ---------------------------------------------------------------------------
# C 工具里的宏名 → vision_axi.v 的 localparam 名
OFF_PAIRS = {
    "OFF_CTRL":     "ADDR_CTRL",
    "OFF_FLEN":     "ADDR_FLEN",
    "OFF_FRAME":    "ADDR_FRAME",
    "OFF_SEQ":      "ADDR_SEQ",
    "OFF_STAMP0":   "ADDR_STAMP0",
    "OFF_STAMP1":   "ADDR_STAMP1",
    # 像素通路（PS 推像素那条）：`tools/pix_overflow.c` 只用这三个，
    # 而它要在真芯片上灌满 FIFO —— 偏移漂了会去写别的寄存器，且不报错。
    "OFF_PIXPUSH":  "ADDR_PIXPUSH",
    "OFF_PIXSTAT":  "ADDR_PIXSTAT",
    "OFF_PIXCTL":   "ADDR_PIXCTL",
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


def test_soak_default_table_matches_vision_top_reset_block() -> None:
    """axi_soak.c 的 DEF[] 必须等于 vision_top.v 复位块里的值。

    为什么重要：Phase F 的 8 个期望 CRC 是**在默认配置下**跑出来的。DEF[] 是
    同一份知识的**第三份拷贝**（另两份是 RTL 复位块与 VR_DEFAULT_REF_PARAMS），
    用来判断"当前配置是不是默认值" —— 它漂了，工具就会在配置其实正常时
    错误地警告、或者在配置不正常时不警告，两种都会把失败归错因。
    """
    rst = _vision_top_reset_defaults()
    assert rst, "vision_top.v 的复位块没解析出任何 r_xxx <= N'dV，正则该更新了"
    table = _soak_cfg_table()
    defs = _soak_default_table()
    assert len(defs) == len(table), (
        f"axi_soak.c 的 DEF[] 有 {len(defs)} 项，CFG[] 有 {len(table)} 项 —— 必须一一对应")

    for (off, _bits, cname), got in zip(table, defs):
        rtl_name = "CFG_" + cname
        assert rtl_name in CFG_TO_REG, f"{rtl_name} 没在 CFG_TO_REG 里登记"
        reg = CFG_TO_REG[rtl_name]
        assert reg in rst, (
            f"vision_top.v 的复位块里找不到 {reg}（{cname} 的目标寄存器）")
        assert got == rst[reg], (
            f"{cname} 的复位默认值漂了：axi_soak.c 的 DEF[] 写 {got}，"
            f"而 vision_top.v 复位块里 {reg} <= {rst[reg]}。"
            "Phase F 的期望 CRC 是以默认配置为前提的，漂了会把失败归错因")


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


# ---------------------------------------------------------------------------
# Windows 侧的两个 .ps1：编码和顺序都是"错了就白折腾一轮"的坑
# ---------------------------------------------------------------------------
def test_windows_ps1_files_are_utf8_with_bom() -> None:
    """两个 `.ps1` 都必须是 **UTF-8 with BOM**。

    Windows PowerShell 5.1 对没有 BOM 的脚本按系统 ANSI 代码页（中文机 GBK）解码：
    中文注释被解坏会把语法拆掉，症状是 `-File` 起来就 ParserError，而且只在注释
    恰好解出引号类字符时才发作 —— 改一条注释就可能突然坏掉，极难查。
    """
    # win_sendinput.ps1 住在 core/rp2040_keyboard/（它是键盘链路的），不在 tools/
    for path in (TOOLS / "win_console.ps1",
                 ROOT / "core" / "rp2040_keyboard" / "win_sendinput.ps1"):
        raw = path.read_bytes()
        assert raw.startswith(b"\xef\xbb\xbf"), \
            f"{path.name} 少了 UTF-8 BOM（PowerShell 5.1 会按 GBK 解坏中文注释）"
        raw.decode("utf-8-sig")          # 必须是合法 UTF-8


def test_capture_boot_warns_about_capture_before_power_on() -> None:
    """抓启动日志必须**先开始抓、再上电**：UART 没有回放缓冲。

    顺序反了会一个字都拿不到，而排查时最容易白折腾的就是这一轮。
    """
    sh = (TOOLS / "capture_boot.sh").read_text(encoding="utf-8")
    assert "win_console.ps1" in sh, "capture_boot.sh 应当复用 Windows 侧读串口的脚本"
    assert "没有回放缓冲" in sh, "必须写明 UART 没有回放缓冲"
    assert "断电再上电" in sh or "上电**之前**" in sh, "必须给出正确的操作顺序"
    assert "MobaXterm" in sh, "必须提醒先关掉别的串口程序（一个串口只能一个程序读）"
    # 结果要落到 data/ 下（gitignore 的目录），别塞进仓库
    assert "data/fpga/boot_logs" in sh, "日志应当写进 data/fpga/boot_logs/"


def test_run_loop_is_safe_and_self_restarting() -> None:
    """常驻闭环脚本的几条硬约束：默认一直跑、有停止钩子、停相机的方式必须安全。"""
    sh = (TOOLS / "run_loop.sh").read_text(encoding="utf-8")
    assert "FRAMES=0" in sh, "默认必须不限帧数（一条命令一直跑）"
    assert "trap on_stop INT TERM" in sh, "必须有停止钩子：收尾要松键 + 停相机"
    # 停相机：按进程名精确匹配（-x），不能用 -f（会连命令行含该串的 shell 一起杀，
    # 这个坑在 WSL 上踩过一次）；ffmpeg 一律 -INT，绝不 -9（-9 会把 V4L2 设备卡住）。
    assert "pkill -INT -x ffmpeg" in sh, "停 ffmpeg 要用 -INT -x"
    # ⚠ 只查**非注释行**：脚本注释里正好在讲"不能用 pkill -f"这个教训（把坑写下来），
    #   按字面断言会把这段说明本身判成违规 —— 断言要盯代码，不盯注释。
    code = "\n".join(ln for ln in sh.splitlines()
                     if not ln.lstrip().startswith("#"))
    assert "pkill -f" not in code, "不能用 pkill -f（会误杀自己的 shell）"
    assert "pkill -9" not in code and "kill -9" not in code, "ffmpeg 绝不 -9"
    assert "--probe" in sh, "开跑前必须先自检注入器（否则按键可能全打空）"
    assert "focus" in sh.lower() or "焦点" in sh, "必须提醒/报出焦点窗口"
    assert "data/fpga/loop_logs" in sh, "全程日志要落到 data/ 下"


def test_pix_overflow_tool_is_hardware_safe() -> None:
    """强制触发像素 FIFO 溢出的工具：只准碰像素寄存器，且必须能自证失败。

    它要在**真芯片**上灌满 FIFO，所以"别碰不该碰的寄存器"是硬要求：写 CTRL 的
    start 位会把流水线搅起来、写配置寄存器会把阈值弄乱。
    """
    c = (TOOLS / "pix_overflow.c").read_text(encoding="utf-8")
    for off in ("OFF_PIXPUSH", "OFF_PIXSTAT", "OFF_PIXCTL"):
        assert off in c, f"缺少 {off}"
    assert "0x0C0" not in c and "OFF_CTRL" not in c, \
        "不准碰 CTRL（start 位会把流水线搅起来）"
    assert "0x000" not in c and "kOffConfig" not in c, "不准碰配置寄存器"
    assert "return fails ? 1 : 0" in c, "断言不成立时必须用非零退出码报告"
    assert "*(volatile" not in c or "*ctl = 0x0u" in c, \
        "收尾要把像素源切回 din_*（0），否则后续 run_frame 会以为还有像素"


def test_soak_tools_are_read_only_and_wait_for_the_app() -> None:
    """长跑工具的两条硬约束：采样只读、等程序出现（别固定 sleep）。"""
    sh = (TOOLS / "soak_loop.sh").read_text(encoding="utf-8")
    assert "pidof vision_board_devmem" in sh, "要按进程名找 PID"
    assert "for _ in $(seq 1 60)" in sh, \
        "必须**轮询**等程序出现：相机热身要几秒，固定 sleep 会白跑一整轮（踩过）"
    assert "pkill -INT -x" in sh, "停程序按进程名精确匹配（不用 -f）"
    assert "kill -9" not in sh and "killall" not in sh, "绝不用 -9"
    c = (TOOLS / "soak_sampler.c").read_text(encoding="utf-8")
    assert "OFF_PIXSTAT" in c and "OFF_CTRL" in c and "OFF_FLEN" in c
    # 采样器只能读。⚠ 要排除**指针声明行**（`volatile uint32_t *ctrl = ...` 里也含
    # `*ctrl =`，按字面查会把声明判成写 —— 这条断言第一次就是这么误报的）。
    body = "\n".join(ln for ln in c.splitlines() if "volatile uint32_t *" not in ln)
    for bad in ("*ctrl =", "*pstat =", "*flen ="):
        assert bad not in body, f"采样器不该写寄存器（发现 {bad}）"
    assert "uint32_t c = *ctrl" in c, "采样器应当只读这三个寄存器"
    assert "read_frame" not in sh, \
        "别拿 read_frame 顶替采样：它会写 CTRL 启动一整帧，搅乱正在跑的闭环"


def test_cam_live_never_crops_horizontally() -> None:
    """相机宽度与流水线不同时必须**缩放**，不能裁 —— 裁掉的是半个视野。

    `crop=640:360:0:Y` 对 640x480 是对的（裁中间几行、视野不变）；但如果换成
    1280x720，同一条 crop 会把 1280 宽的画面**只取左半边**：右半场的平台看不见，
    而且不报错，只表现为"检测不到东西"。所以 VF 的选择必须带上宽度的判断。
    """
    sh = (TOOLS / "cam_live.sh").read_text(encoding="utf-8")
    code = "\n".join(ln for ln in sh.splitlines() if not ln.lstrip().startswith("#"))
    assert 'CAM_W" -eq "$PIPE_W"' in code, \
        "裁之前必须判断宽度相同（否则宽度不同时会把画面裁掉一半）"
    assert "scale=${PIPE_W}:${PIPE_H}" in code, "宽度不同时要走缩放"
    assert "crop=${PIPE_W}:${PIPE_H}:0:" in code, "宽度相同时才裁中间几行"
