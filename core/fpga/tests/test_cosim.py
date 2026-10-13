#!/usr/bin/env python3
"""P2/P3 验收：**RTL 与 C 参考模型在真实录像帧上逐位一致**。

跑什么
------
`core/fpga/cosim/` 把 Verilator 编出来的 `vision_top` 和 `core/fpga/ref/vision_ref.c`
链进同一个可执行文件，同一帧同一批像素同时喂给两边，逐位比较：

  * 灰度 / 墙 / 亮 三张掩膜、行直方图
  * 行带、场地 x 范围
  * 墙段（障碍）记录、缺口记录
  * 角色位置与连通域包围盒
  * **最终几何帧的全部字节**（含 CRC32）
  * 配置寄存器（阈值等）写下去之后仍然逐位一致

为什么默认只跑几帧
------------------
构建 + 逐帧仿真是**秒级**的活儿，放进默认测试套件会拖慢所有无关改动。
默认帧数由环境变量 `FPGA_COSIM_FRAMES` 控制：

    FPGA_COSIM_FRAMES=320 python -m pytest core/fpga/tests/test_cosim.py -q

320 帧（约 35 s 仿真，2.2 亿次比对，零差异）是**已经跑过并记录在案**的结果，
见 core/fpga/rtl/README.md。

依赖
----
* Verilator（免 root 装在 ``$HOME/.local/verilator-5.020``，见 core/fpga/ref/README.md）
* 真实录像 ``data/vision/run4/screen.mp4``（仓库里不带数据，没有就跳过）
"""

from __future__ import annotations

import os
import pathlib
import re
import shutil
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
COSIM = ROOT / "core" / "fpga" / "cosim"
RTL = ROOT / "core" / "fpga" / "rtl"
REF = ROOT / "core" / "fpga" / "ref"
VIDEO = ROOT / "data" / "vision" / "run4" / "screen.mp4"
EXE = COSIM / "obj_dir" / "Vvision_top"
EXE_AXI = COSIM / "obj_dir" / "Vvision_axi"
EXE_ST = COSIM / "obj_dir" / "Vvision_selftest"

VERILATOR_ROOT = pathlib.Path(
    os.environ.get("VERILATOR_ROOT", pathlib.Path.home() / ".local" / "verilator-5.020")
)
VERILATOR = VERILATOR_ROOT / "bin" / "verilator"

# 默认帧数：够抓住"第二帧起行号不归零"这类状态残留，又不拖慢套件。
DEFAULT_FRAMES = 4


def _verilator() -> str:
    if VERILATOR.exists():
        return str(VERILATOR)
    found = shutil.which("verilator")
    if found:
        return found
    pytest.skip("没有 Verilator（见 core/fpga/ref/README.md 的免 root 装法）")


def _env() -> dict:
    e = dict(os.environ)
    e["VERILATOR_ROOT"] = str(VERILATOR_ROOT)
    e["PATH"] = f"{VERILATOR_ROOT / 'bin'}:{e.get('PATH', '')}"
    return e


# ---------------------------------------------------------------------------
# 1) RTL 必须过 lint（-Wall，除了有意记录在案的几类）
# ---------------------------------------------------------------------------
def test_rtl_lint_clean():
    v = _verilator()
    r = subprocess.run(
        [v, "--lint-only", "-Wall", "-Wno-DECLFILENAME", "--top-module", "vision_axi",
         f"-I{RTL}", str(RTL / "vision_selftest.v"), str(RTL / "vision_tpg.v"),
         str(RTL / "vision_axi.v"), str(RTL / "vision_top.v"),
         str(RTL / "vision_ccl.v"), str(RTL / "vision_div.v")],
        capture_output=True, text=True, env=_env(), cwd=str(COSIM),
    )
    assert r.returncode == 0, r.stdout + r.stderr
    assert "%Warning" not in r.stdout + r.stderr, r.stdout + r.stderr


# ---------------------------------------------------------------------------
# 构建 / 激励：整个模块共用一次（构建 ~20 s，不值得每个用例来一遍）
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def exe() -> pathlib.Path:
    _verilator()
    r = subprocess.run(["make", "-j", "4"], cwd=str(COSIM),
                       capture_output=True, text=True, env=_env())
    assert r.returncode == 0, r.stdout + r.stderr
    assert EXE.exists() and EXE_AXI.exists() and EXE_ST.exists()
    return EXE


@pytest.fixture(scope="module")
def stim(tmp_path_factory: pytest.TempPathFactory) -> tuple[pathlib.Path, int]:
    """导一批彩色 PPM 真实帧。灰度会让 R-B 判据恒为 0，墙根本测不出来。"""
    if not VIDEO.exists():
        pytest.skip("没有真实录像 data/vision/run4/screen.mp4")
    want = int(os.environ.get("FPGA_COSIM_FRAMES", DEFAULT_FRAMES))
    out = tmp_path_factory.mktemp("stim")
    r = subprocess.run(
        [sys.executable, str(REF / "export_frames.py"),
         "--video", str(VIDEO), "--out", str(out),
         "--frames", str(want), "--start", "200", "--stride", "2",
         "--width", "640", "--color"],
        capture_output=True, text=True,
    )
    assert r.returncode == 0, r.stdout + r.stderr
    files = sorted(out.glob("*.ppm"))
    assert files, "一帧激励都没导出来"
    return out, len(files)


def _run(exe: pathlib.Path, stim_dir: pathlib.Path, frames: int,
         *extra: str, timeout: int = 1800) -> subprocess.CompletedProcess:
    return subprocess.run([str(exe), str(stim_dir), str(frames), *extra],
                          capture_output=True, text=True, timeout=timeout)


def _fingerprint(stdout: str) -> str:
    m = re.search(r"几何帧指纹 ([0-9a-f]{16})", stdout)
    assert m, stdout
    return m.group(1)


# ---------------------------------------------------------------------------
# 2) 逐位一致（真实帧）
# ---------------------------------------------------------------------------
def test_cosim_bit_exact_on_real_frames(exe, stim):
    stim_dir, frames = stim
    r = _run(exe, stim_dir, frames)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout
    assert "有差异的帧 0" in r.stdout, r.stdout
    # 每一帧至少要比 3*w*h + h 项（三张掩膜 + 行直方图），防止
    # "一帧都没真比" 也报 0 不符。
    n_checks = int(r.stdout.split("比对项 ")[1].split("，")[0])
    assert n_checks > 600_000 * frames, r.stdout


# ---------------------------------------------------------------------------
# 3) 配置寄存器真的接到了判据上
#
# 只比"两边一致"是不够的 —— 如果 RTL 忽略了这个寄存器、而 C 模型也恰好
# 没因此改变输出，两边照样"一致"。所以这里同时要求**指纹变化**：
# 参数改了、输出没变，就说明这条参数在这批帧上没被触发，用例是空跑的。
# 每个取值都是实测"会改变指纹"的（见 core/fpga/rtl/README.md）。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("args,what", [
    (("--bright", "100"), "bright 阈值"),
    (("--wall-rb", "20"), "墙色 R-B 阈值"),
    (("--wall-min-g", "120"), "墙 G 分量下限"),
    (("--p-min-area", "200"), "角色最小面积"),
])
def test_config_registers_are_wired(exe, stim, args, what):
    stim_dir, frames = stim
    base = _run(exe, stim_dir, frames)
    assert base.returncode == 0, base.stdout + base.stderr
    fp0 = _fingerprint(base.stdout)

    r = _run(exe, stim_dir, frames, *args)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, f"{what}：RTL 与 C 不一致\n{r.stdout}"
    fp1 = _fingerprint(r.stdout)
    assert fp1 != fp0, (
        f"{what}：改了参数但几何帧指纹没变（{fp0}）—— 这条参数在这批帧上"
        f"根本没被触发，用例是空跑的。换一个能触发它的取值。")


# ---------------------------------------------------------------------------
# 4) 配置通路的反面控制：**只改 RTL 的寄存器、不改 C 的参数**，两边必须不一致。
#
# 没有这一条，"配置寄存器接对了"可能只是"谁都没在看那个寄存器"。
# ---------------------------------------------------------------------------
def test_config_negative_control(exe, stim):
    stim_dir, frames = stim
    # addr 2 = CFG_BRIGHT_MIN；只写 RTL。C 侧仍用默认 195。
    r = _run(exe, stim_dir, frames, "--rtl-only", "2", "100")
    assert r.returncode != 0, ("只改 RTL 的 bright 阈值却报'一致' —— "
                               "说明这个寄存器根本没参与判据\n" + r.stdout)
    assert "不符 0" not in r.stdout, r.stdout


# ---------------------------------------------------------------------------
# 5) 反面控制：全黑帧两边都必须判"没有角色"
#
# 这正是 vision_ref 早先"空帧给出默认角色"那个 bug 的守卫。
# ---------------------------------------------------------------------------
def test_cosim_negative_control_black_frame(exe, tmp_path: pathlib.Path):
    stim_dir = tmp_path / "black"
    stim_dir.mkdir()
    w, h = 640, 360      # 必须是 RTL 参数化的帧尺寸，否则 RTL 收不满一帧
    (stim_dir / "000000.ppm").write_bytes(b"P6\n640 360\n255\n" + bytes(w * h * 3))
    r = _run(exe, stim_dir, 1, "--verbose", timeout=600)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout
    assert "角色 0" in r.stdout, r.stdout


# ---------------------------------------------------------------------------
# 6) **PS 面向接口**（AXI4-Lite）走通
#
# tb_vision 直接驱动 vision_top 的引脚；上板时 PS 碰到的是 AXI 口。
# 地址译码、握手、窗口读、CTRL 位定义 —— 这些只有走 AXI 才测得到。
# 这一条当场抓到过一个真 bug：`dout_last` 每帧拉高两次（payload 尾 + CRC 尾），
# 纯字节流看不出来，但包装层拿它定帧长会算成 4 字节。
# ---------------------------------------------------------------------------
def test_axi_path_matches_c_reference(exe, stim):
    stim_dir, frames = stim
    r = subprocess.run([str(EXE_AXI), str(stim_dir), str(frames)],
                       capture_output=True, text=True, timeout=1800)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout
    n_checks = int(r.stdout.split("比对项 ")[1].split("，")[0])
    assert n_checks > 100 * frames, r.stdout


def test_axi_path_with_changed_params(exe, stim):
    """两边一起改参数 → AXI 路径仍然逐字节一致。"""
    stim_dir, frames = stim
    r = subprocess.run([str(EXE_AXI), str(stim_dir), str(frames), "--bright", "100"],
                       capture_output=True, text=True, timeout=1800)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout


def test_axi_path_negative_control(exe, stim):
    """**只改 RTL 的寄存器**（C 侧不改）→ AXI 路径必须报不一致。

    没有这一条，"AXI 路径一致"可能只是"窗口读回来的是常量"，
    或者"那个寄存器根本没人看"。
    """
    stim_dir, frames = stim
    # 0x008 = A_CFG_BRIGHT_MIN
    r = subprocess.run([str(EXE_AXI), str(stim_dir), str(frames), "--rtl-only", "0x008", "100"],
                       capture_output=True, text=True, timeout=1800)
    assert r.returncode != 0, ("只改 RTL 的 bright 阈值却报'一致' —— "
                               "说明 AXI 写进去的寄存器没参与判据\n" + r.stdout)
    assert "不符 0" not in r.stdout, r.stdout


# ---------------------------------------------------------------------------
# 7) **PL 自检图案**：不用相机、也不用 C 参考模型
#
# 这一条比"RTL == C 模型"更硬：它比的是**手算出来的几何**。
# 前两个测试台都在拿 C 模型当参照，两边一起错是测不出来的；自检图案把手算的
# 墙带行范围、缺口中心、角色质心直接写进断言，给了 PL 通路一个独立参照。
#
# 上板 bring-up 的第一步就是它：相机通路通常最后才通，但"PL 到底对不对"
# 必须能提前回答。板上跑 auto_run=1（内部节拍自转），结果从调试口读。
# ---------------------------------------------------------------------------
def test_selftest_pattern_matches_hand_computed_geometry(exe):
    r = subprocess.run([str(EXE_ST), "8", "--verbose"],
                       capture_output=True, text=True, timeout=900)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout
    # 8 个场景全部要跑到（防止循环提前退化成 0 帧也算过）
    assert r.stdout.count("全部与手算一致") == 8, r.stdout
    assert "auto: 完成帧数" not in r.stdout   # 自由运行只是内部检查，不打印


# ---------------------------------------------------------------------------
# 8) **假通过防线**：什么都没验证的运行，不能长得像全绿。
#
# 这不是假想出来的风险。cosim/README.md 里的相对路径依赖 cwd，从
# core/fpga/cosim/ 下少敲一层（`../../data/...` 而不是 `../../../data/...`）
# 就会命中 —— 实测旧代码打印"比对项 0，不符 0，有差异的帧 0"，并以 **0** 退出。
# 也就是说：一次什么都没验证的运行，和一次全绿通过，输出与退出码完全一样。
#
# 这一组用例守的是**二进制自己的行为**。test_cosim_bit_exact_on_real_frames
# 那种"比对项 > 600000*frames"的断言只保护 pytest 这条路，保护不了 README
# 里手敲命令的人 —— 而手敲正是文档规定的用法。
# ---------------------------------------------------------------------------
def test_missing_stimulus_is_a_failure(exe, tmp_path: pathlib.Path):
    r = _run(exe, tmp_path / "does_not_exist", 4, timeout=600)
    assert r.returncode != 0, f"激励目录不存在却报成功（假通过）：\n{r.stdout}"


def test_short_stimulus_is_a_failure(exe, stim, tmp_path: pathlib.Path):
    """请求 4 帧、目录里只有 2 帧 —— 必须失败，不能"比了 2 帧"就报绿。"""
    stim_dir, _ = stim
    few = tmp_path / "few"
    few.mkdir()
    for p in sorted(stim_dir.glob("*.ppm"))[:2]:
        shutil.copy(p, few / p.name)
    r = _run(exe, few, 4, timeout=600)
    assert r.returncode != 0, f"只给了 2 帧却请求 4 帧，仍然报成功：\n{r.stdout}"
    assert "只比对了 2 / 4 帧" in r.stdout, r.stdout


def test_axi_missing_stimulus_is_a_failure(exe, tmp_path: pathlib.Path):
    r = subprocess.run([str(EXE_AXI), str(tmp_path / "does_not_exist"), "4"],
                       capture_output=True, text=True, timeout=600)
    assert r.returncode != 0, f"AXI 路径激励读不到却报成功：\n{r.stdout}"


@pytest.mark.parametrize("n", ["0", "-3"])
def test_selftest_with_no_scenes_is_a_failure(exe, n: str):
    """0 个场景时自由运行段照样产生 3 个比对项，所以"比对项 > 0"抓不住它。"""
    r = subprocess.run([str(EXE_ST), n], capture_output=True, text=True, timeout=600)
    assert r.returncode != 0, f"请求 {n} 个场景却报成功：\n{r.stdout}"
    assert "一次比对都没发生" in r.stdout or "没有任何几何比对" in r.stdout, r.stdout


# ---------------------------------------------------------------------------
# 9) 真实世界的结构性盲区
#
# 合成语料（录屏导帧）在结构上是"良性"的：段永远短小、标签表永远填不满、
# 几乎每帧都选得出玩家。于是 620 帧逐位对账全绿，却漏掉了三个只在**真实输入**
# 下才暴露的缺陷（2026-10-13 一起修掉，见 core/fpga/docs/帧存改造.md §14.4）：
#
#   ① **标签表打满时 C_SEL 死循环**
#      `sel_i` 只有 `clog2(MAX_LABELS)=8` 位（0..255），而 `n_label` 能饱和到
#      256 —— 判据 `{1'b0,sel_i} >= n_label` **恒为假**，sel_i 绕圈永不退出。
#      症状是 RTL **完全不出帧**（vision_top 卡在 S_CCL 等 ccl_done）。
#
#   ② **超大墙段把 16 位累加器冲爆**
#      `sg_total` 原来只有 16 位，而一段最多覆盖 `640×360=230400` 个墙像素。
#      它是除数，一回绕 cyq 就算错（实测算出 y=453，图只有 360 高）。
#      症状是**只有 cyq 一个字段**不符，且只在 2/32 张真实照片上出现。
#
#   ③ **没选中玩家时输出不清零**
#      C 侧是 `memset(f,0,sizeof(*f))` + 提前 return；RTL 侧它们是寄存器，
#      不清零就保留上一帧的值，而几何帧照打包那 16 个字节。
#
# 这一节用**确定性生成的**图像把三条钉住（不依赖 data/，裸检出也跑得了），
# 另外再挂一条真实照片语料（有就跑，没有就 skip）。
# ---------------------------------------------------------------------------
WALL_RGB  = (200, 120, 100)   # R-B=100>40、R=200>110、G=120>60 → 判墙
WHITE_RGB = (255, 255, 255)   # min=255>195、max-min=0<45      → 判亮
BLACK_RGB = (0, 0, 0)


def _ppm_write(path: pathlib.Path, w: int, h: int, buf: bytearray) -> None:
    path.write_bytes(b"P6\n%d %d\n255\n" % (w, h) + bytes(buf))


def _fill(buf: bytearray, w: int, x0: int, x1: int, y0: int, y1: int,
          color: tuple[int, int, int]) -> None:
    px = bytes(color)
    for y in range(y0, y1 + 1):
        base = y * w
        for x in range(x0, x1 + 1):
            i = (base + x) * 3
            buf[i:i + 3] = px


def _blindspot_frames(dirpath: pathlib.Path) -> tuple[pathlib.Path, int]:
    """三帧，分别打中三个盲区。**确定性**，不依赖任何数据文件。"""
    dirpath.mkdir(parents=True, exist_ok=True)
    W, H = 640, 360
    blank = lambda: bytearray(W * H * 3)          # noqa: E731

    # 帧 0：一面几乎铺满的墙（604 列 × 341 行）+ 一个合法玩家。
    #   列计数总和 ≈ 2e5 ≫ 65535 —— 旧的 16 位 sg_total 必回绕。
    #   玩家 20×20=400 px，落在 [60,1500]、填充 100%、长宽比 100%、不贴边。
    a = blank()
    _fill(a, W, 0, 603, 0, 340, WALL_RGB)
    _fill(a, W, 300, 319, 200, 219, WHITE_RGB)
    _ppm_write(dirpath / "000000.ppm", W, H, a)

    # 帧 1：左上角 24×24 的亮色点阵（点间空 1 像素，8 邻接下互不相连）
    #   ⇒ 576 个独立连通域 ⇒ 标签表 256 必定打满 ⇒ 走 C_SEL 的终止边界。
    #   ★ 点阵只占 x,y ∈ [0,46]，与 (300,200) 的玩家方块离得很远，
    #     不会和它连成一片把面积顶出 [60,1500]。
    b = blank()
    for j in range(24):
        for i in range(24):
            _fill(b, W, 2 * i, 2 * i, 2 * j, 2 * j, WHITE_RGB)
    _fill(b, W, 300, 319, 200, 219, WHITE_RGB)
    _ppm_write(dirpath / "000001.ppm", W, H, b)

    # 帧 2：全黑 ⇒ 一个玩家都选不出来。上一帧刚选过 —— 输出寄存器若不清零，
    #   就会把上一帧的 player_x/y/r 与 blob_* 打进这一帧的字节流。
    _ppm_write(dirpath / "000002.ppm", W, H, blank())
    return dirpath, 3


@pytest.fixture(scope="module")
def blindspot(tmp_path_factory: pytest.TempPathFactory) -> tuple[pathlib.Path, int]:
    return _blindspot_frames(tmp_path_factory.mktemp("blindspot"))


def test_blindspot_frames_bit_exact(exe, blindspot):
    """三个真实世界盲区：标签表打满 / 超大墙段溢出 / 没玩家时输出不清零。"""
    d, n = blindspot
    r = _run(exe, d, n, "--verbose")
    assert "有差异的帧 0" in r.stdout, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout + r.stderr


def test_real_photo_frames_bit_exact(exe):
    """真实照片（`data/fpga/stim_photo`）逐位对账。

    **没有语料就 skip** —— `data/` 是 gitignore 的，队友那边拿不到。
    但只要语料在，它就是保真度最高的那道闸：录屏合成帧的段结构太良性，
    上面三个缺陷一个都逼不出来。语料用
    `python core/fpga/ref/export_photos.py --out data/fpga/stim_photo` 生成。
    """
    d = ROOT / "data" / "fpga" / "stim_photo"
    ppts = sorted(d.glob("*.ppm"))
    if not ppts:
        pytest.skip(f"没有真实照片语料（{d}）")
    for exe_path, name in ((EXE, "vision_top"), (EXE_AXI, "vision_axi")):
        r = subprocess.run([str(exe_path), str(d), str(len(ppts))],
                           capture_output=True, text=True, timeout=1800)
        assert "不符 0" in r.stdout, f"{name}：\n{r.stdout}{r.stderr}"


def test_rtl_has_no_verilator_metacomment_lookalikes():
    """RTL 注释不许长得像 Verilator 元注释。

    踩过一次：`//   Verilator 会照写，…` 被 Verilator 当成元注释（它认的是
    `// verilator <指令>`，**大小写不敏感**），直接
    `%Error: Unknown verilator comment` —— 报错行号指在注释上，一眼看不出是
    "中文注释碰巧以 Verilator 开头"。这条把合法指令白名单固化下来，
    以后要用新指令就得显式加进来（有意为之，而不是碰巧）。
    """
    ok = ("lint_on", "lint_off", "tracing_on", "tracing_off",
          "coverage_on", "coverage_off", "public", "noinline",
          "isolate_assignments", "waiver", "hierarchical", "clock_enable",
          "inline", "no_inline", "lint_only", "stats", "trace")
    bad = []
    for f in sorted(RTL.glob("*.v")) + sorted(RTL.glob("*.vh")):
        for i, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
            m = re.match(r"\s*(?://|/\*)\s*[Vv]erilator\s+(\S+)", line)
            if m and not m.group(1).startswith(ok):
                bad.append(f"{f.name}:{i}: {line.strip()}")
    assert not bad, (
        "这些注释会被 Verilator 当成元注释，改掉措辞或在白名单里登记：\n  "
        + "\n  ".join(bad))
