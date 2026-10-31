"""C 参考模型 ↔ Python 视觉链的**语义一致性**守卫。

为什么必须有
------------
整条验证链是::

    RTL  ==  C 参考模型        ←  tb_vision / tb_axi 逐位对账，重兵把守
    C    ==  Python 视觉链     ←  **本文件之前一条测试都没有**
                  ↑
            Python 才是"语义真值"（core/vision/wall_with_gap.py）

第一条有几十项测试；第二条只被**手工跑过一次**，数字记在
`docs/工具链现状.md` §5。也就是说：**C 与 Python 一旦分家，RTL 比对照样全绿** ——
它只是忠实地复现了一个已经跑偏的参考模型。这正是"验证了一个错的东西"
那一类风险，而且它比 RTL 的 bug 更隐蔽：RTL 有对账兜着，这条没有。

本文件把第二条钉住，分两档：

**① 部署场景（录像导帧）—— 硬指标。**
录像帧代表真实部署里那条合成渲染的 `wall_with_gap`（每堵墙恰好 2 段）。要求：

* 缺口中心误差 ≤ 0.1（`docs/交接文档.md` 的差异预算）
* 墙段角度误差 ≤ **1e-3 rad** —— 决策的分桶键是 `round(rotation * 1000)`，
  角度差 ×1000 就是桶键差；超过 1e-3 就会把同一堵墙分进不同的桶，
  `plan()` 于是重建不出缺口、一直喊 NO_GAP。
* 角色位置误差中位 ≤ 1 px —— 角色是小车自身位姿，直接进决策。

**② 真实照片（非部署场景）—— 只钉上限，不要求零分歧。**
`docs/工具链现状.md` §5 已经定性并**明确留给以后**：真实照片里同一"墙带"
会出现 **>2 段**，C 用"首段心→末段心"的弦、Python 用全部段心的 TLS 主方向，
两者不等价；而且两边偶尔多检/少检一段。所以这里不设"零分歧"的硬指标，
只钉一个**量化上限**防止它悄悄变坏。

实测（2026-10-13，32 张 640×360 居中裁剪的真实照片）::

    缺口条数   Python 15 / C 18        墙段条数 Python 55 / C 59
    缺口中心   中位 0.00  max 0.01     ← 这条仍然达标
    角度       中位 0.004 max 0.397    ← 已知分歧
    角色位置   **双峰**：9 帧 ≤ 0.4 px，16 帧 60~610 px

角色那一列的双峰很说明问题：**两边选中同一个连通域时位置差 < 0.4 px**
（矩/质心的算术是对的），差得多的那些是**选中的连通域根本不是同一个** ——
即分歧在 CCL 合并/选优，不在几何计算。这与"偶尔多检/少检一段"同源。

### ⚠️ 这套护栏的**力度受语料限制**（实测，别高估）

写完之后逐个突变做了反向验证，结论要如实写下来：

| 突变 | 是否被抓到 |
|---|---|
| `wall_min_r` 110 → 150（墙判据收紧） | ✅ `test_deployment_scene_gap_center_within_budget` 变红 |
| 角度：整墙弦 → 逐段各自 | ❌ **抓不到** |
| 选优：最优分 → 第一个通过判据的 | ❌ **抓不到** |
| `min_gap_px` 8 → 20 | ❌ 抓不到（语料里缺口远大于 20，本就不起作用） |
| `player_min_area/max_area` 改默认值 | ❌ 无效（§见下） |

两条原因，都值得记住：

1. **角度那条**：录像语料里每堵墙都是**直的**，于是"整墙弦"与"逐段各自拟合"
   数值上**恰好相等**（实测两者 max 都是 0.000623，一字不差）。文档里
   "修复前 48/48 个决策时刻全不同"是在**另一个场景**（墙不直 / 有噪声）上测的。
   所以这条测试钉的是**要求**（≤1e-3），**不能**证明"弦 vs 逐段"这个具体退化会被发现。
2. **选优那条**：这套语料每一步只有一个连通域能通过判据，选优规则怎么改都一样。
   要覆盖它得造**多候选**的场景。

**`player_min_area` / `player_max_area` 改 C 的编译期默认值是无效突变**：
`compare_with_python.py` 会用 `--set` 把 Python 侧参数**显式**传给 C
（`_SETTABLE` 那 8 项），并且有 `_check_params_agree` 守着。要动玩家选择，
得改 `vr_extract_player` 的**选优逻辑**本身 —— 但见上面第 2 条，这套语料也看不出来。

**小结**：这个文件解决的是"**Python↔C 完全没有自动护栏**"这个空白（原来一条都没有），
把部署场景的硬指标钉住了；但它**不构成**对弦/TLS、选优这些细节退化的完整覆盖。
补那一半需要专门造语料，属于"以后"。

需要 `gcc` 与语料；没有就 skip（不假装通过）。
"""

from __future__ import annotations

import json
import pathlib
import shutil
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
REF_C = ROOT / "core" / "fpga" / "ref" / "vision_ref.c"
COMPARE = ROOT / "core" / "fpga" / "ref" / "compare_with_python.py"
VIDEO = ROOT / "data" / "vision" / "run4" / "screen.mp4"
PHOTOS = ROOT / "data" / "fpga" / "stim_photo"

pytestmark = pytest.mark.skipif(
    shutil.which("gcc") is None or not REF_C.exists(),
    reason="需要 gcc 与 core/fpga/ref/vision_ref.c")


# ---------------------------------------------------------------------------
# 夹具
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def ref_bin(tmp_path_factory) -> pathlib.Path:
    out = tmp_path_factory.mktemp("refbin") / "vision_ref"
    r = subprocess.run(["gcc", "-O2", "-o", str(out), str(REF_C)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return out


def _compare(ref_bin: pathlib.Path, stim: pathlib.Path, limit: int,
             tmp_path: pathlib.Path) -> list[dict]:
    out = tmp_path / "cmp.json"
    r = subprocess.run(
        [sys.executable, str(COMPARE), "--stim", str(stim),
         "--ref", str(ref_bin), "--limit", str(limit), "--json-out", str(out)],
        capture_output=True, text=True, timeout=1800,
    )
    assert r.returncode == 0, f"对账脚本失败：\n{r.stdout}\n{r.stderr}"
    return json.loads(out.read_text(encoding="utf-8"))


def _stat(rows: list[dict], key: str) -> tuple[float, float, int] | None:
    vals = [r[key] for r in rows if r.get(key) is not None]
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2], vals[-1], len(vals)


@pytest.fixture(scope="module")
def video_frames(tmp_path_factory) -> pathlib.Path:
    """录像导帧（部署场景的代表）。没有录像就 skip。"""
    if not VIDEO.exists():
        pytest.skip(f"没有真实录像 {VIDEO}")
    out = tmp_path_factory.mktemp("refstim")
    r = subprocess.run(
        [sys.executable, str(ROOT / "core" / "fpga" / "ref" / "export_frames.py"),
         "--video", str(VIDEO), "--out", str(out),
         "--frames", "40", "--start", "200", "--stride", "2",
         "--width", "640", "--color"],
        capture_output=True, text=True,
    )
    assert r.returncode == 0, r.stdout + r.stderr
    assert sorted(out.glob("*.ppm")), "一帧都没导出来"
    return out


# ---------------------------------------------------------------------------
# ① 部署场景：硬指标
# ---------------------------------------------------------------------------
def test_deployment_scene_gap_center_within_budget(ref_bin, video_frames, tmp_path) -> None:
    """缺口中心误差 ≤ 0.1（`docs/交接文档.md` 的差异预算）。"""
    rows = _compare(ref_bin, video_frames, 40, tmp_path)
    st = _stat(rows, "gap_err")
    assert st, "没有任何一帧两边都检出了缺口，这条测试就是空的"
    med, mx, n = st
    assert mx <= 0.1, f"缺口中心误差 max {mx:.3f} > 预算 0.1（{n} 帧）"


def test_deployment_scene_angle_does_not_split_buckets(ref_bin, video_frames, tmp_path) -> None:
    """角度误差 ≤ 1e-3 rad —— 否则 `round(rotation*1000)` 的桶键会跨档。

    跨档的后果不是"角度不准"，而是**闭环直接失效**：`group_walls` 把同一堵墙
    的两段分进不同的桶，`plan()` 重建不出缺口，一直喊 NO_GAP。
    文档记的修复前症状就是"48/48 个决策时刻全不同"。

    ⚠ **实测：这条测试抓不到"整墙弦 → 逐段各自"那个具体退化** ——
    录像语料里墙是直的，两种算法数值上恰好相等（都是 max 0.000623）。
    它钉的是**要求**（≤1e-3 rad ⇒ 桶键不跨档），不是那个具体 bug。
    详见模块 docstring 的"力度受语料限制"一节。
    """
    rows = _compare(ref_bin, video_frames, 40, tmp_path)
    st = _stat(rows, "angle_err")
    assert st, "没有任何一帧两边都检出了墙段"
    med, mx, n = st
    assert mx <= 1e-3, (
        f"墙段角度误差 max {mx:.6f} rad > 1e-3（{n} 帧）—— "
        f"决策分桶键会跨档（桶键 = round(rotation*1000)，差 {mx * 1000:.1f} 格）"
    )


def test_deployment_scene_counts_agree_exactly(ref_bin, video_frames, tmp_path) -> None:
    """部署场景下**条数必须完全一致**（多一段少一段都会让缺口错位）。"""
    rows = _compare(ref_bin, video_frames, 40, tmp_path)
    py_g = sum(r.get("py_n_gaps") or 0 for r in rows)
    c_g = sum(r.get("c_n_gaps") or 0 for r in rows)
    py_o = sum(r.get("py_n_obs") or 0 for r in rows)
    c_o = sum(r.get("c_n_obs") or 0 for r in rows)
    assert (py_g, py_o) == (c_g, c_o), (
        f"条数不一致：缺口 Python {py_g} vs C {c_g}；"
        f"墙段 Python {py_o} vs C {c_o}"
    )


def test_deployment_scene_player_position_matches(ref_bin, video_frames, tmp_path) -> None:
    """角色位置是小车自身位姿，直接进决策 —— 中位 ≤ 1 px、最大 ≤ 5 px。"""
    rows = _compare(ref_bin, video_frames, 40, tmp_path)
    st = _stat(rows, "player_err")
    assert st, "没有任何一帧两边都检出了角色"
    med, mx, n = st
    assert med <= 1.0 and mx <= 5.0, (
        f"角色位置误差中位 {med:.2f} max {mx:.2f}（{n} 帧），超出 1 / 5 px"
    )


# ---------------------------------------------------------------------------
# ② 真实照片：只钉上限（已知且已定性的分歧，见模块 docstring）
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def photo_frames() -> pathlib.Path:
    ppts = sorted(PHOTOS.glob("*.ppm")) if PHOTOS.exists() else []
    if not ppts:
        pytest.skip(f"没有真实照片语料（{PHOTOS}；用 core/fpga/ref/export_photos.py 生成）")
    return PHOTOS


def test_real_photos_gap_center_still_within_budget(ref_bin, photo_frames, tmp_path) -> None:
    """**缺口中心**在真实照片上仍然达标 —— 这一条不因为"非部署场景"而放宽。

    真实照片上条数会差（见下一条），但只要两边都检出的那些帧，缺口中心
    实测中位 0.00 / max 0.01，离 0.1 的预算很远。
    """
    rows = _compare(ref_bin, photo_frames, 64, tmp_path)
    st = _stat(rows, "gap_err")
    assert st, "没有任何一帧两边都检出了缺口"
    med, mx, n = st
    assert mx <= 0.1, f"缺口中心误差 max {mx:.3f} > 预算 0.1（{n} 帧）"


def test_real_photos_angle_divergence_stays_bounded(ref_bin, photo_frames, tmp_path) -> None:
    """真实照片上角度分歧**已知**（>2 段的墙带，C 用弦、Python 用 TLS）。

    这里不要求零分歧 —— `docs/工具链现状.md` §5 明确"留给以后"。
    但必须**钉住上限**：它是"部署不受影响"这个结论的前提，
    悄悄变到 1 rad 就说明弦/TLS 之外又多了别的原因，得重新判断。
    """
    rows = _compare(ref_bin, photo_frames, 64, tmp_path)
    st = _stat(rows, "angle_err")
    if not st:
        pytest.skip("真实照片上没有两边都检出墙段的帧")
    med, mx, n = st
    # 实测 2026-10-13（32 张 640×360 居中裁剪）：中位 0.004137、max 0.397022。
    # 上限给 1.5~2 倍余量：够容忍换语料/换裁剪带来的正常波动，
    # 又能在"多出一个新原因"时报警。
    assert med <= 0.02, f"角度误差中位 {med:.6f} rad 超出实测基线（0.0041）的 5 倍"
    assert mx <= 0.8, f"角度误差 max {mx:.6f} rad 超出实测基线（0.397）的 2 倍"


def test_real_photos_known_count_divergence_is_recorded(ref_bin, photo_frames, tmp_path) -> None:
    """把"真实照片上条数会差"这件事**显式记录**成断言，而不是留个印象。

    文档 §5 写的是"两边**偶尔**多检/少检一段"。把它量化：
    实测 32 张里缺口 15 vs 18、墙段 55 vs 59（差 20% 上下）。
    哪边多不做断言 —— 实测两组都有（Python 多检的帧也存在）。
    真正的意义是：**如果哪天这个差距突然变大**（比如从 20% 变成 100%），
    说明 C 侧的某个过滤阈值又与 Python 分家了，而不是"本来就有的已知分歧"。
    """
    rows = _compare(ref_bin, photo_frames, 64, tmp_path)
    py_g = sum(r.get("py_n_gaps") or 0 for r in rows)
    c_g = sum(r.get("c_n_gaps") or 0 for r in rows)
    py_o = sum(r.get("py_n_obs") or 0 for r in rows)
    c_o = sum(r.get("c_n_obs") or 0 for r in rows)

    def _rel(a: int, b: int) -> float:
        base = max(a, b, 1)
        return abs(a - b) / base

    assert _rel(py_g, c_g) <= 0.5, (
        f"缺口条数分歧 {py_g} vs {c_g} 超过 50% —— 已知分歧在 20% 上下，"
        f"突然变大说明又分家了"
    )
    assert _rel(py_o, c_o) <= 0.5, (
        f"墙段条数分歧 {py_o} vs {c_o} 超过 50% —— 已知分歧在 7% 上下"
    )
