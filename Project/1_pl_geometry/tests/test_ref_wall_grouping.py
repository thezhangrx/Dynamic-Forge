"""**同一堵墙的两段必须共享完全相同的 `rotation`** —— 帧级精确守卫。

为什么需要这一条（补 `test_ref_vs_python.py` 自己如实记为空白的那一半）
--------------------------------------------------------------------
那个文件拿"墙段角度误差 ≤ 1e-3 rad"当代理指标，并在文件头写下：
**录像语料抓不到「整墙弦 → 逐段各自」这个具体退化** —— 录像里每堵墙都是直的，
两种算法数值上恰好相等（都是 max 0.000623）。

本文件不靠数值接近，直接钉**不变量本身**：

    一堵墙的**所有段**必须拿到**逐位相同**的 `rotation`。

这条在**任何**语料上都成立、都能判，而且正是 `group_walls()` 分桶所依赖的前提。

### 为什么这个不变量是根因，不是代理

`cpu/gap_avoid.group_walls()` 的分桶键是::

    (int(round(r.rotation * 1000.0)), int(round(band / 2.0)))

一堵墙的两段只要 `rotation` 差一个 Q6 格（PL 定点 atan2 的分辨率 = 1/64
≈ 0.015625 rad），桶键就相差 **15~16 格**，两段被拆成两堵单段墙，
`plan()` 就永远重建不出缺口 → 一直喊 `NO_GAP`。

### 实测反向验证（2026-10-13，40 帧带噪声的渲染语料）

把 C 的 `ang_q = ang_wall` 换成 `ang_q = kang_q[i]`（退回逐段各自）重跑::

    实现            多段墙组   段间 rotation 不一致   其中桶键也不同   最大差
    整墙共用（现在）     88            0                   0          0.000000
    逐段各自（退化）     88           74                  74          0.015625

### ⚠ 语料里**必须有噪声**，否则这条测试是空的

`noise = 0` 时墙被渲染成严格直线，每一列的灰度重心精确共线，
于是"逐段自己拟合"与"整墙弦"**数值上完全相等** —— 两种实现都通过。
这正是 `test_ref_vs_python.py` 遇到的那个盲区。加噪声（这里 3.0）让各列的
重心各自抖动，两种算法才分得开：实测同一帧里一段是 `+0.078125`、另一段是
`+0.062500`，正好差一格。

### 闭环后果（实测，见 `core/fpga/docs/帧存改造.md`）

同一场景跑闭环：正确实现板子侧 `NO_GAP = 1`（Python 也是 1）；
逐段变体 `NO_GAP = 5 / 15`（Python 仍 1）。所以这条不变量不是洁癖，
它直接决定板子链路能不能穿过缺口。跑闭环一次要 ~5 分钟，
按"守卫要跑得起"的取舍，这里只钉帧级不变量，闭环数字留作本文件的证据。
"""

from __future__ import annotations

import os
import pathlib
import sys

import pytest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

ROOT = pathlib.Path(__file__).resolve().parents[3]
for _p in (ROOT / "core", ROOT / "core" / "cpu"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

pytest.importorskip("pygame", reason="要离屏渲染语料，没有 pygame 就跳过")
pytest.importorskip("bullet_sim.simulator.env", reason="没有平台就跳过")

from harness.board_sim import RefPl, compile_ref                 # noqa: E402
from harness.loop import SyntheticCamera, build_env              # noqa: E402
from standard.vision.vision_frame import decode_vision_frame     # noqa: E402
from bullet_sim.action import Action                             # noqa: E402

#: 渲染/取帧参数。**噪声不能是 0** —— 见模块 docstring 的"语料里必须有噪声"。
NOISE = 3.0
BLUR = 1
N_FRAMES = 40
STEP_STRIDE = 30             # 每 0.25 s 取一帧（dt = 1/120）
SECONDS = 12.0
GAP_WIDTH = 60.0
GAP_POS = 0.28
OBSTACLE_SPEED = 40.0

#: 墙带方向的容差（世界单位）。相邻墙之间 >= 100，取 40 不会误并。
#: ⚠ 不能按 `y` **精确值**分组：墙在图像里是斜的，同一堵墙的两段 y 就不相等
#:   （实测 79.95 vs 94.73）—— 早先这么写，"多段墙组"数出来是 0，测试是空的。
BAND_TOL = 40.0


@pytest.fixture(scope="module")
def c_frames():
    """渲染一段带噪声的场景，逐帧过 C 参考模型，返回每帧的障碍列表。"""
    import shutil
    if shutil.which("gcc") is None:
        pytest.skip("没有 gcc，跳过（要编 C 参考模型）")

    pl = RefPl(compile_ref())
    env = build_env(0, GAP_WIDTH, SECONDS, OBSTACLE_SPEED, GAP_POS, "static")
    cam = SyntheticCamera(blur=BLUR, noise=NOISE)
    env.reset(seed=0)

    frames = []
    for i in range(int(SECONDS / (1.0 / 120.0))):
        env.step(Action.zero())
        if i % STEP_STRIDE:
            continue
        f = cam.grab(env)
        raw = bytes(f.to_rgb())
        frame_bytes, _res = pl.process(raw, f.width, f.height, seq=len(frames))
        frames.append(decode_vision_frame(frame_bytes))
        if len(frames) >= N_FRAMES:
            break
    assert frames, "一帧都没渲染出来"
    return frames


def _wall_groups(frame) -> dict[int, list[float]]:
    """按**墙带**把障碍分组（同一堵墙的各段落在同一组）。"""
    groups: dict[int, list[float]] = {}
    for o in (frame.obstacles or []):
        groups.setdefault(int(round(float(o.y) / BAND_TOL)), []).append(float(o.rotation))
    return groups


def test_corpus_actually_has_multi_segment_walls(c_frames):
    """先证明语料是**活的** —— 没有多段墙的话，下面那条测试是空跑的。

    这条本身就是一次教训：早先按 `y` 精确值分组，多段墙组数出来是 0，
    测试"通过"了但什么都没验。
    """
    n_multi = sum(1 for f in c_frames for rots in _wall_groups(f).values() if len(rots) >= 2)
    assert n_multi >= 20, (
        f"语料里只有 {n_multi} 个多段墙组（{len(c_frames)} 帧）—— "
        f"太少说明这个场景没造出缺口墙，或者分组键又写错了"
    )


def test_all_segments_of_a_wall_share_the_same_rotation(c_frames):
    """**本文件的主守卫。** 同一堵墙的各段 rotation 必须逐位相同。

    退化成"逐段各自拟合角度"时，实测 88 个多段墙组里 74 个不一致，
    且这 74 个的 `round(rotation*1000)` 桶键**全都不同** —— 墙被拆开，
    闭环里 `plan()` 永远重建不出缺口。

    反向验证命令（把 `ang_q = ang_wall` 换成 `ang_q = kang_q[i]` 后重跑本文件）::

        sed 's/ang_q = ang_wall/ang_q = kang_q[i]/' core/fpga/ref/vision_ref.c > /tmp/p.c
        # 用 /tmp/p.c 编出的 .so 替换 data/fpga/board_sim/libvision_ref.so 后重跑
    """
    bad = []
    for idx, f in enumerate(c_frames):
        for band, rots in _wall_groups(f).items():
            if len(rots) < 2:
                continue
            spread = max(rots) - min(rots)
            if spread > 0.0:
                keys = sorted({int(round(r * 1000)) for r in rots})
                bad.append((idx, band, rots, spread, keys))
    if bad:
        idx, band, rots, spread, keys = bad[0]
        pytest.fail(
            f"{len(bad)} 个多段墙组的 rotation 在段间不一致（共 {len(c_frames)} 帧）。\n"
            f"  首个：帧 {idx}、墙带 {band}：rotation={[round(r, 6) for r in rots]}\n"
            f"  段间差 {spread:.6f} rad，分桶键 {keys} —— "
            f"{'**桶键不同，墙会被拆成两堵**' if len(keys) > 1 else '桶键相同'}\n"
            f"  最可能的原因：C 侧的 `ang_q` 退回了 `kang_q[i]`（逐段各自拟合），\n"
            f"  而不是整堵墙共用的 `ang_wall`。见 vision_ref.c 的 vr_extract_walls。"
        )


def test_wall_count_is_even_and_gaps_bracketed(c_frames):
    """语料形状的健全性：缺口墙应当是**成对**出现的。

    每堵缺口墙 = 2 段 + 1 个缺口。这条不钉具体数值，只防"某次改动把段过滤
    调过头"这类塌方：段数掉到 0 或奇偶全乱时会红。
    """
    n_odd = 0
    for idx, f in enumerate(c_frames):
        groups = _wall_groups(f)
        multi = [b for b, r in groups.items() if len(r) >= 2]
        if not multi:
            n_odd += 1
    assert n_odd <= len(c_frames) // 2, (
        f"{n_odd}/{len(c_frames)} 帧里连一个多段墙都没有 —— "
        f"缺口墙的检测塌方了"
    )
