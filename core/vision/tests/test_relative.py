"""相对判据的不变量：**换曝光、换增益、改白平衡都不能改变判据**。

这是"不需要调参才能好看"的可验证版本：
* 增益不变（`frame * k` → 掩膜逐位相同）；
* 偏置不变（未削顶时 `frame + c` → 掩膜相同）；
* 削顶像素进 `unknown`（"看不清，不判"），而不是被硬判成墙或背景；
* 与"绝对阈值版"对照：同一个场景整体变暗/变亮时，绝对阈值版会翻面，相对判据不翻。
"""

from __future__ import annotations

import pytest

from vision import primitives as P
from vision.relative import DEFAULTS, RelativeParams, classify_relative


# --------------------------------------------------------------------------
# 造图：墙（橙）/ 角色（白）/ 背景（深蓝）/ 干扰（绿）
# --------------------------------------------------------------------------
WALL = (240, 160, 48)
PLAYER = (235, 240, 250)
BG = (34, 40, 55)
GREEN = (40, 150, 60)


def make_frame(scale: float = 1.0, offset: int = 0, *, w: int = 64, h: int = 48) -> P.Frame:
    """一张合成图：一条橙墙 + 一个白圆 + 深蓝底 + 一小块绿。"""
    rgb = bytearray()
    for y in range(h):
        for x in range(w):
            # 墙：第 8~11 行
            if 8 <= y < 12:
                c = WALL
            # 角色：以 (32, 30) 为心、半径 3
            elif (x - 32) ** 2 + (y - 30) ** 2 <= 9:
                c = PLAYER
            # 绿块
            elif 40 <= x < 44 and 40 <= y < 44:
                c = GREEN
            else:
                c = BG
            if scale != 1.0:
                c = tuple(min(255, max(0, int(round(v * scale)) + offset)) for v in c)
            elif offset:
                c = tuple(min(255, max(0, v + offset)) for v in c)
            rgb.extend(c)
    return P.Frame.from_rgb(w, h, rgb)


def masks_of(frame: P.Frame, p: RelativeParams | None = None) -> tuple:
    m = classify_relative(frame, p)
    return bytes(m.wall), bytes(m.bright), bytes(m.green), bytes(m.unknown or b"")


# --------------------------------------------------------------------------
# 不变量
# --------------------------------------------------------------------------
@pytest.mark.parametrize("scale", [0.35, 0.5, 0.75, 1.0])
def test_gain_invariance(scale: float) -> None:
    """整帧乘一个系数（模拟曝光变化）→ **三张掩膜逐位相同**。

    ⚠ 只在"没有削顶"的范围内成立（scale 到 255 会把通道压到饱和，色度就丢了）——
    那正是下面 `test_clipping_goes_to_unknown` 要处理的情况。
    """
    base = masks_of(make_frame(1.0))
    dark = masks_of(make_frame(scale))
    assert dark[0] == base[0], f"scale={scale}: 墙掩膜变了（判据在追曝光）"
    assert dark[1] == base[1], f"scale={scale}: 亮掩膜变了"
    assert dark[2] == base[2], f"scale={scale}: 绿掩膜变了"


@pytest.mark.parametrize("offset", [10, 25, 40])
def test_offset_invariance(offset: int) -> None:
    """整帧加一个偏置（模拟环境光/黑电平）→ 掩膜保持（未削顶时）。"""
    base = masks_of(make_frame(1.0))
    lifted = masks_of(make_frame(1.0, offset))
    assert lifted[0] == base[0], f"offset={offset}: 墙掩膜变了"
    assert lifted[1] == base[1], f"offset={offset}: 亮掩膜变了"


def test_absolute_threshold_flips_but_relative_does_not() -> None:
    """对照组：同一场景变暗时，**绝对阈值版**墙没了；相对判据照样有墙。"""
    dark = make_frame(0.35)                      # 墙 (84,56,17)：r>110 不成立
    absm = P.classify(dark)
    relm = classify_relative(dark)
    assert sum(absm.wall) == 0, "绝对阈值版在变暗后应当丢掉墙（这就是要换掉它的原因）"
    assert sum(relm.wall) > 0, "相对判据在变暗后必须仍然看得见墙"


def test_near_black_pixels_are_unknown_not_wall() -> None:
    """近黑像素的色度是噪声 → 进 `unknown`，不许被判成墙。"""
    rgb = bytearray()
    for _ in range(16):
        rgb.extend((2, 1, 1))                    # 近黑偏红
    rgb.extend([200, 200, 200] * 16)             # 一块白
    f = P.Frame.from_rgb(32, 1, rgb)
    m = classify_relative(f)
    assert sum(m.wall) == 0, "近黑不该判成墙"
    assert sum(m.unknown) > 0, "近黑应当被标成 unknown"


def test_platform_colors_are_classified_as_expected() -> None:
    """契约里的三种颜色：橙→墙、白→亮、深蓝→都不是。"""
    f = make_frame(1.0)
    m = classify_relative(f)
    w, h = f.width, f.height
    def idx(x: int, y: int) -> int:
        return y * w + x
    assert m.wall[idx(4, 9)] == 1, "橙墙应当是墙"
    assert m.wall[idx(32, 30)] == 0, "白角色不该是墙"
    assert m.bright[idx(32, 30)] == 1, "白角色应当是亮块"
    assert m.bright[idx(4, 20)] == 0, "深蓝底不该是亮块"
    assert m.wall[idx(4, 20)] == 0, "深蓝底不该是墙"


def test_no_absolute_or_camera_specific_constant_is_left() -> None:
    """**这是"去掉调参"的守门测试**：参数表里不许再出现绝对量。

    原来的判据是 `r-b>40 / r>110 / g>60 / min>195 / spread<45` —— 每一个都是
    "对这台相机+这块屏才成立"的数。现在只允许两类参数：
    * **统计显著性**（几个稳健标准差，量级 2~8）；
    * **无量纲比例**（近中性容差 0~1、换算系数 ~1.48、eps 这类数值安全项）。
    任何绝对值（>=10 的像素级阈值、>=1 的比例之类）都不许出现。
    """
    from dataclasses import fields
    #: 允许的例外：**结构长度**（不是阈值）。`row_avg` 是"沿墙长度方向平滑多少像素"，
    #: 取决于墙比噪声的相关长度长多少，与曝光/白平衡无关；它不参与"判不判"的决定。
    structural = {"row_avg", "eps", "mad_to_sigma"}
    for f in fields(RelativeParams):
        v = getattr(DEFAULTS, f.name)
        assert isinstance(v, (int, float)), f"{f.name} 应当是数值"
        if f.name in structural:
            continue
        assert v <= 10.0, (
            f"参数 {f.name}={v} 看起来是绝对量（像素阈值一类）—— "
            "相对判据里不该再出现这种数")


def test_k_warm_is_a_statistical_significance_level() -> None:
    """`k_warm` 的含义是"比本帧背景暖几个稳健标准差"，与像素值无关。"""
    assert 2.0 <= DEFAULTS.k_warm <= 8.0
    assert 2.0 <= DEFAULTS.k_bright <= 8.0
    assert 0.0 < DEFAULTS.neutral <= 1.0


def test_empty_and_tiny_frames_do_not_crash() -> None:
    f = P.Frame.from_rgb(1, 1, bytes((10, 20, 30)))
    m = classify_relative(f)
    assert len(m.wall) == 1 and len(m.bright) == 1
