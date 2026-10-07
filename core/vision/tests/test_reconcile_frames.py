"""``vision_reconcile`` 里"读帧源 + 找四角标记标定"的回归测试。

为什么值得单独测
----------------
这两件事都在**调用脚本**里（不是 core 模块），所以最容易被"只测了 core"
漏掉。而它们恰恰是最靠近真实数据的一层：一旦这里坏了，报出来的是
``NameError`` / 找不到标记，而不是一个可疑的数字。

**这里钉的是一个真实事故**：把 ``_calibrate_from_video`` 重构成
``_calibrate_from_frames``（同时支持图序和视频）时，``import cv2`` 只在
``_iter_probe_frames`` 的**局部作用域**里，而 ``cv2.resize`` 在
``_calibrate_from_frames`` 里用 —— 于是**只有走降采样分支时**才
``NameError: name 'cv2' is not defined``。

为什么没在第一时间发现：当时的合成图只有 1280 px 宽，小于
``search_width=1600``，所以 ``scale == 1.0``、**降采样分支一次都没进去**。
真实录像 1920 px 宽，第一次跑就炸。教训是"分支要按它的**判定条件**去覆盖，
不是拿手上恰好有的数据顺手跑一遍"。
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("cv2")
cv2 = __import__("cv2")

# 被测实现住在 core/vision/sources.py（原先挤在根脚本 vision_reconcile.py 里）。
from vision.sources import (  # noqa: E402
    _calibrate_from_frames,
    _stream_frames,
)


def _screen(width: int, height: int, layout) -> np.ndarray:
    """按真实布局画一张"屏幕"（灰度，用真实标记灰度），返回 HxWx3 BGR。"""
    from standard.fiducial import FIDUCIAL_LEVEL

    g = np.full((height, width), 60, np.uint8)
    sx, sy = width / layout.field_w, height / layout.field_h
    for slot in range(4):
        for (x, y, side, white) in layout.rects(slot):
            ix0, ix1 = int(round(x * sx)), int(round((x + side) * sx))
            iy0 = int(round((layout.field_h - (y + side)) * sy))
            iy1 = int(round((layout.field_h - y) * sy))
            cv2.rectangle(g, (ix0, iy0), (ix1, iy1),
                          FIDUCIAL_LEVEL if white else 0, -1)
    return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)


def _write_video(path: pathlib.Path, frames) -> None:
    h, w = frames[0].shape[:2]
    vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 20.0, (w, h))
    for f in frames:
        vw.write(f)
    vw.release()


from standard.fiducial import FiducialLayout  # noqa: E402

LAYOUT = FiducialLayout(field_w=640.0, field_h=480.0, marker_side=48.0, margin=8.0)


@pytest.mark.parametrize("width,height", [
    (640, 480),      # scale == 1.0 -> 不降采样（旧测试只覆盖了这一档）
    (1280, 960),     # 小于 search_width=1600 -> 仍不降采样
    (1920, 1080),    # **大于** search_width -> 走 cv2.resize，就是这个分支炸的
    (3840, 2160),    # 4K：降得更多，角点要按比例放大回去
])
def test_calibration_works_at_every_width(width: int, height: int, tmp_path) -> None:
    """四种分辨率都要能标定 —— 尤其是**触发降采样**的那两种。

    断言标定**质量**而不只是"没抛异常"：降采样后如果角点忘了按比例放大回
    原分辨率，单应照样能解出来、RMS 也"正常"，但换算出来的场地单位整体偏小
    一个 ``scale`` 倍 —— 那是最危险的一类错（跑得通但全错）。
    """
    video = tmp_path / "v.mp4"
    _write_video(video, [_screen(width, height, LAYOUT) for _ in range(3)])
    calib, rep, idx = _calibrate_from_frames(video, LAYOUT, scan_frames=3)
    assert rep.n_markers == 4, f"{width}x{height}: 只找到 {rep.n_markers} 枚标记"
    # 像素 RMS 会随分辨率放大，所以除了它，还要验**比例**：
    # 场地宽度在图像里应当正好占 field_w 个场地单位 -> 反算回去要一致。
    from vision.localize import to_world
    from vision.primitives import Frame
    from vision import to_standard
    from vision.wall_with_gap import WallWithGapParams, detect

    bgr = _screen(width, height, LAYOUT)
    fr = Frame(width, height, bgr[:, :, 2].tobytes(), bgr[:, :, 1].tobytes(),
               bgr[:, :, 0].tobytes())
    from vision.localize import field_roi
    roi = field_roi(calib)
    assert roi is not None
    # 场地四角在图像里的位置 = ROI 四角；映射回场地应当接近 (0,0)-(640,480)
    x0, y0, x1, y1 = roi
    assert abs(x0) < 0.15 * width and abs(int(x1) - width) < 0.15 * width, \
        f"{width}x{height}: ROI x 跨度 {x0:.1f}..{x1:.1f} 与图像宽度 {width} 不符" \
        "（降采样后角点比例多半没还原）"


def test_images_source_calibrates_too(tmp_path) -> None:
    """图序来源（录制台产物）同样要能标定，并且和视频来源给出同一结果。"""
    frames = [_screen(1920, 1080, LAYOUT) for _ in range(3)]
    for i, f in enumerate(frames):
        cv2.imwrite(str(tmp_path / f"{i:04d}.jpg"), f)
    calib, rep, _idx = _calibrate_from_frames(tmp_path, LAYOUT, images=tmp_path,
                                                 scan_frames=3)
    assert rep.n_markers == 4
    assert rep.rms_px < 5.0


def test_missing_fiducials_reports_what_to_do(tmp_path) -> None:
    """一张纯背景（没有标记）必须**说清怎么办**，而不是抛一个看不懂的异常。"""
    v = tmp_path / "blank.mp4"
    _write_video(v, [np.full((480, 640, 3), 40, np.uint8) for _ in range(3)])
    with pytest.raises(SystemExit) as ei:
        _calibrate_from_frames(v, LAYOUT, scan_frames=3)
    msg = str(ei.value)
    assert "标定标记" in msg and "vision_capture_rig" in msg


# ---------------------------------------------------------------------------
# 流式取帧的两个开关：--process-width（缩放）与 --stride（隔帧取样）
# ---------------------------------------------------------------------------
def test_stride_keeps_original_stamps(tmp_path) -> None:
    """``--stride`` 下时戳必须取**原始序号**那一个，不能按取样下标取。

    踩法：图序分支里 ``picked = files[::stride]``，若用 ``stamps[i]``（i 是
    取样下标）而不是 ``stamps[i*stride]``，整条时戳序列会**整体前移**
    ``stride`` 倍 —— 配对照样能成（门限内），但配到的是错误时刻，
    误差里混进的是时间错而不是识别错。这条 bug 不看时戳本身发现不了。
    """
    frames = [_screen(640, 480, LAYOUT) for _ in range(6)]
    for i, f in enumerate(frames):
        cv2.imwrite(str(tmp_path / f"{i:04d}.jpg"), f)
    stamps = [10.0, 10.5, 11.0, 11.5, 12.0, 12.5]

    got = [s for s, _b in _stream_frames(tmp_path, tmp_path, stamps=stamps,
                                             stride=2, process_width=0,
                                             with_stamps=True)]
    assert got == [10.0, 11.0, 12.0], f"隔帧取样的时戳错了: {got}"


def test_process_width_resizes_consistently(tmp_path) -> None:
    """缩放必须**等比**，而且标定与识别走同一个出口（否则单应差一个比例）。"""
    frames = [_screen(1920, 1080, LAYOUT) for _ in range(3)]
    for i, f in enumerate(frames):
        cv2.imwrite(str(tmp_path / f"{i:04d}.jpg"), f)

    for w in (0, 1280, 960, 640):
        sizes = {b.shape[1] for _i, b in _stream_frames(
            tmp_path, tmp_path, process_width=w, with_stamps=False)}
        want = 1920 if w == 0 else w
        assert sizes == {want}, f"process_width={w} 得到宽度 {sizes}"
        # 等比：高度按同一比例
        for _i, b in _stream_frames(tmp_path, tmp_path, process_width=w):
            s = b.shape[1] / 1920.0
            assert abs(b.shape[0] - round(1080 * s)) <= 1, \
                f"process_width={w} 不是等比缩放: {b.shape}"


def test_calibration_at_process_width_stays_accurate(tmp_path) -> None:
    """缩放之后标定仍要准 —— 单应是在**缩放后**的像素里解的，不能差一个比例。"""
    frames = [_screen(1920, 1080, LAYOUT) for _ in range(3)]
    video = tmp_path / "v.mp4"
    _write_video(video, frames)
    for w in (0, 1280, 960):
        calib, rep, _idx = _calibrate_from_frames(
            video, LAYOUT, scan_frames=3, process_width=w)
        assert rep.n_markers == 4, f"process_width={w}: 只找到 {rep.n_markers} 枚"
        # 场地单位下的换算必须仍然对：把场地角映射到图像，跨度应≈图像里的场地占比
        from vision.localize import field_roi
        roi = field_roi(calib)
        assert roi is not None
        x0, _y0, x1, _y1 = roi
        img_w = 1920 if w == 0 else w
        # 场地宽 640 单位；两种极端比例下 ROI 宽度都应落在画面宽度的 60%~105%
        frac = (x1 - x0) / img_w
        assert 0.55 < frac < 1.05, f"process_width={w}: ROI 占画面宽度 {frac:.2f} 不合理"


def test_max_frames_limits_after_stride(tmp_path) -> None:
    """``--max-frames`` 限的是**处理帧数**（与 --stride 组合时语义要明确）。"""
    frames = [_screen(640, 480, LAYOUT) for _ in range(10)]
    for i, f in enumerate(frames):
        cv2.imwrite(str(tmp_path / f"{i:04d}.jpg"), f)
    got = list(_stream_frames(tmp_path, tmp_path, max_frames=3,
                                  stride=2, with_stamps=False))
    assert len(got) == 3
    assert [i for i, _b in got] == [0, 2, 4]
