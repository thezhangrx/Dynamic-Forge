"""实时视频循环的回归测试 —— **不需要摄像头、不需要显示器**。

用假的 source 替掉 :class:`CameraSource`：这正是把采集关在
``read() -> (ok, frame, stamp)`` 一个小接口后面的好处 —— 循环本身可以被完整测试。
"""

from __future__ import annotations

import time

from vision.live_view import FpsMeter, ViewState, run


class FakeSource:
    """按脚本产帧的假相机。``frames=None`` 表示无限产帧。"""

    def __init__(self, frames: int | None = 5, *, fail_after: int | None = None,
                 interval: float = 0.0) -> None:
        self.device = 0
        self.frames = frames
        self.fail_after = fail_after
        self.interval = interval
        self.reads = 0

    def read(self):
        self.reads += 1
        if self.fail_after is not None and self.reads > self.fail_after:
            return False, None, time.perf_counter()
        if self.frames is not None and self.reads > self.frames:
            return False, None, time.perf_counter()
        if self.interval:
            time.sleep(self.interval)
        return True, object(), time.perf_counter()     # headless 不碰帧内容

    @property
    def width(self) -> int:
        return 640

    @property
    def height(self) -> int:
        return 480

    def describe(self) -> str:
        return "fake source"


def test_headless_runs_until_max_frames(capsys):
    src = FakeSource(frames=None)
    assert run(src, headless=True, max_frames=7, verbose_every=0) == 0
    assert src.reads == 7
    assert "共 7 帧" in capsys.readouterr().out


def test_seconds_is_wall_clock_not_frame_count():
    """--seconds 用墙钟：驱动标称帧率常常是实际的两倍，按帧数换算会差一倍。"""
    src = FakeSource(frames=None, interval=0.005)
    t0 = time.perf_counter()
    assert run(src, headless=True, seconds=0.2, verbose_every=0) == 0
    elapsed = time.perf_counter() - t0
    assert 0.2 <= elapsed < 1.0, f"应在约 0.2 s 后退出，实际 {elapsed:.2f} s"


def test_read_failure_returns_3():
    src = FakeSource(frames=None, fail_after=3)
    assert run(src, headless=True, max_frames=100, verbose_every=0) == 3
    assert src.reads == 4          # 第 4 次读到失败


def test_fps_meter_uses_timestamps():
    """帧率由时间戳算，不由帧号算。"""
    m = FpsMeter(window=10)
    for i in range(11):
        m.tick(i * 0.1)            # 每帧 0.1 s -> 10 fps
    assert abs(m.fps - 10.0) < 1e-9
    m2 = FpsMeter(window=10)
    assert m2.fps == 0.0           # 一帧都没有时不能除零
    m2.tick(1.0)
    assert m2.fps == 0.0           # 只有一帧也算不出


def test_fps_meter_window_is_rolling():
    m = FpsMeter(window=5)
    for i in range(100):           # 前面慢、后面快
        m.tick(i * (0.1 if i < 50 else 0.01))
    assert m.fps > 50.0            # 窗口只看最近 5 帧，不应被历史拖住


# --------------------------------------------------------------------------
# 缩放与"两个按钮"
# --------------------------------------------------------------------------
def test_layout_puts_exactly_two_buttons_top_right():
    st = ViewState()
    st.layout(640, 480)
    assert sorted(st.buttons) == ["in", "out"]        # 就两个，不多不少
    names = sorted(st.buttons)
    (ax0, ay0, ax1, ay1) = st.buttons["out"]
    (bx0, by0, bx1, by1) = st.buttons["in"]
    # 都在画面内、都在右上角（"-" 在左，"+" 在右）
    assert ax0 < bx0 and by0 == ay0
    assert ax1 <= bx1 and bx1 <= 640 and by1 <= 40
    assert ax0 > 640 // 2, "按钮应该在右上角，不要挡住画面主体"
    assert ax1 < bx0, "两个按钮不能重叠"


def test_hit_maps_click_to_the_right_button():
    st = ViewState()
    st.layout(640, 480)
    out_r, in_r = st.buttons["out"], st.buttons["in"]
    assert st.hit(out_r[0] + 2, out_r[1] + 2) == "out"
    assert st.hit(in_r[0] + 2, in_r[1] + 2) == "in"
    assert st.hit(320, 240) is None                   # 画面中间不是按钮


def test_click_and_wheel_change_zoom():
    import cv2
    st = ViewState()
    st.layout(640, 480)
    in_r = st.buttons["in"]
    st.on_mouse(cv2.EVENT_LBUTTONDOWN, in_r[0] + 1, in_r[1] + 1, 0, None)
    assert st.zoom > 1.0
    z = st.zoom
    st.on_mouse(cv2.EVENT_MOUSEWHEEL, 0, 0, 1, None)   # 滚轮向上 = 放大
    assert st.zoom > z
    st.on_mouse(cv2.EVENT_MOUSEWHEEL, 0, 0, -1, None)  # 滚轮向下 = 缩小
    assert st.zoom < z + 1e-9
    st.reset()
    assert st.zoom == 1.0


def test_zoom_is_clamped():
    st = ViewState()
    for _ in range(50):
        st.zoom_by(1.5)
    assert st.zoom <= st.max_zoom
    for _ in range(80):
        st.zoom_by(1 / 1.5)
    # 数字变焦下最小就是 1.0：传感器视场就那么大，再"缩小"也看不到更多东西
    assert st.zoom >= 1.0


def test_apply_zoom_keeps_size_and_crops_the_centre():
    """数字变焦 = 中心裁剪 + 放大回原尺寸：**尺寸不变、视野变小**。

    这跟"把整帧放大"是两回事——后者只是显示变大、视野不变。
    """
    import numpy as np
    from vision.live_view import apply_zoom
    h, w = 40, 80
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:, :w // 4] = (0, 0, 255)              # 左 1/4 红（放大后应被裁掉）
    frame[:, w // 4: 3 * w // 4] = (0, 255, 0)   # 中间一半绿（放大后应占满）
    frame[:, 3 * w // 4:] = (255, 0, 0)          # 右 1/4 蓝（放大后应被裁掉）

    assert apply_zoom(frame, 1.0) is frame       # 100% 原样返回，不拷贝

    out = apply_zoom(frame, 2.0)
    assert out.shape == frame.shape, "数字变焦后尺寸必须不变（不是把窗口撑大）"
    assert (out[:, :, 0] == 0).all(), "红色边缘应被裁掉"
    assert (out[:, :, 2] == 0).all(), "蓝色边缘应被裁掉"
    assert (out[:, :, 1] > 200).all(), "中间区域应放大占满"


def test_zoom_out_below_one_is_a_noop():
    """缩小到 1 倍以下没有意义：视场不会变大，所以原样返回。"""
    import numpy as np
    from vision.live_view import apply_zoom
    frame = np.zeros((16, 16, 3), dtype=np.uint8)
    assert apply_zoom(frame, 0.5) is frame


def test_camera_source_without_zoom_reports_unsupported():
    """相机不支持变焦时，set_zoom 必须安全地返回 False（好让调用方降级）。"""
    from vision.live_view import CameraSource
    src = CameraSource(0)
    assert src.has_zoom is False          # 没 open() 时默认就是不支持
    assert src.set_zoom(1.5) is False     # 不炸，只返回 False


def test_window_alive_is_false_for_a_window_that_never_existed():
    """关窗检测不能因为窗口不存在就抛异常——它必须安全地返回 False。"""
    from vision.live_view import _window_alive
    assert _window_alive("definitely-not-a-real-window") is False
