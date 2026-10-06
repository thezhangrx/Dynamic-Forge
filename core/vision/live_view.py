"""实时视频：连续采集 + 窗口显示（带放大/缩小两个按钮）。

**可迁移性放在第一位**：采集被关在 :class:`CameraSource` 后面，整个循环只依赖
``read() -> (ok, frame, stamp)`` 这一个形状。将来把摄像头换成 V4L2 直控、换成 FPGA
的像素流、或换成录像文件回放，只需要再写一个同形状的 source，**循环一行都不用改**。

窗口为什么长这样
----------------
OpenCV 的 Qt 后端默认会给窗口加一排按钮（保存 / 放大 / 缩小 / 原始尺寸…），
**这排按钮无法从 Python 裁剪**——要么全要，要么全不要。所以这里的做法是：

* ``WINDOW_GUI_NORMAL`` 把 Qt 那排按钮整个去掉；
* 自己在画面右上角画**两个**按钮（放大 ``+`` / 缩小 ``-``），用鼠标回调响应点击；
* 键盘 ``+`` / ``-`` / ``0`` 与滚轮同样可用。

另外修掉了"点 × 关掉窗口后它又弹回来"的问题：每帧检查 ``WND_PROP_VISIBLE``，
窗口被关掉就退出循环。

缩放：优先驱动摄像头，做不到才做数字变焦
----------------------------------------
按钮想控制的是**摄像头自己的缩放**，而不是把显示画面放大。所以打开设备时会探测
``CAP_PROP_ZOOM``（对应 V4L2 的 ``zoom_absolute``）：

* **相机支持** → 两个按钮直接写摄像头的变焦控制项（真·摄像头缩放，视野与分辨率都由
  镜头/传感器决定）；
* **相机不支持**（实测本机的 USB 相机就是这种：没有 ``zoom_absolute``、没有
  focus/pan/tilt，``VIDIOC_G_CROP`` 也返回 EINVAL）→ 退回**中心裁剪式数字变焦**：
  把画面中心按 zoom 倍裁一块再放大回原尺寸。视野**真的变小**（像拉近），窗口尺寸不变；
  代价是分辨率下降（裁出来的像素本来就少），而且传感器其实把整个场景都拍下来了。

叠加读数会标明当前是哪种（``zoom 150% digital`` / ``zoom 150% camera``），
免得把数字变焦误当成光学变焦。

四个采集要点（都有实测依据）

1. **先设 FOURCC 再设分辨率**。MJPG 是压缩格式，同带宽下能跑更高帧率；很多驱动
   只有在先声明 MJPG 之后才开放高帧率档位。顺序反了会静默退化。
2. **把驱动内部缓冲压到 1**。默认缓冲很深，处理一慢就"越看越滞后"；设完读回确认。
3. **FPS 用真实时间戳算，不用帧号**。显示的帧率与标称帧率是两回事，分别报。
4. **``stamp`` 是"取到帧"的时刻，不是"曝光"的时刻**。真要算速度必须走 V4L2 读
   ``v4l2_buffer.timestamp``（OpenCV 不暴露）。
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import cv2

#: 默认窗口名。
DEFAULT_WINDOW = "vision live"
#: 退出键。
QUIT_KEYS = (27, ord("q"))          # ESC / q
#: 切换叠加信息的键。
OVERLAY_KEY = ord("h")
#: 缩放键。
ZOOM_IN_KEYS = (ord("+"), ord("="))
ZOOM_OUT_KEYS = (ord("-"), ord("_"))
ZOOM_RESET_KEYS = (ord("0"),)

#: 窗口标志：AUTOSIZE 让窗口跟着画面尺寸走，GUI_NORMAL 去掉 Qt 自带的那排按钮。
WINDOW_FLAGS = cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL

ZOOM_MIN, ZOOM_MAX, ZOOM_STEP = 1.0, 4.0, 1.25
BUTTON_PX, BUTTON_GAP, BUTTON_MARGIN = 30, 6, 8

#: UVC 的 zoom_absolute 以 100 为 1 倍。
UVC_ZOOM_UNIT = 100


# --------------------------------------------------------------------------
# 采集源（可替换的一层）
# --------------------------------------------------------------------------
class CameraSource:
    """OpenCV 采集源：``read() -> (ok, frame, stamp)``。

    只暴露三个动作（``open`` / ``read`` / ``close``），所以换实现很便宜：
    想换成 V4L2 直控或 FPGA 像素流时，复制这个类的**方法签名**即可。
    """

    def __init__(
        self,
        device: int = 0,
        *,
        width: int = 640,
        height: int = 480,
        fps: int | None = None,
        fourcc: str = "MJPG",
        buffersize: int = 1,
    ) -> None:
        self.device = int(device)
        self.req_width = int(width)
        self.req_height = int(height)
        self.req_fps = None if fps is None else int(fps)
        self.fourcc = fourcc
        self.buffersize = int(buffersize)
        self._cap: Any = None
        #: 设完之后读回来的实际值（-1 表示后端不支持该属性）。
        self.effective_buffersize: int = -1
        #: 摄像头自己是否支持变焦（``zoom_absolute``）。
        self.has_zoom: bool = False

    # -- 生命周期 --------------------------------------------------------
    def open(self) -> "CameraSource":
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(
                f"打不开 /dev/video{self.device}；确认设备存在（ls -l /dev/video*）、"
                "当前用户在 video 组、且没有别的程序占用它"
            )
        if self.fourcc:
            # 顺序：FOURCC -> 分辨率 -> 帧率 -> 缓冲
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.req_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.req_height)
        if self.req_fps is not None:
            cap.set(cv2.CAP_PROP_FPS, self.req_fps)
        if self.buffersize > 0:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffersize)
            self.effective_buffersize = int(cap.get(cv2.CAP_PROP_BUFFERSIZE))
        self._cap = cap
        self.has_zoom = self._probe_zoom()
        return self

    def _probe_zoom(self) -> bool:
        """这个相机支持 ``zoom_absolute`` 吗？

        OpenCV 没有"查询支持哪些属性"的接口，只能试探：不支持的属性 ``get`` 返回
        ``-1``、``set`` 返回 ``False``。这里把当前值**原样设回去**——不改变状态，
        但能问出驱动收不收这个属性。
        """
        try:
            current = self._cap.get(cv2.CAP_PROP_ZOOM)
        except cv2.error:                      # pragma: no cover - 后端相关
            return False
        if current is None or current < 0:
            return False
        try:
            return bool(self._cap.set(cv2.CAP_PROP_ZOOM, current))
        except cv2.error:                      # pragma: no cover - 后端相关
            return False

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self) -> "CameraSource":
        return self.open()

    def __exit__(self, *exc: object) -> bool:
        self.close()
        return False

    # -- 读一帧 ----------------------------------------------------------
    def read(self) -> tuple[bool, Any, float]:
        """``(ok, frame, stamp)``；``stamp`` 是取到这一帧的时刻（秒，单调）。"""
        if self._cap is None:
            raise RuntimeError("CameraSource 还没 open()")
        ok, frame = self._cap.read()
        return bool(ok), frame, time.perf_counter()

    # -- 摄像头变焦 ------------------------------------------------------
    def set_zoom(self, zoom: float) -> bool:
        """把摄像头自己的变焦设成 ``zoom`` 倍（UVC 以 100 为 1 倍）。

        相机不支持时返回 ``False``，由调用方退回数字变焦。
        """
        if self._cap is None or not self.has_zoom:
            return False
        return bool(self._cap.set(cv2.CAP_PROP_ZOOM, int(round(zoom * UVC_ZOOM_UNIT))))

    # -- 元信息（诊断用） -------------------------------------------------
    @property
    def width(self) -> int:
        return int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)) if self._cap else 0

    @property
    def height(self) -> int:
        return int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) if self._cap else 0

    @property
    def reported_fps(self) -> float:
        """驱动标称帧率。**不是**实测帧率，别拿它算速度。"""
        return float(self._cap.get(cv2.CAP_PROP_FPS)) if self._cap else 0.0

    def describe(self) -> str:
        buf = self.effective_buffersize
        buf_s = f"{buf}" if buf >= 0 else "不支持该属性"
        zoom_s = "支持" if self.has_zoom else "不支持（用数字变焦代替）"
        return (f"/dev/video{self.device}  {self.width}x{self.height}  "
                f"标称 {self.reported_fps:.1f} fps  内部缓冲={buf_s}  摄像头变焦={zoom_s}")


# --------------------------------------------------------------------------
# 帧率计
# --------------------------------------------------------------------------
@dataclass
class FpsMeter:
    """滑动窗口实测帧率（用时间戳差，不用帧号）。"""

    window: int = 30
    _stamps: deque = field(default_factory=lambda: deque(maxlen=30))

    def __post_init__(self) -> None:
        self._stamps = deque(maxlen=max(2, int(self.window)))

    def tick(self, stamp: float) -> None:
        self._stamps.append(float(stamp))

    @property
    def fps(self) -> float:
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 0 else 0.0


# --------------------------------------------------------------------------
# 缩放 + 自己画的两个按钮
# --------------------------------------------------------------------------
@dataclass
class ViewState:
    """缩放状态、右上角两个按钮的矩形，以及鼠标命中处理。

    ``zoom`` 是**倍率**：1.0 = 不变。数字变焦下最小就是 1.0——传感器的视场就那么大，
    再"缩小"也看不到更多东西。
    """

    zoom: float = 1.0
    min_zoom: float = ZOOM_MIN
    max_zoom: float = ZOOM_MAX
    #: 按钮名 -> (x0, y0, x1, y1)；每帧按当前画面尺寸刷新。
    buttons: dict[str, tuple[int, int, int, int]] = field(default_factory=dict)

    def zoom_by(self, factor: float) -> None:
        self.zoom = min(self.max_zoom, max(self.min_zoom, self.zoom * factor))

    def reset(self) -> None:
        self.zoom = 1.0

    # -- 布局 ------------------------------------------------------------
    def layout(self, width: int, height: int) -> None:
        """两个按钮横排在右上角：``-`` 在左，``+`` 在右。"""
        y0 = BUTTON_MARGIN
        y1 = y0 + BUTTON_PX
        x1 = width - BUTTON_MARGIN
        out_x0 = x1 - BUTTON_PX - BUTTON_GAP - BUTTON_PX
        self.buttons = {
            "out": (out_x0, y0, out_x0 + BUTTON_PX, y1),
            "in": (x1 - BUTTON_PX, y0, x1, y1),
        }

    def hit(self, x: int, y: int) -> str | None:
        for name, (x0, y0, x1, y1) in self.buttons.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
        return None

    # -- 事件 ------------------------------------------------------------
    def on_mouse(self, event: int, x: int, y: int, flags: int, _param: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            name = self.hit(x, y)
            if name == "in":
                self.zoom_by(ZOOM_STEP)
            elif name == "out":
                self.zoom_by(1.0 / ZOOM_STEP)
        elif event == cv2.EVENT_MOUSEWHEEL:
            self.zoom_by(ZOOM_STEP if flags > 0 else 1.0 / ZOOM_STEP)


def draw_buttons(frame: Any, state: ViewState) -> None:
    """把两个按钮画到画面上（不依赖任何 GUI 控件）。"""
    for name, (x0, y0, x1, y1) in state.buttons.items():
        cv2.rectangle(frame, (x0, y0), (x1, y1), (40, 40, 40), -1)
        cv2.rectangle(frame, (x0, y0), (x1, y1), (220, 220, 220), 1)
        cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
        if name == "in":                                  # 加号
            cv2.line(frame, (cx - 7, cy), (cx + 7, cy), (255, 255, 255), 2)
            cv2.line(frame, (cx, cy - 7), (cx, cy + 7), (255, 255, 255), 2)
        else:                                             # 减号
            cv2.line(frame, (cx - 7, cy), (cx + 7, cy), (255, 255, 255), 2)


def apply_zoom(frame: Any, zoom: float) -> Any:
    """**中心裁剪式数字变焦**：按 ``zoom`` 倍裁中心一块，再放大回原尺寸。

    与"把整帧放大"的区别：那个只是显示变大、视野不变；这个是**视野真的变小**
    （像拉近），窗口尺寸也保持不变。代价是分辨率下降——裁出来的像素本来就少。

    ``zoom <= 1`` 时原样返回（传感器的视场就那么大，缩小看不到更多东西）。
    """
    if zoom <= 1.0 + 1e-6:
        return frame
    h, w = frame.shape[:2]
    cw, ch = max(1, int(round(w / zoom))), max(1, int(round(h / zoom)))
    x0, y0 = (w - cw) // 2, (h - ch) // 2
    crop = frame[y0:y0 + ch, x0:x0 + cw]
    return cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)


# --------------------------------------------------------------------------
# 主循环
# --------------------------------------------------------------------------
def _draw_overlay(frame: Any, lines: list[str]) -> None:
    """左上角画几行小字（带描边，任何背景都看得清）。"""
    y = 18
    for text in lines:
        cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        y += 18


def _window_alive(window: str) -> bool:
    """窗口还在吗？点右上角 × 之后这里会变成 <1，循环据此退出。"""
    try:
        return cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def run(
    source: CameraSource,
    *,
    window: str = DEFAULT_WINDOW,
    headless: bool = False,
    overlay: bool = True,
    max_frames: int | None = None,
    seconds: float | None = None,
    verbose_every: int = 60,
) -> int:
    """连续采集并显示，直到按 ESC/q、点窗口的 ×（或跑满 ``max_frames`` / ``seconds``）。

    ``seconds`` 用**墙钟**而不是换算成帧数：驱动标称帧率经常与实际不符
    （实测某相机标称 60 fps 只跑 30 fps），拿标称值换算会差一倍。

    返回进程退出码：0 正常，3 收不到帧，4 无法开窗（建议 ``headless=True``）。
    """
    meter = FpsMeter()
    state = ViewState()
    frames = 0
    started = time.perf_counter()
    created = False
    #: 摄像头自己能不能变焦；不能就走数字变焦。
    camera_zoom = bool(getattr(source, "has_zoom", False))
    mode = "camera" if camera_zoom else "digital"

    try:
        while True:
            ok, frame, stamp = source.read()
            if not ok or frame is None:
                print("读取失败：收不到帧。检查顺序：①设备被占用/掉线；"
                      "②该格式或分辨率不被支持（试试 --fourcc MJPG）；"
                      "③USB 带宽不足（未压缩的 YUYV 最容易超时）")
                return 3

            frames += 1
            meter.tick(stamp)

            if headless:
                if verbose_every and frames % verbose_every == 0:
                    print(f"  frame {frames:6d}  实测 {meter.fps:5.1f} fps")
            else:
                if not created:
                    # 自己建窗口，才能带上 GUI_NORMAL（去掉 Qt 那排按钮）
                    cv2.namedWindow(window, WINDOW_FLAGS)
                    cv2.setMouseCallback(window, state.on_mouse)
                    created = True

                # 缩放：能驱动摄像头就驱动摄像头，否则中心裁剪数字变焦
                if camera_zoom:
                    if not source.set_zoom(state.zoom):
                        camera_zoom = False          # 中途失败就降级
                        mode = "digital"
                    shown = frame
                else:
                    shown = apply_zoom(frame, state.zoom)

                state.layout(shown.shape[1], shown.shape[0])
                if overlay:
                    _draw_overlay(shown, [
                        f"{meter.fps:5.1f} FPS  (measured)",
                        f"{source.width}x{source.height}  dev{source.device}"
                        f"  zoom {state.zoom * 100:.0f}% {mode}",
                    ])
                draw_buttons(shown, state)
                try:
                    cv2.imshow(window, shown)
                except cv2.error as exc:            # pragma: no cover - 依赖显示环境
                    print(f"无法创建窗口（{exc}）；无显示环境请加 --headless")
                    return 4
                key = cv2.waitKey(1) & 0xFF
                if key in QUIT_KEYS:
                    break
                if key == OVERLAY_KEY:
                    overlay = not overlay
                if key in ZOOM_IN_KEYS:
                    state.zoom_by(ZOOM_STEP)
                if key in ZOOM_OUT_KEYS:
                    state.zoom_by(1.0 / ZOOM_STEP)
                if key in ZOOM_RESET_KEYS:
                    state.reset()
                if not _window_alive(window):
                    print("窗口已关闭，退出")
                    break

            if max_frames is not None and frames >= max_frames:
                break
            if seconds is not None and (time.perf_counter() - started) >= seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if not headless:
            cv2.destroyAllWindows()

    elapsed = time.perf_counter() - started
    avg = frames / elapsed if elapsed > 0 else 0.0
    print(f"共 {frames} 帧 / {elapsed:.1f} s，平均 {avg:.1f} fps（实测）；缩放模式={mode}")
    return 0


__all__ = [
    "DEFAULT_WINDOW", "QUIT_KEYS", "WINDOW_FLAGS", "UVC_ZOOM_UNIT",
    "CameraSource", "FpsMeter", "ViewState", "apply_zoom",
    "draw_buttons", "run",
]
