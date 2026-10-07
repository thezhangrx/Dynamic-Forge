"""相机取帧（连续/后台线程）与"这一帧是不是平台"的判据。

从根脚本 ``vision_capture_rig.py`` 下沉而来：那个文件按约定只该放"调用代码"，
而 ``Camera``（后台线程抓帧）与 ``looks_like_platform``（画面判据）都是可复用的
实现，属于视觉模块。

``Camera`` 为什么必须独立成线程
-------------------------------
``cap.read()`` 在实测的外接 UVC 摄像头上**一帧要 ~120 ms**（8.3 fps）。
放在渲染主循环里会把平台拖到 8 fps —— 现象是"平台卡卡的"，
而根因是相机而不是渲染。放进后台线程后主循环只管步进+渲染（~60 fps），
有新的帧就取走存一张。
"""

from __future__ import annotations

import threading
import time

import numpy as np


class Camera(threading.Thread):
    """后台抓帧。主线程随时 ``take()`` 拿最新的一帧。"""

    def __init__(self, device: int, width: int, height: int, warmup: int) -> None:
        super().__init__(daemon=True)
        import cv2
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # 队列只留 1 帧：默认缓冲会让"新拿到的帧其实是旧的"，而现象只是速度算错。
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit(f"打不开 /dev/video{device}")
        for _ in range(warmup):
            self.cap.grab()
        self._lock = threading.Lock()
        self._latest: tuple[float, object] | None = None
        self._stop = threading.Event()

    def run(self) -> None:
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok or frame is None:
                continue
            # 时戳取 read() 返回的瞬间 = "拿到这帧"的时刻。真正的曝光比它早一个
            # 固定时延，那部分由对账时的凸包法估进 offset；可变的那部分才是敌人。
            with self._lock:
                self._latest = (time.monotonic(), frame)

    def take(self):
        """取走最新一帧（没有新的返回 None）。"""
        with self._lock:
            item, self._latest = self._latest, None
        return item

    def close(self) -> None:
        self._stop.set()
        self.cap.release()


def looks_like_platform(bgr) -> tuple[bool, str]:
    """场地底是 (34,40,55)：整体暗、且偏蓝。编辑器主题是中性灰白，一测就分开。"""
    a = np.asarray(bgr)[:, :, ::-1].astype(int)
    mx = a.max(axis=2)
    dark = float((mx < 140).mean())
    bluish = float(((a[:, :, 2] > a[:, :, 0] + 8) & (mx < 180)).mean())
    return (dark > 0.5 and bluish > 0.25), f"暗={100 * dark:.0f}% 偏蓝={100 * bluish:.0f}%"

