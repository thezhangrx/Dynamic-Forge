"""通用图像原语 —— 面向 Verilog 转化的纯 Python 实现。

设计前提（重要）
----------------
这套代码将来要**逐条翻译成 Verilog**，所以刻意遵守三条约束：

1. **不依赖 numpy / opencv 做运算**。只用 Python 内建类型（``bytes`` / ``bytearray`` / ``list`` /
   ``int``），每个函数都对应硬件上一个明确的小部件。第三方库只出现在 ``image_io.py``
   这一个文件里，且只负责"把图片读成像素、把像素写成图片"。
2. **只做流式 / 一维操作**。没有轮廓、没有 `minAreaRect`、没有形态学闭运算——这些在
   Verilog 里没有对应物。取而代之的是：逐像素比较器、行/列累加器、一维游程状态机、
   扫描线连通域、纵向差分（行缓冲）。
3. **每个函数都是纯函数**：不吃全局状态，输入输出都是普通值，方便单独写 testbench。

与 detector 的分工：本文件不含任何"墙 / 玩家 / 缺口"语义，那些判断在
``wall_with_gap.py`` 里。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

#: 文件名里的日期戳格式（与启动调试日志一致）。
STAMP_FORMAT = "%Y%m%d_%H%M%S"


# --------------------------------------------------------------------------
# 路径（纯 stdlib）
# --------------------------------------------------------------------------
def repo_root() -> Path | None:
    """仓库根；装成 wheel 后返回 ``None``。"""
    # core/vision/primitives.py -> parents[2] == <repo>
    root = Path(__file__).resolve().parents[2]
    return root if (root / "core").is_dir() else None


def data_dir(module: str = "vision") -> Path:
    """模块产物目录 ``<repo>/data/<module>``。"""
    root = repo_root()
    base = (root / "data") if root is not None else Path.cwd() / "data"
    return base / module


def stamp(now: float | None = None) -> str:
    return time.strftime(STAMP_FORMAT, time.localtime(now))


def newest_image(directory: Path | None = None) -> Path | None:
    """目录里最近修改的一张图片。"""
    directory = data_dir() if directory is None else Path(directory)
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    files = [p for p in directory.glob("*") if p.suffix.lower() in exts]
    return max(files, key=lambda p: p.stat().st_mtime) if files else None


# --------------------------------------------------------------------------
# 帧：三个平面（R/G/B），每个像素一个 byte
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Frame:
    """一帧图像。三个通道分开存 —— 硬件里就是三条并行的像素流。"""

    width: int
    height: int
    r: bytes
    g: bytes
    b: bytes

    @property
    def n(self) -> int:
        return self.width * self.height

    def pixel(self, x: int, y: int) -> tuple[int, int, int]:
        i = y * self.width + x
        return self.r[i], self.g[i], self.b[i]

    def to_rgb(self) -> bytearray:
        """交错成 ``R,G,B,R,G,B,...``（画图 / 存盘用）。"""
        out = bytearray(self.n * 3)
        out[0::3] = self.r
        out[1::3] = self.g
        out[2::3] = self.b
        return out

    @staticmethod
    def from_rgb(width: int, height: int, rgb: bytes | bytearray) -> "Frame":
        buf = bytes(rgb)
        return Frame(width, height, buf[0::3], buf[1::3], buf[2::3])


# --------------------------------------------------------------------------
# 逐像素分类（三个并行比较器）
# --------------------------------------------------------------------------
@dataclass
class Masks:
    """三张二值掩膜，每张都是长度 ``w*h`` 的 0/1 字节。"""

    wall: bytearray
    bright: bytearray
    green: bytearray


def classify(
    frame: Frame,
    *,
    wall_diff: int = 40,
    wall_min_r: int = 110,
    wall_min_g: int = 60,
    bright_min: int = 195,
    bright_spread: int = 45,
    green_score: int = 10,
    green_min_g: int = 90,
) -> Masks:
    """一帧 → 三张掩膜。Verilog：三个并行的减法器 + 比较器，一拍一像素。

    判据用**通道差**而不是 HSV：屏幕眩光会把饱和度洗没，``R-B`` 这类差值更抗洗白。
    """
    n = frame.n
    wall = bytearray(n)
    bright = bytearray(n)
    green = bytearray(n)
    for i in range(n):
        r = frame.r[i]
        g = frame.g[i]
        b = frame.b[i]
        if r - b > wall_diff and r > wall_min_r and g > wall_min_g:
            wall[i] = 1
        hi = r if r > g else g
        if b > hi:
            hi = b
        lo = r if r < g else g
        if b < lo:
            lo = b
        if lo > bright_min and hi - lo < bright_spread:
            bright[i] = 1
        if g - (r if r > b else b) > green_score and g > green_min_g:
            green[i] = 1
    return Masks(wall, bright, green)


# --------------------------------------------------------------------------
# 一维游程（状态机）
# --------------------------------------------------------------------------
def runs(flags) -> list[tuple[int, int]]:
    """布尔序列 → 连续 True 的区间列表 ``[(start, end), ...]``（闭区间）。

    Verilog：一个"上升沿开计数器、下降沿输出"的小状态机。
    """
    out: list[tuple[int, int]] = []
    start = None
    for i, v in enumerate(flags):
        if v and start is None:
            start = i
        elif not v and start is not None:
            out.append((start, i - 1))
            start = None
    if start is not None:
        out.append((start, len(flags) - 1))
    return out


def join_close(rs: list[tuple[int, int]], max_gap: int) -> list[tuple[int, int]]:
    """把间隔不超过 ``max_gap`` 的相邻区间合并（补掉网格线造成的断口）。"""
    if not rs:
        return []
    out = [list(rs[0])]
    for a, b in rs[1:]:
        if a - out[-1][1] - 1 <= max_gap:
            out[-1][1] = b
        else:
            out.append([a, b])
    return [(a, b) for a, b in out]


def drop_short(rs: list[tuple[int, int]], min_len: int) -> list[tuple[int, int]]:
    return [(a, b) for a, b in rs if b - a + 1 >= min_len]


# --------------------------------------------------------------------------
# 行 / 列直方图（累加器）
# --------------------------------------------------------------------------
def row_counts(mask: bytearray, width: int, y0: int, y1: int,
               x0: int, x1: int) -> list[int]:
    """``[y0, y1]`` 每行的掩膜像素数（列范围 ``[x0, x1]``）。"""
    return [sum(mask[y * width + x0: y * width + x1 + 1]) for y in range(y0, y1 + 1)]


def col_counts(mask: bytearray, width: int, x0: int, x1: int,
               y0: int, y1: int) -> list[int]:
    """``[x0, x1]`` 每列的掩膜像素数（行范围 ``[y0, y1]``）。"""
    return [sum(mask[y * width + x] for y in range(y0, y1 + 1))
            for x in range(x0, x1 + 1)]


# --------------------------------------------------------------------------
# 纵向差分（行缓冲）—— 抗"灯反光把颜色洗白"
# --------------------------------------------------------------------------
def col_step_max(frame: Frame, y0: int, y1: int, x0: int, x1: int,
                 k: int = 2) -> list[int]:
    """每列在 ``[y0, y1]`` 内最大的**持续台阶**：行缝 ``e`` 上"上 ``k`` 行之和"与
    "下 ``k`` 行之和"之差的最大值。

    这是整套识别里最关键的一步。为什么不用相邻行差分：墙被灯光洗白后**颜色没了、
    但墙的整块亮度跳变还在**，而场地网格线只有 1 像素宽——平均到 k 行之后，网格线的
    台阶被摊薄到 1/k，墙的台阶几乎不变。于是同一个阈值就能同时做到
    "认出被洗白的墙"和"不把网格线当墙"。

    Verilog：一个行缓冲 + 两个 k 抽头累加器 + 一个减法器 + 比较器。
    """
    width = frame.width
    g = frame.g
    if y1 - y0 + 1 < 2 * k + 1:
        k = 1
    out = []
    for x in range(x0, x1 + 1):
        best = 0
        for e in range(y0 + k, y1 - k + 1):
            above = 0
            below = 0
            for i in range(1, k + 1):
                above += g[(e - i) * width + x]
                below += g[(e + i) * width + x]
            d = below - above
            if d < 0:
                d = -d
            if d > best:
                best = d
        out.append(best)
    return out


# --------------------------------------------------------------------------
# 扫描线连通域（只存上一行的游程）
# --------------------------------------------------------------------------
@dataclass
class Blob:
    """一个连通域的外接框与一阶矩。"""

    x0: int
    y0: int
    x1: int
    y1: int
    area: int
    sum_x: int
    sum_y: int

    @property
    def cx(self) -> float:
        return self.sum_x / self.area if self.area else 0.0

    @property
    def cy(self) -> float:
        return self.sum_y / self.area if self.area else 0.0

    @property
    def w(self) -> int:
        return self.x1 - self.x0 + 1

    @property
    def h(self) -> int:
        return self.y1 - self.y0 + 1

    @property
    def fill(self) -> float:
        """面积 / 外接框面积。正圆 ≈ π/4 ≈ 0.785，圆环低得多。"""
        box = self.w * self.h
        return self.area / box if box else 0.0

    @property
    def aspect(self) -> float:
        short = self.w if self.w < self.h else self.h
        return (self.w if self.w > self.h else self.h) / short if short else float("inf")


def scanline_blobs(mask: bytearray, width: int, y0: int, y1: int,
                   x0: int, x1: int, *, min_area: int = 1) -> list[Blob]:
    """单遍连通域标记：只把**上一行的游程**放在手里，逐行匹配重叠。

    Verilog：一行游程缓存 + 比较器组 + 几个累加器，不需要整帧缓存。
    """
    active: list[Blob] = []
    done: list[Blob] = []
    for y in range(y0, y1 + 1):
        row_flags = [mask[y * width + x] for x in range(x0, x1 + 1)]
        row_runs = runs(row_flags)
        used = [False] * len(row_runs)
        next_active: list[Blob] = []
        for blob in active:
            hit = -1
            for i, (a, b) in enumerate(row_runs):
                if not used[i] and b >= blob.x0 and a <= blob.x1:
                    hit = i
                    break
            if hit < 0:
                done.append(blob)
                continue
            a, b = row_runs[hit]
            used[hit] = True
            blob.x0 = min(blob.x0, a + x0)
            blob.x1 = max(blob.x1, b + x0)
            blob.y1 = y
            blob.area += b - a + 1
            blob.sum_x += sum(range(a + x0, b + x0 + 1))
            blob.sum_y += y * (b - a + 1)
            next_active.append(blob)
        for i, (a, b) in enumerate(row_runs):
            if used[i]:
                continue
            n = b - a + 1
            next_active.append(Blob(a + x0, y, b + x0, y, n,
                                    sum(range(a + x0, b + x0 + 1)), y * n))
        active = next_active
    done.extend(active)
    return [b for b in done if b.area >= min_area]


# --------------------------------------------------------------------------
# 画图（纯 Python，直接写交错 RGB 缓冲）
# --------------------------------------------------------------------------
def draw_rect_outline(buf: bytearray, w: int, h: int,
                      x0: int, y0: int, x1: int, y1: int,
                      colour: tuple[int, int, int]) -> None:
    for x in range(max(0, x0), min(w - 1, x1) + 1):
        for y in (y0, y1):
            if 0 <= y < h:
                i = (y * w + x) * 3
                buf[i:i + 3] = bytes(colour)
    for y in range(max(0, y0), min(h - 1, y1) + 1):
        for x in (x0, x1):
            if 0 <= x < w:
                i = (y * w + x) * 3
                buf[i:i + 3] = bytes(colour)


def draw_line(buf: bytearray, w: int, h: int,
              x0: int, y0: int, x1: int, y1: int,
              colour: tuple[int, int, int], thick: int = 1) -> None:
    """Bresenham 直线（整数运算，硬件里就是两个累加器）。"""
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        for ox in range(-(thick // 2), thick // 2 + 1):
            for oy in range(-(thick // 2), thick // 2 + 1):
                x, y = x0 + ox, y0 + oy
                if 0 <= x < w and 0 <= y < h:
                    i = (y * w + x) * 3
                    buf[i:i + 3] = bytes(colour)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def draw_circle(buf: bytearray, w: int, h: int,
                cx: int, cy: int, rad: int,
                colour: tuple[int, int, int], thick: int = 1) -> None:
    """中点圆算法（整数运算）。"""
    for ring in range(thick):
        rr = rad - ring
        if rr < 0:
            break
        x, y, err = rr, 0, 1 - rr
        while x >= y:
            for px, py in ((x, y), (y, x), (-x, y), (-y, x),
                           (-x, -y), (-y, -x), (x, -y), (y, -x)):
                sx, sy = cx + px, cy + py
                if 0 <= sx < w and 0 <= sy < h:
                    i = (sy * w + sx) * 3
                    buf[i:i + 3] = bytes(colour)
            y += 1
            if err < 0:
                err += 2 * y + 1
            else:
                x -= 1
                err += 2 * (y - x) + 1


__all__ = [
    "STAMP_FORMAT",
    "Blob",
    "Frame",
    "Masks",
    "classify",
    "col_counts",
    "col_step_max",
    "data_dir",
    "draw_circle",
    "draw_line",
    "draw_rect_outline",
    "drop_short",
    "join_close",
    "newest_image",
    "repo_root",
    "row_counts",
    "runs",
    "scanline_blobs",
    "stamp",
]
