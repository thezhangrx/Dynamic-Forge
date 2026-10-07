"""任务一：参考物 → 场地坐标系（单应矩阵 + 尺度）。

**这一步解决什么**
------------------
摄像头的画面是**斜视的像素**；而"角色能不能通过这个缺口"必须用**世界长度**来判
（平台硬约束：缺口宽度 ≥ 玩家圆直径）。所以要先建立一个场地坐标系：

    参考物（已知真实尺寸的矩形）  →  单应 H  →  场地坐标（世界单位）
                                              →  尺度（1 世界单位 = 多少米）

之后任何像素点都能映射成场地坐标，长度、距离、缺口宽度全部有了物理含义。

**数学**（为什么是 8 个自由度）
------------------------------
相机看一个**平面**时，投影退化成 3×3 齐次矩阵：

    [X']     [h11 h12 h13] [x]
    [Y'] =   [h21 h22 h23] [y] ,      X = X'/W',  Y = Y'/W'
    [W']     [h31 h32  1 ] [1]

因为世界平面取 Z = 0，原本的 ``H = K·[r₁ r₂ t]`` 只剩两个旋转列加一个平移 ——
**8 个未知量**（整体尺度不可辨，故令 h33 = 1）。每一组对应点给 2 个方程，
所以 **4 组点就能解**；多于 4 组就用最小二乘。

DLT 的行形式（一組点 ``(x, y) → (X, Y)``）：

    [ x  y  1  0  0  0  −Xx  −Xy ] · h = X
    [ 0  0  0  x  y  1  −Yx  −Yy ] · h = Y

2N 个方程、8 个未知量 → 正规方程 ``(AᵀA)·h = Aᵀb`` → 高斯消元。

**为什么先做坐标归一化**（Hartley）：像素值动辄几百，直接解正规方程条件数很差
（``AᵀA`` 里会出现 x² 量级 ~1e5 的元素）。先把两组点各自平移到质心、缩放到
"平均距离 = √2"，解完再反归一化 ``H = T_dst⁻¹ · Ĥ · T_src``，数值稳定性天差地别。

**可迁移性**：`solve_homography` 是**一次性**的 CPU 工作（标定一次存下来），
不必上板；真正上 FPGA 的是 :func:`apply_homography` —— 每个点 8 乘 2 加 1 除，
也就是 6 个乘加 + 一次倒数。这条分界线在文件末尾也有说明。

参考的开源项目
--------------
* ``opencv-python-tutorials/15_image_transformations`` —— ``getPerspectiveTransform`` /
  ``warpPerspective`` 的用法与 4 点定标思路（本文件用纯 Python 重写了它的数学）；
* ``Stereo_Vision_Camera``（``stereo_cam_calibration/``）—— **把靶标固定为世界原点**
  再用 ``solvePnP`` 求相机位姿；它证明了 ``calibrateCamera`` 返回的 rvecs/tvecs
  本来就是"靶标 → 相机"的位姿；
* ``common_interfaces/nav_msgs/MapMetaData.msg`` —— ``origin`` + ``resolution`` 这两个
  字段就是"场地原点 + 每单位多少米"，本模块的字段与它对齐。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

from . import primitives as P

#: 本规范版本（与 core/standard 的字段命名保持一致）。
CALIBRATION_VERSION = 1

Point = tuple[float, float]
Mat3 = tuple[float, float, float, float, float, float, float, float, float]


# ==========================================================================
# 3×3 齐次矩阵的基本运算（纯 Python，全部只用加减乘除）
# ==========================================================================
def apply_homography(h: Sequence[float], x: float, y: float) -> Point:
    """把点 ``(x, y)`` 用单应 ``h`` 映射过去。

    Verilog：8 乘 2 加得 (X', Y', W')，再各乘一次倒数 —— 一次除法即可
    （``1/W'`` 只算一次，两个乘法复用）。
    """
    X = h[0] * x + h[1] * y + h[2]
    Y = h[3] * x + h[4] * y + h[5]
    W = h[6] * x + h[7] * y + h[8]
    if W == 0.0:
        raise ZeroDivisionError("单应映射到无穷远（W=0），该点在相机平面上")
    return X / W, Y / W


def invert_homography(h: Sequence[float]) -> Mat3:
    """3×3 求逆（伴随矩阵 / 行列式）。整数与浮点皆可，只有乘加除。"""
    a, b, c, d, e, f, g, i, j = h
    A = e * j - f * i
    B = f * g - d * j
    C = d * i - e * g
    det = a * A + b * B + c * C
    if det == 0.0:
        raise ValueError("单应矩阵不可逆（退化：参考物四个角共线或重复）")
    return (
        A / det, (c * i - b * j) / det, (b * f - c * e) / det,
        B / det, (a * j - c * g) / det, (c * d - a * f) / det,
        C / det, (b * g - a * i) / det, (a * e - b * d) / det,
    )


def _solve8(a: list[list[float]], b: list[float]) -> list[float]:
    """8×8 线性方程组：带部分主元的高斯消元。一次性工作，不上板。"""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]        # 增广矩阵
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            raise ValueError("标定方程奇异：参考物的点退化（共线或重合）")
        m[col], m[piv] = m[piv], m[col]
        inv = 1.0 / m[col][col]
        for r in range(col + 1, n):
            factor = m[r][col] * inv
            if factor == 0.0:
                continue
            for k in range(col, n + 1):
                m[r][k] -= factor * m[col][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):                          # 回代
        s = m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))
        x[r] = s / m[r][r]
    return x


def _normalize(points: Sequence[Point]) -> tuple[list[Point], Mat3]:
    """Hartley 归一化：平移到质心、缩放到平均距离 √2。返回 (新点, 变换矩阵)。"""
    n = len(points)
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    mean_d = sum(math.hypot(p[0] - cx, p[1] - cy) for p in points) / n
    if mean_d <= 0.0:
        raise ValueError("参考物的点全部重合，无法标定")
    s = math.sqrt(2.0) / mean_d
    t = (s, 0.0, -s * cx, 0.0, s, -s * cy, 0.0, 0.0, 1.0)
    return [(s * (p[0] - cx), s * (p[1] - cy)) for p in points], t


def solve_homography(src: Sequence[Point], dst: Sequence[Point]) -> Mat3:
    """DLT 解单应：``src``（图像点）→ ``dst``（场地坐标点）。需要 ≥4 组不共线点。

    用最小二乘，所以支持多余 4 组点做平均（推荐 6~8 组，标定残差能压下来）。
    """
    if len(src) != len(dst):
        raise ValueError(f"两组点数不一致：{len(src)} vs {len(dst)}")
    if len(src) < 4:
        raise ValueError(f"至少需要 4 组对应点，得到 {len(src)}")
    sn, ts = _normalize(src)
    dn, td = _normalize(dst)

    # 正规方程 (AᵀA) h = Aᵀb，未知量是 h11..h32（h33 固定为 1）
    ata = [[0.0] * 8 for _ in range(8)]
    atb = [0.0] * 8
    for (x, y), (X, Y) in zip(sn, dn):
        r1 = [x, y, 1.0, 0.0, 0.0, 0.0, -X * x, -X * y]
        r2 = [0.0, 0.0, 0.0, x, y, 1.0, -Y * x, -Y * y]
        for row, rhs in ((r1, X), (r2, Y)):
            for i in range(8):
                atb[i] += row[i] * rhs
                for j in range(8):
                    ata[i][j] += row[i] * row[j]
    h8 = _solve8(ata, atb)
    hn: Mat3 = (h8[0], h8[1], h8[2], h8[3], h8[4], h8[5], h8[6], h8[7], 1.0)

    # 反归一化：H = T_dst⁻¹ · Ĥ · T_src
    return _mat_mul(invert_homography(td), _mat_mul(hn, ts))


def _mat_mul(a: Sequence[float], b: Sequence[float]) -> Mat3:
    return tuple(  # type: ignore[return-value]
        sum(a[3 * r + k] * b[3 * k + c] for k in range(3))
        for r in range(3) for c in range(3)
    )


def rms_error(h: Sequence[float], src: Sequence[Point],
              dst: Sequence[Point]) -> float:
    """重投影 RMS（单位与 ``dst`` 相同）。标定质量就看这个数。"""
    if not src:
        return 0.0
    total = 0.0
    for (x, y), (X, Y) in zip(src, dst):
        px, py = apply_homography(h, x, y)
        total += (px - X) ** 2 + (py - Y) ** 2
    return math.sqrt(total / len(src))


# ==========================================================================
# 参考物：一枚已知真实尺寸的矩形
# ==========================================================================
@dataclass(frozen=True)
class Marker:
    """参考物 = 一个已知真实尺寸的矩形（打印的方块 / A4 纸 / 地板贴纸都行）。

    ``size_m`` 是它的**真实宽高（米）**，度量衡就来自这里：标完之后
    "1 个世界单位等于多少米"是确定的。
    """

    name: str = "marker"
    width_m: float = 0.30
    height_m: float = 0.20

    def world_corners(self) -> tuple[Point, Point, Point, Point]:
        """参考物在自己坐标系里的四角（原点在左下，单位=米，逆时针）。

        约定：**场地坐标系就建在这枚参考物上** —— 它的左下角是原点，
        ``+x`` 沿它的宽、``+y`` 沿它的高。这与 ``MapMetaData.origin`` 的语义一致
        （origin = 地图第 0 格在世界里的位姿）。
        """
        return ((0.0, 0.0), (self.width_m, 0.0),
                (self.width_m, self.height_m), (0.0, self.height_m))


def detect_marker_quad(frame: "P.Frame", *, min_area_frac: float = 0.02,
                       bright_min: int = 170, spread: int = 60) -> tuple[Point, ...] | None:
    """在画面里找那枚**最亮的矩形参考物**，返回它的 4 个角点（图像坐标）。

    做法与整个 vision 模块一致，全是能翻 Verilog 的算子：

    1. 逐像素阈值（比较器）→ 掩膜；
    2. 扫描线连通域找**最大**的一块（一行游程缓存 + 累加器）；
    3. 用"对角极值"取四角：``min(x+y) / max(x−y) / max(x+y) / min(x−y)``
       —— 就是 4 个累加器，比轮廓/多边形拟合便宜得多，而且对凸四边形是准确的。

    相机不可用时这个函数不参与测试（用合成图测）。
    """
    w, h = frame.width, frame.height
    mask = bytearray(frame.n)
    lo = bright_min
    for i in range(frame.n):
        b, g, r = frame.b[i], frame.g[i], frame.r[i]
        hi = r if r > g else g
        if b > hi:
            hi = b
        low = r if r < g else g
        if b < low:
            low = b
        if low > lo and (hi - low) < spread:
            mask[i] = 1

    blobs = P.scanline_blobs(mask, w, 0, h - 1, 0, w - 1, min_area=32)
    if not blobs:
        return None
    best = max(blobs, key=lambda b: b.area)
    if best.area < min_area_frac * w * h:
        return None

    # 对角极值 → 四角（凸四边形上这四个点就是角点）
    # 注意四个量都取"最小化"：min(x+y) / min(x−y) / min(−(x+y)) / min(−(x−y))，
    # 所以初值**全部**是 +inf。把 max 那两个的初值写成 −inf 会让分支永远不触发。
    corners: dict[str, tuple[float, float] | None] = {
        "sum_min": None, "diff_min": None, "sum_max": None, "diff_max": None,
    }
    best_v = {"sum_min": math.inf, "diff_min": math.inf,
              "sum_max": math.inf, "diff_max": math.inf}
    for y in range(best.y0, best.y1 + 1):
        row = y * w
        for x in range(best.x0, best.x1 + 1):
            if not mask[row + x]:
                continue
            s, d = x + y, x - y
            for key, val in (("sum_min", s), ("diff_min", d),
                             ("sum_max", -s), ("diff_max", -d)):
                if val < best_v[key]:
                    best_v[key] = val
                    corners[key] = (float(x), float(y))
    if any(v is None for v in corners.values()):
        return None
    # 按"图像里"的左上/右上/右下/左下顺序返回，便于和 Marker 的四角一一对应
    pts = [corners["sum_min"], corners["diff_max"],
           corners["sum_max"], corners["diff_min"]]
    if len({p for p in pts}) < 4:
        return None
    return tuple(pts)  # type: ignore[return-value]


# ==========================================================================
# 标定记录
# ==========================================================================
@dataclass
class FieldCalibration:
    """一次标定的全部产物：图像 ↔ 场地的映射 + 度量衡 + 质量指标。

    存成 JSON（``save`` / ``load``），标一次用很久 —— 这与平台"生成阶段校验一次、
    存 JSON 复用"的做法一致。
    """

    #: 图像 → 场地 的单应（3×3 行主序）。
    H: Mat3
    #: 场地尺寸（世界单位）。0 表示不限制。
    field_width: float = 0.0
    field_height: float = 0.0
    #: **度量衡**：1 个世界单位 = 多少米。标定后必然确定。
    metres_per_unit: float = 1.0
    #: 标定所用的图像尺寸。
    image_width: int = 0
    image_height: int = 0
    #: 参考物信息。
    marker: str = "marker"
    marker_width_m: float = 0.0
    marker_height_m: float = 0.0
    #: 标定残差（像素）。越小越可信。
    rms_px: float = 0.0
    #: 可选内参/畸变（有就先做去畸变再映射；相机不可用时留空）。
    K: tuple[float, ...] | None = None
    D: tuple[float, ...] | None = None
    version: int = CALIBRATION_VERSION
    notes: str = ""

    # -- 映射 ------------------------------------------------------------
    def image_to_field(self, x: float, y: float) -> Point:
        return apply_homography(self.H, x, y)

    def field_to_image(self, X: float, Y: float) -> Point:
        return apply_homography(self.H_inv, X, Y)

    @property
    def H_inv(self) -> Mat3:
        return invert_homography(self.H)

    # -- 度量衡 ----------------------------------------------------------
    def distance_metres(self, p1: Point, p2: Point) -> float:
        """两个**图像点**之间的真实距离（米）。

        注意不能简单地把像素距离乘一个系数：透视下像素↔米的比值随位置变化
        （近处一个像素代表的距离远小于远处）。正确做法是**先把两点映射到场地坐标，
        再量欧氏距离** —— 这也是这一整套标定的意义。
        """
        X1, Y1 = self.image_to_field(*p1)
        X2, Y2 = self.image_to_field(*p2)
        return self.unit_to_metres(math.hypot(X2 - X1, Y2 - Y1))

    def unit_to_metres(self, value: float) -> float:
        return value * self.metres_per_unit

    def metres_to_unit(self, value: float) -> float:
        return value / self.metres_per_unit

    # -- 校验 ------------------------------------------------------------
    def validate(self) -> None:
        if len(self.H) != 9:
            raise ValueError("H 必须是 3×3 行主序的 9 个数")
        if self.metres_per_unit <= 0.0:
            raise ValueError(f"metres_per_unit 必须 > 0，得到 {self.metres_per_unit!r}")

    # -- 序列化 ----------------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "H": list(self.H),
            "field_width": self.field_width,
            "field_height": self.field_height,
            "metres_per_unit": self.metres_per_unit,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "marker": self.marker,
            "marker_width_m": self.marker_width_m,
            "marker_height_m": self.marker_height_m,
            "rms_px": self.rms_px,
            "K": list(self.K) if self.K is not None else None,
            "D": list(self.D) if self.D is not None else None,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "FieldCalibration":
        def tup(key: str) -> tuple[float, ...] | None:
            v = data.get(key)
            return None if v is None else tuple(float(x) for x in v)

        return cls(
            H=tuple(float(x) for x in data["H"]),  # type: ignore[arg-type]
            field_width=float(data.get("field_width", 0.0)),
            field_height=float(data.get("field_height", 0.0)),
            metres_per_unit=float(data.get("metres_per_unit", 1.0)),
            image_width=int(data.get("image_width", 0)),
            image_height=int(data.get("image_height", 0)),
            marker=str(data.get("marker", "marker")),
            marker_width_m=float(data.get("marker_width_m", 0.0)),
            marker_height_m=float(data.get("marker_height_m", 0.0)),
            rms_px=float(data.get("rms_px", 0.0)),
            K=tup("K"), D=tup("D"),
            version=int(data.get("version", CALIBRATION_VERSION)),
            notes=str(data.get("notes", "")),
        )

    def save(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2, ensure_ascii=False),
                     encoding="utf-8")
        return p

    @classmethod
    def load(cls, path: str | Path) -> "FieldCalibration":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def calibrate_from_marker(
    marker_quad_px: Sequence[Point],
    marker: Marker,
    *,
    image_size: tuple[int, int] = (0, 0),
    extra_pairs: Iterable[tuple[Point, Point]] = (),
) -> FieldCalibration:
    """用参考物的 4 个角点做标定。

    ``marker_quad_px`` 是参考物在**图像里**的四角，顺序必须与
    :meth:`Marker.world_corners` 一致（左下 → 右下 → 右上 → 左上）。
    ``extra_pairs`` 可以再补几组已知点（例如场地四角）提高精度。
    """
    if len(marker_quad_px) != 4:
        raise ValueError(f"参考物需要 4 个角点，得到 {len(marker_quad_px)}")
    src = [(float(p[0]), float(p[1])) for p in marker_quad_px]
    dst = list(marker.world_corners())
    for s, d in extra_pairs:
        src.append((float(s[0]), float(s[1])))
        dst.append((float(d[0]), float(d[1])))

    H = solve_homography(src, dst)
    rms = rms_error(H, src, dst)
    return FieldCalibration(
        H=H,
        field_width=marker.width_m,
        field_height=marker.height_m,
        metres_per_unit=1.0,          # 世界单位就是米 —— 尺度由参考物的真实尺寸给出
        image_width=image_size[0],
        image_height=image_size[1],
        marker=marker.name,
        marker_width_m=marker.width_m,
        marker_height_m=marker.height_m,
        rms_px=rms,
        notes="世界单位 = 米；原点在参考物左下角",
    )


__all__ = [
    "CALIBRATION_VERSION",
    "Point", "Mat3",
    "apply_homography", "invert_homography", "solve_homography", "rms_error",
    "Marker", "detect_marker_quad",
    "FieldCalibration", "calibrate_from_marker",
]
