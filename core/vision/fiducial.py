"""标定标记（ArUco）—— 检测在 cv2，标定数学在本项目。

为什么把洋红方块换成它
----------------------
原来那块"占场地 90%x90% 的洋红矩形"有三个死穴，实测都撞过：

1. **靠色度**。判据是 ``R-B``，而过曝会把它从 192 打到 35~80（``docs/vision/adaptive.md`` §7.1）。
   黑白标记靠**灰阶差**，与色相无关 —— 实测黑白差只剩 38 个灰阶、再叠 5px 模糊仍可检出。
2. **太占地方**。它盖住场地 90%x90%，开着就测不了墙和缺口，只能开头露几秒。
   四角小标记占场地面积约 4%，**可以全程开着**，于是每帧都能反解一次单应，
   ``out_of_field`` 才真正成为"标定还成立吗"的哨兵。
3. **只有一个矩形 = 4 个点**。单应是精确解，``rms_px`` **结构性恒为 0**、不含任何质量信息。
   四枚标记 = **16 组对应点** -> 最小二乘 + **真实残差** + 可留出校验。

参考项目
--------
* ``docs/repo/Sign/apriltag``：标记布局的权威定义。``tag36h11`` 是
  ``width_at_border=8`` / ``total_width=10``，即 **6x6 数据 + 1 模块黑边 + 各 1 模块白边**。
  它把家族编码硬编码成 ``uint64_t *codes``，本模块照这个做法把 4 枚标记的位图写死，
  于是**渲染器不需要引入 cv2**。
* ``docs/repo/Sign/apex/opencv_contrib``：**里面已经没有 aruco 了** ——
  OpenCV 5 把 ``aruco`` 并进主仓库，所以本地 ``cv2 5.0.0`` 直接就有
  ``ArucoDetector`` / ``generateImageMarker``，**零新依赖**。
* ``docs/repo/Sign/apriltag_ros``：只是 ROS 封装，对渲染无新增信息。

cv2 的边界
----------
``image_io.py`` 一直自称"vision 模块里唯一依赖第三方库的地方"。本模块是**第二处**，
而且只用在 :func:`detect_markers` 一个函数里：ArUco 检测是一整套算法
（自适应阈值 -> 连通域 -> 四边形拟合 -> 位采样 -> 汉明纠错），我们**有意不重写**它。
其余部分（布局、角点对应、单应、残差）全是纯 Python，可翻 Verilog。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

from .calibration import FieldCalibration, rms_error, solve_homography

# 位图与几何布局属于**共享契约**（平台要画、视觉要认），所以放在
# ``core/standard/fiducial.py``；这里只做"在图像里找它"和"据此标定"。
from standard.fiducial import (          # noqa: E402
    BORDER_MODULES, DATA_MODULES, GRID_MODULES, MARKER_BITS, QUIET_MODULES, SLOTS,
    FiducialLayout, Point, snap_marker_side, slot_of,
)

__all__ = [
    "MARKER_BITS", "DATA_MODULES", "BORDER_MODULES", "GRID_MODULES", "QUIET_MODULES",
    "SLOTS", "FiducialLayout", "snap_marker_side", "slot_of",
    "FiducialError", "FiducialObs", "FiducialReport",
    "detector_params", "detect_markers", "calibrate_from_fiducials", "reprojection_rms",
]


class FiducialError(ValueError):
    """标记相关的输入问题。**不静默兜底。**"""


# ---------------------------------------------------------------------------
# 布局：标记在场地里的已知位置
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class FiducialLayout:
    """四角标记在**场地坐标**里的位置。这就是整套标定的"已知量"。"""

    field_w: float = 640.0
    field_h: float = 480.0
    #: 标记**黑边外沿**的边长（场地单位）。检测到的四边形就是这个尺寸。
    marker_side: float = 48.0
    #: 黑边外沿到场地边的距离（场地单位）。
    margin: float = 8.0

    @property
    def module(self) -> float:
        """一个模块多少场地单位。"""
        return self.marker_side / GRID_MODULES

    @property
    def quiet(self) -> float:
        """留白边宽度（场地单位）。"""
        return self.module * QUIET_MODULES

    def centre(self, slot: int) -> Point:
        """第 ``slot`` 枚标记的中心（场地坐标）。"""
        _, sx, sy = SLOTS[slot]
        x = self.margin + self.marker_side / 2.0 if sx < 0 else \
            self.field_w - self.margin - self.marker_side / 2.0
        y = self.margin + self.marker_side / 2.0 if sy < 0 else \
            self.field_h - self.margin - self.marker_side / 2.0
        return (x, y)

    def corners(self, slot: int) -> tuple[Point, Point, Point, Point]:
        """第 ``slot`` 枚标记的四角（场地坐标）。

        顺序与 ``cv2.aruco.detectMarkers`` 在**正立摆放**时返回的顺序一致：
        **左上 -> 右上 -> 右下 -> 左下**（实测确认，见
        ``test_fiducial_corner_order_matches_opencv``）。
        场地 y 与图像 y 同向（向下为正），所以"上下"就是字面意思。
        """
        cx, cy = self.centre(slot)
        h = self.marker_side / 2.0
        return ((cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h))

# ---------------------------------------------------------------------------
# 检测（唯一用到 cv2 的地方）
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class FiducialObs:
    """一帧里看到的一枚标记。"""

    id: int
    #: 四个角（图像坐标），顺序与 :meth:`FiducialLayout.corners` 一致。
    corners: tuple[Point, Point, Point, Point]

    @property
    def centre(self) -> Point:
        return (sum(p[0] for p in self.corners) / 4.0,
                sum(p[1] for p in self.corners) / 4.0)


def detector_params(*, fast: bool = False):
    """``cv2.aruco.DetectorParameters``，按**拍屏场景实测**调过。

    默认值(``3/23/10`` 的窗口)在"低对比 + 模糊"下会失败；下表是本项目实测
    （1920x1080 合成图，标记一起被压缩，再叠高斯模糊）：

    ==========  ====  ====  ====  ======
    对比度      黑电平  模糊   默认   调参后
    ==========  ====  ====  ====  ======
    0.35        80     5     ✗     ✓
    0.25        90     5     ✗     ✓
    0.25        90     9     ✗     ✓
    0.15        100    5     ✗     ✓
    ==========  ====  ====  ====  ======

    ``fast=True`` 关掉亚像素细化，用于"先粗找一下看有没有标记"。
    """
    import cv2
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin = 5
    p.adaptiveThreshWinSizeMax = 83        # 默认 23：拍屏 + 模糊需要更大的窗
    p.adaptiveThreshWinSizeStep = 6
    p.adaptiveThreshConstant = 5.0
    p.perspectiveRemovePixelPerCell = 6    # 默认 4：模糊时每格多采几个像素
    p.polygonalApproxAccuracyRate = 0.05
    p.minMarkerPerimeterRate = 0.01        # 默认 0.03：小标记要放宽
    p.errorCorrectionRate = 0.8
    if fast:
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    else:
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # 默认是 NONE
        p.cornerRefinementWinSize = 9
        p.cornerRefinementMaxIterations = 50
        p.cornerRefinementMinAccuracy = 0.05
    return p


def _dictionary(ids: Sequence[int] | None = None):
    import cv2
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return d


def detect_markers(frame, *, fast: bool = False,
                   valid_ids: Sequence[int] | None = None) -> list[FiducialObs]:
    """一帧 -> 看到的标记列表。**这是本模块唯一用 cv2 的地方。**

    ``valid_ids`` 只保留布局里真正会用到的 id（默认 :data:`MARKER_BITS` 的键）——
    场上别的 ArUco（比如背景里贴的）不会混进来。

    **返回值不保证有序、也不保证齐全**：调用方必须自己判断"够不够标定"，
    并且**在不够时明确报错**，不能静默退化成"没有标定"。
    """
    import cv2
    import numpy as np
    from .primitives import Frame

    bgr = np.empty((frame.height, frame.width, 3), dtype=np.uint8)
    # Frame 是三个平面；cv2 要 HxWx3
    for ch, plane in enumerate((frame.b, frame.g, frame.r)):
        bgr[:, :, ch] = np.frombuffer(plane, dtype=np.uint8).reshape(
            frame.height, frame.width)
    det = cv2.aruco.ArucoDetector(_dictionary(), detector_params(fast=fast))
    corners, ids, _ = det.detectMarkers(bgr)
    if ids is None or len(ids) == 0:
        return []
    keep = set(valid_ids) if valid_ids is not None else set(MARKER_BITS)
    out: list[FiducialObs] = []
    for quad, mid in zip(corners, ids.ravel()):
        mid = int(mid)
        if mid not in keep:
            continue
        # OpenCV 5 给的是 (N, 1, 4, 2)，旧版是 (N, 4, 2)。用 reshape 一次抹平，
        # 别依赖版本 —— 早先按 (4,2) 直接索引，拿到的是数组而不是标量。
        pts = np.asarray(quad, dtype=float).reshape(-1, 2)
        if pts.shape[0] == 4:
            corners4 = tuple((float(a), float(b)) for a, b in pts)
            out.append(FiducialObs(id=mid, corners=corners4))   # type: ignore[arg-type]
    return out


# ---------------------------------------------------------------------------
# 标定
# ---------------------------------------------------------------------------
@dataclass
class FiducialReport:
    """标定质量。**这次是真的有信息**（16 组点，不是精确解的 4 组）。"""

    n_markers: int = 0
    n_points: int = 0
    rms_px: float = 0.0
    #: 留出校验：用其中 3 枚解、第 4 枚验，返回那个残差（场地单位）。
    #: 单枚标记时无法留出，为 ``None``。
    holdout: float | None = None
    missing: tuple[int, ...] = ()

    def summary(self) -> str:
        s = (f"标定: {self.n_markers}/4 枚标记 {self.n_points} 组点  "
             f"RMS={self.rms_px:.3f} 场地单位")
        if self.holdout is not None:
            s += f"  留出校验={self.holdout:.3f}"
        if self.missing:
            s += f"  **缺 id {list(self.missing)}**"
        return s


def calibrate_from_fiducials(obs: Sequence[FiducialObs],
                             layout: FiducialLayout,
                             image_size: tuple[int, int],
                             *, min_markers: int = 1) -> tuple[FieldCalibration, FiducialReport]:
    """用看到的标记解单应。

    ``min_markers=1`` 是**下限**（1 枚标记 = 4 点，单应恰好定死，没有多余点做校验）；
    能用 4 枚就用 4 枚 —— 16 组点做最小二乘，残差才有意义。

    返回 ``(标定, 质量报告)``；**标记数不够时抛 :class:`FiducialError`**。
    """
    if not obs:
        raise FiducialError("一帧标记都没看到 —— 无法标定")
    seen = {o.id for o in obs}
    missing = tuple(sorted(set(MARKER_BITS) - seen))
    if len(seen) < min_markers:
        raise FiducialError(
            f"只看到 {len(seen)} 枚标记（需要至少 {min_markers}），缺 id {list(missing)}")

    src: list[Point] = []
    dst: list[Point] = []
    for o in sorted(obs, key=lambda o: o.id):
        want = layout.corners(o.id)
        for got, ref in zip(o.corners, want):
            src.append(got)
            dst.append(ref)
    if len(src) < 4:
        raise FiducialError(f"只有 {len(src)} 组点，单应至少需要 4 组")

    H = solve_homography(src, dst)
    rms = rms_error(H, src, dst)

    # 留出校验：拿"最多点的那几枚"解，剩下一枚验（evo 的 n_to_align 那套思路）
    holdout = None
    if len(seen) >= 2:
        hold_id = max(seen)
        s2 = [p for o in sorted(obs, key=lambda o: o.id) if o.id != hold_id
              for p in o.corners]
        d2 = [q for o in sorted(obs, key=lambda o: o.id) if o.id != hold_id
              for q in layout.corners(o.id)]
        if len(s2) >= 4:
            H2 = solve_homography(s2, d2)
            ho = next(o for o in obs if o.id == hold_id)
            holdout = rms_error(H2, list(ho.corners), list(layout.corners(hold_id)))

    area = sum(layout.marker_side ** 2 for _ in seen)
    coverage = area / (layout.field_w * layout.field_h) if layout.field_w > 0 else 0.0
    calib = FieldCalibration(
        H=H,
        field_width=float(layout.field_w), field_height=float(layout.field_h),
        metres_per_unit=1.0,
        image_width=int(image_size[0]), image_height=int(image_size[1]),
        marker="fiducial4",
        marker_width_m=float(layout.marker_side), marker_height_m=float(layout.marker_side),
        rms_px=rms,
        unit="field",
        window=None,
        marker_coverage=coverage,
        notes=(f"4 角 ArUco 标记（DICT_4X4_50, id {sorted(seen)}）；"
               f"场地 {layout.field_w:g}x{layout.field_h:g} 单位"),
    )
    rep = FiducialReport(n_markers=len(seen), n_points=len(src), rms_px=rms,
                         holdout=holdout, missing=missing)
    return calib, rep


def reprojection_rms(calib: FieldCalibration, obs: Sequence[FiducialObs],
                     layout: FiducialLayout) -> float:
    """把**当前这一帧**的标记按已有单应投回场地，算残差。

    用途：标定只在"相机与屏幕相对几何不变"时有效。标记全程可见，
    所以每帧都能算一次这个数 —— 它一跳起来就说明相机被碰了/窗口挪了。
    """
    src: list[Point] = []
    dst: list[Point] = []
    for o in obs:
        for got, ref in zip(o.corners, layout.corners(o.id)):
            src.append(got)
            dst.append(ref)
    if len(src) < 4:
        return float("nan")
    return rms_error(calib.H, src, dst)


__all__ = [
    "MARKER_BITS", "DATA_MODULES", "BORDER_MODULES", "GRID_MODULES", "QUIET_MODULES",
    "SLOTS", "FiducialError", "FiducialLayout", "FiducialObs", "FiducialReport",
    "snap_marker_side", "detector_params", "detect_markers",
    "calibrate_from_fiducials", "reprojection_rms", "marker_bitmap",
]
