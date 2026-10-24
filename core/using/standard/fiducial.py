"""标定标记的**契约**：位图与几何布局。平台画它，视觉识别它。

为什么单独放在 ``core/standard``
--------------------------------
它同时被两边用：

* ``bullet_sim.render.pygame_view`` 把它**画**到窗口四角；
* ``vision.fiducial`` 在图像里**找**它并据此标定。

如果放 ``vision`` 里，平台就得反过来依赖 ``vision``（实测现在两边互不依赖，
这条方向不该破）；放 ``bullet_sim`` 里，视觉又得依赖平台。所以它属于
共享契约层 —— 和 ``obstacle`` / ``player`` / ``vision_output`` 同一个位置。

**本文件是纯 Python，不依赖 numpy / cv2 / pygame**，两边都能直接吃。

标记长什么样
------------
``DICT_4X4_50`` 的一枚标记 = **1 模块黑边 + 4x4 数据**（共 6x6 模块），
外面还要留 **1 模块白边**。这几条都来自 ``docs/repo/Sign/apriltag``：
``tag36h11`` 的 ``width_at_border=8`` / ``total_width=10`` 就是同一套结构，
而 apriltag 也是把家族编码硬编码成 ``uint64_t *codes`` —— 本文件照做。

实测（``docs/vision/fiducial.md``）：
* **纯黑底上不留白边会检不出**（黑边与背景糊在一起）；留白边或背景不是纯黑才行。
* 四个角的标记占场地面积约 4%，所以**可以全程开着**，不必像原来那块洋红矩形
  那样"开头露几秒就得关掉"（它盖住 90% 场地）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

Point = tuple[float, float]

#: ``DICT_4X4_50`` 四个标记的数据位（4x4，行主序，1=白 0=黑）。
#: 生成方式：``cv2.aruco.generateImageMarker(id, 120, borderBits=1)`` 按模块中心采样。
#: 选 ``4x4_50`` 而不是 ``APRILTAG_36h11``（6x6 数据）：同渲染尺寸下**每模块大 33%**，
#: 更抗模糊；只用 4 个 id，不需要长编码。
MARKER_BITS: dict[int, tuple[int, ...]] = {
    0: (1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0),
    1: (0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0),
    2: (0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1),
    3: (1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0),
}
#: 数据区模块数（"4x4" 的那个 4）。
DATA_MODULES = 4
#: 黑边宽度（模块）。1 与 ``DICT_4X4_50`` 一致。
BORDER_MODULES = 1
#: 含黑边的总模块数。**检测到的四边形就是这一圈的外沿**。
GRID_MODULES = DATA_MODULES + 2 * BORDER_MODULES
#: 留白边宽度（模块）。apriltag 的 ``total_width - width_at_border = 2`` 就是两侧各 1。
QUIET_MODULES = 1

#: 标记里"亮"那一半的灰度值（0~255）。**不是纯白 255，这是有意的。**
#:
#: 视觉链把"亮块"定义为 ``min(R,G,B) > 195``（``vision.primitives`` 的
#: ``bright_min``），并用它找**角色**（白圆）。四个标记的留白边 + 数据模块
#: 如果画成纯白，画面四角就会出现四块比角色还大的亮块 —— 实测在 1280x960
#: 渲染下，右上标记留白边的碎块得分 768，而**真正的角色只有 631**：
#: 于是"角色"被钉死在标记上、逐帧一动不动，对账报出 360 场地单位的位置误差
#: （而且看起来像"识别算法不行"，很难反推到"标记把角色挤掉了"）。
#:
#: 取 128 的依据，两头都要顾：
#:
#: * **上界**：不能进亮块判据。128 距离 195 有 1.5 倍余量，也就是相机曝光
#:   涨到 1.5 倍之前都还安全（实测场地底 34 被过曝到 86，约 2.2 倍 ——
#:   那种程度下**任何**绝对阈值都会被冲垮，该换自适应分类器，
#:   见 ``docs/vision/adaptive.md``，不是靠标记调色能救的）。
#: * **下界**：要和黑边 (0) 有足够对比度给 ArUco 的**自适应**阈值分割。
#:   128 对 0 的对比度约 0.5，远高于实测可用的下限 0.15
#:   （``test_detects_under_screen_photo_degradation``）。
#:
#: 颜色契约的同类先例见 ``pygame_view.WALL_COLOR``：渲染出来的颜色不是审美，
#: 是**和视觉链的接口**，改之前要先看判据。
FIDUCIAL_LEVEL = 128

#: 槽位顺序：0=左上 1=右上 2=右下 3=左下，与 id 一一对应。**按"屏幕上的方位"命名。**
#:
#: **场地 y 向上为正**（``Viewport.world_to_screen`` 把 y 翻了：
#: ``sy = h/2 - (y-cy)*scale``）。所以"屏幕上的左上"对应**较大的**场地 y。
#: 这条踩过一次：按"y 向下"画出来的标记是**上下镜像**的，
#: 镜像后的 ArUco 不是合法编码，相机根本解不出来。
#: 每个槽位是 ``(名字, x 方向, y 方向)``，方向 +1 表示靠场地大坐标一侧。
SLOTS: tuple[tuple[str, int, int], ...] = (
    ("左上", -1, +1), ("右上", +1, +1), ("右下", +1, -1), ("左下", -1, -1),
)


@dataclass(frozen=True)
class FiducialLayout:
    """四角标记在**场地坐标**里的位置 —— 这就是标定全部已知量。"""

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
        x = (self.margin + self.marker_side / 2.0
             if sx < 0 else self.field_w - self.margin - self.marker_side / 2.0)
        y = (self.margin + self.marker_side / 2.0
             if sy < 0 else self.field_h - self.margin - self.marker_side / 2.0)
        return (x, y)

    def corners(self, slot: int) -> tuple[Point, Point, Point, Point]:
        """第 ``slot`` 枚标记的四角（场地坐标）。

        顺序与 ``cv2.aruco.detectMarkers`` 对一枚**在画面上正立**的标记
        返回的顺序一致：**画面左上 -> 右上 -> 右下 -> 左下**。

        注意场地 y 向上为正，所以"画面左上"= **场地 y 较大**那一侧：
        ``(cx-h, cy+h) -> (cx+h, cy+h) -> (cx+h, cy-h) -> (cx-h, cy-h)``。
        这条由测试 ``test_corner_order_matches_opencv`` 现场验证（真跑一次检测），
        不是靠推理 —— 弄反了单应会整体转 90°。
        """
        cx, cy = self.centre(slot)
        h = self.marker_side / 2.0
        return ((cx - h, cy + h), (cx + h, cy + h), (cx + h, cy - h), (cx - h, cy - h))

    def rects(self, slot: int) -> list[tuple[float, float, float, float, bool]]:
        """第 ``slot`` 枚标记拆成"要画的方块"：``[(x, y, side, 是白)]``（场地单位）。

        **按模块对齐**、并且先铺满整块黑（含黑边）再盖白块 —— 这样模块边界不会
        因为浮点舍入出现半个像素的缝，相机一模糊就连成一片。调用方应先用
        :func:`snap_marker_side` 把边长取成"模块正好整数像素"。
        """
        bits = MARKER_BITS[slot]
        cx, cy = self.centre(slot)
        x0 = cx - self.marker_side / 2.0
        # **bitmap 的第 0 行画在场地 y 最大处**（= 画面顶部），否则标记在画面里
        # 是上下镜像的，而镜像后的 ArUco 不是合法编码、相机解不出来。
        y_top = cy + self.marker_side / 2.0
        m = self.module
        out: list[tuple[float, float, float, float, bool]] = [
            # 白留白（在标记外面一圈）
            (x0 - self.quiet, y_top - self.marker_side - self.quiet,
             self.marker_side + 2.0 * self.quiet, True),
            # 整块黑（含黑边）
            (x0, y_top - self.marker_side, self.marker_side, False),
        ]
        for r in range(DATA_MODULES):
            for c in range(DATA_MODULES):
                if bits[r * DATA_MODULES + c]:
                    out.append((x0 + (c + BORDER_MODULES) * m,
                                y_top - (r + BORDER_MODULES + 1) * m, m, True))
        return out

    def bitmap(self, slot: int) -> list[list[int]]:
        """完整位图（含黑边），``GRID_MODULES x GRID_MODULES`` 的 0/1 表。"""
        if slot not in MARKER_BITS:
            raise ValueError(f"没有 id={slot} 的标记位图")
        g = [[0] * GRID_MODULES for _ in range(GRID_MODULES)]
        bits = MARKER_BITS[slot]
        for r in range(DATA_MODULES):
            for c in range(DATA_MODULES):
                g[r + BORDER_MODULES][c + BORDER_MODULES] = bits[r * DATA_MODULES + c]
        return g

    def to_dict(self) -> dict:
        return {"field_w": self.field_w, "field_h": self.field_h,
                "marker_side": self.marker_side, "margin": self.margin,
                "grid_modules": GRID_MODULES, "quiet_modules": QUIET_MODULES,
                "ids": sorted(MARKER_BITS)}


def snap_marker_side(layout: FiducialLayout, scale: float, *,
                     min_module_px: float = 6.0) -> FiducialLayout:
    """把边长调成"一个模块正好整数像素"，并保证模块不小于 ``min_module_px``。

    ``scale`` 是渲染时的 ``像素 / 场地单位``。不做这一步，模块边界落在非整数像素上，
    pygame 画出的白块之间会出现半个像素的缝，相机一模糊就连片，检测直接废。
    """
    if scale <= 0:
        return layout
    module_px = max(float(min_module_px), float(round(layout.module * scale)))
    return FiducialLayout(field_w=layout.field_w, field_h=layout.field_h,
                          marker_side=module_px * GRID_MODULES / scale,
                          margin=layout.margin)


def slot_of(marker_id: int) -> int:
    """标记 id 就是槽位号（0..3）。留个函数是为了让"id 与槽位一一对应"这条
    契约只有一个出处。"""
    if marker_id not in MARKER_BITS:
        raise ValueError(f"未知标记 id={marker_id}")
    return marker_id


__all__ = [
    "Point", "MARKER_BITS", "DATA_MODULES", "BORDER_MODULES", "GRID_MODULES",
    "QUIET_MODULES", "FIDUCIAL_LEVEL", "SLOTS", "FiducialLayout",
    "snap_marker_side", "slot_of",
]
