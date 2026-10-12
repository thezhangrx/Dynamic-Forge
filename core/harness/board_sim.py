"""板子全流程仿真 —— 在本机把**板上那条链**整条跑起来，并在屏幕上可视化。

板上是这条链
------------

    摄像头 ──► PL(视觉) ──► 几何帧(AXI) ──► PS(单应 + 决策) ──► 动作
     ①          ②              ③                  ④⑤              ⑥

本机拿什么顶替每一段（**顶替物都标出来，不许含糊**）
----------------------------------------------------

| 段 | 板上 | 本机 | 保真度 |
|---|---|---|---|
| ① | USB 摄像头 | 录像文件逐帧读（cv2） | 像素数据真实，但**不是实时** |
| ② | `vision_axi`（RTL） | `core/fpga/ref/vision_ref.c`（走 ctypes） | **与 RTL 逐位一致**（620 帧 4.3 亿次比对零差异） |
| ③ | AXI → PS 内存 | 同一份 v3 线协议字节（含 CRC32） | **完全一样** |
| ④ | PS 上跑单应 | `core/vision/localize.to_world` | 同一份代码 |
| ⑤ | PS 上跑决策 | `cpu.adapters.vision_adapter` → `gap_avoid.plan()` | 同一份代码 |
| ⑥ | 电机 | 屏幕可视化 + 打印 | 仿真 |

**为什么 ② 用 C 模型而不是 Verilator**：两者逐位一致，但 C 模型能直接吃一帧吐一帧、
走 ctypes 零进程开销；Verilator 那边要先给 `tb_axi` 加"吐帧到文件"。等这条链跑通了，
把 :class:`RefPl` 换成 Verilator 后端即可 —— 接口就是 :meth:`PlBackend.process`。

**纪律：决策层只能看见视觉。** 录像里的真值一概不进决策，本模块也不持有真值。
这条和 `core/harness/loop.py` 同一条纪律。

用法见根脚本 ``board_sim_run.py``。
"""

from __future__ import annotations

import ctypes
import json
import pathlib
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Iterator, Optional, Sequence

# ---------------------------------------------------------------------------
# 路径：core/ 下各模块用绝对导入，所以先把 core/ 放进 sys.path
# ---------------------------------------------------------------------------
_CORE = pathlib.Path(__file__).resolve().parents[1]
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from cpu.adapters.vision_adapter import (            # noqa: E402
    VisionWorldConfig,
    VisionWorldStream,
)
from cpu.gap_avoid import AlgoConfig                  # noqa: E402
from standard.vision.vision_frame import decode_vision_frame   # noqa: E402
from vision.calibration import FieldCalibration       # noqa: E402
from vision.localize import to_world                  # noqa: E402

_ROOT = _CORE.parent
REF_C = _ROOT / "core" / "fpga" / "ref" / "vision_ref.c"
BUILD_DIR = _ROOT / "data" / "fpga" / "board_sim"

#: 与 ``core/fpga/ref/ref_api.h`` 的 ``VR_API_*`` 必须逐个一致。
#: 不一致会让 ctypes 读越界（有测试守着 C 侧，这里靠常量对齐）。
MAX_W, MAX_H = 640, 480
MAX_OBS, MAX_GAPS, MAX_BANDS = 32, 16, 64


# ===========================================================================
# 1. PL 后端：C 参考模型，走 ctypes（不是每帧开进程）
# ===========================================================================
class VrRefParams(ctypes.Structure):
    """镜像 ``ref_api.h`` 的 ``VrRefParams``（21 个 int，顺序必须一致）。"""

    _fields_ = [(n, ctypes.c_int) for n in (
        "wall_rb", "wall_min_r", "wall_min_g",
        "bright_min", "bright_spread", "green_gm", "green_min",
        "band_row_pct", "band_join_rows", "band_min_rows", "field_col_min",
        "wall_col_pct", "wall_col_min", "edge_min", "edge_min_cols",
        "min_seg_px", "min_gap_px",
        "player_min_area", "player_max_area",
        "player_min_fill_pct", "player_max_aspect_pct",
    )]


class VrRefResult(ctypes.Structure):
    """镜像 ``ref_api.h`` 的 ``VrRefResult``。"""

    _fields_ = [
        ("width", ctypes.c_int), ("height", ctypes.c_int),
        ("n_bands", ctypes.c_int),
        ("band_y0", ctypes.c_int * MAX_BANDS),
        ("band_y1", ctypes.c_int * MAX_BANDS),
        ("x_min", ctypes.c_int), ("x_max", ctypes.c_int),
        ("n_obstacles", ctypes.c_int), ("n_gaps", ctypes.c_int),
        ("obstacles", (ctypes.c_int32 * 9) * MAX_OBS),
        ("gaps", (ctypes.c_int32 * 9) * MAX_GAPS),
        ("player_valid", ctypes.c_int),
        ("player", ctypes.c_int32 * 4),
        ("blob_x0", ctypes.c_int32), ("blob_x1", ctypes.c_int32),
        ("blob_y0", ctypes.c_int32), ("blob_y1", ctypes.c_int32),
        ("blob_area", ctypes.c_int32),
        ("n_labels", ctypes.c_int),
        ("frame_bytes", ctypes.c_size_t),
    ]


def compile_ref(force: bool = False) -> pathlib.Path:
    """把 ``vision_ref.c`` 编成共享库（带缓存）。

    为什么要 .so 而不是每帧跑一次可执行文件：一帧一个进程的 fork/exec 开销
    比视觉本身还大，可视化里根本跟不上。走 ctypes 就是一帧一次函数调用。
    """
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    so = BUILD_DIR / "libvision_ref.so"
    if so.exists() and not force and so.stat().st_mtime > REF_C.stat().st_mtime:
        return so
    cmd = ["gcc", "-O2", "-fPIC", "-shared", "-DVR_NO_MAIN",
           "-o", str(so), str(REF_C), "-lm"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"编译 C 参考模型失败：\n{r.stdout}\n{r.stderr}")
    return so


class RefPl:
    """PL 后端：C 参考模型。**与 RTL 逐位一致**，是本机跑板子链路的 PL 顶替物。"""

    name = "C 参考模型 vision_ref.c（与 RTL 逐位一致）"

    def __init__(self, so: Optional[pathlib.Path] = None):
        so = so or compile_ref()
        self.lib = ctypes.CDLL(str(so))
        self.lib.vr_ref_process_ex.restype = ctypes.c_int
        self.lib.vr_ref_process_ex.argtypes = [
            ctypes.POINTER(ctypes.c_uint8), ctypes.c_int, ctypes.c_int, ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint16),
            ctypes.POINTER(VrRefResult), ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t,
            ctypes.POINTER(VrRefParams), ctypes.c_uint32, ctypes.c_uint64,
        ]
        self.lib.vr_ref_default_params.argtypes = [ctypes.POINTER(VrRefParams)]
        self.params = VrRefParams()
        self.lib.vr_ref_default_params(ctypes.byref(self.params))
        # 中间量缓冲：板上这些在片内是掩膜帧存，本机只是为了让 API 满足而已
        self._gray = (ctypes.c_uint8 * (MAX_W * MAX_H))()
        self._wall = (ctypes.c_uint8 * (MAX_W * MAX_H))()
        self._bright = (ctypes.c_uint8 * (MAX_W * MAX_H))()
        self._rowc = (ctypes.c_uint16 * MAX_H)()
        self._out = (ctypes.c_uint8 * 8192)()
        self._res = VrRefResult()

    def process(self, rgb: bytes, w: int, h: int, *, seq: int = 0,
                stamp_ns: int = 0) -> tuple[bytes, VrRefResult]:
        """一帧 RGB → ``(几何帧字节, 结构化中间量)``。

        ``seq``/``stamp_ns`` 会写进帧头 —— 板上是 PS 通过 AXI 写 0x0C4/0x0C8，
        这里走同一个字段，所以帧字节与板上同构（决策侧要靠它算 ``dt``）。
        """
        buf = (ctypes.c_uint8 * len(rgb)).from_buffer_copy(rgb)
        rc = self.lib.vr_ref_process_ex(
            buf, w, h, 1,
            self._gray, self._wall, self._bright, self._rowc,
            ctypes.byref(self._res), self._out, ctypes.c_size_t(len(self._out)),
            ctypes.byref(self.params), ctypes.c_uint32(seq), ctypes.c_uint64(stamp_ns),
        )
        if rc != 0:
            raise RuntimeError(f"vr_ref_process_ex 返回 {rc}（内存不足？）")
        n = int(self._res.frame_bytes)
        return bytes(self._out[:n]), self._res


# ===========================================================================
# 2. 取图：录像文件（顶替摄像头）
# ===========================================================================
@dataclass
class VideoSource:
    """录像文件逐帧读。**这是摄像头的顶替物，不是摄像头。**"""

    path: pathlib.Path
    width: int = 0            # 0 = 不缩放
    stride: int = 1           # 每几帧取一帧
    start: int = 0
    limit: int = 0            # 0 = 全片

    def __post_init__(self) -> None:
        import cv2  # noqa: F401  （延迟导入：没装 cv2 时不该拖垮整个模块）
        self._cv2 = cv2
        self.cap = cv2.VideoCapture(str(self.path))
        if not self.cap.isOpened():
            raise SystemExit(f"打不开录像 {self.path}")
        if self.start:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.start)
        self.fps = float(self.cap.get(cv2.CAP_PROP_FPS) or 30.0)

    def __iter__(self) -> Iterator[tuple[bytes, int, int, int]]:
        """产出 ``(rgb 字节, w, h, 原始帧号)``。"""
        cv2 = self._cv2
        got, src = 0, self.start
        while True:
            ok, frame = self.cap.read()
            if not ok:
                break
            if src % self.stride:
                src += 1
                continue
            if self.width and frame.shape[1] != self.width:
                scale = self.width / frame.shape[1]
                frame = cv2.resize(frame, (self.width, int(round(frame.shape[0] * scale))),
                                   interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            yield rgb.tobytes(), w, h, src
            got += 1
            src += 1
            if self.limit and got >= self.limit:
                break

    def close(self) -> None:
        self.cap.release()


# ===========================================================================
# 3. 板子全流程
# ===========================================================================
@dataclass
class BoardFrame:
    """一帧走过全链路之后的**全部**中间产物 —— 可视化就是把它画出来。"""

    index: int = 0
    rgb: bytes = b""
    w: int = 0
    h: int = 0
    frame_bytes: bytes = b""          # ③ PL 吐出的几何帧（像素坐标）
    vision: Any = None                # 解码后（像素坐标）
    world_frame: Any = None           # ④ 单应之后（世界坐标）
    decision: Any = None              # ⑤ 决策
    view: Any = None
    meta: Any = None
    t_pl_ms: float = 0.0
    t_total_ms: float = 0.0


class BoardSim:
    """摄像头 → PL → 几何帧 → PS(单应+决策) 的整条链。

    换 PL 后端（C 模型 → Verilator RTL）只需要换 ``pl=``，其余不动。
    """

    def __init__(self, *, pl: Any = None, calib: FieldCalibration,
                 cfg: Optional[AlgoConfig] = None,
                 field_w: float = 640.0, field_h: float = 480.0,
                 require_calibration: bool = True):
        self.pl = pl or RefPl()
        self.calib = calib
        self.cfg = cfg or AlgoConfig()
        self.stream = VisionWorldStream(
            VisionWorldConfig(field_w=field_w, field_h=field_h,
                              require_calibration=require_calibration))

    def step(self, rgb: bytes, w: int, h: int, *, index: int,
             stamp_ns: int = 0) -> BoardFrame:
        bf = BoardFrame(index=index, rgb=rgb, w=w, h=h)
        t0 = time.perf_counter()

        # ② PL：像素 → 几何帧。板上这一段的输入来自相机，输出去 AXI。
        frame_bytes, _res = self.pl.process(rgb, w, h, seq=index, stamp_ns=stamp_ns)
        bf.frame_bytes = frame_bytes
        t1 = time.perf_counter()

        # ③ PS 收帧：v3 线协议 → VisionFrame。坐标**还是像素**（PL 不做单应）。
        bf.vision = decode_vision_frame(frame_bytes)

        # ④ PS 做单应：像素 → 世界单位。单应错了不会报错、只会让角色乱跑，
        #    所以这一步的产物要显式检查（见 localize.to_world 与 out_of_field）。
        bf.world_frame = to_world(bf.vision, self.calib)

        # ⑤ 适配 → 决策。**注意喂进去的是视觉产出的帧，不是真值。**
        view, dec = self.stream.step(bf.world_frame, cfg=self.cfg)
        bf.view, bf.decision = view, dec

        t2 = time.perf_counter()
        bf.t_pl_ms = (t1 - t0) * 1e3
        bf.t_total_ms = (t2 - t0) * 1e3
        return bf


# ===========================================================================
# 4. 可视化（pygame）—— 左边"相机看到什么"，右边"决策在想什么"
# ===========================================================================
#: 配色沿用平台渲染器的接口约定（core/bullet_sim/render），别自己发明。
_BG = (12, 14, 22)
_FG = (232, 236, 244)
_DIM = (96, 104, 124)
_WALL = (240, 160, 48)          # 与平台一致：墙=橙
_TARGET = (90, 200, 140)        # 目标=绿
_PLAYER = (235, 240, 250)       # 角色=近白
_GAP = (120, 205, 255)
_AIM = (255, 130, 200)
_PANEL = (24, 28, 40)


def _load_font(size: int):
    """找一个**带中文字形**的字体；找不到就退回默认（英文仍可读）。"""
    import pygame
    for p in ("/usr/share/fonts/NotoSansCJK-Regular.otf",
              "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
        if pathlib.Path(p).exists():
            try:
                return pygame.font.Font(p, size)
            except Exception:            # noqa: BLE001
                pass
    return pygame.font.Font(None, size)


class BoardView:
    """两栏窗口：左=相机帧+视觉检测，右=决策场视图。"""

    def __init__(self, *, panel: int = 520, title: str = "board sim"):
        import pygame
        self.pygame = pygame
        pygame.init()
        self.panel = panel
        self.screen = pygame.display.set_mode((panel * 2, panel))
        pygame.display.set_caption(title)
        self.font = _load_font(17)
        self.small = _load_font(14)
        self._cam_rect: Optional[tuple[int, int, int, int]] = None

    # -- 左栏：相机 + 视觉检测（**像素坐标**）-----------------------------
    def _draw_camera(self, bf: BoardFrame) -> None:
        pg = self.pygame
        n = self.panel
        if not bf.rgb:
            return
        surf = pg.image.frombuffer(bf.rgb, (bf.w, bf.h), "RGB")
        scale = min(n / bf.w, n / bf.h)
        dw, dh = max(1, int(bf.w * scale)), max(1, int(bf.h * scale))
        img = pg.transform.smoothscale(surf, (dw, dh))
        ox, oy = (n - dw) // 2, (n - dh) // 2
        self.screen.blit(img, (ox, oy))
        self._cam_rect = (ox, oy, dw, dh)

        def px(x: float, y: float) -> tuple[int, int]:
            return int(ox + x * scale), int(oy + y * scale)

        vis = bf.vision
        # 障碍：像素坐标下的矩形（C 模型给的是中心 + 半宽/半高 + 旋转）
        for o in (getattr(vis, "obstacles", None) or []):
            cx, cy = float(getattr(o, "x", 0.0)), float(getattr(o, "y", 0.0))
            hw = float(getattr(o, "half_w", 0.0))
            hh = float(getattr(o, "half_h", 0.0))
            if hw <= 0 and hh <= 0:
                continue
            if hw <= 0:
                hw = hh
            if hh <= 0:
                hh = hw
            import math
            th = math.radians(float(getattr(o, "rotation", 0.0) or 0.0))
            cs, sn = math.cos(th), math.sin(th)
            pts = []
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                dx, dy = sx * hw, sy * hh
                pts.append(px(cx + dx * cs - dy * sn, cy + dx * sn + dy * cs))
            pg.draw.polygon(self.screen, _WALL, pts, 2)
        # 角色
        pl = getattr(vis, "player", None)
        if pl is not None:
            c = px(float(getattr(pl, "x", 0.0)), float(getattr(pl, "y", 0.0)))
            r = max(3, int(float(getattr(pl, "radius", 3.0)) * scale))
            pg.draw.circle(self.screen, _PLAYER, c, r, 2)
        # 缺口
        for g in (getattr(vis, "gaps", None) or []):
            c = px(float(getattr(g, "cx", 0.0)), float(getattr(g, "cy", 0.0)))
            pg.draw.circle(self.screen, _GAP, c, 5, 1)

        self._label(6, 6, f"① 相机（像素 {bf.w}×{bf.h}）  "
                          f"②PL {bf.t_pl_ms:.1f}ms  ③帧 {len(bf.frame_bytes)}B")

    # -- 右栏：决策场视图（**世界坐标**）----------------------------------
    def _draw_field(self, bf: BoardFrame) -> None:
        pg = self.pygame
        n = self.panel
        ox = n
        pg.draw.rect(self.screen, _PANEL, (ox, 0, n, n))
        vf = bf.world_frame
        fw = float(getattr(self, "field_w", 640.0))
        fh = float(getattr(self, "field_h", 480.0))
        scale = min((n - 20) / fw, (n - 20) / fh)
        def w2s(x: float, y: float) -> tuple[int, int]:
            return int(ox + 10 + x * scale), int(n - 10 - y * scale)   # y 轴向上

        # 场地边框（没有它就看不出哪些东西跑到场外去了）
        a, b = w2s(0.0, fh), w2s(fw, 0.0)
        pg.draw.rect(self.screen, _DIM,
                     pg.Rect(a[0], a[1], max(1, b[0] - a[0]), max(1, b[1] - a[1])), 1)

        v = bf.view
        # 障碍
        for r in (getattr(v, "rects", None) or []):
            cx, cy = float(r.x), float(r.y)
            hw, hh = float(getattr(r, "half_w", 0.0)), float(getattr(r, "half_h", 0.0))
            if hw <= 0:
                hw = hh
            if hh <= 0:
                hh = hw
            a, b = w2s(cx - hw, cy + hh), w2s(cx + hw, cy - hh)
            pg.draw.rect(self.screen, _WALL,
                         pg.Rect(a[0], a[1], max(1, b[0] - a[0]), max(1, b[1] - a[1])), 2)
        # 角色
        pc = None
        pl = getattr(v, "player", None)
        if pl is not None and len(pl) >= 2:
            pc = w2s(float(pl[0]), float(pl[1]))
            rr = max(3, int(float(pl[4]) * scale)) if len(pl) > 4 else 6
            pg.draw.circle(self.screen, _PLAYER, pc, rr, 2)

        d = bf.decision
        if d is not None:
            # 缺口线段
            gl = getattr(d, "gap_line", None)
            if gl:
                p0, p1 = gl
                pg.draw.line(self.screen, _GAP, w2s(float(p0[0]), float(p0[1])),
                             w2s(float(p1[0]), float(p1[1])), 2)
            # **速度指令**：dec.wx/wy 是归一化速度（模长<=1），不是瞄准点 ——
            # 早先这里当成"瞄准点"画了个叉，结果它落在场地外，看着像 bug。
            wx, wy = float(getattr(d, "wx", 0.0)), float(getattr(d, "wy", 0.0))
            if pc is not None and (abs(wx) > 1e-9 or abs(wy) > 1e-9):
                L = 60.0                                   # 箭头固定视觉长度
                tip = (int(pc[0] + wx * L), int(pc[1] - wy * L))   # 屏幕 y 向下
                pg.draw.line(self.screen, _AIM, pc, tip, 3)
                pg.draw.circle(self.screen, _AIM, tip, 4)
            tier = getattr(d, "tier", "?")
            self._label(ox + 6, 6,
                        f"④⑤ 决策（世界单位 {fw:.0f}×{fh:.0f}）  档位={tier}  "
                        f"总 {bf.t_total_ms:.1f}ms")
            self._label(ox + 6, n - 22,
                        f"速度指令=({wx:+.2f},{wy:+.2f})  "
                        f"缺口宽={getattr(d, 'gap', float('nan')):.1f}  "
                        f"余隙={getattr(d, 'clearance', float('inf')):.1f}  "
                        f"障碍={getattr(d, 'n_rects', 0)}")

    def _label(self, x: int, y: int, text: str) -> None:
        self.screen.blit(self.small.render(text, True, _FG), (x, y))

    def draw(self, bf: BoardFrame) -> None:
        self.screen.fill(_BG)
        self._draw_camera(bf)
        self._draw_field(bf)
        self.pygame.display.flip()

    def snapshot(self, path: Any) -> None:
        """把当前画面存成 PNG。

        为什么需要：可视化对不对**不能靠"窗口开起来了"来判断** —— 渲染成一片
        黑、或者坐标轴搞反了，窗口照样开得起来。存图才能真的看一眼。
        """
        path = pathlib.Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.pygame.image.save(self.screen, str(path))

    def poll(self) -> bool:
        """返回 True 表示用户要求退出。"""
        for e in self.pygame.event.get():
            if e.type == self.pygame.QUIT:
                return True
            if e.type == self.pygame.KEYDOWN and e.key in (
                    self.pygame.K_ESCAPE, self.pygame.K_q):
                return True
        return False

    def close(self) -> None:
        self.pygame.quit()


# ===========================================================================
# 5. 顶层：跑一条录像，逐帧显示
# ===========================================================================
@dataclass
class RunStats:
    frames: int = 0
    decided: int = 0
    rejected: int = 0
    tiers: dict = field(default_factory=dict)
    pl_ms: list = field(default_factory=list)
    total_ms: list = field(default_factory=list)
    reasons: list = field(default_factory=list)

    def summary(self) -> dict:
        def med(v):
            if not v:
                return 0.0
            s = sorted(v)
            return s[len(s) // 2]
        return {
            "frames": self.frames,
            "decided": self.decided,
            "rejected": self.rejected,
            "tiers": dict(self.tiers),
            "pl_ms_median": round(med(self.pl_ms), 3),
            "total_ms_median": round(med(self.total_ms), 3),
            "reject_reasons": self.reasons[:5],
        }


def run(source: Any, sim: BoardSim, *, view: Optional[BoardView] = None,
        fps: float = 0.0, verbose: bool = False, quiet: bool = False,
        save_png: Optional[pathlib.Path] = None, save_at: int = 0) -> RunStats:
    """把一路帧源跑完整条链。``fps>0`` 时限速（可视化看得清）。

    ``save_png``：在第 ``save_at`` 帧存一张 PNG（用来**验证渲染本身**，
    而不是只证明"窗口能开"）。
    """
    st = RunStats()
    per_frame = 1.0 / fps if fps > 0 else 0.0
    try:
        for rgb, w, h, src_idx in source:
            t_next = time.perf_counter() + per_frame
            stamp_ns = int(src_idx * 1e9 / max(1e-6, getattr(source, "fps", 30.0)))
            bf = sim.step(rgb, w, h, index=src_idx, stamp_ns=stamp_ns)
            st.frames += 1
            st.pl_ms.append(bf.t_pl_ms)
            st.total_ms.append(bf.t_total_ms)
            if bf.decision is not None:
                st.decided += 1
                t = getattr(bf.decision, "tier", "?")
                st.tiers[t] = st.tiers.get(t, 0) + 1
            if view is not None:
                view.draw(bf)
                if save_png is not None and st.frames - 1 == save_at:
                    view.snapshot(save_png)
                    if not quiet:
                        print(f"  已存图（第 {st.frames} 帧）：{save_png}")
                if view.poll():
                    break
            if verbose and st.frames % 20 == 0:
                print(f"  帧 {st.frames}: PL {bf.t_pl_ms:.1f}ms "
                      f"总 {bf.t_total_ms:.1f}ms tier={getattr(bf.decision, 'tier', '?')}")
            if per_frame:
                dt = t_next - time.perf_counter()
                if dt > 0:
                    time.sleep(dt)
    except KeyboardInterrupt:
        pass
    except Exception as exc:                       # noqa: BLE001
        st.rejected += 1
        st.reasons.append(f"{type(exc).__name__}: {exc}")
        if not quiet:
            print(f"  第 {st.frames} 帧被拒：{type(exc).__name__}: {exc}", file=sys.stderr)
    return st


def load_calibration(path: pathlib.Path) -> FieldCalibration:
    data = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    return FieldCalibration.from_dict(data)


def calibrate_from_video(path: pathlib.Path, *, process_width: int = 640,
                         field_w: float = 640.0, field_h: float = 480.0,
                         scan_frames: int = 20) -> tuple[FieldCalibration, Any, int]:
    """用录像里**四角 ArUco 标记**现算单应 —— 和板上真机标定同一条路。

    为什么非要有这个：``data/vision/calibration.json`` 的 ``image_width`` 是 0
    （没记），而实测它对不上这段录像 —— 它把场四角映射到像素 x[199,531]
    y[62,268]，而录像（缩到 640 宽后）里四角实际在 x[104,536] y[24,336]。
    **拿错单应不会报错**，只会让墙和角色被搬到场地外面去（可视化里一眼就看得见）。
    四角标记全程可见，所以直接现标最可靠。

    ⚠ **必须用 ``standard.fiducial.FiducialLayout``**：``vision/fiducial.py:65``
    里有一个**同名的重复定义**，它的 ``corners()`` 角点顺序是反的
    （16 点重投影 RMS 47.9 px，正确的那份是 0.31 px）。这是踩过的坑，
    见 ``坐标系约定.md`` §9。

    返回 ``(标定, 质量报告, 用的帧号)``。
    """
    import cv2
    from standard.fiducial import FiducialLayout
    from vision.fiducial import calibrate_from_fiducials, detect_markers
    from vision.primitives import Frame

    layout = FiducialLayout(field_w=field_w, field_h=field_h)
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"打不开录像 {path}")
    best: Optional[tuple[Any, Any, int]] = None
    try:
        for i in range(scan_frames):
            ok, bgr = cap.read()
            if not ok:
                break
            h0, w0 = bgr.shape[:2]
            scale = 1.0 if (not process_width or w0 <= process_width) \
                else process_width / float(w0)
            small = bgr if scale == 1.0 else cv2.resize(
                bgr, (max(1, int(w0 * scale)), max(1, int(h0 * scale))),
                interpolation=cv2.INTER_AREA)
            fr = Frame(small.shape[1], small.shape[0],
                       small[:, :, 2].tobytes(),      # R
                       small[:, :, 1].tobytes(),      # G
                       small[:, :, 0].tobytes())      # B
            obs = detect_markers(fr)
            if not obs:
                continue
            try:
                calib, rep = calibrate_from_fiducials(
                    obs, layout, (small.shape[1], small.shape[0]))
            except Exception:                     # noqa: BLE001 标记不够就换下一帧
                continue
            if best is None or float(getattr(rep, "rms_px", 1e9)) < float(
                    getattr(best[1], "rms_px", 1e9)):
                best = (calib, rep, i)
            if int(getattr(rep, "n_markers", 0)) == 4 and float(
                    getattr(rep, "rms_px", 1e9)) < 1.0:
                break
    finally:
        cap.release()
    if best is None:
        raise SystemExit(
            f"在 {path} 的前 {scan_frames} 帧里没找到可用的四角标记，无法标定。\n"
            "  排查：标记是否被裁掉、画面是否太暗、或改用 --calibration 指定文件")
    return best
