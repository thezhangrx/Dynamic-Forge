"""闭环测试台：把 bullet_sim 当"真实世界"，视觉当唯一传感器。

    ┌──────────────┐  渲染/显示   ┌──────────┐  取帧   ┌──────────────┐
    │  bullet_sim  │ ───────────▶ │ 相机/屏幕 │ ──────▶ │  core/vision │
    │  "真实世界"   │              └──────────┘         │ 检测→世界坐标 │
    └──────┬───────┘                                   └──────┬───────┘
           │ Action                                           │ VisionFrame
           │                    ┌──────────────┐              │
           └────────────────────│  core/cpu    │◀─────────────┘
                                │ gap_avoid.plan│
                                └──────────────┘

**关键纪律：决策层只能看见摄像头。** ``plan()`` 拿到的永远是视觉产出的
``WorldView``；``world`` 只用来推进物理和渲染，不喂给算法。一旦偷看真值，
这个环路就什么都证明不了。

为什么单独一个模块（``core/harness/``）
--------------------------------------
这些代码原来堆在根脚本 ``cpu_vision_loop.py`` 里（578 行，58% 是非 main 逻辑），
而仓库约定是"根目录只放 ``<模块>_*.py`` 调用代码"。它也不能放进 ``core/cpu/``：
那里有"只依赖 bullet_sim 的**类型**、不依赖其实现"的硬约束，而本模块要用
``bullet_sim.render`` 离屏渲染、``bullet_sim.simulator`` 建场景。
``core/harness/`` 是集成层，依赖 ``bullet_sim`` + ``cpu`` + ``vision`` + ``standard``，
放在这里两边都不脏。

取帧方式的两种实现见 ``SyntheticCamera`` / ``RealCamera``；一局的完整流程见
:func:`run_episode`。
"""

from __future__ import annotations

import math
import sys

from bullet_sim.action.base import action_to_codec_input  # noqa: E402
from bullet_sim.action.types import Action  # noqa: E402
from bullet_sim.obstacles.scenario import ObstacleScenario  # noqa: E402
from bullet_sim.simulator.env import BulletHellEnv  # noqa: E402
from cpu.adapters.vision_adapter import (  # noqa: E402
    VisionWorldConfig,
    VisionWorldStream,
)
from cpu.gap_avoid import AlgoConfig  # noqa: E402
from vision import to_standard  # noqa: E402
from vision.calibration import (  # noqa: E402
    FieldCalibration,
    invert_homography,
    solve_homography,
)
from vision.localize import field_roi, out_of_field, to_world  # noqa: E402
from vision.wall_with_gap import WallWithGapParams, detect  # noqa: E402

# 真值读数的**唯一**出处：和离线对账（vision_reconcile）用同一套实现。
# 这里只用来**打分**（存活时间、决策输入误差），绝不进决策。
# 放在 bullet_sim 里是因为它读的就是平台自己的状态；放在 vision_reconcile 里
# 会让两个根脚本互相 import（循环导入，实测直接 ImportError）。
from bullet_sim.simulator.truth import truth_gaps, truth_player  # noqa: E402

SIM_DT = 1.0 / 120.0
FIELD = (640.0, 480.0)
#: 合成相机：窗口 -> 相机 的单应（一个透视四边形）
CAMERA_QUAD = [(60.0, 25.0), (580.0, 60.0), (530.0, 455.0), (95.0, 425.0)]
WIN = (960, 720)          # 渲染窗口
CAM = (640, 480)          # "相机"分辨率


def _classifier(name: str):
    """把 ``--classifier`` 的名字翻成 ``detect(classifier=...)`` 要的可调用对象。

    ``absolute`` 返回 ``None``，表示沿用 ``detect`` 自己的默认（绝对灰度阈值）——
    和这个开关存在之前的行为**一模一样**。
    ``adaptive`` 返回一个闭包，产出带弃权掩膜的 ``Masks``。
    """
    if name == "adaptive":
        from vision.adaptive import AdaptiveParams, classify_adaptive

        def adaptive_classifier(frame):
            masks, _ = classify_adaptive(frame, AdaptiveParams())
            return masks

        return adaptive_classifier
    return None


# --------------------------------------------------------------------------
# 取帧
# --------------------------------------------------------------------------
class SyntheticCamera:
    """离屏渲染 + 已知单应 —— 一台可以精确复现的相机。

    ``blur`` / ``noise`` 用来**故意把图像搞糊、搞脏**：合成图太干净，
    直接拿它的 0 碰撞去推真机是不负责任的。真实相机有镜头模糊、有传感器噪声，
    所以这里可以显式加上，看闭环能扛到什么程度。
    """

    def __init__(self, seed: int = 1, blur: int = 1, noise: float = 0.0):
        import numpy as np
        from bullet_sim.render.pygame_view import PygameRenderer
        self._np = np
        self.blur = int(blur)
        self.noise = float(noise)
        self._rng = np.random.default_rng(seed)
        self.win = WIN
        self.cam = CAM
        self._r = PygameRenderer(mode="rgb_array", window_size=WIN, show_hitboxes=False,
                                 show_world_axes=True, show_calibration_marker=False)
        self._h_inv = invert_homography(solve_homography(
            [(0.0, 0.0), (WIN[0] - 1.0, 0.0), (WIN[0] - 1.0, WIN[1] - 1.0), (0.0, WIN[1] - 1.0)],
            CAMERA_QUAD))

    def calibrate(self, env) -> FieldCalibration:
        """用同一台相机拍一张带**四角标记**的图，走真机同一条标定流程。

        为什么不再用那块洋红矩形（旧路径）：它盖住 90% 场地，只能在"开头露几秒"
        标定，而且标定实现和录制台/离线对账是**另一套**
        （``detect_marker_quad`` + ``calibrate_from_marker``）。
        四角标记只占场地约 5%、全程可见，而且和
        ``vision_capture_rig`` / ``vision_reconcile`` **共用同一个实现**
        （``vision.fiducial.detect_markers`` + ``calibrate_from_fiducials``），
        所以闭环和离线对账的标定质量是**同一个量**，可以直接比。
        """
        from standard.fiducial import FiducialLayout, snap_marker_side
        from vision.fiducial import calibrate_from_fiducials, detect_markers
        from bullet_sim.render.pygame_view import PygameRenderer

        r = PygameRenderer(mode="rgb_array", window_size=WIN, show_hitboxes=False,
                           show_world_axes=True, show_fiducials=True)
        photo = self._warp(r.render(env.world))
        # 边长必须和渲染器**实际画的**一致：渲染器会把边长吸附到"整数模块像素"
        # （否则模块间有半像素缝）。这里用同一个函数、同一个 scale 复算一遍，
        # 而不是假设 48.0 没变 —— scale 不是 3.0 的倍数时它真的会变。
        layout = FiducialLayout(field_w=FIELD[0], field_h=FIELD[1])
        scale = float(getattr(r.viewport, "scale", 1.0) or 1.0)
        layout = snap_marker_side(layout, scale)
        obs = detect_markers(self._frame(photo))
        if not obs:
            raise SystemExit("合成相机没检测到四角标记 —— 场景配置有问题")
        calib, rep = calibrate_from_fiducials(obs, layout, self.cam)
        self.last_fiducial_report = rep
        return calib

    def grab(self, env):
        return self._frame(self._degrade(self._warp(self._r.render(env.world))))

    def _degrade(self, img):
        """模糊 + 噪声。都是逐元素运算，和真机链路上发生的事同构。"""
        np = self._np
        out = img
        if self.blur > 1:
            k = self.blur
            pad = k // 2
            a = out.astype(np.float32)
            ap = np.pad(a, ((pad, pad), (pad, pad), (0, 0)), mode="edge")
            acc = np.zeros_like(a)
            for dy in range(k):
                for dx in range(k):
                    acc += ap[dy:dy + a.shape[0], dx:dx + a.shape[1]]
            out = np.clip(acc / (k * k), 0, 255).astype(np.uint8)
        if self.noise > 0.0:
            n = self._rng.normal(0.0, self.noise, out.shape)
            out = np.clip(out.astype(np.float32) + n, 0, 255).astype(np.uint8)
        return out

    def _warp(self, img):
        """窗口图像 -> "相机照片"。最近邻取样。

        **试过双线性和它比过**（别重复这次实验）：双线性更像真实光学（带限），
        但在这台相机上代价约 **3.6 倍**（86 vs 24 ms/帧），换来的只是标定
        RMS 0.449 -> 0.388 场地单位，而且留出/RMS 比反而从 1.81 变成 2.33。
        也就是说：**这里的瓶颈是相机分辨率，不是取样核**。相机 640x480、
        场地在画面里约占 520 px -> 0.81 px/场地单位，标记一个模块只有约 6.5 px，
        角点定位的量化误差就是这么来的。

        这带来一个必须说清的性质：**合成台的标定质量（~0.4 单位）比真实录屏
        （0.071 单位）更差**。方向是安全的 —— 合成台不会高估算法能力，
        它给决策输入误差定了一个约 0.4 单位的地板。要更高的保真度应该抬相机
        分辨率（代价是识别耗时按像素数涨），而不是换取样核。
        """
        np = self._np
        w, h = self.cam
        ww, wh = self.win
        H = self._h_inv
        ys, xs = np.mgrid[0:h, 0:w]
        d = H[6] * xs + H[7] * ys + H[8]
        u = (H[0] * xs + H[1] * ys + H[2]) / d
        v = (H[3] * xs + H[4] * ys + H[5]) / d
        ui = np.rint(u).astype(np.int64)
        vi = np.rint(v).astype(np.int64)
        ok = (ui >= 0) & (ui < ww) & (vi >= 0) & (vi < wh)
        out = np.empty((h, w, 3), np.uint8)
        out[:, :] = (18, 18, 20)
        out[ok] = img[vi[ok], ui[ok]]
        return out

    @staticmethod
    def _frame(photo):
        from vision.primitives import Frame
        h, w = photo.shape[:2]
        return Frame(w, h, photo[:, :, 0].tobytes(), photo[:, :, 1].tobytes(),
                     photo[:, :, 2].tobytes())


class RealCamera:
    """真的开摄像头。每次 ``grab()`` 抓一帧（时间戳用本机单调时钟）。"""

    def __init__(self, device: int, width: int, height: int):
        import cv2
        self._cv2 = cv2
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise SystemExit(f"打不开 /dev/video{device}")
        for _ in range(25):
            self.cap.grab()

    def grab(self, env=None):
        from vision.primitives import Frame
        ok, fr = self.cap.read()
        if not ok or fr is None:
            return None
        import numpy as np
        a = np.ascontiguousarray(fr)
        return Frame(a.shape[1], a.shape[0], a[:, :, 2].tobytes(),
                     a[:, :, 1].tobytes(), a[:, :, 0].tobytes())

    def close(self):
        self.cap.release()


# --------------------------------------------------------------------------
def build_env(seed: int, gap_width: float, seconds: float, obstacle_speed: float,
              gap_position: float = 0.5, gap_motion: str = "static"):
    """``wall_with_gap`` 障碍场景 —— 和 core/cpu 的算法一一对应。

    ``gap_position`` 是缺口在场地宽度上的位置（0~1）。**默认 0.5 会让实验变得
    没有意义**：角色恰好出生在场地中线（x=320），而缺口也在 320，于是它原地不动
    就能活下来。要看"视觉控制真的在起作用"，得把缺口挪开。
    """
    sc = ObstacleScenario(
        name="wall_with_gap", seed=seed, dt=SIM_DT,
        field_w=FIELD[0], field_h=FIELD[1],
        obstacles=[{"type": "wall_with_gap", "gap_width": float(gap_width),
                    "gap_position": float(gap_position),
                    "gap_motion": str(gap_motion),
                    "speed": float(obstacle_speed), "interval": 1.6}],
        duration=float(seconds),
        player_hitbox_radius=10.0,
    )
    # collision=None 是关键：形参默认 "circle" 会覆盖 spec 自带的 "obstacle"，
    # 那样旋转矩形的墙根本不做碰撞、玩家直接穿墙，整个实验就白跑了。
    return BulletHellEnv(sc.to_spec(), seed=seed, codec="action", collision=None)


def to_action(dec) -> Action:
    if math.hypot(dec.wx, dec.wy) < 1e-9:
        return Action.zero()
    return Action.from_vector((dec.wx, dec.wy))


def _contacts(env):
    """当前碰撞状态。**不要**用 ``getattr(env.world, "collision_events", 0)``。

    踩过：``env.world`` 上根本没有 ``collision_events`` 这个属性（那是
    ``env.step()`` 返回的 ``info`` 字典里的键），于是 ``getattr(..., 0)``
    永远拿到 0 —— **"0 碰撞"这个结论一直是假的**，而且它不会报错，
    只是安静地一直报 0。整条闭环的验收指标就这样空了。

    真正的计数器在 ``env.world.contacts``（``ContactTracker``）：
    ``entered_total`` 单调累加"接触进入"次数，``in_contact`` 是当前是否重叠，
    ``overlap_frames`` 是累计重叠帧数（擦过去的严重程度），
    ``first_event_frame`` 给首次碰撞发生在哪一步（-> 存活时间）。
    """
    c = env.world.contacts
    first = c.first_event_frame
    return {
        "entered": int(c.entered_total),
        "in_contact": bool(c.in_contact),
        "overlap_frames": int(c.overlap_frames),
        "first_event_frame": None if first is None else int(first),
    }


def run_episode(env, camera, calib, *, steps: int, cfg: AlgoConfig,
                control_hz: float = 30.0, verbose: bool = False,
                classifier=None, policy: str = "gap_avoid") -> dict:
    """跑一局。**决策只用摄像头**。

    ``control_hz`` 把"物理步进"和"看一次摄像头"解耦：相机不会 120 Hz 出图，
    真机也不是每个仿真步都决策。两次决策之间**保持上一次的动作**。

    ``policy``：

    * ``gap_avoid``（默认）—— 真链路：渲染 → core/vision → vision_adapter → plan()；
    * ``hold`` —— **阴性对照**：角色一动不动。它的作用不是"跑一个基线"，
      而是**证明碰撞指标是活的**：如果连"站着不动"都报 0 碰撞，
      那说明计数器读错了（这个错真发生过），于是 gap_avoid 的 0 碰撞毫无意义。
      任何"无碰撞率"结论都必须和这个对照一起看。
    """
    from bullet_sim.entities.player import P_SPEED

    env.reset(seed=int(env.spec.seed))
    stream = VisionWorldStream(VisionWorldConfig(field_w=FIELD[0], field_h=FIELD[1]))
    codec = env.world.codec
    speed = float(env.world.player[P_SPEED])

    action = Action.zero()
    dec = None
    decide_every = max(1, int(round((1.0 / max(1e-6, control_hz)) / SIM_DT)))
    stats = {"steps": 0, "decided": 0, "rejected": 0, "tiers": {},
             "first_tier": None, "control_hz": control_hz, "policy": policy,
             "start": None, "end": None, "max_y": None, "trace": [],
             "out_of_field": 0, "entered": 0, "in_contact": False,
             "overlap_frames": 0, "ever_collided": False, "survived_s": None}
    for k in range(steps):
        # ① 推进"真实世界"：用的是**上一次决策**出来的动作 —— 一拍延迟，和真机一样
        env.step(action_to_codec_input(action, codec, speed))
        stats["steps"] += 1

        # ② 碰撞状态**每一步**都读，这样存活时间不会被决策周期量化
        c = _contacts(env)
        stats["entered"] = c["entered"]
        stats["in_contact"] = c["in_contact"]
        stats["overlap_frames"] = c["overlap_frames"]
        if c["entered"] and not stats["ever_collided"]:
            stats["ever_collided"] = True
            stats["first_event_frame"] = c["first_event_frame"]
            stats["survived_s"] = k * SIM_DT

        if policy == "hold":
            action = Action.zero()          # 阴性对照：不看不决策，只让墙撞上来
            continue

        if k % decide_every:
            continue

        # ③ 取一帧（合成 or 真摄像头）—— 这是决策层唯一的输入通道
        frame = camera.grab(env)
        if frame is None:
            continue

        # ④ 视觉：像素 -> 标准帧 -> 世界坐标
        stamp = float(env.world.state.timestamp)
        std = to_standard.frame_to_standard(
            detect(frame, WallWithGapParams(), roi=field_roi(calib),
                   classifier=classifier),
            seq=k, stamp=stamp)
        world_frame = to_world(std, calib)

        # ④.5 标定还成立吗？**闭环里这一步尤其不能省**：错的单应不会报错，
        # 它会让角色被一堆垃圾坐标驱动着跑。宁可停帧，也不能拿错坐标去动电机。
        bad = out_of_field(world_frame, calib)
        if bad:
            stats["out_of_field"] += 1
            if stats["out_of_field"] == 1:
                print(f"  ⚠ 第{k}帧越界：{bad[0]}", file=sys.stderr)
                if calib.window is not None:
                    print(f"    标定记的是窗口 x={calib.window[0]} y={calib.window[1]} "
                          f"{calib.window[2]}x{calib.window[3]}；挪过窗口就会这样",
                          file=sys.stderr)
            action = Action.zero()          # 停住，别让错坐标驱动角色
            continue

        # ⑤ 决策：注意这里传的是**视觉**产出的帧，不是 env.world
        try:
            view, dec = stream.step(world_frame, cfg=cfg)
        except Exception as exc:                      # noqa: BLE001
            stats["rejected"] += 1
            if verbose and stats["rejected"] <= 3:
                print(f"    第{k}帧被决策层拒绝：{type(exc).__name__}: {exc}")
            continue
        stats["decided"] += 1
        stats["tiers"][dec.tier] = stats["tiers"].get(dec.tier, 0) + 1
        # 用**视觉**给的位置记录轨迹（真值只用来推进物理，不进这里）
        pos = (world_frame.player.x, world_frame.player.y)
        if stats["start"] is None:
            stats["start"] = pos
        stats["end"] = pos
        stats["max_y"] = pos[1] if stats["max_y"] is None else max(stats["max_y"], pos[1])
        # 决策层的输入误差：视觉给出的缺口中心 vs 同一帧真值的缺口中心。
        # **只用来打分**，不进决策（下面的 stats 在跑完之前没有任何东西会读它）。
        # 决策层的输入误差：**每个**视觉缺口到最近真值缺口的距离（倒角），
        # 取该帧的中位。不能用 vg[0] 去配"x 最近的真值缺口"——场上有 ~8 堵墙，
        # 那样会把两条不同的墙配到一起，实测报出 256 场地单位的假误差。
        # 倒角对"多一个/少一个"不敏感，和内容对齐用的是同一套语义。
        tg = truth_gaps(env.world) or ()
        if tg and world_frame.gaps:
            ds = [min(math.hypot(g.center[0] - tx, g.center[1] - ty)
                      for (tx, _tw, ty, _tt) in tg) for g in world_frame.gaps]
            ds.sort()
            stats.setdefault("gap_in_err", []).append(ds[len(ds) // 2])
            stats.setdefault("gap_in_n", []).append((len(world_frame.gaps), len(tg)))
        tpx, tpy, _tr = truth_player(env.world)
        stats.setdefault("player_in_err", []).append(
            math.hypot(pos[0] - tpx, pos[1] - tpy))
        if verbose:
            stats["trace"].append((round(stamp, 2), dec.tier,
                                   round(pos[0]), round(pos[1]),
                                   round(dec.gap, 1) if math.isfinite(dec.gap) else None,
                                   (round(action.direction[0], 2), round(action.direction[1], 2))))
        if stats["first_tier"] is None:
            stats["first_tier"] = dec.tier
        action = to_action(dec)

    stats["final_tier"] = dec.tier if dec is not None else "NONE"
    # 前向进展（诊断量，**不是**验收判据）：本算法对"缺口在 x 上不动的墙"有
    # 一个显式的 HOLD 分支，理由是 ``gap_will_pass_me``（缺口会自己滑过角色，
    # 停住就是正确解，见 core/cpu/gap_avoid.py 的 tier 说明）。所以进展为 0
    # 本身不是缺陷。之所以还是要报：万一哪天算法退化成"什么都不做"，
    # 无碰撞率照样是 100%（站着不动在别的场景里也能碰巧躲过），
    # 只有进展 + 档位分布能把那种退化露出来。
    if stats["start"] is not None:
        stats["forward_gain"] = float(stats["max_y"] - stats["start"][1])
    else:
        stats["forward_gain"] = 0.0
    return stats


