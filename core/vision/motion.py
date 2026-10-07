"""任务二：从视频取速度。

链条
----
``检测(世界坐标) → 跨帧关联 → α-β 滤波 → 速度``

三个关键决定，每个都有依据
--------------------------
**1. dt 必须用 ``stamp`` 之差，不能用帧号。**
三个参考项目全在这里翻车：``optical-flow-tracker`` 从不读 ``CAP_PROP_FPS``；
``ByteTrack`` 的 ``dt`` 在 ``kalman_filter.py:41`` 写死为 1（速度单位变成"像素/帧"）；
``Stereo_Vision_Camera`` 定义了 ``v4l2_buffer.timestamp`` 却从未读取。
本模块的 ``update()`` 强制要求显式传 ``stamp``，不给"用帧号凑"的机会。

**2. 关联用"最近邻 + 门限"，不用 LAPJV。**
``ByteTrack`` 用的是 LAPJV（最优分配），但它的**增广路径长度可变**，
在 FPGA 上无法做定长流水（参考笔记里记为"最难上板的一环"）。
目标数少（个位数）时，贪心最近邻与最优解几乎总是相同，而且只要 N×M 个比较器 +
比较树 —— 可以流水。

**3. 滤波用定增益 α-β，而不是完整 Kalman。**
``ByteTrack`` 的推导显示：Kalman 增益矩阵**与框高 h 无关** —— 因为 P0/Q/R 在
x/y/h 三个通道都正比于 h²、a 通道全是常数，归一化后 Riccati 递推与 h 无关
（实测 h=100 vs h=400 前 40 帧逐帧 ``max|ΔK| = 0.0``，稳态 α=0.6608、β=0.0728）。
所以完整的矩阵递推可以**退化成两个常数**：

    x̂ ← x̂ + v̂·dt
    r  = z − x̂
    x̂ ← x̂ + α·r
    v̂ ← v̂ + (β/dt)·r

每帧每目标 4 乘 4 加，**没有 Cholesky、没有除法（除 dt 可预先取倒数）、没有开方** ——
纯整数/定点即可上 PL。

参考的开源项目
--------------
* ``ByteTrack``（``yolox/tracker/kalman_filter.py``、``byte_tracker.py``）——
  上面那条"增益与 h 无关"的结论、以及"速度存在 ``state[4:6]`` 但单位是像素/帧"的坑；
* ``optical-flow-tracker`` —— 反面教材：只有 px/帧，没有 fps，也丢掉了带符号流场；
* ``docs/reference/三个任务的技术路线.md`` §3.2 —— 三级换算
  （px/帧 → px/s → 世界单位/s）必须自己补齐。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence

Point = tuple[float, float]

#: ByteTrack 实测的稳态 α-β 增益（对应它的 P0/Q/R 与 dt=1）。
#: α 修正位置、β 修正速度；数值越大越跟手、越小越平滑。
DEFAULT_ALPHA = 0.6608
DEFAULT_BETA = 0.0728


@dataclass
class Track:
    """一条轨迹：世界坐标下的位置与速度。"""

    id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    stamp: float = 0.0
    #: 命中次数 / 连续丢失次数 / 总存活帧数。
    hits: int = 1
    misses: int = 0
    age: int = 1

    @property
    def speed(self) -> float:
        """速率（世界单位/秒）。"""
        return math.hypot(self.vx, self.vy)

    @property
    def heading(self) -> float:
        """运动方向（弧度）。静止时为 0。"""
        return 0.0 if (self.vx == 0.0 and self.vy == 0.0) else math.atan2(self.vy, self.vx)

    def predict(self, dt: float) -> Point:
        return self.x + self.vx * dt, self.y + self.vy * dt


@dataclass
class Tracker:
    """最近邻关联 + 定增益 α-β 滤波。

    输入是**世界坐标**的点（先用 :func:`vision.localize.to_world` 把像素换过去），
    所以输出速度天然是"世界单位/秒"。若直接喂像素，速度就是像素/秒 ——
    物理意义由调用方负责，本模块只保证 **dt 来自 stamp**。
    """

    alpha: float = DEFAULT_ALPHA
    beta: float = DEFAULT_BETA
    #: 关联门限的**基础**值（世界单位）。超出就不认为是同一个目标。
    #:
    #: 默认 24 ≈ 1.2 个玩家直径（平台把间距都以 ``player_diameter = 2r`` 计价，
    #: 障碍场景里 r=10）。**这个数不能拍脑袋选小**：实测在 640x480 的场地里，
    #: 角色一帧能走 1.67 个单位，而门限如果按"米"的习惯写成 0.6，
    #: 结果每一帧都判成新目标 —— 每条轨迹只活一帧，速度恒为 0，
    #: 而位置看起来还是对的，所以这个 bug 特别难发现。
    gate: float = 24.0
    #: 门限随目标**自身速度**放宽的比例：``gate + frac * |v| * dt``。
    #:
    #: 快速目标本来就该有更大的搜索窗，否则它总会"跑出门限"。
    #: 取 0.5 表示"允许比按惯性预测的落点再多偏半个身位"。
    gate_speed_frac: float = 0.5
    #: 连续丢失多少帧后删除轨迹。
    max_misses: int = 5
    #: 两帧间隔超过它就**不做预测**（速度置零重来），避免跨大间隔乱推。
    max_dt: float = 0.5
    _tracks: list[Track] = field(default_factory=list)
    _next_id: int = 1
    _last_stamp: float | None = None

    # ------------------------------------------------------------------
    def update(self, detections: Sequence[Point], stamp: float) -> list[Track]:
        """喂进这一帧的目标点，返回更新后的活跃轨迹（含速度）。

        ``stamp`` 是**采集时刻**（秒）。第一次调用用来建立时基。
        """
        dt = 0.0 if self._last_stamp is None else float(stamp) - self._last_stamp
        if dt < 0.0:
            raise ValueError(f"stamp 必须单调递增：{self._last_stamp} -> {stamp}")
        fresh = self._last_stamp is None or dt > self.max_dt
        self._last_stamp = float(stamp)

        # -- 1. 预测：把每条轨迹推到当前时刻 ------------------------------
        predicted: list[Point] = []
        for t in self._tracks:
            if fresh or dt <= 0.0:
                predicted.append((t.x, t.y))          # 不预测，原地等
            else:
                predicted.append(t.predict(dt))

        # -- 2. 关联：贪心最近邻 + 门限（固定 N×M 次距离比较，可流水）------
        used = [False] * len(detections)
        matched: dict[int, int] = {}                  # 轨迹下标 -> 检测下标
        for ti, (px, py) in enumerate(predicted):
            t = self._tracks[ti]
            # 自适应门限：跑得快的目标给更大的搜索窗（一次乘加，仍然可流水）
            gate = self.gate + self.gate_speed_frac * t.speed * (dt if dt > 0.0 else 0.0)
            best_j, best_d = -1, gate
            for j, (zx, zy) in enumerate(detections):
                if used[j]:
                    continue
                d = math.hypot(zx - px, zy - py)
                if d <= best_d:
                    best_j, best_d = j, d
            if best_j >= 0:
                used[best_j] = True
                matched[ti] = best_j

        # -- 3. 滤波：α-β 更新 ------------------------------------------
        survivors: list[Track] = []
        for ti, t in enumerate(self._tracks):
            if ti not in matched:
                t.misses += 1
                if t.misses <= self.max_misses:
                    if fresh or dt <= 0.0:
                        t.vx = t.vy = 0.0             # 间隔太大，速度不可信
                    else:
                        t.x, t.y = predicted[ti]      # 惯性外推
                    t.age += 1
                    survivors.append(t)
                continue

            zx, zy = detections[matched[ti]]
            if fresh or dt <= 0.0:
                # 第一帧或大间隔：直接落位，速度保持 0
                t.x, t.y = zx, zy
                t.vx = t.vy = 0.0
            else:
                px, py = predicted[ti]
                rx, ry = zx - px, zy - py
                t.x = px + self.alpha * rx
                t.y = py + self.alpha * ry
                t.vx += (self.beta / dt) * rx
                t.vy += (self.beta / dt) * ry
            t.stamp = float(stamp)
            t.hits += 1
            t.misses = 0
            t.age += 1
            survivors.append(t)

        # -- 4. 新目标：未被任何轨迹认领的检测 ----------------------------
        for j, (zx, zy) in enumerate(detections):
            if used[j]:
                continue
            survivors.append(Track(id=self._next_id, x=zx, y=zy,
                                   stamp=float(stamp)))
            self._next_id += 1

        self._tracks = survivors
        return list(survivors)

    # ------------------------------------------------------------------
    @property
    def tracks(self) -> list[Track]:
        return list(self._tracks)

    def reset(self) -> None:
        self._tracks.clear()
        self._last_stamp = None
        self._next_id = 1

    def speeds(self) -> list[float]:
        return [t.speed for t in self._tracks]


def track_points(points_per_frame: Iterable[Sequence[Point]],
                 stamps: Sequence[float], **kwargs: object) -> list[list[Track]]:
    """把"每帧一组点 + 每帧时间戳"跑成轨迹序列，方便离线验证。"""
    frames = [list(p) for p in points_per_frame]     # 先落地，生成器只能消费一次
    if len(frames) != len(stamps):
        raise ValueError(f"帧数与时间戳数不一致：{len(frames)} vs {len(stamps)}")
    tr = Tracker(**kwargs)  # type: ignore[arg-type]
    return [tr.update(pts, ts) for pts, ts in zip(frames, stamps)]


__all__ = ["DEFAULT_ALPHA", "DEFAULT_BETA", "Track", "Tracker", "track_points"]
