#!/usr/bin/env python3
"""wall_with_gap 缺口墙避障算法 —— 临时可视化验证脚本（用完即删）。

    .venv/bin/python gap_wall_demo.py                       # 5 局，缺口静止
    .venv/bin/python gap_wall_demo.py --gap-sweep-px 120    # 5 局，缺口真的在扫动
    .venv/bin/python gap_wall_demo.py --gap-width 40 --obstacle-speed 200

============================================================================
这个文件是**一次性的**：它不改动仓库里任何既有文件，看完了就删掉。
它做三件事：
  1. 把「移动缺口墙」的避障算法实现成一个纯函数：世界状态 -> Action；
  2. 用仓库自带的 pygame 渲染器把它画出来（含算法内部量的叠加层）；
  3. 连跑 5 局，每局换种子，最后打印统计。
============================================================================

关于「缺口到底会不会移动」——实测结论
--------------------------------------
README :300 写的是「位置随机抖动、可扫动」，catalog 的默认参数也是
``gap_motion="sweep"``。但在**障碍**这条路径上（也就是 ``--obstacle-type
wall_with_gap`` 和 ``--level hard/extreme`` 实际走的那条），
``gap_motion`` / ``gap_position`` / ``gap_speed`` 这三个字段
**只被声明、只被写进 to_dict()，从头到尾没有任何代码读它们**：

    obstacles/spec.py:129-131   声明
    obstacles/spec.py:174-176   只写进 to_dict()
    => 没有任何地方拿它去改 x / half_w

真正实现了 sweep 的是旧的装饰性 pattern 生成器
``generators/patterns.py::_layout_wall_gap``（用 ``amp*sin(2*pi*speed*t)``），
但那条路已经不可达：``available_patterns() == ['obstacle']``。

所以实测（120Hz，seed 3，默认障碍场景）：
    每一堵墙在其整个下降过程中，缺口中心**是不变的**
        y=479→344 : 320.0     (持续 5 秒)
        y=420→209 : 320.0
        y=429→217 : 264.3
        y=438→150 : 212.1 ...
    变化只发生在**换墙**的时候：320 → 264.3 → 212.1 → 166.7 → 130.9 → 107.0
    墙每 ~1.6-2.5 秒生成一堵，所以看上去就像"缺口在场地里来回跑"。

结论：**在现有代码里，缺口不会在一堵墙的生命周期内移动**，它是在"换墙"时跳变。
本脚本提供 ``--gap-sweep-px`` 直接**手动**把缺口改成真正扫动
（外缘钉在场地边界、内缘整体平移），用来验证算法到底扛不扛得住会动的缺口。

算法一句话
----------
墙跨越整个场地（README :69），缺口是唯一通路。墙从场地一侧匀速压过来，
玩家要做的是**在墙到达自己这条线之前，横向移动到缺口里**（README :300 的原话：
"定位缺口并判断能否在墙到达前抵达"）。

与早先纸面版的一处重要修正
--------------------------
纸面版里有"已对齐就沿 n̂ 穿过去（T1_CROSS）"这一支。跑了真实场景发现它是**自杀**：
这里的墙是"从上方压下来的整面隔断"，沿 n̂ 前进 = 迎面撞上去。
README :487 又写着 success_terminates_episode=False（环境没有目的地），
所以根本不存在"必须穿到墙另一侧"的理由。因此本实现默认**只做横向对齐 + 保持**，
穿越分支留在 --allow-cross 后面，方便你对比看差别。

试过但已删掉的两个做法（结论都保留，代码已移除）
------------------------
其一：前后移动 / 沿墙法向后退买时间（实测无效）
-------------------------
一度给车加过"沿墙法向后退买时间"（把速度预算在横向和后退之间分摊，
`r = min(0.7, v_max/v_n)`，`u = sqrt(1-r²)`，并用场地边界算出还能退多远）。
5 局 × 12s 的 A/B 结果是**不改善，在缺口会扫动的场景里明显更差**：

    场景                      只横向   前后也能动
    默认 gap200 v90              0        0
    扫动 ±120 @0.6Hz             6       12   <- 变差
    扫动 ±200 @0.3Hz             3        9   <- 变差
    墙速 2400（t_arr=0.17s）     25       25
    墙速 3200（t_arr=0.12s）     30       30

原因是瓶颈不在反应时间，而在瞄准：车出生在 y=72（场高 480），**身后只有 62px**
（0.31 秒）可退；本场景相邻两堵墙的缺口只差 ~50px，而 t_arr 有 0.5~4.4 秒，
横向跑完全场也只要 3.2 秒 —— 时间一直富余。真正卡时间的档位（墙速 >=1600）
缺口已经无解，退不退都撞。而扫动场景的失败是"用直线去拟合正弦"的预测误差，
**再多时间也修不了一个错的方向**。所以整段删掉了。

其二：每过一堵墙提速（实测有效，但按要求已删除）
------------------------
`v = min(v0 * (1 + k*通过墙数), v0*cap)`，写回 `player[P_SPEED]`。
它落地时是有效的 —— 4 局 × 14s 的 A/B，**每个困难档都改善，没有一个变差**：

    场景                    不提速   提速
    默认 gap200 v90            0      0
    窄缺口 gap40 v800          0      0
    扫动 ±120 @0.6Hz           8      4   <- 改善
    扫动 ±200 @0.3Hz           4      0   <- 改善
    扫动 ±120 @1.0Hz          16     12   <- 改善
    墙速 2400                  0      0

对照上一节的"前后移动"，差别正好说明了瓶颈在哪：后退是拿横向进度去换时间，
而时间本来就不缺；提速是直接给横向**可达距离和跟踪带宽**，这才是追一个会扫动
的缺口真正需要的东西。所以"给车更多能力"这件事，方向对才有用。

两个实现要点（将来要恢复的话别踩）：
  * "通过一堵墙"要用**墙自己的速度方向** a = normalize(v_wall) 判定，不能用 n̂：
    n̂ 是"从墙指向玩家"，墙一越过玩家它就翻向，同一堵墙会被算成两堵。
        s_wall = max(corner·a) > p·a + r_p  =>  记一次
  * 去重要用**实体 id 集合**，且只要任意一段的 id 记过就算这堵墙记过 ——
    否则一段被剔除（实测约 4% 的帧）时会重复计数、白送一档速度。
  * 副作用：它会让 README :519 的判据②（相邻位移不超过 `player_speed`）失效，
    因为校验用的是场景的初始 `player_speed`。要恢复就得同时改校验。

实测暴露的坑（算法必须处理）
----------------------------
1. **所有墙段的 group_id 都是 0** => 不能用 group_id 配墙，改用几何分组。见 group_walls()。
2. **只看得见一段的情况真的会发生**（实测约 4% 的帧）：剔除按"中心点"判
   （physics/bounds.py，margin = 32），宽 440 的墙中心一越过 640+32 就整段消失。
3. **缺口可用区间必须按场地裁剪**：实测缺口会跑到场地边缘甚至外面。
4. **墙的身份不能用 min(ent_id)**：一段被剔除后 id 跳变，记忆就丢了。
   改成按"上一帧最近的墙"关联。
5. **墙正压在玩家身上时不能判成"已过去"**：必须按玩家半径膨胀墙带、分三段判断。
6. **浮点容差**：缺口算出来 200.00000000000003、自由空间 200.0，`>=` 会判 False。
7. **每一堵墙都要重建缺口**，不能只算选中的那堵（全局缺口宽度提示要靠它学习）。
8. **collision=None**：BulletHellEnv 形参默认 "circle" 会覆盖 spec.collision
   （= "obstacle"），不传 None 的话墙根本不做碰撞，玩家穿墙而过。

依赖：numpy + pygame（仓库 .venv 里都有）。
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

# --- 让这个"退役"副本仍然可运行 -------------------------------------------
# 本文件现在住在 Delete_old/ 下面，而 Delete_old/ 里还有一个**同名的
# bullet_sim/ 包**（旧的 cpu / ai 决策栈）。直接跑的话，Python 会把脚本所在目录
# 放在 sys.path[0]，于是 ``import bullet_sim`` 命中 Delete_old/bullet_sim，
# 那个包下面没有 action/，直接 ModuleNotFoundError。
# 把项目根目录插到 sys.path 最前面，就能稳定命中活树里的 bullet_sim。
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
# ---------------------------------------------------------------------------

from bullet_sim.action import Action
from bullet_sim.action.base import action_to_codec_input
from bullet_sim.entities.player import P_RADIUS, P_SPEED, P_VX, P_VY, P_X, P_Y
from bullet_sim.obstacles.scenario import ObstacleScenario
from bullet_sim.render.pygame_view import PygameRenderer
from bullet_sim.simulator.env import BulletHellEnv

# ---------------------------------------------------------------------------
# 常量：与 README / 帧协议一致
# ---------------------------------------------------------------------------
SHAPE_RECT = 1                # README :125 SHAPE_CODES {"circle": 0, "rect": 1}
EPS = 1e-9
#: 几何比较的容差。实测 gap 是 200.00000000000003 而自由空间是 200.0，
#: 直接用 >= 会因为这个 3e-14 的误差判成"放不下"，算法就整帧摆烂。
TOL = 1e-6
SIM_DT = 1.0 / 120.0          # 仿真固定步长


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1])


# ===========================================================================
# 1. 配置
# ===========================================================================
@dataclass(frozen=True)
class AlgoConfig:
    #: README 判据 ①：膨胀量 = 玩家外接圆 + safety_margin。
    #: README 没有给出这个数值，这里取 safety/validate.py 里 SafetyConfig 的默认值 6.0。
    safety_margin: float = 6.0
    #: README 判据 ③：验证时窗 = min(回合剩余, max_window)。同样取 SafetyConfig 默认 8.0。
    max_window: float = 8.0
    #: 「已经在缺口里」的判定：横向差 <= align_frac * 可用半孔径 就算到位。
    align_frac: float = 0.6
    #: 横向差小于这个比例 * A 时连微调都不做（防抖）。
    dead_frac: float = 0.05
    #: 是否允许"穿缺口到墙另一侧"。默认关：见文件头说明。
    allow_cross: bool = False
    #: 瞄准用的预测时长（秒）。
    #:
    #: 诚实记录：这一项**在 A/B 里没有测出明显收益**。
    #: 原理上缺口会动时不该用 t_arr（可达 4 秒）去外推 —— 线性拟合外推正弦是垃圾；
    #: 但实测把它从 0.25s 放开到 99s（等于退回"瞄准 t_arr"）后，
    #: 3 局 × 14s 的碰撞数是 0/0/0/0/3/12/6（分别对应下面的扫动档位），
    #: 与 0.25s 的 0/0/0/0/3/6/12 相比，差异都在噪声量级且方向不一致。
    #: 原因大概是：算法每个仿真步都重算，就算瞄准点算错了，"往哪边动"的符号
    #: 仍然是对的，于是它实际上退化成了追踪，最后 t_arr→0 时预测自动收敛到真值。
    #: 保留这个旋钮的理由只是：它把外推误差限制在短时距内，更不容易出怪事；
    #: 嫌多一个参数的话，把它设大即可回到旧行为。
    aim_lookahead: float = 0.25

    #: 允许控制**前后**速度（含减到 0 停住）。
    #:
    #: 方向定义：n̂ 从墙指向车，所以 −n̂ = 朝墙 = 前进，+n̂ = 远离墙 = 后退。
    #: （上一版把这两个方向写反了：代码是 "− n̂" 却注释成"后退"，实测那是在朝墙冲，
    #:   所以那次 A/B 根本没测到后退。这次先把方向钉死。）
    #:
    #: 为什么在"墙朝车压过来"的场景里前后移动没用、在"车朝墙开过去"的场景里却是关键：
    #: 两者相对运动相同，但可用的自由度不同 —— 墙有法向速度时，车停住也拦不住墙；
    #: 墙在行驶方向静止时，**车停住 = 对齐时间变成无限**，这就是"先过前两堵、
    #: 停在第三堵前面慢慢对缺口"能成立的物理原因。
    front_back: bool = False
    #: 关掉前后控制时，恒定的前进速度比例（车不能停）。0 = 完全不前进。
    fwd_speed: float = 0.0
    #: 对齐时与墙保持的净距离（世界单位）。大于 0 才有"停下来对准"的余地。
    hold_clearance: float = 10.0
    #: 固定行驶轴（如 (0,1) = 一直朝 +y 开）。给了它，墙的法向就不再靠
    #: "车在墙哪一侧"来判断 —— 静态墙那个判断是退化的，会让前进方向抖动。
    forward: Optional[tuple[float, float]] = None
    #: 指令方向的**最大转向角速度**（度/秒）。0 = 不限（默认，保持旧行为）。
    #:
    #: 为什么需要：目标墙交接的那一帧，两道缺口可能差 180px，指令方向会瞬间
    #: 转 100° 以上 —— 对真车来说是无穷大横向加速度，画面上就是"突然旋转"。
    #: 限住它才是真车能执行的指令。注意这是**指令层**的限制，不是动力学模型：
    #: 真正的车还要有最小转弯半径（README 里目前没有这个约束）。
    max_turn_rate: float = 0.0


# ===========================================================================
# 2. 手动让缺口真的扫动（因为 gap_motion 在障碍路径上是死参数）
# ===========================================================================
def apply_gap_sweep(world: Any, amp_px: float, freq_hz: float,
                    sim_time: float, field_w: float,
                    base_positions: dict) -> None:
    """把每堵墙的两段重排成"缺口按正弦扫动"的样子。

    为什么本脚本要自己干这件事：见文件头 —— ``gap_motion`` / ``gap_position`` /
    ``gap_speed`` 在 obstacles/spec.py 里只被存下来、从来没有被消费，
    所以 ``--obstacle-type wall_with_gap`` 的缺口在每堵墙的一生里都是不动的。

    重排方式（和生成器原本的布局一致）：
        左段 = [0, new_lo]                外缘钉在场地左边界，内缘平移
        右段 = [new_hi, field_w]          内缘平移，外缘钉在场地右边界
        new_lo = clamp(base_lo + amp*sin(2*pi*freq*t), 0, field_w - gap)

    ``base_positions`` 必须记住每堵墙**最初的**缺口左缘。
    第一版忘了这件事，写成 ``clamp(当前左缘 + delta)`` —— 那等于对正弦做积分，
    缺口会一路漂到边界并来回撞限位（实测 ±5px 的设置居然让缺口在 0 和 440 之间
    猛冲，估出来的缺口速度飙到 ±600 px/s）。那不是算法的问题，是这段测试脚手架的
    问题；修好之后才是真正的"±N 像素正弦扫动"。
    """
    if amp_px <= 0.0:
        return
    pool = world.state.bullets
    d = pool.data
    idx = pool.active_indices()
    bands: dict[float, list[int]] = {}
    for i in idx:
        if int(d["shape"][i]) != SHAPE_RECT:
            continue
        bands.setdefault(round(float(d["y"][i]), 1), []).append(i)
    delta = float(amp_px) * math.sin(2.0 * math.pi * float(freq_hz) * float(sim_time))
    for ids in bands.values():
        if len(ids) != 2:
            continue
        ids.sort(key=lambda i: float(d["x"][i]))
        il, ir = ids
        e_lo = float(d["x"][il]) + float(d["half_w"][il])
        e_hi = float(d["x"][ir]) - float(d["half_w"][ir])
        gap = e_hi - e_lo
        if gap <= 0.0 or gap >= field_w:
            continue
        key = int(d["id"][il])
        if key not in base_positions:
            base_positions[key] = e_lo          # 第一次看见这堵墙时记下它的基准位置
        new_lo = min(max(base_positions[key] + delta, 0.0), field_w - gap)
        new_hi = new_lo + gap
        d["half_w"][il] = new_lo / 2.0
        d["x"][il] = new_lo / 2.0
        d["half_w"][ir] = (field_w - new_hi) / 2.0
        d["x"][ir] = new_hi + (field_w - new_hi) / 2.0


# ===========================================================================
# 3. 只读观察：把帧里的矩形按"同一堵墙"聚起来
# ===========================================================================
@dataclass(frozen=True)
class Rect:
    """一个矩形实体（README :116-132 的字段子集）。"""

    ent_id: int
    x: float
    y: float
    vx: float
    vy: float
    half_w: float
    half_h: float
    rotation: float
    angular_velocity: float


@dataclass(frozen=True)
class Wall:
    """一堵缺口墙：1~2 段共线矩形 + 它的墙坐标系。

    û = 墙的跨度方向（缺口沿它排布，实测就是 half_w 轴）
    n̂ = 墙厚方向，**规定从墙指向玩家**，(v_wall - v_p)·n̂ > 0 表示"墙在逼近"。
    """

    rects: tuple[Rect, ...]
    u_hat: tuple[float, float]
    n_hat: tuple[float, float]

    @property
    def key(self) -> int:
        """最小实体 id（仅作诊断；真正的跨帧身份见 GapMemory.match_keys）。"""
        return min(r.ent_id for r in self.rects)


def _half_extent_along(r: Rect, d: Sequence[float]) -> float:
    """旋转矩形在单位方向 d 上的半投影长度（精确支撑函数，不是小角度近似）。

        h(d) = half_w * |d · x̂_local| + half_h * |d · ŷ_local|
    """
    ct, st = math.cos(r.rotation), math.sin(r.rotation)
    aw = (ct, st)                 # 局部 x 轴 = half_w 方向
    ah = (-st, ct)                # 局部 y 轴 = half_h 方向
    return r.half_w * abs(_dot(d, aw)) + r.half_h * abs(_dot(d, ah))


def _rect_sdf(r: Rect, px: float, py: float) -> tuple[float, tuple[float, float]]:
    """旋转矩形 SDF + 它的单位梯度（README :496：引擎自带圆+旋转矩形 SDF）。"""
    ct, st = math.cos(r.rotation), math.sin(r.rotation)
    dx, dy = px - r.x, py - r.y
    lx = dx * ct + dy * st                 # 世界 -> 局部
    ly = -dx * st + dy * ct
    qx = abs(lx) - r.half_w
    qy = abs(ly) - r.half_h
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    sdf = outside + inside

    sx = 1.0 if lx >= 0 else -1.0
    sy = 1.0 if ly >= 0 else -1.0
    if qx > 0.0 and qy > 0.0:              # 角点区
        gx_l, gy_l = sx * qx, sy * qy
        n = math.hypot(gx_l, gy_l) or 1.0
        gx_l, gy_l = gx_l / n, gy_l / n
    elif qx > 0.0:
        gx_l, gy_l = sx, 0.0
    elif qy > 0.0:
        gx_l, gy_l = 0.0, sy
    else:                                  # 在矩形内部：沿最浅的轴推出
        gx_l, gy_l = (sx, 0.0) if qx > qy else (0.0, sy)
    return sdf, (gx_l * ct - gy_l * st, gx_l * st + gy_l * ct)


def observe_rects(world: Any) -> list[Rect]:
    """只读地把当前帧里所有矩形障碍读出来（protocol v3 的几何字段，README :202）。"""
    pool = world.state.bullets
    d = pool.data
    out: list[Rect] = []
    for i in pool.active_indices():
        if int(d["shape"][i]) != SHAPE_RECT:
            continue
        out.append(
            Rect(
                ent_id=int(d["id"][i]),
                x=float(d["x"][i]), y=float(d["y"][i]),
                vx=float(d["vx"][i]), vy=float(d["vy"][i]),
                half_w=float(d["half_w"][i]), half_h=float(d["half_h"][i]),
                rotation=float(d["rotation"][i]),
                angular_velocity=float(d["angular_velocity"][i]),
            )
        )
    return out


def group_walls(rects: Sequence[Rect], px: float, py: float,
               forward: Optional[tuple[float, float]] = None) -> list[Wall]:
    """把矩形聚成"一堵墙"。

    **不能用 group_id**：实测本场景里所有墙段的 group_id 都是 0。
    改用几何分组：同一堵墙的各段在"墙厚方向"上坐标几乎相同，
    把该坐标量化到 2 个世界单位一档作为分组键（相邻墙之间 >= 100 单位，不会误并）。
    """
    buckets: dict[tuple[int, int], list[Rect]] = {}
    for r in rects:
        nx = -math.sin(r.rotation)
        ny = math.cos(r.rotation)
        band = r.x * nx + r.y * ny            # 墙厚方向的坐标
        key = (int(round(r.rotation * 1000.0)), int(round(band / 2.0)))
        buckets.setdefault(key, []).append(r)

    walls: list[Wall] = []
    for members in buckets.values():
        members = sorted(members, key=lambda r: r.x)
        ct, st = math.cos(members[0].rotation), math.sin(members[0].rotation)
        u_hat = (ct, st)                      # half_w 轴 = 墙的跨度方向
        # 规范化 û 让它大致指向 +x，这样 e_lo/e_hi、"左侧/右侧"才有稳定含义
        if u_hat[0] < -EPS or (abs(u_hat[0]) <= EPS and u_hat[1] < 0.0):
            u_hat = (-u_hat[0], -u_hat[1])
        n_hat = (-u_hat[1], u_hat[0])

        mx = sum(r.x for r in members) / len(members)
        my = sum(r.y for r in members) / len(members)
        to_player = (px - mx, py - my)
        if forward is not None:
            # 固定行驶轴的场景（小车朝 +forward 开）：n̂ 从墙指向车，恒等于 -forward。
            # **必须这样钉死**：静态墙的法向速度是 0，而"车在墙的哪一侧"在车正好
            # 走到墙这条线上时是退化的（点积 ≈ 0），于是每帧重新判定的 n̂ 会来回翻，
            # "前进"跟着隔帧反号，车就原地抖、永远过不去（实测卡在墙 y 上不动）。
            n_hat = (-forward[0], -forward[1])
        elif abs(_dot(n_hat, to_player)) > TOL:
            if _dot(n_hat, to_player) < 0.0:
                n_hat = (-n_hat[0], -n_hat[1])
        else:
            # 退化：玩家正好落在墙所在的那条线上，用墙自己的速度定向
            vx = sum(r.vx for r in members) / len(members)
            vy = sum(r.vy for r in members) / len(members)
            if _dot((vx, vy), n_hat) < 0.0:
                n_hat = (-n_hat[0], -n_hat[1])
        walls.append(Wall(rects=tuple(members[:2]), u_hat=u_hat, n_hat=n_hat))
    return walls


# ===========================================================================
# 4. 跨帧记忆：缺口轨迹 + "看不见的那一段"的补全依据
# ===========================================================================
@dataclass
class WallMemory:
    ring: deque = field(default_factory=lambda: deque(maxlen=24))
    gap_width_seen: Optional[float] = None      # 曾经亲眼看到过的缺口宽度
    gap_center_seen: Optional[float] = None     # 曾经亲眼看到过的缺口中心
    n_single_frames: int = 0                    # 只有一段可见的帧数（诊断用）


@dataclass
class GapMemory:
    """跨帧记忆。墙身份按"上一帧最近的墙"关联（不能用 min(ent_id)，见文件头第 4 条）。"""

    walls: dict[int, WallMemory] = field(default_factory=dict)
    global_gap_width: Optional[float] = None
    step_index: int = -1
    _next_key: int = 1
    _prev: list = field(default_factory=list)   # [(band, key)]
    last_dir: Optional[tuple[float, float]] = None   # 上一帧下发过的方向（转向限速用）

    def get(self, key: int) -> WallMemory:
        return self.walls.setdefault(key, WallMemory())

    def match_keys(self, bands: Sequence[float], tol: float = 24.0) -> list[int]:
        prev = list(self._prev)
        keys: list[int] = []
        for band in bands:
            pick = None
            for i, (pb, pk) in enumerate(prev):
                dd = abs(pb - band)
                if pick is None or dd < pick[0]:
                    pick = (dd, i, pk)
            if pick is not None and pick[0] <= tol:
                keys.append(pick[2])
                prev.pop(pick[1])              # 一堵旧墙只能被认领一次
            else:
                keys.append(self._next_key)
                self._next_key += 1
        self._prev = list(zip(bands, keys))
        return keys

    @property
    def single_segment_frames(self) -> int:
        return sum(w.n_single_frames for w in self.walls.values())


# ===========================================================================
# 5. 缺口重建
# ===========================================================================
@dataclass(frozen=True)
class Aperture:
    e_lo: float          # 可用区间的左端（沿 û，已按场地裁剪）
    e_hi: float          # 右端
    gap_raw: float       # 未裁剪的缺口宽度
    center: float        # 可用区间中心
    usable_half: float   # A = 可用宽度 / 2 - r_p - margin
    complete: bool       # True = 两段都看见了


def reconstruct_aperture(
    wall: Wall, mem: WallMemory, r_p: float, margin: float, field_w: float,
    global_gap: Optional[float] = None,
) -> Aperture:
    """从可见的矩形反算缺口。

    两段都在：e_lo / e_hi = 两条内缘取 min/max，gap = e_hi - e_lo。
    只有一段可见（实测会发生）：用"别处见过"的缺口宽度补出来 ——
      1. 优先用这堵墙自己历史上量到过的宽度；
      2. 否则用任意一堵墙量到过的宽度（同一场景 gap_width 是同一个参数）；
      3. 缺口在可见段的哪一侧：有本墙记忆就按记忆方位，没记忆就看哪一侧放得下。
    """
    rects = sorted(wall.rects, key=lambda r: _dot((r.x, r.y), wall.u_hat))
    u_hat = wall.u_hat

    if len(rects) >= 2:
        rl, rr = rects[0], rects[1]
        inner_a = _dot((rl.x, rl.y), u_hat) + _half_extent_along(rl, u_hat)
        inner_b = _dot((rr.x, rr.y), u_hat) - _half_extent_along(rr, u_hat)
        e_lo, e_hi = min(inner_a, inner_b), max(inner_a, inner_b)
        gap = e_hi - e_lo
        if gap > 0.0:
            mem.gap_width_seen = gap
            mem.gap_center_seen = 0.5 * (e_lo + e_hi)
        complete = True
    else:
        mem.n_single_frames += 1
        r = rects[0]
        proj = _dot((r.x, r.y), u_hat)
        half = _half_extent_along(r, u_hat)
        g_mem = mem.gap_width_seen if mem.gap_width_seen is not None else global_gap
        if g_mem is None or g_mem <= 0.0:
            return Aperture(0.0, 0.0, 0.0, 0.0, -math.inf, complete=False)

        if mem.gap_center_seen is not None:
            on_left = proj < mem.gap_center_seen
        else:
            free_left = proj - half
            free_right = field_w - (proj + half)
            if free_left + TOL >= g_mem and free_left >= free_right:
                on_left = False                      # 缺口在左边 => 可见的是右段
            elif free_right + TOL >= g_mem:
                on_left = True                       # 缺口在右边 => 可见的是左段
            else:
                return Aperture(0.0, 0.0, 0.0, 0.0, -math.inf, complete=False)

        if on_left:
            e_lo = proj + half
            e_hi = e_lo + g_mem
        else:
            e_hi = proj - half
            e_lo = e_hi - g_mem
        gap = e_hi - e_lo
        complete = False

    lo = max(e_lo, 0.0)
    hi = min(e_hi, field_w)
    usable_w = hi - lo
    return Aperture(lo, hi, gap, 0.5 * (lo + hi),
                    0.5 * usable_w - r_p - margin, complete)


# ===========================================================================
# 6. 决策
# ===========================================================================
@dataclass
class Decision:
    wx: float = 0.0
    wy: float = 0.0
    tier: str = "IDLE"
    reason: str = ""
    t_arr: float = math.inf
    clearance: float = math.inf
    gap: float = math.nan
    usable_half: float = math.nan
    e_lo: float = math.nan
    e_hi: float = math.nan
    aim_lat: float = math.nan
    s_hat: float = 0.0
    n_rects: int = 0
    gap_line: tuple[tuple[float, float], tuple[float, float]] = (
        (math.nan, math.nan), (math.nan, math.nan))
    aim_world: tuple[float, float] = (math.nan, math.nan)
    wall_band: float = math.nan


def _band_point(wall: Wall, lateral: float, band: float) -> tuple[float, float]:
    """（沿 û 的横向坐标, 沿 n̂ 的坐标）-> 世界坐标：P = lateral*û + band*n̂。"""
    ux, uy = wall.u_hat
    nx, ny = wall.n_hat
    return (lateral * ux + band * nx, lateral * uy + band * ny)


def plan(world: Any, mem: GapMemory, cfg: AlgoConfig, horizon_remaining: float) -> Decision:
    """每帧调用一次：世界状态 -> Decision。只读，不修改世界。"""
    p = world.player
    px, py = float(p[P_X]), float(p[P_Y])
    pvx, pvy = float(p[P_VX]), float(p[P_VY])
    r_p = float(p[P_RADIUS])
    v_max = float(p[P_SPEED])
    dt = float(world.state.env.dt)
    step = int(world.state.env.step_index)
    field_w = float(world.state.env.field_w)
    mem.step_index = step

    walls = group_walls(observe_rects(world), px, py, cfg.forward)
    if not walls:
        mem.match_keys(())
        return Decision(tier="IDLE", reason="no_wall")

    bands = [sum(_dot((r.x, r.y), w.n_hat) for r in w.rects) / len(w.rects)
             for w in walls]
    keys = mem.match_keys(bands)

    # --- 选最近的那堵正在逼近的墙；每一堵都要重建缺口（学习全局缺口宽度）---
    cands: list = []
    for w, key in zip(walls, keys):
        front = max(_dot((r.x, r.y), w.n_hat) + _half_extent_along(r, w.n_hat)
                    for r in w.rects)
        thick = 2.0 * max(_half_extent_along(r, w.n_hat) for r in w.rects)
        back = front - thick
        pn = _dot((px, py), w.n_hat)
        vn_wall = sum(_dot((r.vx, r.vy), w.n_hat) for r in w.rects) / len(w.rects)
        v_n = vn_wall - _dot((pvx, pvy), w.n_hat)

        # 必须按玩家半径膨胀墙带、分三段判断（见文件头第 5 条）
        near = front + r_p
        far = back - r_p
        if pn > near:
            t_arr = (pn - near) / v_n if v_n > EPS else math.inf
            prio, key_d = 1, pn - near           # 还在前面：越近越优先
        elif pn >= far:
            t_arr = 0.0                          # 玩家的圆正压在墙带上
            prio, key_d = 0, 0.0                 # 正压着它 => 必须继续盯住这一堵
        else:
            t_arr = math.inf                     # 整面墙已经在身后
            prio, key_d = 2, 0.0                 # 身后 => 不再是约束

        # 选目标墙**必须按位置**（prio/key_d），不能按 t_arr：
        # t_arr = d_n / v_n 里含**车自己的速度**。静态墙场景下，车只要往后动一点点
        # （v_n <= 0），所有墙的 t_arr 同时变成 inf，min() 就会随便挑一堵 ——
        # 目标在两道缺口之间来回跳（实测 ±175px），输出的速度方向隔帧反号，
        # 车原地抖 1.5 秒过不去，画面上就是"过完第一堵墙突然旋转"。
        # 位置判据是单调的，不含速度，永远不会这样退化。

        sdf = min(_rect_sdf(r, px, py)[0] for r in w.rects)
        clearance = sdf - r_p - cfg.safety_margin

        wmem = mem.get(key)
        ap_w = reconstruct_aperture(w, wmem, r_p, cfg.safety_margin, field_w,
                                    mem.global_gap_width)
        if ap_w.complete and ap_w.gap_raw > 0.0:
            mem.global_gap_width = ap_w.gap_raw
        cands.append((w, key, t_arr, clearance, v_n, front, thick, ap_w,
                      prio, key_d))

    (wall, key, t_arr, clearance, v_n, front, thickness, ap, _p, _d) = min(
        cands, key=lambda c: (c[8], c[9]))
    wmem = mem.get(key)
    lat = _dot((px, py), wall.u_hat)

    if ap.usable_half <= 0.0:
        # 推不出缺口 / 膨胀后装不下人：朝远离最近那段墙的方向横移兜底，
        # 别把自己顶到场地边界上。
        r_near = min(wall.rects, key=lambda r: _rect_sdf(r, px, py)[0])
        proj_near = _dot((r_near.x, r_near.y), wall.u_hat)
        sgn = 1.0 if lat < proj_near else -1.0
        if not (0.0 <= lat + sgn * 40.0 <= field_w):
            sgn = -sgn
        return Decision(wx=sgn * wall.u_hat[0], wy=sgn * wall.u_hat[1],
                        tier="NO_GAP", reason="aperture_unknown_flee",
                        t_arr=t_arr, clearance=clearance, gap=ap.gap_raw,
                        usable_half=ap.usable_half, e_lo=ap.e_lo, e_hi=ap.e_hi,
                        n_rects=len(wall.rects),
                        gap_line=(_band_point(wall, ap.e_lo, front),
                                  _band_point(wall, ap.e_hi, front)))

    # --- 缺口轨迹：按墙分别估计（线性拟合 + 残差包络）---
    wmem.ring.append((step, ap.center))
    s_hat, resid = 0.0, 0.0
    if len(wmem.ring) >= 2:
        t0, _s0 = wmem.ring[0]
        n = len(wmem.ring)
        st = stt = ss = sts = 0.0
        for (ti, si) in wmem.ring:
            tt = (ti - t0) * dt
            st += tt
            stt += tt * tt
            ss += si
            sts += tt * si
        den = n * stt - st * st
        s_hat = 0.0 if abs(den) < EPS else (n * sts - st * ss) / den
        b = (ss - s_hat * st) / n
        for (ti, si) in wmem.ring:
            resid = max(resid, abs(si - (b + s_hat * (ti - t0) * dt)))

    def s_lo(t: float) -> float:
        return ap.center + min(0.0, s_hat) * t - resid * t

    def s_hi(t: float) -> float:
        return ap.center + max(0.0, s_hat) * t + resid * t

    H = min(float(horizon_remaining), cfg.max_window)

    def worst_offset(t: float) -> float:
        return max(abs(s_lo(t) - lat), abs(s_hi(t) - lat)) + 0.5 * (s_hi(t) - s_lo(t))

    def reach(t: float) -> float:
        return v_max * max(0.0, t)          # README 判据 ②（本脚本不开加速度）

    def feasible(t: float) -> bool:
        off = worst_offset(t)
        return off <= ap.usable_half or (off - ap.usable_half) <= reach(t)

    # --- 瞄准与判定 ------------------------------------------------------
    # 关键判据（README :300）：墙到达我这条线时，我必须已经在缺口里。
    #
    # 瞄准时刻不取 t_arr 而取 min(t_arr, aim_lookahead)：缺口会动时，用线性拟合
    # 外推 t_arr（开局可达 4 秒）得到的位置毫无意义；短视距 + 每步重算 = 追踪。
    # 瞄准点用**拦截时间**求，而不是固定前瞻：
    #     T 满足 |s(T) - lat| = v_max * T     （车横移 T 秒到达，缺口也走了 T 秒）
    # 迭代几次就收敛（只要缺口速度 < 车速）。为什么必须这样：缺口在滑动时，
    # 固定前瞻 + 比例控制会有稳态滞后 —— 车永远差一截追不上（实测 drive 场景里
    # 车停在墙前左右追着缺口跑、始终不进去）。缺口静止时 s(T) 恒等于 s0，
    # 结果与原来完全一致，所以 incoming 场景的基线不受影响。
    t_aim = abs(0.5 * (s_lo(0.0) + s_hi(0.0)) - lat) / max(v_max, EPS)
    for _ in range(4):
        cand = abs(0.5 * (s_lo(t_aim) + s_hi(t_aim)) - lat) / max(v_max, EPS)
        t_aim = cand if not math.isfinite(t_arr) else min(t_arr, cand)
    aim_lat = 0.5 * (s_lo(t_aim) + s_hi(t_aim))
    d_lat = aim_lat - lat
    need = max(0.0, abs(d_lat) - ap.usable_half)
    sgn_lat = 1.0 if d_lat >= 0 else -1.0

    band = front
    gl = (_band_point(wall, ap.e_lo, band), _band_point(wall, ap.e_hi, band))

    # 横向分量（分级）。这里不再直接 return，因为下面还要叠加前后分量。
    if abs(d_lat) <= cfg.dead_frac * ap.usable_half:
        tier, reason, u_mag = "HOLD", "gap_will_pass_me", 0.0
    elif abs(d_lat) <= cfg.align_frac * ap.usable_half:
        u_mag = min(1.0, abs(d_lat) / max(ap.usable_half, EPS))
        tier, reason = "TRIM", f"recenter dlat={d_lat:+.1f}"
    else:
        u_mag, tier, reason = 1.0, "ALIGN", f"move_to_gap dlat={d_lat:+.1f}"

    # 纯横向来不来得及。注意：前后可控时"减速/停住"能把这条救回来，
    # 所以那种情况下不判 PANIC —— 那正是前后移动的价值所在。
    if math.isfinite(t_arr) and need > reach(t_arr) and not cfg.front_back:
        u_mag, tier, reason = 1.0, "PANIC", "unreachable_before_arrival"

    # --- 前后分量 -------------------------------------------------------
    # n̂ 从墙指向车 => 前进（朝墙）= −n̂，后退（远离墙）= +n̂。
    fwd = (-wall.n_hat[0], -wall.n_hat[1])
    d_n = max(0.0, _dot((px, py), wall.n_hat) - (front + r_p))   # 车到墙的净距离

    u_speed = u_mag * v_max
    w_cap = math.sqrt(max(0.0, v_max * v_max - u_speed * u_speed))  # 剩余速度预算
    if cfg.front_back:
        t_align = need / max(u_speed, EPS) if need > 1e-6 else 0.0
        room = max(0.0, d_n - cfg.hold_clearance)
        t_reach = room / max(w_cap, EPS)
        if t_reach >= t_align:
            w = w_cap                                   # 时间够：全速通过
            if tier == "PANIC":
                tier, reason = "ALIGN", reason
        else:
            # 时间不够：减速到"恰好对齐完成时抵达"，必要时直接停住等
            w = min(w_cap, room / max(t_align, EPS))
            tier = "SLOW" if w > 1e-6 else "STOP"
            reason = f"hold_to_align need={need:.1f} t_align={t_align:.2f}"
    else:
        # 不允许前后控制：**油门是固定的**（车不能停、不能减速），横向只能用剩下的预算。
        # 这才是"只会左右移动的车"的诚实模型：|v| <= v_max 是同一份预算。
        w = min(cfg.fwd_speed * v_max, v_max)
        u_cap = math.sqrt(max(0.0, v_max * v_max - w * w))
        u_mag = min(u_mag, u_cap / max(v_max, EPS))
        if cfg.fwd_speed > 0.0:
            tier, reason = "CRUISE", reason

    w_frac = w / v_max
    wx = u_mag * sgn_lat * wall.u_hat[0] + w_frac * fwd[0]
    wy = u_mag * sgn_lat * wall.u_hat[1] + w_frac * fwd[1]

    # 转向限速：把这一帧的方向相对上一帧的夹角限住。这是"真车做不出来"的那一步。
    if cfg.max_turn_rate > 0.0 and mem.last_dir is not None:
        mag = math.hypot(wx, wy)
        if mag > EPS:
            a_new = math.atan2(wy, wx)
            a_old = math.atan2(mem.last_dir[1], mem.last_dir[0])
            d = (a_new - a_old + math.pi) % (2.0 * math.pi) - math.pi   # 归一化到 (-pi, pi]
            lim = math.radians(cfg.max_turn_rate) * dt
            if abs(d) > lim:
                a_new = a_old + math.copysign(lim, d)
                wx, wy = mag * math.cos(a_new), mag * math.sin(a_new)
    if math.hypot(wx, wy) > EPS:
        mem.last_dir = (wx, wy)

    return Decision(wx=wx, wy=wy, tier=tier, reason=reason,
                    t_arr=t_arr, clearance=clearance, gap=ap.gap_raw,
                    usable_half=ap.usable_half, e_lo=ap.e_lo, e_hi=ap.e_hi,
                    aim_lat=aim_lat, s_hat=s_hat, n_rects=len(wall.rects),
                    gap_line=gl, aim_world=_band_point(wall, aim_lat, band),
                    wall_band=band)


def to_action(dec: Decision) -> Action:
    """Decision -> Action（README :174：Action 是对外唯一动作表示）。"""
    if math.hypot(dec.wx, dec.wy) < EPS:
        return Action.zero()
    return Action.from_vector((dec.wx, dec.wy))


# ===========================================================================
# 7. 可视化：在仓库自带渲染器上叠一层"算法内部量"
# ===========================================================================
class AlgoRenderer(PygameRenderer):
    """复用 bullet_sim 的 pygame 渲染器，只在 HUD 之后补画算法叠加层。

    render() 顺序：... -> _draw_obstacles -> _draw_player -> _draw_hud -> flip()。
    把叠加层挂在 _draw_hud 末尾就能在**同一次 flip** 里画出来，不会闪烁。
    """

    def __init__(self, **kw: Any) -> None:
        super().__init__(**kw)
        self.decision: Decision = Decision()
        self.trail: list[tuple[float, float]] = []
        self.episode_label: str = ""
        self._algo_font: Any = None

    def _font_algo(self) -> Any:
        if self._algo_font is None:
            import pygame
            try:
                self._algo_font = pygame.font.SysFont("monospace", 13, bold=True)
            except Exception:            # pragma: no cover
                self._algo_font = pygame.font.Font(None, 15)
        return self._algo_font

    def _draw_hud(self, screen: Any, world: Any, fps: float) -> None:
        super()._draw_hud(screen, world, fps)
        self._draw_algo_overlay(screen, world)

    def _draw_algo_overlay(self, screen: Any, world: Any) -> None:
        import pygame

        vp = self.viewport
        if vp is None:
            return
        d = self.decision

        # 玩家轨迹
        if len(self.trail) >= 2:
            pts = [vp.world_to_screen(x, y) for (x, y) in self.trail]
            for i in range(1, len(pts)):
                a = i / len(pts)
                col = (int(60 + 150 * a), int(70 + 70 * a), 90)
                pygame.draw.line(screen, col, pts[i - 1], pts[i], 1)

        # 缺口的可用区间：墙上一条粗绿线
        p0w, p1w = d.gap_line
        if math.isfinite(p0w[0]) and math.isfinite(p1w[0]):
            pygame.draw.line(screen, (60, 255, 120),
                             vp.world_to_screen(*p0w), vp.world_to_screen(*p1w), 3)

        # 瞄准点
        if math.isfinite(d.aim_world[0]):
            ap_pt = vp.world_to_screen(*d.aim_world)
            pygame.draw.circle(screen, (255, 220, 60),
                               (int(ap_pt[0]), int(ap_pt[1])), 6, 2)

        # 指令速度箭头
        pl = world.player
        if math.hypot(d.wx, d.wy) > 1e-6:
            nrm = math.hypot(d.wx, d.wy)
            ox, oy = vp.world_to_screen(float(pl[P_X]), float(pl[P_Y]))
            L = 44.0
            ex = ox + d.wx / nrm * L
            ey = oy - d.wy / nrm * L
            pygame.draw.line(screen, (255, 140, 40), (ox, oy), (ex, ey), 3)
            pygame.draw.circle(screen, (255, 140, 40), (int(ex), int(ey)), 4)

        # 文本面板
        f = self._font_algo()
        rows = [
            f"EPISODE {self.episode_label}",
            f"tier     : {d.tier}",
            f"reason   : {d.reason}",
            f"segs     : {d.n_rects}    t_arr: {d.t_arr:7.2f}s",
            f"clearance: {d.clearance:7.2f}",
            f"gap      : {d.gap:7.1f}",
            f"A(half)  : {d.usable_half:7.1f}",
            f"gap x    : [{d.e_lo:6.1f},{d.e_hi:6.1f}]",
            f"aim x    : {d.aim_lat:7.1f}",
            f"s_hat    : {d.s_hat:+7.2f} u/s",
        ]
        x0, y0 = 8, 8
        wpx = max(f.size(r)[0] for r in rows) + 12
        hpx = f.get_height() * len(rows) + 10
        panel = pygame.Surface((wpx, hpx), pygame.SRCALPHA)
        panel.fill((0, 0, 0, 170))
        screen.blit(panel, (x0, y0))
        tier_col = {"HOLD": (120, 255, 140), "ALIGN": (120, 200, 255),
                    "TRIM": (120, 255, 240), "PANIC": (255, 90, 90),
                    "NO_GAP": (255, 150, 90), "CROSS": (255, 200, 60)}.get(
                        d.tier, (200, 200, 200))
        for i, r in enumerate(rows):
            col = tier_col if r.startswith("tier") else (255, 255, 255)
            screen.blit(f.render(r, True, col), (x0 + 6, y0 + 5 + i * f.get_height()))


# ===========================================================================
# 8. 「小车向前行驶」赛道：静态墙 + 缺口左右移动
# ===========================================================================
def make_wall_layout(seed: int, *, walls: int, field_w: float, field_h: float,
                     gap_width: float, spacing: float, max_slide: float,
                     r_p: float, jitter: float = 0.25):
    """按种子生成一条**随机**赛道：墙的间隔、缺口位置、缺口滑动速度都随机。

    用 seed 驱动的 numpy Generator，所以**同种子必得同赛道**（可复现），
    换种子就换一套布局 —— 这是为了能拿一批不同场景去检验算法，而不是每次都跑
    同一条手搭的赛道。

    约束（不满足就是不可行的场景，不是算法的问题）：
      * 最后一堵墙必须留在可达范围内：车被夹在 y <= field_h - r_p，墙要留出冲过去的余量；
      * 缺口中心必须能让"膨胀后的口径"留在场内，否则等于没有缺口。
    """
    rng = random.Random(int(seed))    # 标准库即可：同种子必得同布局
    # 墙位：从 y=60 起按 spacing 累加，每段加一点抖动；并整体压缩到可达范围内
    usable = max(120.0, field_h - r_p - 80.0 - 60.0)
    step = min(spacing, usable / max(1, walls))
    # 先抽带抖动的间隔，再整体缩放，保证**最后一堵墙一定留在可达范围内**
    # （车被夹在 y <= field_h - r_p；墙后面要留出冲过去的余量，否则"通过"不可能）
    steps = [step * (1.0 + (rng.uniform(-jitter, jitter) if jitter > 0 else 0.0))
             for _ in range(int(walls))]
    run_out = 40.0
    cap = field_h - r_p - run_out - 60.0
    total = sum(steps)
    if total > cap > 0.0:
        steps = [t * cap / total for t in steps]
    ys: list[float] = []
    y = 60.0
    for st in steps:
        y += st
        ys.append(round(float(y), 1))
    # 缺口中心：让膨胀后的可用区间完整落在场内
    lo = gap_width / 2.0 + r_p + 6.0
    hi = field_w - gap_width / 2.0 - r_p - 6.0
    centers = [round(float(rng.uniform(lo, max(lo, hi))), 1) for _ in range(int(walls))]
    # 每堵墙的滑动速度与方向都随机
    slides = [round(float(rng.uniform(-max_slide, max_slide)), 1)
              for _ in range(int(walls))]
    return ys, centers, slides


def _spawn_wall_pair(pool: Any, field_w: float, y: float, center: float,
                     gap: float, half_h: float) -> tuple[int, int]:
    """在 y 处放一堵墙：左段 [0, lo] + 右段 [hi, field_w]，缺口 [lo, hi]。

    返回两段的实体 id（不是槽位号 —— 槽位会被 compact 挪动，id 不会）。
    """
    lo = center - gap / 2.0
    hi = center + gap / 2.0
    out = []
    for (cx, hw) in ((lo / 2.0, lo / 2.0),
                     (hi + (field_w - hi) / 2.0, (field_w - hi) / 2.0)):
        idx = pool.spawn(cx, y, 0.0, 0.0, shape=1, half_w=hw, half_h=half_h,
                         radius=half_h, ttl=float("inf"), type_id=9, group_id=0)
        out.append(int(pool.data["id"][idx[0]]))
    return out[0], out[1]


@dataclass
class DriveCourse:
    """N 堵**沿行驶方向静止**的墙，缺口可以左右滑动。

    为什么非要有这个场景：现有的 wall_with_gap 场景是"墙朝静止的车压过来"
    （墙有法向速度）。那种几何下"前进"就是撞墙，所以上一轮无论怎么测都得出
    "前后移动没用"。而真实小车是**车向前开、墙不动**，前进是唯一的推进手段，
    而且"停在墙前面把缺口对准再走"因此变成可用策略 —— 这正是需求里说的
    "迅速过前两道墙，然后留更多时间过第三道墙"。

    注入方式：清空生成器给的实体，自己用 pool.spawn(shape=1, ...) 放 2N 段矩形。
    演示级手搭场景，不走 scenarios/generators 那套管线。
    """

    field_w: float
    ys: tuple[float, ...]
    gap_width: float
    centers: list[float]
    slides: list = field(default_factory=list)   # 每堵墙各自的缺口横向速度（px/s）
    half_h: float = 4.0
    ids: list = field(default_factory=list)
    passed: list = field(default_factory=list)
    _t0: float = 0.0

    def build(self, world: Any) -> None:
        pool = world.state.bullets
        pool.clear()                                  # 干掉生成器排好的墙
        self.ids = []
        self.passed = [False] * len(self.ys)
        self.centers = [min(max(c, self.gap_width / 2.0 + 1.0),
                            self.field_w - self.gap_width / 2.0 - 1.0)
                        for c in self.centers]
        for y, c in zip(self.ys, self.centers):
            self.ids.append(_spawn_wall_pair(pool, self.field_w, y, c,
                                             self.gap_width, self.half_h))
        self.tick(world, 0.0)

    def tick(self, world: Any, sim_time: float) -> None:
        """按时间推进缺口的横向位置（外缘钉在场边，内缘平移）。"""
        pool = world.state.bullets
        d = pool.data
        slot_of = {int(d["id"][i]): i for i in pool.active_indices()}
        for k, (y, ids) in enumerate(zip(self.ys, self.ids)):
            sl = self.slides[k] if k < len(self.slides) else 0.0
            c = self.centers[k] + sl * (sim_time - self._t0)
            c = min(max(c, self.gap_width / 2.0 + 1.0),
                    self.field_w - self.gap_width / 2.0 - 1.0)
            lo = c - self.gap_width / 2.0
            hi = c + self.gap_width / 2.0
            il, ir = ids
            if il in slot_of:
                i = slot_of[il]
                d["x"][i] = lo / 2.0
                d["half_w"][i] = lo / 2.0
            if ir in slot_of:
                i = slot_of[ir]
                d["x"][i] = hi + (self.field_w - hi) / 2.0
                d["half_w"][i] = (self.field_w - hi) / 2.0

    def update_passed(self, world: Any) -> int:
        """车的前沿越过某堵墙 => 记一次通过。返回已通过数。"""
        py = float(world.player[P_Y])
        r_p = float(world.player[P_RADIUS])
        for k, y in enumerate(self.ys):
            if not self.passed[k] and py > y + self.half_h + r_p:
                self.passed[k] = True
        return sum(self.passed)

    def next_wall_y(self) -> Optional[float]:
        for k, y in enumerate(self.ys):
            if not self.passed[k]:
                return y
        return None


# ===========================================================================
# 8. 主程序：连跑 N 局
# ===========================================================================
def build_spec(seed: int, gap_width: float, seconds: float, obstacle_speed: float,
               player_y: Optional[float] = None,
               player_speed: Optional[float] = None,
               interval: float = 1.6):
    sc = ObstacleScenario(
        name="wall_with_gap",
        seed=seed,
        dt=SIM_DT,
        field_w=640.0,
        field_h=480.0,
        obstacles=[{
            "type": "wall_with_gap",
            "gap_width": float(gap_width),
            "speed": float(obstacle_speed),      # README :417 --obstacle-speed
            # drive 场景自己放墙，所以把生成器的发射间隔拉到无穷，别来捣乱
            "interval": float(interval),
        }],
        duration=float(seconds),
        # README :353：障碍场景玩家半径默认 10 => d = 20
        player_hitbox_radius=10.0,
        # 玩家出生高度：默认 None => 障碍场景的 field_h*0.15 = 72。
        # 身后能退多远完全由它决定，所以做"前后移动"实验时必须能改。
        player_y=player_y,
        player_speed=float(player_speed if player_speed is not None else 200.0),
        require_valid=False,
    )
    return sc.to_spec()


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="wall_with_gap 避障算法可视化（临时脚本）")
    ap.add_argument("--scene", choices=("incoming", "drive"), default="incoming",
                    help="incoming = 墙朝静止的车压过来（现有 wall_with_gap 场景）；"
                         "drive = 车朝静止的墙开过去（需求里的真实模型）")
    ap.add_argument("--walls", type=int, default=3, help="drive 场景的墙数")
    ap.add_argument("--wall-spacing", type=float, default=110.0,
                    help="drive 场景墙间距（默认 110 => 墙在 y=170/280/390）")
    ap.add_argument("--gap-slide", type=float, default=40.0,
                    help="drive 场景缺口横向速度的**上限**（px/s）；"
                         "每堵墙的速度和方向在 ±这个值里随机取")
    ap.add_argument("--wall-seed", type=int, default=None,
                    help="墙布局的随机种子。不给就用本局种子（每局不同）")
    ap.add_argument("--wall-jitter", type=float, default=0.25,
                    help="墙间距的随机抖动比例（0 = 等间距）")
    ap.add_argument("--quiet-layout", action="store_true",
                    help="不打印每局的墙布局")
    ap.add_argument("--player-speed", type=float, default=None,
                    help="玩家的最大速度（默认用场景值 200）")
    ap.add_argument("--front-back", dest="front_back", action="store_true", default=None,
                    help="允许控制前后速度（含停住）。默认：drive 开、incoming 关")
    ap.add_argument("--no-front-back", dest="front_back", action="store_false",
                    help="关掉前后控制（只左右移动）")
    ap.add_argument("--fwd-speed", type=float, default=None,
                    help="关掉前后控制时的恒定前进速度比例（默认 drive=1.0, incoming=0.0）")
    ap.add_argument("--hold-clearance", type=float, default=10.0,
                    help="对齐时与墙保持的净距离（世界单位）")
    ap.add_argument("--episodes", type=int, default=5, help="跑几局（默认 5）")
    ap.add_argument("--seed", type=int, default=3, help="第一局的种子，之后 +1")
    ap.add_argument("--gap-width", type=float, default=200.0)
    ap.add_argument("--obstacle-speed", type=float, default=90.0)
    ap.add_argument("--seconds", type=float, default=12.0, help="每局的仿真时长")
    ap.add_argument("--player-y", type=float, default=None,
                    help="玩家出生 y。默认 None = 场高的 15%%（=72，身后只有 62px 退路）；"
                         "设为 240 就是场地正中，身后有 230px")
    ap.add_argument("--sim-speed", type=float, default=1.0,
                    help="1.0 = 实时；0.5 = 半速慢放（只影响渲染，不影响决策）")
    ap.add_argument("--scale", type=float, default=1.3, help="窗口缩放")
    ap.add_argument("--margin", type=float, default=6.0, help="安全膨胀 safety_margin")
    ap.add_argument("--max-window", type=float, default=8.0)
    ap.add_argument("--gap-sweep-px", type=float, default=0.0,
                    help="让缺口真正扫动：正弦振幅（像素）。0 = 关（默认，也是现有代码的真实行为）")
    ap.add_argument("--gap-sweep-hz", type=float, default=0.3,
                    help="缺口扫动频率（Hz），默认 0.3 => 周期 3.3 秒")
    ap.add_argument("--max-turn-rate", type=float, default=0.0,
                    help="指令方向的最大转向角速度（度/秒）。0 = 不限（默认）")
    ap.add_argument("--allow-cross", action="store_true",
                    help="打开『已对齐就穿过去』的旧分支（默认关，实测是自杀）")
    ap.add_argument("--headless", action="store_true", help="不开窗口，只跑并打印统计")
    args = ap.parse_args(argv)

    # 前后控制/恒定前进速度的默认值随场景走
    front_back = (args.scene == "drive") if args.front_back is None else bool(args.front_back)
    fwd_speed = (args.fwd_speed if args.fwd_speed is not None
                 else (1.0 if args.scene == "drive" else 0.0))
    cfg = AlgoConfig(safety_margin=args.margin, max_window=args.max_window,
                     allow_cross=bool(args.allow_cross),
                     front_back=front_back, fwd_speed=fwd_speed,
                     hold_clearance=args.hold_clearance,
                     forward=((0.0, 1.0) if args.scene == "drive" else None),
                     max_turn_rate=float(args.max_turn_rate))
    render_fps = 60.0
    frame_dt = 1.0 / render_fps
    sim_per_frame = max(1, int(round(args.sim_speed / render_fps / SIM_DT)))

    renderer: Optional[AlgoRenderer] = None
    if not args.headless:
        renderer = AlgoRenderer(
            mode="human", scale=args.scale, show_hud=True, show_prediction=False,
            show_danger=False, interactive=False, input_source=None, controller=None,
            resizable=True, help_seconds=0.0,
        )

    rows: list[dict[str, Any]] = []
    tier_count: dict[str, int] = {}
    sweep_base: dict[int, float] = {}       # 每堵墙最初的缺口左缘（见 apply_gap_sweep）
    for ep in range(int(args.episodes)):
        seed = int(args.seed) + ep
        drive = (args.scene == "drive")
        spec = build_spec(seed, args.gap_width, args.seconds, args.obstacle_speed,
                          args.player_y, args.player_speed,
                          interval=(1e9 if drive else 1.6))
        # collision=None 是关键：形参默认 "circle" 会覆盖 spec.collision（= "obstacle"），
        # 那样旋转矩形的墙根本不做碰撞，玩家直接穿墙。
        env = BulletHellEnv(spec, seed=seed, codec="action", collision=None)
        trail: list[tuple[float, float]] = []
        try:
            env.reset(seed=seed)
            mem = GapMemory()
            course = None
            if drive:
                # 三堵静态墙，缺口横向滑动。车从 y=60 出发往 +y 开。
                # 墙的布局**按种子随机**（同种子可复现）。--wall-seed 不给就用本局种子。
                wall_seed = int(args.wall_seed) if args.wall_seed is not None else seed
                ys, centers, slides = make_wall_layout(
                    wall_seed, walls=int(args.walls), field_w=float(spec.field_w),
                    field_h=float(spec.field_h), gap_width=float(args.gap_width),
                    spacing=float(args.wall_spacing), max_slide=float(args.gap_slide),
                    r_p=float(spec.player_radius), jitter=float(args.wall_jitter))
                if not args.quiet_layout:
                    print(f"    墙布局 seed={wall_seed}: "
                          + "  ".join(f"#{i+1} y={y:.0f} 缺口x={c:.0f} 滑={s:+.0f}"
                                      for i, (y, c, s) in enumerate(zip(ys, centers, slides))))
                course = DriveCourse(field_w=float(spec.field_w), ys=tuple(ys),
                                     gap_width=float(args.gap_width),
                                     centers=list(centers), slides=list(slides))
                course.build(env.world)
            steps_done = 0
            total = int(spec.total_steps)
            if renderer is not None:
                sweep_note = (f"  sweep±{args.gap_sweep_px:.0f}px"
                              if args.gap_sweep_px > 0 else "")
                renderer.episode_label = f"{ep + 1}/{args.episodes}  seed={seed}"
                renderer.trail = trail
                renderer.mode_banner = (f"EPISODE {ep + 1}/{args.episodes}  seed={seed}"
                                       f"  gap={args.gap_width:.0f}{sweep_note}")
                renderer.mode_banner_until = time.monotonic() + 2.5

            dec = Decision()
            while steps_done < total:
                if renderer is not None and renderer.closed:
                    break
                # 决策每个仿真步都重算，渲染才按 --sim-speed 抽样。
                for _ in range(sim_per_frame):
                    if steps_done >= total:
                        break
                    remaining = (total - steps_done) * float(spec.dt)
                    dec = plan(env.world, mem, cfg, remaining)
                    tier_count[dec.tier] = tier_count.get(dec.tier, 0) + 1
                    raw = action_to_codec_input(
                        to_action(dec), env.world.codec,
                        float(env.world.player[P_SPEED]),
                    )
                    env.step(raw)
                    steps_done += 1
                    if course is not None:
                        course.tick(env.world, float(env.world.state.env.sim_time))
                        passed = course.update_passed(env.world)
                        if passed >= len(course.ys):
                            break          # 三堵都过了，本局结束
                    else:
                        # 手动让缺口扫动（现有代码里 gap_motion 是死参数，见文件头）
                        apply_gap_sweep(env.world, args.gap_sweep_px, args.gap_sweep_hz,
                                        float(env.world.state.env.sim_time),
                                        float(spec.field_w), sweep_base)
                    trail.append((float(env.world.player[P_X]),
                                  float(env.world.player[P_Y])))
                    if len(trail) > 1200:
                        del trail[: len(trail) - 1200]
                if renderer is not None:
                    renderer.decision = dec
                    renderer.render(env.world)
                    time.sleep(frame_dt)

            m = env.episode_metrics()
            rows.append({
                "ep": ep + 1, "seed": seed,
                "collisions": int(m.get("total_collision_count", 0)),
                "reward": float(m.get("cumulative_reward", 0.0)),
                "steps": steps_done,
                "single": mem.single_segment_frames,
                "passed": (course.update_passed(env.world) if course else -1),
                "n_walls": (len(course.ys) if course else -1),
            })
            print(f"[ep {ep + 1}/{args.episodes}] seed={seed} "
                  f"collisions={rows[-1]['collisions']} "
                  f"steps={steps_done}/{total} "
                  f"reward={rows[-1]['reward']:+.1f} "
                  + (f"通过墙 {rows[-1]['passed']}/{rows[-1]['n_walls']}  "
                     if course else "")
                  + f"single_segment_frames={rows[-1]['single']}")
        finally:
            env.close()
        if renderer is not None and renderer.closed:
            break

    if renderer is not None:
        renderer.close()

    print()
    print("=" * 68)
    print(f"  统计（gap_width={args.gap_width:.0f}，墙速={args.obstacle_speed:.0f}，"
          f"margin={args.margin:.1f}，缺口扫动±{args.gap_sweep_px:.0f}px"
          f"@{args.gap_sweep_hz:.2f}Hz）")
    print("=" * 68)
    tot = 0
    for r in rows:
        tot += r["collisions"]
        print(f"  第 {r['ep']} 局  seed={r['seed']:3d}  碰撞 {r['collisions']:3d} 次   "
              f"奖励 {r['reward']:+8.1f}   仿真 {r['steps'] * SIM_DT:5.2f}s"
              + (f"   通过 {r['passed']}/{r['n_walls']} 堵" if r["passed"] >= 0 else ""))
    print(f"  合计碰撞事件: {tot}")
    print("  决策分布   : " + ", ".join(
        f"{k}={v}" for k, v in sorted(tier_count.items(), key=lambda kv: -kv[1])))
    print("=" * 68)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
