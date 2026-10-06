"""Catalogue of real-world dynamic obstacle types.

Every entry answers the four questions the specification demands, and the
answers are what ``describe()``, the CLI and the README print - there is one
source of truth, not a document that drifts from the code.

Design rule: **start from the real situation, not from a pretty pattern.**
Only obstacles that simulate something real are kept.  Decorative
centre-fired patterns ("radial", "spiral", "aimed", "burst", "line", "wall",
"random") have been removed: the environment is a dynamic-obstacle simulator,
not a bullet-hell game.

The player is a **circle** whose drawn size equals its collision size, so every
``size`` here is quoted as a multiple of the player *diameter*.
"""

from __future__ import annotations

from typing import Any

from bullet_sim.obstacles.spec import ObstacleType

# --------------------------------------------------------------------------
# 1. moving block
# --------------------------------------------------------------------------
MOVING_BLOCK = ObstacleType(
    key="moving_block",
    label="Moving Block",
    label_zh="移动大方块",
    simulates=(
        "运动过程中迎面或横向接近的大型物体：前方突然出现的大型障碍、"
        "移动的机器/车辆/设备、建筑或货架被推着移动。"
    ),
    motion=(
        "从场地**边界的一段区间**进入（不是从一个点冒出），沿该边的法线方向"
        "以恒定速度朝场内平移，可以配置进入区间的宽度与进入角。"
        "尺寸与玩家圆形成明确倍数关系（默认 2× 玩家直径）。"
    ),
    player_problem=(
        "必须在物体到达之前提前判断绕行方向：与玩家发生相对运动的物体"
        "不能靠'撞到后再反应'处理，必须提前选择从哪一侧通过。"
    ),
    layout="block",
    shape="rect",
    safety_strategy="bypass",
    #: ``entry_span`` = fraction of the edge the block may be born along.
    defaults={
        "spawn": "top",
        "min_player_distance": 0.0,
        "size": 2.0,
        "entry_span": 0.6,
    },
)

# --------------------------------------------------------------------------
# 2. wall with gap
# --------------------------------------------------------------------------
WALL_WITH_GAP = ObstacleType(
    key="wall_with_gap",
    label="Wall With Gap",
    label_zh="有缺口墙",
    simulates=(
        "室内通道、建筑之间的狭窄缝隙、围栏开口：整体移动但只有局部可通行。"
        "（原先单独的 closing_gap 类型已删除：缺口收窄只是同一个墙的参数，"
        "不是一个新障碍。）"
    ),
    motion=(
        "由两段矩形组成一整面墙，中间保留宽度为 gap_width 的缺口；"
        "缺口宽度可调（**不得小于玩家圆直径**），位置可以固定或正弦扫动。"
        "墙整体匀速前进。"
    ),
    player_problem=(
        "必须先定位缺口，再判断'按当前速度能否在墙到达前抵达缺口'。"
        "这是几何可通行性 + 动态可达性的联合问题。"
    ),
    layout="wall_gap",
    shape="rect",
    safety_strategy="gap",
    defaults={
        "spawn": "top",
        "gap_ratio": 3.0,          # gap = 3 x the player diameter (world units
                                   # are derived in resolve_spec)
        "gap_position": 0.5,
        "gap_motion": "sweep",
    },
)

# --------------------------------------------------------------------------
# 3. small obstacles (sparse + dense merged)
# --------------------------------------------------------------------------
SMALL_OBSTACLES = ObstacleType(
    key="small_obstacles",
    label="Small Obstacles",
    label_zh="小型动态障碍",
    simulates=(
        "树林/复杂自然环境中的枝叶、漂浮物，或人群/车流中的细小动态物体。"
        "稀疏与密集只是同一个障碍的两个参数，因此合并为一个类型。"
    ),
    motion=(
        "尺寸远小于玩家的圆形障碍从各个边界进入，方向在进入方向附近随机扰动，"
        "彼此分散或密集（由 count / interval / size 决定），相对运动以"
        "'擦身而过'为主，而非正面对撞。"
    ),
    player_problem=(
        "在连续的小扰动中保持一条长期的可行通道；密度提高后自由空间被压缩成"
        "狭窄连通区域，重点是局部自由空间的连贯性而不是单次极限闪避。"
    ),
    layout="small",
    shape="circle",
    safety_strategy="local_free",
    defaults={"size": 0.35, "angle_jitter": 30.0, "count": 12},
)

# --------------------------------------------------------------------------
# 4. corridor (two walls symmetric about the player)
# --------------------------------------------------------------------------
CORRIDOR = ObstacleType(
    key="corridor",
    label="Corridor",
    label_zh="通道 / 道路变化",
    simulates=(
        "室内通道、建筑之间的狭长区域、树林中的飞行通道；"
        "以及'路面收窄 / 张开 / 转向'这类需要提前判断的通行问题。"
    ),
    motion=(
        "角色一开始就生成在两堵墙**正中间**，两堵墙以角色为中心对称分布，"
        "并**沿通道方向一直延伸到场地两端**（`wall_length` 可覆盖，默认自动跨场）；"
        "墙的**中心坐标不变**，只通过三种方式模拟道路变化："
        "(a) 张开 / 收缩（沿法线对称平移，宽度变化）；"
        "(b) 同向旋转（两堵墙一起转，通道整体倾斜，宽度不变）；"
        "(c) 反向旋转（两堵墙对转成剪刀口，角度被限制在安全范围内）。"
        "变化量按整段回放分摊成恒定速度/角速度，因此不会中途失控；"
        "最小间距被限制为玩家圆直径 ×1.25（**绝不压死角色**）；"
        "也可以只放一堵墙（walls=1）。"
    ),
    player_problem=(
        "在通道变窄 / 张开 / 偏转时判断'现在能否通过'、'是否需要等待'，"
        "并提前把位置放到变化后仍然安全的一侧。"
    ),
    layout="corridor",
    shape="rect",
    safety_strategy="corridor",
    defaults={
        "spawn": "left",
        "size": 1.5,                # wall thickness driver
        "corridor_ratio": 4.0,      # initial width = 4 x player diameter
        "corridor_min_ratio": 1.25, # gap floor = 1.25 x player diameter
        "walls": 2,                 # 1 = a single barrier, 2 = a symmetric channel
        "motion": "static",         # static|open|close|rotate_same|rotate_opposite
        "change": 0.0,              # open/close: width delta in player diameters
                                    # rotate_*: total rotation in degrees
        "wall_length_ratio": None,  # None = extend across the whole field
        "tilt_deg": 0.0,            # initial inclination of the whole channel
    },
)

# --------------------------------------------------------------------------
# 5. cross traffic
# --------------------------------------------------------------------------
CROSS_TRAFFIC = ObstacleType(
    key="cross_traffic",
    label="Cross Traffic",
    label_zh="多方向移动障碍",
    simulates=(
        "复杂动态交通：十字路口、多方向来流、"
        "周围多个独立移动物体同时靠近。"
    ),
    motion=(
        "障碍从四条（或指定几条）边界同时进入，各自沿进入方向横穿场地，"
        "彼此独立、速度可不同。相对运动方向各不相同。"
    ),
    player_problem=(
        "在多个不同方向的相对运动之间选择时机与间隙，"
        "本质上是'多方向流的间隙选择'问题。"
    ),
    layout="cross",
    shape="rect",
    safety_strategy="generic",
    defaults={"size": 0.9, "count": 8, "interval": 1.1},
)

#: The catalogue: only obstacle types that simulate something real.
CATALOG: dict[str, ObstacleType] = {
    t.key: t
    for t in (
        MOVING_BLOCK,
        WALL_WITH_GAP,
        SMALL_OBSTACLES,
        CORRIDOR,
        CROSS_TRAFFIC,
    )
}

#: Every catalogued type can be generated directly; compose several by listing
#: several ``--obstacle-type`` arguments (there is no "mixed" pseudo-type).
GENERATABLE: tuple[str, ...] = tuple(CATALOG)


def get_obstacle_type(key: str) -> ObstacleType:
    try:
        return CATALOG[str(key)]
    except KeyError as exc:
        raise KeyError(
            f"unknown obstacle type {key!r}; available: {sorted(CATALOG)}"
        ) from exc


def catalogue_summary() -> list[dict[str, Any]]:
    return [t.to_dict() for t in CATALOG.values()]


__all__ = [
    "CATALOG",
    "GENERATABLE",
    "MOVING_BLOCK",
    "WALL_WITH_GAP",
    "SMALL_OBSTACLES",
    "CORRIDOR",
    "CROSS_TRAFFIC",
    "get_obstacle_type",
    "catalogue_summary",
]
