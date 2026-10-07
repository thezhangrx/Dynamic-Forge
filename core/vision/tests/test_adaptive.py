"""自适应分类 + 弃权的回归测试。

**完全自包含**：合成图直接用纯 Python 画进 ``bytearray``（和算法本身一样可以翻成
Verilog），不读文件、不用 numpy/opencv。

这里锁住四条性质，每一条都对应一次**实测踩到的错**：

1. **光照不变**：整体明暗/梯度变化时输出不变 —— 这是换掉绝对阈值的全部理由。
   绝对阈值在压暗到 0.2x 时会**全部判空**（场地和目标一起掉到阈值以下）。
2. **和绝对阈值在标准场景上逐像素等价**：换了分类层但不能改变平台上的既有行为。
3. **饱和必须弃权**：目标色度被截顶后信息就没了，这时报"没有目标"是最危险的失败。
4. **``unknown_frac`` 必须和 ``Masks.unknown`` 是同一个定义** ——
   早先 ``local_fields`` 自己数 ``ct == 0``，掩膜用的是 ``ct < min_contrast``，
   两个数不是一个东西，读出来的"弃权比例"全是错的。
"""

from __future__ import annotations

import pytest

from vision import primitives as P
from vision.adaptive import (
    NOISE_TILE,
    SCALE,
    AdaptiveParams,
    classify_adaptive,
    local_fields,
)

W, H = 640, 480
FIELD = (34, 40, 55)
WALL = (240, 160, 48)
PLAYER = (235, 240, 250)
TARGET = (110, 200, 90)

#: 三种目标在标准场景里的**真值像素数**（矩形/圆盘面积，直接数得出来）：
#: 墙 (600-60)x8、玩家 20x20、目标环 16x16。
TRUTH = (4320, 400, 256)


def _scene() -> bytearray:
    """一张标准合成图：一堵带缺口的墙 + 一个亮玩家 + 一个目标环。"""
    buf = bytearray(bytes(FIELD) * (W * H))

    def put(x: int, y: int, c: tuple[int, int, int]) -> None:
        if 0 <= x < W and 0 <= y < H:
            i = (y * W + x) * 3
            buf[i:i + 3] = bytes(c)

    for y in range(100, 108):
        for x in range(0, 600):
            if not (300 <= x < 360):        # 缺口
                put(x, y, WALL)
    for y in range(300, 320):
        for x in range(300, 320):
            put(x, y, PLAYER)
    for y in range(200, 216):
        for x in range(100, 116):
            put(x, y, TARGET)
    return buf


def _light(buf: bytearray, lo: float, hi: float) -> bytearray:
    """叠一层纵向光照斜坡/整体增益（模拟镜头与屏幕亮度不均）。"""
    out = bytearray(len(buf))
    for y in range(H):
        s = lo + (hi - lo) * y / (H - 1)
        row = y * W * 3
        for i in range(row, row + W * 3):
            v = int(buf[i] * s)
            out[i] = 255 if v > 255 else v
    return out


def _frame(buf: bytearray) -> P.Frame:
    return P.Frame.from_rgb(W, H, buf)


def _counts(m: P.Masks) -> tuple[int, int, int]:
    return sum(m.wall), sum(m.bright), sum(m.green)


def _pixels_equal(a: P.Masks, b: P.Masks, n: int) -> bool:
    return all(getattr(a, k)[i] == getattr(b, k)[i]
               for k in ("wall", "bright", "green") for i in range(n))


# --------------------------------------------------------------------------
# 1. 光照不变性 —— 换掉绝对阈值的全部理由
# --------------------------------------------------------------------------
@pytest.mark.parametrize("lo,hi", [
    (1.0, 1.0),          # 无梯度
    (0.35, 1.25),        # 3 倍梯度（和实拍量到的 2.9 倍同量级）
    (0.2, 0.2),          # 整体压暗 5 倍
    (0.5, 0.5),          # 整体压暗 2 倍
])
def test_adaptive_survives_illumination(lo: float, hi: float) -> None:
    """整体明暗 / 梯度变化时，三张掩膜的像素数必须**一模一样**。

    注意参数都取"不会把任何目标推过 254"的增益。这不是为了凑测试：
    一旦目标截顶，色度信息就真的没了（见
    :func:`test_uniform_brightening_destroys_green_target`），
    那时候"不变"本来就不该成立。
    """
    m, _ = classify_adaptive(_frame(_light(_scene(), lo, hi)), AdaptiveParams())
    assert _counts(m) == TRUTH


def test_uniform_brightening_destroys_green_target() -> None:
    """整体提亮 3 倍后目标环被洗成纯白 —— 必须**认不出来**，而不是硬判。

    ``(110,200,90) * 3`` 三通道全部截顶到 255，绿色信息归零。
    绝对阈值在这里会把绿色**误判成玩家**（实测 bright 从 400 涨到 656），
    自适应则是"没有绿色"—— 因为剩下的确实只有白色。
    """
    f = _frame(_light(_scene(), 3.0, 3.0))
    m, _ = classify_adaptive(f, AdaptiveParams())
    assert sum(m.green) == 0
    # 反面对照：绝对阈值把洗白的绿环判成了亮目标
    assert sum(P.classify(f).bright) > TRUTH[1]


def test_absolute_thresholds_collapse_when_dark() -> None:
    """反面对照：绝对阈值在压暗 5 倍后**全部判空**，所以必须换。"""
    dark = _frame(_light(_scene(), 0.2, 0.2))
    assert _counts(P.classify(dark)) == (0, 0, 0)
    # 同一个场景，自适应照样完全正确
    m, _ = classify_adaptive(dark, AdaptiveParams())
    assert _counts(m) == TRUTH


# --------------------------------------------------------------------------
# 2. 和绝对阈值在标准场景上逐像素等价
# --------------------------------------------------------------------------
def test_matches_absolute_on_standard_scene() -> None:
    """标准场景下自适应必须**逐像素**等于绝对阈值 —— 换分类层不能改既有行为。"""
    f = _frame(_scene())
    m, _ = classify_adaptive(f, AdaptiveParams())
    a = P.classify(f)
    assert _pixels_equal(m, a, f.n)


# --------------------------------------------------------------------------
# 3. 饱和必须弃权（而不是自信地报"没有目标"）
# --------------------------------------------------------------------------
def test_saturated_target_is_abstained_not_denied() -> None:
    """提亮到色度被截顶时，目标区域必须进 ``unknown``，而不是被判成背景。

    实测把场景提亮 4 倍，墙 ``(240,160,48) -> (255,255,192)``，色差从 192 掉到
    63；目标环被洗成纯白。信息是被**饱和毁掉的**，报"干净场地"就是撒谎。
    """
    f = _frame(_light(_scene(), 4.0, 4.0))
    m, fields = classify_adaptive(f, AdaptiveParams())
    assert fields.unknown_frac > 0.0
    # 墙原本所在的位置必须被标成 unknown（既不是墙，也不是"自信的背景"）
    y, x = 104, 200
    assert m.wall[y * W + x] == 0
    assert m.unknown[y * W + x] == 1
    # 目标环被洗白后不能算成"玩家"
    assert sum(m.green) == 0


# --------------------------------------------------------------------------
# 4. unknown_frac 与掩膜同源
# --------------------------------------------------------------------------
@pytest.mark.parametrize("lo,hi", [(1.0, 1.0), (4.0, 4.0), (0.35, 1.25)])
def test_unknown_frac_agrees_with_mask(lo: float, hi: float) -> None:
    """``unknown_frac`` 必须**恰好**是 ``Masks.unknown`` 的占比（同一个阈值）。"""
    f = _frame(_light(_scene(), lo, hi))
    m, fields = classify_adaptive(f, AdaptiveParams())
    assert fields.unknown_frac == pytest.approx(sum(m.unknown) / f.n)


def test_local_fields_does_not_guess_unknown_frac() -> None:
    """``local_fields`` 不知道真实判据，所以它**不许**自己填 ``unknown_frac``。"""
    fields = local_fields(_frame(_scene()))
    assert fields.unknown_frac == 0.0


# --------------------------------------------------------------------------
# 噪声标量：必须是"噪声"，不是边界落差
# --------------------------------------------------------------------------
def test_thin_target_survives_per_pixel_structure() -> None:
    """比噪声瓦片还薄的目标（3 px 墙），逐像素 ``noise`` 在它身上很大。

    ``noise`` 是 4 px 窗的 min/max 再模糊，**跨边界的瓦片**才有非零值；
    墙一旦薄于 4 px，墙上的每个像素都落在跨边界瓦片上。实测这里
    ``noise=61``，而整帧噪声标量是 0。拿逐像素那个当门限，
    等于要求"余量超过整个边沿高度" —— 薄目标会**整条**被判成弃权。
    """
    buf = bytearray(bytes(FIELD) * (W * H))
    for y in range(100, 103):                    # 3 px 厚，薄于 NOISE_TILE
        for x in range(100, 500):
            i = (y * W + x) * 3
            buf[i:i + 3] = bytes(WALL)
    f = _frame(buf)
    m, fields = classify_adaptive(f, AdaptiveParams())
    x, y = 300, 101
    assert fields.noise[y * W + x] > 50          # 逐像素结构幅度很大
    assert fields.noise_level < 20               # 噪声标量很小
    assert m.wall[y * W + x] == 1                # 所以墙照样被判出来
    assert sum(m.wall) == 400 * 3


def test_noise_tile_constant_is_apriltag_size() -> None:
    """噪声窗用 apriltag 的 4 px：这么小的窗内光照几乎不变。"""
    assert NOISE_TILE == 4


# --------------------------------------------------------------------------
# 判据方向：三个条件是"与"，余量必须取 **min**
# --------------------------------------------------------------------------
def test_wall_needs_warm_chroma_not_just_bright() -> None:
    """白色（r-b == 0）**不能**算墙。曾经写成 ``if m < other: m = other``
    取到了最大值，余量被抬成正的，于是任何亮东西都成了墙。"""
    buf = bytearray(bytes(FIELD) * (W * H))
    for y in range(50, 58):
        for x in range(50, 250):
            i = (y * W + x) * 3
            buf[i:i + 3] = bytes((250, 250, 250))     # 纯白长条
    f = _frame(buf)
    m, _ = classify_adaptive(f, AdaptiveParams())
    assert sum(m.wall) == 0
    assert sum(m.bright) > 0        # 它是亮目标，不是墙


def test_green_needs_green_chroma() -> None:
    """玩家（近白）不能算目标环。"""
    f = _frame(_scene())
    m, _ = classify_adaptive(f, AdaptiveParams())
    py, px = 310, 310
    assert m.bright[py * W + px] == 1
    assert m.green[py * W + px] == 0


# --------------------------------------------------------------------------
# 注入 detect：换的只是分类层
# --------------------------------------------------------------------------
def test_detect_accepts_injected_classifier() -> None:
    """``detect(classifier=...)`` 换分类层，下游一字不动。"""
    from vision.wall_with_gap import detect

    f = _frame(_scene())

    def adaptive(fr: P.Frame) -> P.Masks:
        masks, _ = classify_adaptive(fr, AdaptiveParams())
        return masks

    base = detect(f, name="abs")
    alt = detect(f, name="adaptive", classifier=adaptive)
    # 真值：墙带 y 落在 100..108，缺口中心 x=330 附近
    assert base.player is not None and alt.player is not None
    assert (base.player.x, base.player.y) == (alt.player.x, alt.player.y)
    assert len(base.walls) == len(alt.walls) == 1
    assert base.walls[0].gap is not None and alt.walls[0].gap is not None
    assert base.walls[0].gap.cx == pytest.approx(alt.walls[0].gap.cx, abs=0.5)
    assert base.walls[0].gap.width_px == pytest.approx(alt.walls[0].gap.width_px, abs=0.5)


# --------------------------------------------------------------------------
# 定点算术：判据里不许出现除法（要能翻成 Verilog）
# --------------------------------------------------------------------------
def test_scale_is_fixed_point() -> None:
    assert SCALE == 100
