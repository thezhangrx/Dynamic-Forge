"""任务一回归测试：参考物 → 坐标系（单应 + 尺度）。

不需要摄像头：全部用**合成对应点**验证。数学对不对可以离线判定，
这也是"相机不可用也能把代码写完"的前提。
"""

from __future__ import annotations

import math

import pytest

from vision.calibration import (
    FieldCalibration,
    Marker,
    apply_homography,
    calibrate_from_marker,
    detect_marker_quad,
    invert_homography,
    marker_channels,
    rms_error,
    solve_homography,
)

#: 造一个"斜视相机"：世界(米) → 图像(px)。有透视、有旋转。
H_TRUE = (620.0, 40.0, 320.0,
          -12.0, 520.0, 300.0,
          0.0012, 0.0007, 1.0)


def to_px(X: float, Y: float) -> tuple[float, float]:
    return apply_homography(H_TRUE, X, Y)


MARKER = Marker("test-marker", 0.30, 0.20)


# --------------------------------------------------------------------------
# 单应本身
# --------------------------------------------------------------------------
def test_four_points_recover_the_homography_exactly():
    src = [to_px(X, Y) for X, Y in MARKER.world_corners()]
    dst = list(MARKER.world_corners())
    H = solve_homography(src, dst)
    assert rms_error(H, src, dst) < 1e-9
    # 任意点往返
    for X, Y in ((0.15, 0.10), (0.07, 0.03), (0.25, 0.18), (0.0, 0.0)):
        u, v = apply_homography(H_TRUE, X, Y)
        X2, Y2 = apply_homography(H, u, v)
        assert math.hypot(X2 - X, Y2 - Y) < 1e-9


def test_more_points_with_noise_gives_sub_millimetre_rms():
    """多余点做最小二乘：±0.5 px 的角点噪声应压到毫米以下。"""
    import random
    random.seed(7)
    world = [(0, 0), (0.3, 0), (0.3, 0.2), (0, 0.2),
             (0.15, 0), (0.3, 0.1), (0.15, 0.2), (0, 0.1)]
    src, dst = [], []
    for X, Y in world:
        u, v = to_px(X, Y)
        src.append((u + random.uniform(-0.5, 0.5), v + random.uniform(-0.5, 0.5)))
        dst.append((X, Y))
    H = solve_homography(src, dst)
    assert rms_error(H, src, dst) < 0.002          # < 2 mm


def test_fewer_than_four_points_is_rejected():
    with pytest.raises(ValueError, match="至少需要 4 组"):
        solve_homography([(0, 0), (1, 0), (0, 1)], [(0, 0), (1, 0), (0, 1)])


def test_collinear_points_are_rejected():
    src = [(0, 0), (1, 1), (2, 2), (3, 3)]
    dst = [(0, 0), (1, 0), (2, 0), (3, 0)]
    with pytest.raises(ValueError, match="奇异|退化"):
        solve_homography(src, dst)


def test_inverse_round_trips():
    H = solve_homography([to_px(X, Y) for X, Y in MARKER.world_corners()],
                         list(MARKER.world_corners()))
    Hi = invert_homography(H)
    for X, Y in ((0.1, 0.05), (0.28, 0.19)):
        u, v = apply_homography(H, X, Y)
        X2, Y2 = apply_homography(Hi, u, v)
        assert math.hypot(X2 - X, Y2 - Y) < 1e-9


# --------------------------------------------------------------------------
# 标定记录
# --------------------------------------------------------------------------
def test_calibrate_from_marker_gives_metric_scale():
    quad = [to_px(X, Y) for X, Y in MARKER.world_corners()]
    cal = calibrate_from_marker(quad, MARKER, image_size=(640, 480))
    cal.validate()
    assert cal.rms_px < 1e-9
    # 度量衡：用图像点量参考物自己的宽高，必须等于真实尺寸
    assert cal.distance_metres(quad[0], quad[1]) == pytest.approx(0.30, abs=1e-6)
    assert cal.distance_metres(quad[0], quad[3]) == pytest.approx(0.20, abs=1e-6)
    # 世界单位就是米
    assert cal.metres_per_unit == 1.0
    assert cal.unit_to_metres(2.5) == 2.5


def test_calibration_json_round_trip(tmp_path):
    quad = [to_px(X, Y) for X, Y in MARKER.world_corners()]
    cal = calibrate_from_marker(quad, MARKER, image_size=(640, 480))
    path = cal.save(tmp_path / "cal.json")
    back = FieldCalibration.load(path)
    assert back.H == pytest.approx(cal.H)
    assert back.metres_per_unit == cal.metres_per_unit
    assert back.rms_px == cal.rms_px
    assert back.image_to_field(*quad[0]) == pytest.approx(cal.image_to_field(*quad[0]))


def test_validate_rejects_bad_scale():
    quad = [to_px(X, Y) for X, Y in MARKER.world_corners()]
    cal = calibrate_from_marker(quad, MARKER)
    cal.metres_per_unit = 0.0
    with pytest.raises(ValueError, match="metres_per_unit"):
        cal.validate()


# --------------------------------------------------------------------------
# 参考物检测（合成图，不用摄像头）
# --------------------------------------------------------------------------
def _synthetic_marker_frame(size=(480, 360)):
    """深色背景 + 一块洋红矩形（参考物）。"""
    w, h = size
    buf = bytearray(bytes((30, 34, 44)) * (w * h))
    left, top, right, bottom = 60, 40, 240, 160
    for y in range(top, bottom + 1):
        for x in range(left, right + 1):
            i = (y * w + x) * 3
            buf[i:i + 3] = bytes((255, 0, 255))
    return _frame(w, h, buf)


def _glare_frame(size=(480, 360)):
    """一整帧的**无彩色反光**：大块白光 + 灰白渐变，没有参考物。

    这是真实失败案例的合成版：相机拍显示器，房间灯光糊出一大片白。
    白参考物时代这个画面会被误判成参考物，洋红判据必须让它返回 None。
    """
    w, h = size
    buf = bytearray(bytes((30, 34, 44)) * (w * h))
    for y in range(h):
        for x in range(w):
            # 一个巨大的、边界圆滑的亮斑，中心 255、边缘渐变到背景
            d = math.hypot(x - 110, y - 90) / 130.0
            v = int(255 * max(0.0, 1.0 - d * d))
            v = max(v, 40)
            i = (y * w + x) * 3
            buf[i:i + 3] = bytes((v, v, v))
    return _frame(w, h, buf)


def _frame(w, h, rgb):
    from vision.primitives import Frame
    return Frame.from_rgb(w, h, rgb)


def test_marker_channels_picks_the_weak_channel():
    """弱通道必须是参考物最缺的那个；两个强通道谁先谁后无所谓。"""
    for colour, weak in (((255, 0, 255), 1), ((0, 255, 0), 0), ((255, 255, 0), 2)):
        got_weak, sa, sb = marker_channels(colour)
        assert got_weak == weak, colour
        assert {sa, sb} == {0, 1, 2} - {weak}, colour


def test_detect_marker_quad_finds_a_bright_rectangle():
    frame = _synthetic_marker_frame()
    quad = detect_marker_quad(frame, min_area_frac=0.01)
    assert quad is not None and len(quad) == 4
    xs = [p[0] for p in quad]
    ys = [p[1] for p in quad]
    assert min(xs) == pytest.approx(60, abs=3) and max(xs) == pytest.approx(240, abs=3)
    assert min(ys) == pytest.approx(40, abs=3) and max(ys) == pytest.approx(160, abs=3)
    # 四角必须互不相同（对角极值取角的正确性）
    assert len(set(quad)) == 4


def test_detect_marker_quad_ignores_a_huge_white_glare():
    """回归：整屏反光 + 一块小的参考物，必须选中参考物而不是反光。

    真实数据里反光块的面积是白色参考物的 5.5 倍（36065 px vs 6468 px），
    所以"取最大亮块"这种判据一定选错。这个测试把那个失败场景固化下来。
    """
    w, h = 480, 360
    frame = _glare_frame((w, h))
    assert detect_marker_quad(frame, min_area_frac=0.01) is None, "反光不该被当成参考物"

    # 再在反光旁边放一块**小得多**的洋红参考物：仍然必须选中它
    rgb = bytearray(frame.n * 3)
    for i in range(frame.n):
        rgb[i * 3:i * 3 + 3] = bytes((frame.r[i], frame.g[i], frame.b[i]))
    left, top, right, bottom = 300, 200, 380, 250      # 81×51，只有反光的 ~1/20
    for y in range(top, bottom + 1):
        for x in range(left, right + 1):
            i = (y * w + x) * 3
            rgb[i:i + 3] = bytes((255, 0, 255))
    quad = detect_marker_quad(_frame(w, h, rgb), min_area_frac=0.005)
    assert quad is not None
    xs = [p[0] for p in quad]
    ys = [p[1] for p in quad]
    assert min(xs) == pytest.approx(left, abs=3) and max(xs) == pytest.approx(right, abs=3)
    assert min(ys) == pytest.approx(top, abs=3) and max(ys) == pytest.approx(bottom, abs=3)


def _paint(buf, w, h, colour, pred):
    for y in range(h):
        for x in range(w):
            if pred(x, y):
                i = (y * w + x) * 3
                buf[i:i + 3] = bytes(colour)


def test_detect_marker_quad_rejects_the_scene_palette_shapes():
    """场景里的东西按**真实形状**画出来，都不该被当成参考物。

    注意这里画的是形状而不是整屏色块。参考物的判据是"饱和色 + 大 + 实心"，
    单看颜色分不开同色系的粉子弹（`color_for_type(6) = (230,130,220)`，
    弱/强 = 0.59，比相机拍出来的参考物 0.68 **还更饱和**）——
    所以真正把它们分开的是**尺寸和形状**，测试就必须按尺寸形状来测。
    """
    from bullet_sim.render.overlays import color_for_type

    w, h = 320, 240
    for tid in range(8):
        colour = color_for_type(tid)

        # ① 一颗子弹：半径 6 的实心圆
        buf = bytearray(bytes((34, 40, 55)) * (w * h))
        cx, cy, rad = 160, 120, 6
        _paint(buf, w, h, colour, lambda x, y: (x-cx)**2 + (y-cy)**2 <= rad*rad)
        assert detect_marker_quad(_frame(w, h, buf), min_area_frac=0.004) is None, \
            f"bullet colour {(colour)} was mistaken for the marker"

        # ② 一堵墙：又长又细（26 行高，跨整个场地宽）
        buf = bytearray(bytes((34, 40, 55)) * (w * h))
        _paint(buf, w, h, colour, lambda x, y: 100 <= y <= 125)
        assert detect_marker_quad(_frame(w, h, buf), min_area_frac=0.004) is None, \
            f"wall colour {(colour)} was mistaken for the marker"

        # ③ 一整屏同色（最坏情况）：只有参考物默认色该被认出来
        buf = bytearray(bytes(colour) * (w * h))
        got = detect_marker_quad(_frame(w, h, buf), min_area_frac=0.004)
        if tid == 6:
            assert got is not None, "同色系最坏情况：颜色判据放行了，这是**已知限制**"
        else:
            assert got is None, f"colour_for_type({tid}) = {colour} passed the colour key"


def test_calibration_warns_when_the_marker_only_covers_a_corner():
    """回归：参考物缩在一个角上时，单应外推会把场地预测得很离谱。

    这条是**真机**上发现的：残差 0.0000 px（4 点输入恒为 0，它不衡量外推），
    但预测出来的场地比真实场地小一大截、透视还被拉过头。
    所以可信度指标是 `marker_coverage`，不是 `rms_px`。
    """
    field = (640.0, 480.0)
    # 一角上的小参考物 vs 铺满场地的参考物，用**同一组**真实对应的图像点
    small = Marker("small", 0.30 * 640, 0.20 * 480)
    big = Marker("big", 0.90 * 640, 0.90 * 480)

    cal_small = calibrate_from_marker([(0, 0), (1, 0), (1, 1), (0, 1)],
                                      small, field_size=field)
    cal_big = calibrate_from_marker([(0, 0), (1, 0), (1, 1), (0, 1)],
                                    big, field_size=field)
    assert cal_small.marker_coverage == pytest.approx(0.06, abs=0.01)
    assert cal_big.marker_coverage == pytest.approx(0.81, abs=0.01)
    assert cal_small.marker_coverage < 0.5 <= cal_big.marker_coverage
    # 两者的残差都是 0 —— 这正是"残差不能当可信度"的证据
    assert cal_small.rms_px == pytest.approx(0.0)
    assert cal_big.rms_px == pytest.approx(0.0)


def test_marker_coverage_survives_a_save_load_round_trip(tmp_path):
    cal = calibrate_from_marker([(0, 0), (1, 0), (1, 1), (0, 1)],
                                Marker("m", 192.0, 96.0),
                                field_size=(640.0, 480.0))
    again = FieldCalibration.load(cal.save(tmp_path / "c.json"))
    assert again.marker_coverage == pytest.approx(cal.marker_coverage)


def test_calibration_without_a_field_size_reports_unknown_coverage():
    """不给场地尺寸时不能瞎猜一个覆盖率出来。"""
    cal = calibrate_from_marker([(0, 0), (1, 0), (1, 1), (0, 1)],
                                Marker("m", 192.0, 96.0))
    assert cal.marker_coverage == 0.0


def test_detect_marker_quad_returns_none_on_empty_frame():
    frame = _frame(64, 48, bytes((20, 20, 20)) * (64 * 48))
    assert detect_marker_quad(frame, min_area_frac=0.01) is None


# --------------------------------------------------------------------------
# 接缝测试：平台**画出来**的参考物，视觉模块必须原样认出来
# --------------------------------------------------------------------------
def _platform_frame(**kw):
    """用平台的渲染器离屏画一帧，转成 `vision.primitives.Frame`。"""
    pytest.importorskip("pygame")
    from bullet_sim.obstacles.scenario import scenario_from_types
    from bullet_sim.render.pygame_view import PygameRenderer
    from bullet_sim.simulator.world import World
    from vision.primitives import Frame

    scenario = scenario_from_types(["wall_with_gap"], seed=1, duration=6.0,
                                   require_valid=False, player_hitbox_radius=10.0)
    world = World(scenario.to_spec(), reward="zero", terminate_on_collision=False)
    for _ in range(30):
        world.step(0)
    renderer = PygameRenderer(mode="rgb_array", window_size=(640, 480),
                             show_hitboxes=False, show_world_axes=True, **kw)
    arr = renderer.render(world)                      # (h, w, 3) RGB
    return renderer, world, Frame.from_rgb(
        arr.shape[1], arr.shape[0], arr.astype("uint8").tobytes())


def test_renderer_marker_is_never_occluded_by_obstacles():
    """回归：障碍物曾经画在参考物**之后**，一条墙线把参考物拦腰切成两半。

    连通域是按"上下行游程重叠"连的，被切成两半就只剩一半面积 ——
    实测检测出的四边形因此比真参考物矮。参考物是标定用的，必须完整可见，
    所以它现在画在**所有东西之上**。
    """
    renderer, world, frame = _platform_frame(
        show_calibration_marker=True, calibration_marker=(0.30, 0.20))
    vp = renderer.viewport
    mx0, my0, mx1, my1 = renderer.calibration_marker_world_rect(world)
    sx0, sy1 = vp.world_to_screen(mx0, my0)
    sx1, sy0 = vp.world_to_screen(mx1, my1)
    x0, x1 = int(min(sx0, sx1)) + 2, int(max(sx0, sx1)) - 2
    y0, y1 = int(min(sy0, sy1)) + 2, int(max(sy0, sy1)) - 2
    assert x1 > x0 and y1 > y0

    want = renderer.calibration_marker_color
    bad = sum(1 for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)
              if (frame.r[y * frame.width + x], frame.g[y * frame.width + x],
                  frame.b[y * frame.width + x]) != want)
    total = (x1 - x0 + 1) * (y1 - y0 + 1)
    assert bad == 0, f"{bad}/{total} 个参考物像素被障碍物盖住了"


def test_renderer_marker_round_trips_through_the_detector():
    """最高价值的一条：渲染出来的参考物，四角必须和世界坐标对得上。

    它同时钉住三件事：①参考物确实是饱和色；②它锚在场地原点；
    ③检测器返回的角点顺序与 `Marker.world_corners()` 一致。
    """
    renderer, world, frame = _platform_frame(
        show_calibration_marker=True, calibration_marker=(0.30, 0.20))
    vp = renderer.viewport
    mx0, my0, mx1, my1 = renderer.calibration_marker_world_rect(world)
    assert (mx0, my0, mx1, my1) == (0.0, 0.0, 0.30 * 640, 0.20 * 480)

    quad = detect_marker_quad(frame, min_area_frac=0.004)
    assert quad is not None, "平台画出的参考物没被检测到"

    # 世界四角 → 屏幕像素，作为期望值
    expect = [vp.world_to_screen(mx0, my0), vp.world_to_screen(mx1, my0),
              vp.world_to_screen(mx1, my1), vp.world_to_screen(mx0, my1)]
    got = {(round(x), round(y)) for x, y in quad}
    want = {(round(x), round(y)) for x, y in expect}
    assert got == want, f"检测角点 {sorted(got)} != 期望 {sorted(want)}"


def test_renderer_marker_is_absent_unless_requested():
    """不开开关时画面里一个参考物像素都不能有，否则会污染正常回放。"""
    _renderer, _world, frame = _platform_frame(show_calibration_marker=False)
    assert detect_marker_quad(frame, min_area_frac=0.0005) is None


def test_renderer_marker_colour_is_configurable():
    """换成绿色参考物，用绿键也认得出 —— 判据不是写死洋红的。"""
    renderer, world, frame = _platform_frame(
        show_calibration_marker=True, calibration_marker_color=(0, 255, 0))
    quad = detect_marker_quad(frame, colour=(0, 255, 0), min_area_frac=0.004)
    assert quad is not None
    # 洋红键反过来必须认不出来
    assert detect_marker_quad(frame, colour=(255, 0, 255), min_area_frac=0.004) is None


def _warp_frame(frame, dst_quad):
    """把一帧用单应映到一个**透视四边形**上（纯 Python，最近邻）。

    单应直接复用本模块的 `solve_homography` / `invert_homography` ——
    测试不引入任何新的数学，也不依赖 opencv。
    """
    from vision.primitives import Frame

    w, h = frame.width, frame.height
    src = [(0.0, 0.0), (w - 1.0, 0.0), (w - 1.0, h - 1.0), (0.0, h - 1.0)]
    h_inv = invert_homography(solve_homography(src, dst_quad))
    out = bytearray(bytes((18, 18, 20)) * (w * h))
    for y in range(h):
        for x in range(w):
            u, v = apply_homography(h_inv, x, y)
            ui, vi = int(u + 0.5), int(v + 0.5)
            if 0 <= ui < w and 0 <= vi < h:
                s = vi * w + ui
                d = (y * w + x) * 3
                out[d] = frame.r[s]
                out[d + 1] = frame.g[s]
                out[d + 2] = frame.b[s]
    return Frame.from_rgb(w, h, out)


def test_detect_marker_quad_survives_a_tilted_perspective():
    """回归：相机是**斜视**的，参考物在画面里是斜四边形。

    曾经用 `primitives.stacked_runs`（"上下对齐的游程堆"）当形状判据，
    它要求相邻两行游程端点几乎不动 —— 四边形一斜，靠近上下角的那几行里
    端点每行要移动 ``宽/高`` 个像素（7° 就到 8 px/行），堆只剩中间一条带，
    于是检测出的四边形比真参考物矮了 14 px，而**单应残差依然是 0**，
    光看 RMS 发现不了。这个测试就是钉这一条的。
    """
    _renderer, world, frame = _platform_frame(show_calibration_marker=True)
    w, h = frame.width, frame.height
    dst = [(120.0, 40.0), (520.0, 90.0), (470.0, 430.0), (60.0, 380.0)]
    photo = _warp_frame(frame, dst)

    quad = detect_marker_quad(photo, min_area_frac=0.004)
    assert quad is not None, "斜视下没检测到参考物"

    m = solve_homography([(0.0, 0.0), (w - 1.0, 0.0),
                          (w - 1.0, h - 1.0), (0.0, h - 1.0)], dst)
    # 期望角点直接从渲染器问出来（世界 -> 屏幕 -> 透视），
    # 不写死尺寸 —— 参考物默认大小改过一次，写死的测试就跟着假失败了。
    vp = _renderer.viewport
    mx0, my0, mx1, my1 = _renderer.calibration_marker_world_rect(world)
    want = [apply_homography(m, *vp.world_to_screen(mx0, my0)),   # 左下
            apply_homography(m, *vp.world_to_screen(mx1, my0)),   # 右下
            apply_homography(m, *vp.world_to_screen(mx1, my1)),   # 右上
            apply_homography(m, *vp.world_to_screen(mx0, my1))]   # 左上
    for i, ((gx, gy), (wx, wy)) in enumerate(zip(quad, want)):
        assert abs(gx - wx) <= 2.5 and abs(gy - wy) <= 2.5, \
            f"角点 {i} 偏了: 检测 {(gx, gy)} 期望 {(wx, wy)}（全部: {quad}）"


def test_calibration_records_the_field_size_not_the_marker_size():
    """回归：`field_width/field_height` 曾经装的是**参考物**尺寸。

    名字叫 field_* 却装 marker_*，于是"坐标有没有跑出场外"根本无从判断 ——
    实测就是这么漏掉一次标定失效的（角色算到 (796,602)，场地只有 640x480）。
    """
    cal = calibrate_from_marker([(0, 0), (1, 0), (1, 1), (0, 1)],
                                Marker("m", 576.0, 432.0),
                                field_size=(640.0, 480.0), unit="field")
    assert (cal.field_width, cal.field_height) == (640.0, 480.0)
    # 参考物尺寸另有字段，没丢
    assert (cal.marker_width_m, cal.marker_height_m) == (576.0, 432.0)


def test_calibration_without_a_field_size_records_zero():
    """没告诉场地多大就记 0 —— 越界检查据此**跳过**，而不是瞎猜一个。"""
    cal = calibrate_from_marker([(0, 0), (1, 0), (1, 1), (0, 1)],
                                Marker("m", 192.0, 96.0))
    assert cal.field_width == 0.0 and cal.field_height == 0.0


def test_window_geometry_survives_a_save_load_round_trip(tmp_path):
    """标定只在"窗口没动"时有效，所以窗口几何必须能存能读。"""
    cal = calibrate_from_marker([(0, 0), (1, 0), (1, 1), (0, 1)],
                                Marker("m", 192.0, 96.0), field_size=(640.0, 480.0),
                                unit="field", window=(240, 140, 640, 480))
    again = FieldCalibration.load(cal.save(tmp_path / "c.json"))
    assert again.window == (240, 140, 640, 480)
    assert (again.field_width, again.field_height) == (640.0, 480.0)


def test_window_geometry_defaults_to_none():
    """拿不到窗口几何（没有 X / 平台没跑）不是错误，只是记不上。"""
    cal = calibrate_from_marker([(0, 0), (1, 0), (1, 1), (0, 1)],
                                Marker("m", 192.0, 96.0), field_size=(640.0, 480.0))
    assert cal.window is None
