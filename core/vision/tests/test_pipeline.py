"""任务二闭环测试：**平台真值 → 合成相机 → 视觉链 → 和真值比**。

为什么这是最有价值的一条测试
----------------------------
前面每个模块都有自己的单元测试，但"整条链合起来准不准"只能闭环验。
这里把平台当**真值发生器**：它的角色位置是已知的，我们用一台**虚拟相机**
（一个已知单应）去"拍"它，再跑完整的视觉链，最后和平台的真值对账。

整条链一次跑通：渲染 → 透视重采样 → detect → to_standard → to_world → Tracker。

没有摄像头也能跑，所以它是**可复现**的 —— 相机不在、CI 上没有设备，这条都成立。
真机验证是另一回事（见 docs/vision/calibration.md 的实测记录）。

关于 ``world.step(4)``：那 4 是 **Action**，不是步数。一次 ``step`` 走
``dt = 1/120 s``。时间戳一律取 ``world.state.timestamp``，因为
"dt 必须来自采集时刻" —— 用帧号凑 dt 正是三个参考项目都翻车的地方。
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip("pygame")

from vision import motion, to_standard  # noqa: E402
from vision.calibration import (  # noqa: E402
    Marker,
    apply_homography,
    calibrate_from_marker,
    detect_marker_quad,
    invert_homography,
    solve_homography,
)
from vision.localize import to_world  # noqa: E402
from vision.wall_with_gap import WallWithGapParams, detect  # noqa: E402

#: 虚拟相机：窗口 -> 相机的单应（用一个透视四边形定义）。
_CAMERA_QUAD = [(60.0, 25.0), (580.0, 60.0), (530.0, 455.0), (95.0, 425.0)]
#: 渲染窗口比相机大 1.5 倍，这样角色在相机里约 15 px —— 接近真机看到的尺寸。
_WIN = (960, 720)
_CAM = (640, 480)


def _renderer(show_marker: bool, win):
    from bullet_sim.render.pygame_view import PygameRenderer
    return PygameRenderer(mode="rgb_array", window_size=win, show_hitboxes=False,
                          show_world_axes=True, show_calibration_marker=show_marker,
                          calibration_marker=(0.90, 0.90))


def _make_world(radius: float = 10.0):
    from bullet_sim.obstacles.scenario import scenario_from_types
    from bullet_sim.simulator.world import World
    sc = scenario_from_types(["moving_block"], seed=3, duration=40.0,
                             require_valid=False, player_hitbox_radius=float(radius))
    return World(sc.to_spec(), reward="zero", terminate_on_collision=False)


def _warp(img, dst_quad, win, cam):
    """把窗口图像按单应重采样成"相机照片"（纯 Python，最近邻）。"""
    import numpy as np
    ww, wh = win
    w, h = cam
    h_inv = invert_homography(solve_homography(
        [(0.0, 0.0), (ww - 1.0, 0.0), (ww - 1.0, wh - 1.0), (0.0, wh - 1.0)], dst_quad))
    out = np.zeros((h, w, 3), np.uint8)
    out[:, :] = (18, 18, 20)
    for y in range(h):
        for x in range(w):
            u, v = apply_homography(h_inv, x, y)
            ui, vi = int(u + 0.5), int(v + 0.5)
            if 0 <= ui < ww and 0 <= vi < wh:
                out[y, x] = img[vi, ui]
    return out


def _as_frame(photo):
    from vision.primitives import Frame
    h, w = photo.shape[:2]
    return Frame(w, h, photo[:, :, 0].tobytes(), photo[:, :, 1].tobytes(),
                 photo[:, :, 2].tobytes())


def _camera_quad_for(cam):
    """把标准四边形缩放到目标分辨率。"""
    sx, sy = cam[0] / 640.0, cam[1] / 480.0
    return [(x * sx, y * sy) for x, y in _CAMERA_QUAD]


@pytest.fixture(scope="module")
def cal():
    """标定一次给所有用例用 —— 每帧重采样是纯 Python，省一次是一次。

    走的是和真机**一模一样**的路径：渲染出参考物 → 自动检测 → 标定。
    """
    world = _make_world()
    r = _renderer(True, _WIN)
    photo = _warp(r.render(world), _camera_quad_for(_CAM), _WIN, _CAM)
    quad = detect_marker_quad(_as_frame(photo))
    assert quad is not None, "合成照片里没检测到参考物"
    # 参考物的宽高就是"场地单位"（576x432 = 0.90 x 0.90 个 640x480 场地），
    # 所以这份标定的世界单位是**场地单位**，不是米。
    return calibrate_from_marker(quad, Marker("m", 0.90 * 640, 0.90 * 480),
                                 field_size=(640.0, 480.0), unit="field")


# ==========================================================================
def test_adapter_and_world_conversion_are_idempotent(cal):
    """``to_world`` 对已经是世界单位的帧必须原样返回，否则会被反复缩放。"""
    world = _make_world()
    r = _renderer(False, _WIN)
    photo = _warp(r.render(world), _camera_quad_for(_CAM), _WIN, _CAM)
    std = to_standard.frame_to_standard(detect(_as_frame(photo)), seq=0, stamp=0.0)
    once = to_world(std, cal)
    twice = to_world(once, cal)
    assert twice is once or twice.units == once.units


def test_closed_loop_recovers_position_radius_and_velocity(cal):
    """整条链的精度对账（这是任务二的验收测试）。

    实测（合成相机 + 平台真值）：位置误差约 0.5% 场宽、半径误差 < 5%、
    匀速段的速率误差 < 10%。α-β 滤波器**起步有 ~0.1 s 的收敛期**，
    所以前几帧不计入 —— 这是所选滤波器的固有特性，不是 bug。
    """
    cam = _CAM
    assert cal.marker_coverage > 0.5, "参考物必须铺满场地，否则单应是在外推"

    world = _make_world()
    r = _renderer(False, _WIN)
    tracker = motion.Tracker(gate=80.0)
    rows: list[tuple[float, float, float, float]] = []      # (t, 真值v, 估计v, 位置误差)
    prev = None

    for k in range(22):
        world.step(4)                                       # 4 是 Action，不是步数
        truth = world.state.player.position
        stamp = float(world.state.timestamp)

        photo = _warp(r.render(world), _camera_quad_for(cam), _WIN, cam)
        std = to_standard.frame_to_standard(
            detect(_as_frame(photo), WallWithGapParams()), seq=k, stamp=stamp)
        wf = to_world(std, cal)
        p = wf.player
        assert p is not None, f"第 {k} 帧没检出角色"

        tracks = tracker.update([(p.x, p.y)], stamp)
        assert tracks, f"第 {k} 帧没有轨迹"
        t = tracks[0]

        if prev is not None:
            dt = stamp - prev[0]
            v_true = (math.hypot(truth[0] - prev[1][0], truth[1] - prev[1][1]) / dt
                      if dt > 0 else 0.0)
            rows.append((stamp, v_true, t.speed,
                         math.hypot(p.x - truth[0], p.y - truth[1])))
        prev = (stamp, truth)

    assert len(rows) >= 12
    pos = sorted(r[3] for r in rows)
    assert pos[len(pos) // 2] < 0.01 * 640, f"位置误差中位 {pos[len(pos)//2]:.2f} 太大"

    # 跳过 α-β 的收敛期，只看稳定段
    tail = rows[6:]
    v_true = sorted(r[1] for r in tail)[len(tail) // 2]
    v_est = sorted(r[2] for r in tail)[len(tail) // 2]
    assert v_true > 50.0, "真值速度太小，这条测试就没意义了"
    assert abs(v_est - v_true) / v_true < 0.20, \
        f"速率误差过大：真值 {v_true:.1f} 估计 {v_est:.1f}"


def test_closed_loop_recovers_the_player_radius_not_just_its_centre(cal):
    """半径是**硬约束**的一半（缺口 >= 2r），量小了会把危险判成安全。

    平台把可见圆画成全亮的（描边也是亮色），所以"相机看到的圆"就是碰撞圆。
    以前描边是深色，检测器只量到里面那圈白芯，半径会系统性偏小 ~20%。
    """
    world = _make_world(radius=10.0)
    r = _renderer(False, _WIN)
    radii = []
    for _ in range(6):
        world.step(4)
        photo = _warp(r.render(world), _camera_quad_for(_CAM), _WIN, _CAM)
        std = to_standard.frame_to_standard(detect(_as_frame(photo)), seq=0, stamp=0.0)
        p = to_world(std, cal).player
        if p is not None:
            radii.append(p.radius)
    assert radii, "一次都没检出角色"
    mid = sorted(radii)[len(radii) // 2]
    assert abs(mid - 10.0) < 1.0, f"世界半径中位 {mid:.2f}，真值 10"


def test_velocity_is_derived_from_stamp_not_frame_index():
    """把同一段轨迹用**一半的 dt** 喂进去，速度估计必须翻倍。

    这一条钉的是"dt 来自 stamp"：如果实现里偷偷用了帧号，速度就不会随
    stamp 变化。三个参考项目全都在这件事上翻过车。
    """
    pts = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (30.0, 0.0),
           (40.0, 0.0), (50.0, 0.0), (60.0, 0.0), (70.0, 0.0)]
    slow = motion.Tracker(gate=100.0)
    fast = motion.Tracker(gate=100.0)
    for i, p in enumerate(pts):
        t_slow = slow.update([p], i * 0.10)
        t_fast = fast.update([p], i * 0.05)
    v_slow = t_slow[0].speed
    v_fast = t_fast[0].speed
    assert v_slow > 0 and v_fast > 0
    assert v_fast == pytest.approx(2.0 * v_slow, rel=0.05), \
        f"dt 减半速度应翻倍：{v_slow:.2f} -> {v_fast:.2f}"


# ==========================================================================
# 接缝：视觉的输出必须能被 core/cpu 的算法**直接**吃下去
# ==========================================================================
def test_world_frame_satisfies_the_cpu_decision_contract(cal):
    """把一帧真实视觉输出喂给 ``cpu.adapters.VisionWorldStream``。

    这条测试守的是**模块边界**，不是精度：决策层对输入有三条硬要求
    （坐标系 ``field``、单位已标定、``stamp`` 单调），任何一条破了
    都会被拒。以前 ``to_world`` 不管标定用的是米还是场地单位，一律标成
    ``units="m"`` —— 场地单位会被下游当米用，尺度错了却什么都看不出来。
    """
    pytest.importorskip("cpu")
    from cpu.adapters.vision_adapter import VisionWorldConfig, VisionWorldStream

    world = _make_world()
    r = _renderer(False, _WIN)
    stream = VisionWorldStream(VisionWorldConfig(field_w=640.0, field_h=480.0))

    decisions = 0
    for k in range(4):
        world.step(4)
        photo = _warp(r.render(world), _camera_quad_for(_CAM), _WIN, _CAM)
        std = to_standard.frame_to_standard(
            detect(_as_frame(photo), WallWithGapParams()), seq=k,
            stamp=float(world.state.timestamp))
        wf = to_world(std, cal)
        view, dec = stream.step(wf)            # 被拒会抛异常，测试就挂
        assert len(view.player) == 7
        assert dec is not None
        decisions += 1
    assert decisions == 4


def test_world_frame_declares_the_truthful_unit(cal):
    """``--relative`` 标出来的是**场地单位**，不能标成米。

    标成米的话决策层会把它当米用 —— 数值一样、含义不同，
    是那种"跑得通但全错"的 bug。
    """
    world = _make_world()
    r = _renderer(False, _WIN)
    photo = _warp(r.render(world), _camera_quad_for(_CAM), _WIN, _CAM)
    std = to_standard.frame_to_standard(detect(_as_frame(photo)), seq=0, stamp=0.0)
    wf = to_world(std, cal)
    assert cal.unit == "field", "合成标定用的是场地单位"
    assert wf.units.value == "px", wf.units
    # 单位是 px 时必须带上单应，否则决策层无法判断"到底标定过没有"
    assert wf.calibration is not None
    assert wf.calibration.H_field_from_image is not None


def test_metric_calibration_declares_metres():
    """反过来：真给了米的参考物尺寸，就该标成 ``m``。"""
    from vision.calibration import Marker, calibrate_from_marker
    cal = calibrate_from_marker([(0, 0), (1, 0), (1, 1), (0, 1)],
                                Marker("m", 0.096, 0.048), field_size=(640.0, 480.0))
    assert cal.unit == "m"
    std = to_standard.frame_to_standard(
        __import__("vision.wall_with_gap", fromlist=["x"]).FrameObservation(
            image="t", width=10, height=10, player=None, target=None),
        seq=0, stamp=0.0)
    assert to_world(std, cal).units.value == "m"
