"""**整条 wall_with_gap 闭环**的端到端测试。

    world → 渲染 → 真实 core/vision 检测 → vision_adapter → gap_avoid.plan()
          → Action → world.step

为什么必须有这个文件
--------------------
两半各自都有测试：视觉对真值的离线对账（``vision_reconcile``）、决策层的
单测（``core/cpu/tests``）。**但拼接处没有**。而拼接处恰好是最容易"跑得通但全错"
的地方：坐标系（``frame_id`` 必须 ``field``）、``dt`` 的来源（必须取 ``stamp``
之差，不能取 ``seq`` 之差）、场地尺寸（``VisionFrame`` 不带 ``field_w/field_h``）。
这些错了不会有异常，只会安静地给出错的动作。

本文件同时钉住三个**真实发生过的静默错误**（见各测试的说明）：
1. 碰撞计数器读了一个不存在的属性 -> "0 碰撞"永远是假的；
2. episode 太短，墙根本够不着角色 -> "0 碰撞"是空的；
3. 只换 seed 不换场景参数 -> "N 个 seed 全通过"其实只跑了 1 局。

跑得动的前提
------------
需要 pygame 离屏渲染 + ``core/vision``。闭环一局要跑真视觉（纯 Python），
所以这里刻意**配短局 + 高墙速**，让"撞上"发生在 1~2 秒内 ——
不为覆盖率跑长局，只验证**结构与指标是活的**。
"""

from __future__ import annotations

import pytest

pytest.importorskip("pygame")
pytest.importorskip("cv2")

# 被测对象是 core/harness/loop.py（仓库根的 cpu_vision_loop.py 只是薄入口）。
# 直接按包导入，不再 importlib 加载根脚本 —— 那些代码已经不在根脚本里了。
from harness import loop as _cl  # noqa: E402
from cpu.gap_avoid import AlgoConfig  # noqa: E402

#: 合成相机固定用这个 seed（噪声项），保证测试可复现
CAM_SEED = 1


def _camera():
    return _cl.SyntheticCamera(seed=CAM_SEED)


def _env(seconds: float, *, speed: float, gap_position: float, seed: int = 1):
    return _cl.build_env(seed, 60.0, seconds, speed, gap_position, "static")


def test_world_has_no_collision_events_attribute() -> None:
    """**钉住那个静默错误**：``env.world`` 上没有 ``collision_events``。

    旧代码写的是 ``getattr(env.world, "collision_events", 0)`` —— 属性不存在，
    于是它永远返回 0，"0 碰撞"这个结论一直是假的。真计数器在
    ``env.world.contacts``（``ContactTracker``）。

    这条断言的作用是：如果哪天有人给 world 加了个同名的空属性，
    或者又有人图省事去读那个名字，这里会立刻炸。
    """
    env = _env(1.0, speed=40.0, gap_position=0.28)
    assert not hasattr(env.world, "collision_events"), \
        "world 上不该有 collision_events —— 真值在 world.contacts"
    assert hasattr(env.world, "contacts")
    c = _cl._contacts(env)
    assert set(c) == {"entered", "in_contact", "overlap_frames", "first_event_frame"}
    assert c["entered"] == 0 and c["in_contact"] is False


def test_collision_metric_is_live_negative_control() -> None:
    """**阴性对照**：角色一动不动，墙必须撞上它。

    这是"无碰撞率"能不能当证据的**唯一**根据。没有这条，一个坏掉的计数器
    和一个完美的算法看起来一模一样。

    用高墙速把"撞上"压到 1 秒内（40 单位/秒要等 10.2 秒，测试太慢）：
    墙速 400 时角色 y≈72、墙从 480 下来，(480-72)/400 ≈ 1.0 秒。
    """
    cam = _camera()
    env = _env(2.5, speed=400.0, gap_position=0.28)
    calib = cam.calibrate(env)
    st = _cl.run_episode(env, cam, calib, steps=int(2.5 / _cl.SIM_DT),
                         cfg=AlgoConfig(forward=(0.0, 1.0)), control_hz=30.0,
                         policy="hold")
    assert st["ever_collided"], \
        "站着不动却没有碰撞 —— 碰撞指标是坏的（这个 bug 真发生过），" \
        "任何'无碰撞率'结论都不能信"
    assert st["entered"] >= 1
    assert st["survived_s"] is not None and st["survived_s"] < 2.0, \
        f"首次碰撞时间 {st['survived_s']} 不合理（墙速 400 应约 1s）"
    assert st["decided"] == 0, "hold 策略不应当做决策"


def test_gap_avoid_survives_where_hold_collides() -> None:
    """同一场景：站着不动会撞，真链路不撞 —— 这才叫"控制生效了"。

    同 seed、同墙速，只换策略。两者结果必须**不同**，否则这个场景什么都测不出来
    （比如墙太慢、或者缺口正对着角色出生点，不动也能活）。
    """
    cam = _camera()
    cfg = AlgoConfig(safety_margin=6.0, forward=(0.0, 1.0))
    results = {}
    for policy in ("hold", "gap_avoid"):
        env = _env(2.5, speed=400.0, gap_position=0.28)
        calib = cam.calibrate(env)
        results[policy] = _cl.run_episode(
            env, cam, calib, steps=int(2.5 / _cl.SIM_DT), cfg=cfg,
            control_hz=30.0, policy=policy)
    assert results["hold"]["ever_collided"] is True
    assert results["gap_avoid"]["decided"] > 0, "真链路一帧都没决策"
    assert results["gap_avoid"]["rejected"] == 0, "决策层拒绝了帧，说明适配器契约没对上"
    assert results["gap_avoid"]["out_of_field"] == 0, "有帧越界 -> 标定/坐标系没对上"


def test_decision_input_error_is_small_at_this_calibration() -> None:
    """决策输入误差要有具体的量级，不能只看"跑通了"。

    合成台的标定 RMS 约 0.45 场地单位（相机 640x480、0.81 px/场地单位 ——
    见 ``SyntheticCamera._warp`` 的说明），所以这里给一个**宽松但有意义**的上限：
    中位误差必须远小于缺口半宽（30 单位），否则决策拿到的就是垃圾坐标。
    """
    cam = _camera()
    env = _env(2.5, speed=400.0, gap_position=0.28)
    calib = cam.calibrate(env)
    st = _cl.run_episode(env, cam, calib, steps=int(2.5 / _cl.SIM_DT),
                         cfg=AlgoConfig(safety_margin=6.0, forward=(0.0, 1.0)),
                         control_hz=30.0, policy="gap_avoid")
    assert st.get("gap_in_err"), "一帧都没算出缺口输入误差"
    g = sorted(st["gap_in_err"])[len(st["gap_in_err"]) // 2]
    p = sorted(st["player_in_err"])[len(st["player_in_err"]) // 2]
    assert g < 10.0, f"缺口决策输入误差中位 {g:.2f} 场地单位太大"
    assert p < 10.0, f"角色决策输入误差中位 {p:.2f} 场地单位太大"


def test_synthetic_camera_calibrates_from_the_four_corner_fiducials() -> None:
    """合成相机的标定走**四角标记**，和录制台/离线对账同一套实现。

    这条是"只有一个标定实现"的守卫：以前合成台用那块盖住 90% 场地的洋红矩形
    （``detect_marker_quad`` + ``calibrate_from_marker``），和真链路是两套代码，
    于是"闭环里的标定"和"对账里的标定"根本无法互相印证。
    """
    cam = _camera()
    env = _env(2.0, speed=40.0, gap_position=0.28)
    calib = cam.calibrate(env)
    rep = cam.last_fiducial_report
    assert rep.n_markers == 4, f"只找到 {rep.n_markers} 枚标记"
    assert rep.n_points == 16
    assert 0.0 < rep.rms_px < 1.0, f"合成相机标定 RMS={rep.rms_px:.3f} 不合理"
    # 留出校验必须和 RMS **同量级**：差一个数量级说明四枚标记不在同一个单应里
    assert rep.holdout < 5.0 * rep.rms_px, \
        f"留出校验 {rep.holdout:.3f} 远大于 RMS {rep.rms_px:.3f} —— 标记几何不一致"
    assert calib.marker_coverage < 0.10, \
        "四角标记只该占场地约 5%（旧洋红矩形是 81%），覆盖这么大说明用的是旧路径"


def test_same_seed_reproduces_exactly_in_process() -> None:
    """同 seed 必须逐位复现（跨进程由 CI/命令行验证，这里验进程内）。

    闭环里唯一的随机源是合成相机的噪声项，它的 seed 是**构造时**给的；
    场景 seed 只管场景。两者都要固定，否则"同 seed 复现"是假的。
    """
    out = []
    for _ in range(2):
        cam = _camera()
        env = _env(2.0, speed=400.0, gap_position=0.28)
        calib = cam.calibrate(env)
        st = _cl.run_episode(env, cam, calib, steps=int(2.0 / _cl.SIM_DT),
                             cfg=AlgoConfig(safety_margin=6.0, forward=(0.0, 1.0)),
                             control_hz=30.0, policy="gap_avoid")
        out.append((env.state_hash(), st["decided"], st["end"], st["tiers"]))
    assert out[0] == out[1], f"同 seed 跑两次结果不一致:\n{out[0]}\n{out[1]}"


def test_varying_gap_position_actually_changes_the_episode() -> None:
    """**换 seed 不会换场景** —— 这一点必须显式钉住。

    ``wall_with_gap`` 场景本身没有随机性：seed 只进 RNG，而生成器是确定的，
    所以 ``build_env(seed=1)`` 和 ``build_env(seed=2)`` 给出**逐帧相同**的局
    （实测过）。于是"跑 5 个 seed 全 0 碰撞"其实只跑了一局 —— 那种验收是假的。
    真要把 seed 用起来，必须动场景参数（闭环脚本用 ``--vary-gap``）。
    """
    from bullet_sim.simulator.truth import truth_gaps

    traces = {}
    for seed in (1, 2):
        env = _env(3.0, speed=400.0, gap_position=0.28, seed=seed)
        ys = []
        for _ in range(int(1.0 / _cl.SIM_DT)):
            env.step(_cl.action_to_codec_input(_cl.Action.zero(),
                                               env.world.codec, 1.0))
            g = truth_gaps(env.world)
            ys.append(None if not g else round(g[0][2], 6))
        traces[seed] = ys
    assert traces[1] == traces[2], (
        "两个 seed 的场景居然不同了 —— 如果平台给 wall_with_gap 加了随机性，"
        "那这条测试的前提变了，请重新评估'多 seed 验收'的做法")
