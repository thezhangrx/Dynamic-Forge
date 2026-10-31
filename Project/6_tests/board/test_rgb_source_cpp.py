#!/usr/bin/env python3
"""`StdinPump` 的**采集时间戳**必须是真实时间（决定实时路径的速度标度）。

为什么单独立一条：几何帧的 `stamp_ns` 是"采集时刻"，速度 = Δ位置 / Δstamp。主循环
以前每帧固定 `+= 33.3 ms`（按 30fps 估），而相机实测交付 **40~48 fps** —— 推出来的速度
被系统性缩小 (30/实际fps) 倍（40fps 时低 28%，60fps 时低一半），而决策的
`t_arr = 距离 / v_n` 直接吃这个数。这条测试从管道喂帧、中间 sleep，检查 stamp 差
**跟着真实间隔走**：等于 33.3 ms 就说明退回固定步长了。

不需要板子、不需要相机：`--stdin-rgb` 那条路就是普通管道。
"""

from __future__ import annotations

import pathlib
import shutil
import subprocess

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
BOARD = ROOT / "core" / "board"
SRC = BOARD / "tests" / "test_rgb_source.cpp"
# ⚠ rgb_source.cpp 用到 ppm_reader.cpp 的 read_ppm_p6（PpmSource 那条路），
#   少编一个就是链接期 undefined reference。
IMPLS = [BOARD / "rgb_source.cpp", BOARD / "ppm_reader.cpp"]


@pytest.fixture(scope="module")
def pump_exe(tmp_path_factory: pytest.TempPathFactory) -> pathlib.Path:
    gxx = shutil.which("g++") or shutil.which("c++")
    if gxx is None:
        pytest.skip("没有 g++/c++，跳过（板上有 aarch64-linux-gnu-g++）")
    out = tmp_path_factory.mktemp("board") / "test_rgb_source"
    r = subprocess.run([gxx, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-I", str(BOARD), "-o", str(out), str(SRC),
                        *[str(p) for p in IMPLS], "-lpthread"],
                       capture_output=True, text=True)
    assert r.returncode == 0, f"编译失败：\n{r.stdout}\n{r.stderr}"
    return out


def test_pump_stamps_follow_real_intervals(pump_exe: pathlib.Path) -> None:
    r = subprocess.run([str(pump_exe)], capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout
    n = int(r.stdout.split("比对项 ")[1].split("，")[0])
    assert n >= 5, f"只跑了 {n} 项比对，太少"


def test_main_uses_capture_time_on_the_live_path() -> None:
    """实时路径要用读线程的采集时刻；离线序列才用可配步长（--ppm-fps）。

    两条路必须在代码里分得清清楚楚 —— 混用就是"速度整体差一个倍数"，而它**不会报错**。
    """
    src = (BOARD / "main.cpp").read_text(encoding="utf-8")
    assert "pump->last_stamp_ns()" in src, "实时路径必须用采集时刻当 stamp"
    assert "--ppm-fps" in src, "离线序列要能配帧率（默认 30）"
    assert "1e9 / args.ppm_fps" in src, "离线步长要按 --ppm-fps 算"
    assert "33'333'333ull" not in src, \
        "不该再有写死的 30fps 步长（那正是实时路径速度偏小的原因）"
    hpp = (BOARD / "rgb_source.hpp").read_text(encoding="utf-8")
    assert "uint64_t last_stamp_ns() const" in hpp
    cpp = (BOARD / "rgb_source.cpp").read_text(encoding="utf-8")
    # 时间戳必须打在**读线程读完那一刻**，不是主循环取走时（那会把处理耗时算进 dt）
    assert "const uint64_t ts = mono_ns();" in cpp
    assert "ready_.emplace_back(std::move(buf), ts)" in cpp
