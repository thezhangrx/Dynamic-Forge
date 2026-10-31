#!/usr/bin/env python3
"""矩形层"碎片合并"的 C++ 自测接进 pytest（含"真缺口不许封死"的反面用例）。"""

from __future__ import annotations

import pathlib
import shutil
import subprocess

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
BOARD = ROOT / "core" / "board"
CPP = ROOT / "core" / "cpu" / "cpp"       # gap_avoid.hpp（Rect 定义）在那儿
SRC = BOARD / "tests" / "test_segment_merge.cpp"


@pytest.fixture(scope="module")
def merge_exe(tmp_path_factory: pytest.TempPathFactory) -> pathlib.Path:
    gxx = shutil.which("g++") or shutil.which("c++")
    if gxx is None:
        pytest.skip("没有 g++/c++，跳过（板上有 aarch64-linux-gnu-g++）")
    out = tmp_path_factory.mktemp("board") / "test_segment_merge"
    r = subprocess.run(
        [gxx, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
         "-I", str(BOARD), "-I", str(CPP), "-o", str(out), str(SRC)],
        capture_output=True, text=True)
    assert r.returncode == 0, f"编译失败：\n{r.stdout}\n{r.stderr}"
    return out


def test_segment_merge_self_test_passes(merge_exe: pathlib.Path) -> None:
    r = subprocess.run([str(merge_exe)], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout
    n = int(r.stdout.split("比对项 ")[1].split("，")[0])
    assert n >= 15, f"只跑了 {n} 项比对，太少"


def test_main_merges_before_tracking_and_has_a_switch() -> None:
    """合并必须在**障碍跟踪之前**（速度要按合并后的墙推），而且能关掉。"""
    src = (BOARD / "main.cpp").read_text(encoding="utf-8")
    assert '#include "segment_merge.hpp"' in src
    assert "--merge-gap-px" in src and "--merge-dy-px" in src, "要有开关"
    i_merge = src.index("segment_merge::merge_collinear")
    i_track = src.index("st.obstacle_motion.update")
    assert i_merge < i_track, "先合并再推速度"
