#!/usr/bin/env python3
"""四角标定标记 footprint 判据的 C++ 自测接进 pytest。

板子可能不在手边（掉电/在别人那里），但"哪些位置会被当成标定标记"这条判据必须
随时能验：判据错了的后果很具体 —— 真角色在角落附近时被当成标记（动作发不出去），
或者标记没被排除（角色被推着往边上钻）。
"""

from __future__ import annotations

import pathlib
import shutil
import subprocess

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
BOARD = ROOT / "core" / "board"
SRC = BOARD / "tests" / "test_fiducial_zone.cpp"


@pytest.fixture(scope="module")
def zone_exe(tmp_path_factory: pytest.TempPathFactory) -> pathlib.Path:
    gxx = shutil.which("g++") or shutil.which("c++")
    if gxx is None:
        pytest.skip("没有 g++/c++，跳过（板上有 aarch64-linux-gnu-g++）")
    out = tmp_path_factory.mktemp("board") / "test_fiducial_zone"
    r = subprocess.run(
        [gxx, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
         "-I", str(BOARD), "-o", str(out), str(SRC)],
        capture_output=True, text=True)
    assert r.returncode == 0, f"编译失败：\n{r.stdout}\n{r.stderr}"
    return out


def test_fiducial_zone_self_test_passes(zone_exe: pathlib.Path) -> None:
    r = subprocess.run([str(zone_exe)], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout
    n = int(r.stdout.split("比对项 ")[1].split("，")[0])
    assert n >= 15, f"只跑了 {n} 项比对，太少"


def test_main_uses_the_shared_zone_helper() -> None:
    """判据必须**只有一份**：`main.cpp` 只调用 `fiducial_zone.hpp`，别再内联一套。"""
    src = (BOARD / "main.cpp").read_text(encoding="utf-8")
    assert '#include "fiducial_zone.hpp"' in src, "main.cpp 应当用共享判据"
    assert "zone.contains(" in src, "main.cpp 应当调用 Zone::contains"
    assert "fiducial_side" in src and "--fiducial-side" in src, "标记尺寸要能配"
