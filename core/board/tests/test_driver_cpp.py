#!/usr/bin/env python3
"""把 ps 侧驱动的 C++ 自测接进 pytest。

`core/board/` 是 **PS 侧（板上）** 的代码，语言是 C++17 —— 依据是厂商示例
`fpai_demo_app/examples/.../src/adder_demo.cpp` 与它的 `CMakeLists.txt`
（`CMAKE_CXX_STANDARD 17` + `aarch64-linux-gnu-g++`）。

这条测试**不需要板子、不需要 icraft SDK**：驱动的顺序逻辑跑在
`MemoryRegIo`（一块假寄存器文件）上，模拟的 PL 在
`tests/test_vision_driver.cpp` 里。
"""

from __future__ import annotations

import pathlib
import shutil
import subprocess

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
BOARD = ROOT / "core" / "board"
SRC = BOARD / "tests" / "test_vision_driver.cpp"
# ⚠ 几何帧解码（`parse_header` 等）在 2026-10-12 从 vision_driver.cpp 搬到了
#   geometry_frame.cpp —— 那边才是线协议的归属。**两个源文件都要编**，
#   否则链接期报 `undefined reference to parse_header`。
#   这条只有在把 `core/board/tests` 加进 pyproject 的 `testpaths` 之后才被发现：
#   在那之前裸 `pytest` 根本不收集这个文件，链接坏了也没人知道。
IMPLS = [BOARD / "vision_driver.cpp", BOARD / "geometry_frame.cpp"]


def _gxx() -> str:
    found = shutil.which("g++") or shutil.which("c++")
    if not found:
        pytest.skip("没有 g++/c++，跳过（板上有 aarch64-linux-gnu-g++）")
    return found


@pytest.fixture(scope="module")
def driver_exe(tmp_path_factory: pytest.TempPathFactory) -> pathlib.Path:
    out = tmp_path_factory.mktemp("board") / "test_vision_driver"
    r = subprocess.run(
        [_gxx(), "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
         "-I", str(BOARD),
         "-o", str(out), str(SRC), *[str(p) for p in IMPLS]],
        capture_output=True, text=True)
    assert r.returncode == 0, f"编译失败：\n{r.stdout}\n{r.stderr}"
    return out


def test_driver_self_test_passes(driver_exe: pathlib.Path) -> None:
    """驱动自测全过（含配置顺序、取帧字节、帧头、两条反面控制）。"""
    r = subprocess.run([str(driver_exe)], capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout
    # 防止"一个检查都没跑"也报 0 不符 —— 和 FPGA 那边同一条规矩。
    n = int(r.stdout.split("比对项 ")[1].split("，")[0])
    assert n > 20, f"只跑了 {n} 项比对，太少"


def test_icraft_backend_is_separate_translation_unit() -> None:
    """icraft 依赖必须隔离在一个编译单元里。

    理由：只有板上才有 aarch64 的 icraft 运行时；把 `#include <icraft-xrt/...>`
    混进 `vision_driver.cpp`，本机就连驱动逻辑都测不了了。
    """
    main_src = (BOARD / "vision_driver.cpp").read_text(encoding="utf-8")
    assert "icraft-xrt" not in main_src, (
        "vision_driver.cpp 里出现了 icraft 依赖；它必须保持与本机可编译")
    icraft = BOARD / "vision_driver_icraft.cpp"
    assert icraft.exists(), "缺少 icraft 后端实现"
    assert "icraft-xrt" in icraft.read_text(encoding="utf-8")


def test_main_drops_frames_with_pixel_fifo_overflow() -> None:
    """像素 FIFO 溢出时，`main.cpp` 必须**丢掉这一帧**，不能只用日志提醒。

    为什么单独立一条：溢出时几何帧是"自洽但错"的（CRC 按 PL 实际产出算、长度也对），
    所以线协议校验**过得去**。如果只是打一行警告、继续拿它决策，就会发出一个
    "合法但错"的动作 —— 这类失败最难查。两处调用点（`--verify-both` 与普通
    `--via-pl`）都要有。
    """
    src = (BOARD / "main.cpp").read_text(encoding="utf-8")
    assert src.count("last_push_overflowed()") == 2, \
        "两处 run_frame_push 调用点都要检查溢出"
    assert src.count("++bad; continue;") >= 4, "溢出帧要计入 bad 并跳过本帧"
    # 驱动必须真的把状态暴露出来（而不是只有一句 fprintf）
    hpp = (BOARD / "vision_driver.hpp").read_text(encoding="utf-8")
    assert "bool last_push_overflowed() const" in hpp
    assert "last_push_buffered() const" in hpp
    cpp = (BOARD / "vision_driver.cpp").read_text(encoding="utf-8")
    assert "last_push_overflowed_ = (st & (1u << 16)) != 0u" in cpp, \
        "溢出状态要从 PIXSTAT 的 bit16 取（粘滞位）"
