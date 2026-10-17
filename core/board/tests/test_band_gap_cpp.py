"""`band_gap.hpp`（C++ 结构判据）与 `core/vision/relative.py` 的 `best_door()` **逐项对账**。

为什么要钉这一条：板子上跑的是 C++（`band_gap.hpp`），离线分析和快改用的是 Python
（`relative.py`）。两边一旦悄悄分家，"Python 上验证通过"就一文不值。所以这里在
同一批合成帧 + 真机位帧上把两边的 `found/bands/y0/y1/x0/x1/support` 全部比一遍。

合成帧覆盖真实会遇到的几种情况：干净、曝光偏暗 + 噪声 + MJPG、缺口窄到不该算门。
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]          # 仓库根
BOARD = ROOT / "core" / "board"


def _have_gxx() -> bool:
    try:
        subprocess.run(["g++", "--version"], capture_output=True, check=True)
        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(not _have_gxx(), reason="没有 g++，跳过 C++ 对账")


def build_probe(tmp_path: Path) -> Path:
    exe = tmp_path / "band_gap_probe"
    src = BOARD / "tools" / "band_gap_probe.cpp"
    subprocess.run(["g++", "-std=c++17", "-O2", "-I", str(BOARD), "-o", str(exe), str(src)],
                   check=True, capture_output=True)
    return exe


def run_probe(exe: Path, paths) -> dict:
    out = subprocess.run([str(exe)] + [str(p) for p in paths],
                         capture_output=True, text=True, check=True).stdout
    got = {}
    for line in out.strip().splitlines():
        parts = line.split()
        name = parts[0]
        fields = {}
        for kv in parts[1:]:
            k, _, v = kv.partition("=")
            fields[k] = float(v) if "." in v else int(v)
        got[name] = fields
    return got


def write_ppm(frame, path: Path) -> Path:
    data = bytearray(b"P6\n%d %d\n255\n" % (frame.width, frame.height))
    data += bytes(frame.to_rgb())
    path.write_bytes(bytes(data))
    return path


def field(w=320, h=180, band=(30, 36), walls=((0, 139), (185, 319))):
    from vision import primitives as P

    warm, bg = (240, 160, 48), (34, 40, 55)
    rgb = bytearray()
    for y in range(h):
        for x in range(w):
            c = bg
            if band[0] <= y <= band[1]:
                for a, b in walls:
                    if a <= x <= b:
                        c = warm
                        break
            rgb += bytes(c)
    return P.Frame(width=w, height=h, r=rgb[0::3], g=rgb[1::3], b=rgb[2::3])


def test_cpp_matches_python_on_synthetic_and_real_frames(tmp_path: Path):
    from vision import camera as C
    from vision import primitives as P
    from vision.relative import best_door

    exe = build_probe(tmp_path)
    frames = {
        "clean": field(),
        "dark_noisy_jpeg": C.apply(field(), C.CameraParams(
            gain=0.55, offset=-8, noise=4.0, jpeg=60, seed=3)),
        "gap_too_narrow": C.apply(field(walls=((0, 158), (163, 319))),
                                 C.CameraParams(noise=3.0, seed=1)),
    }
    paths = [write_ppm(f, tmp_path / f"{k}.ppm") for k, f in frames.items()]
    real = sorted((ROOT / "data" / "fpga" / "run4_720to360").glob("*.ppm"))[:2]
    paths += real

    got = run_probe(exe, paths)
    for path in paths:
        frame = P.Frame.from_rgb(
            *(lambda d: (int(d.split(b"\n")[1].split()[0]), int(d.split(b"\n")[1].split()[1]),
                         d.split(b"\n", 3)[3]))(path.read_bytes()))
        ref = best_door(frame)
        c = got[str(path)]
        assert int(c["found"]) == int(bool(ref["found"])), f"{path.name}: found 不一致"
        assert int(c["bands"]) == len(ref["bands"]), f"{path.name}: 带数不一致"
        if ref["found"]:
            for key, pykey in (("y0", "y0"), ("y1", "y1"), ("x0", "x0"), ("x1", "x1")):
                assert abs(int(c[key]) - int(ref[pykey])) <= 1, f"{path.name}: {key} 不一致"
            assert abs(c["support"] - ref["support"]) <= 0.02, f"{path.name}: support 不一致"
