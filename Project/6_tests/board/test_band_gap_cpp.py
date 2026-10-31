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


def run_probe(exe: Path, paths, *, angle0: bool = True) -> dict:
    extra = ["--angle0"] if angle0 else []
    out = subprocess.run([str(exe)] + extra + [str(p) for p in paths],
                         capture_output=True, text=True, check=True).stdout
    got = {}
    for line in out.strip().splitlines():
        if not line.startswith("/"):        # 探测工具还会打一行缩进的 extent 诊断
            continue
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


def shear(frame, deg: float):
    """把画面沿 x 方向剪切（等价于场景整体旋转，小角度下与旋转同构）。"""
    import math

    from vision import primitives as P

    w, h = frame.width, frame.height
    t = math.tan(math.radians(deg))
    out = bytearray(w * h * 3)
    for x in range(w):
        dy = int(round((x - w / 2) * t))
        for y in range(h):
            sy = min(max(y + dy, 0), h - 1)
            si = (sy * w + x) * 3
            di = (y * w + x) * 3
            out[di:di + 3] = bytes((frame.r[sy * w + x], frame.g[sy * w + x], frame.b[sy * w + x]))
    return P.Frame(width=w, height=h, r=bytes(out[0::3]), g=bytes(out[1::3]), b=bytes(out[2::3]))


def test_angle_search_recovers_a_rotated_wall(tmp_path: Path):
    """真机位画面相对场地有 5~10° 旋转：0° 找门会失败，角度搜索要能找回来。

    实测动机见 core/board/docs/CAR_MIGRATION.md §4：斜墙在水平行带里每行只剩很短
    一段，左右肩被带截断，支撑比永远过不了 1。
    """
    exe = build_probe(tmp_path)
    tilted = write_ppm(shear(field(), 8.0), tmp_path / "tilt8.ppm")
    flat = write_ppm(field(), tmp_path / "flat.ppm")

    plain = run_probe(exe, [tilted], angle0=True)[str(tilted)]
    assert int(plain["found"]) == 0, "8° 斜墙上 0° 判据不该找到门（这正是问题本身）"

    searched = run_probe(exe, [tilted], angle0=False)[str(tilted)]
    assert int(searched["found"]) == 1, "角度搜索应当把斜墙找回来"
    assert abs(float(searched["w"]) - 45.0) <= 6.0, f"缺口宽 {searched['w']} 与真值 45 差太多"
    # 水平的那张不该被角度搜索带偏
    ok = run_probe(exe, [flat], angle0=False)[str(flat)]
    assert int(ok["found"]) == 1 and abs(float(ok["w"]) - 45.0) <= 6.0
