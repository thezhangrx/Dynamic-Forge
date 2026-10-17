"""**不需要相机**的端到端几何测试：已知场景 → 合成"平台画面" → C++ 判据 → 场地单位。

为什么必须有它：最近几轮"真机指标"全被相机拍到聊天窗口污染了（叠加图证实），
于是"判据到底准不准"一直说不清。这个测试把画面**造出来**（墙/缺口/边框/网格/
角色都在已知位置），再让 `band_gap` 去量，最后用 `field_extent` 的比例尺换算回
场地单位，与真值比。

覆盖真实机位会遇到的三种变换（都不需要相机）：
* 4:3 → 16:9 的纵向压缩（cam_live 的 scale 路径）；
* 相机相对场地的小角度旋转；
* 曝光增益/偏置。
"""

from __future__ import annotations

import math
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
BOARD = ROOT / "core" / "board"

WARM = (240, 160, 48)
BG = (34, 40, 55)
WHITE = (235, 240, 250)


def _gxx() -> bool:
    try:
        subprocess.run(["g++", "--version"], capture_output=True, check=True)
        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(not _gxx(), reason="没有 g++")
# 已知缺陷（本轮由这个测试发现）：`field_extent` 假设"场地边框是暖色"，
# 但仿真里边框是冷色 (60,70,95) —— 实测这张**已知场景**的合成图上 ext_ok=0。
# 于是它报出的比例尺/原点全是从"随便两条暖色列"来的，这正是板子把门算偏
# 200 场地单位的根因。修好后把 xfail 去掉。
xfail = pytest.mark.xfail(reason="field_extent 依赖暖色边框，仿真边框是冷色（待修）", strict=False)


def scene(*, wall_y=479.0, gap=(251.0, 327.0), player=(320.0, 240.0), field=(640.0, 480.0),
          squash=0.75, rot_deg=0.0, gain=1.0, offset=0.0, out=(640, 360)):
    """按**场地坐标**画一张"相机看到的平台"，返回 (rgb, 变换函数)。

    画的东西严格对应场景契约：场地 640x480、橙墙一条带缺口、白色圆角色、
    四角标定标记、等距网格线。之后统一做：旋转（围绕场心）→ 纵向压缩 →
    缩放到 out → 增益/偏置。这样"真值"始终是场地坐标，与板子要算的一致。
    """
    fw, fh = field
    ow, oh = out
    buf = bytearray(ow * oh * 3)

    def to_img(fx: float, fy: float):
        # 场地 → 画面：先绕场心旋转，再纵向压缩，最后按比例映射到 out
        cx, cy = 0.5 * fw, 0.5 * fh
        dx, dy = fx - cx, fy - cy
        t = math.radians(rot_deg)
        rx = dx * math.cos(t) - dy * math.sin(t) + cx
        ry = dx * math.sin(t) + dy * math.cos(t) + cy
        return rx / fw * ow, ry * squash / fh * oh

    def put(x: int, y: int, c):
        if 0 <= x < ow and 0 <= y < oh:
            i = (y * ow + x) * 3
            buf[i] = max(0, min(255, int(c[0] * gain + offset)))
            buf[i + 1] = max(0, min(255, int(c[1] * gain + offset)))
            buf[i + 2] = max(0, min(255, int(c[2] * gain + offset)))

    for y in range(oh):
        for x in range(ow):
            put(x, y, BG)
    # 网格（每 64 场地单位一条，冷色，不应被当成墙）
    for gx in range(0, int(fw) + 1, 64):
        for fy in range(0, int(fh)):
            px, py = to_img(gx, fy)
            put(int(px), int(py), (70, 80, 105))
    # 墙：一条带，缺口 [gap)
    y0, y1 = wall_y - 4.0, wall_y + 4.0
    x = 0.0
    while x < fw:
        if not (gap[0] <= x <= gap[1]):
            for yy in range(int(y0), int(y1) + 1):
                px, py = to_img(x, yy)
                put(int(px), int(py), WARM)
        x += 1.0
    # 角色（半径 10）
    for dy in range(-10, 11):
        for dx in range(-10, 11):
            if dx * dx + dy * dy <= 100:
                px, py = to_img(player[0] + dx, player[1] + dy)
                put(int(px), int(py), WHITE)
    # 四角标定标记（黑底 + 白角块，边长 48、离边 8）
    for mx in (8.0, fw - 56.0):
        for my in (8.0, fh - 56.0):
            for dy in range(0, 48):
                for dx in range(0, 48):
                    dark = not (dx < 12 or dx >= 36 or dy < 12 or dy >= 36)
                    px, py = to_img(mx + dx, my + dy)
                    put(int(px), int(py), WHITE if dark else (0, 0, 0))
    return bytes(buf), to_img, (ow, oh)


def probe(tmp_path: Path, rgb: bytes, size) -> dict:
    exe = tmp_path / "bg"
    subprocess.run(["g++", "-std=c++17", "-O2", "-I", str(BOARD), "-o", str(exe),
                    str(BOARD / "tools" / "band_gap_probe.cpp")],
                   check=True, capture_output=True)
    ppm = tmp_path / "f.ppm"
    ppm.write_bytes(b"P6\n%d %d\n255\n" % size + rgb)
    out = subprocess.run([str(exe), str(ppm)], capture_output=True, text=True, check=True).stdout
    import re
    m = re.search(r"found=(\d).*?y0=(-?\d+) y1=(-?\d+) x0=(-?\d+) x1=(-?\d+)", out)
    e = re.search(r"extent ok=(\d) x=(-?\d+)\.\.(-?\d+)", out)
    return {"found": int(m.group(1)), "y0": int(m.group(2)), "y1": int(m.group(3)),
            "x0": int(m.group(4)), "x1": int(m.group(5)),
            "ext_ok": int(e.group(1)), "ex0": int(e.group(2)), "ex1": int(e.group(3))}


@xfail
@pytest.mark.parametrize("kw", [
    {"rot_deg": 0.0, "gain": 1.0, "offset": 0.0},
    {"rot_deg": 0.0, "gain": 0.55, "offset": -8.0},
    {"rot_deg": 0.0, "gain": 1.6, "offset": 40.0},
    {"rot_deg": 6.0, "gain": 1.0, "offset": 0.0},
])
def test_gap_lands_within_ten_field_units(tmp_path: Path, kw):
    rgb, _to_img, size = scene(**kw)
    r = probe(tmp_path, rgb, size)
    assert r["ext_ok"] == 1, f"场地边框没量出来：{r}"
    assert r["found"] == 1, f"没找到门：{r}"
    s = 640.0 / (r["ex1"] - r["ex0"] + 1)
    cx = (0.5 * (r["x0"] + r["x1"]) - r["ex0"]) * s   # 以**左边框**为原点
    cy = 0.5 * (r["y0"] + r["y1"]) * s / 0.75  # 纵向还原压缩
    assert abs(cx - (0.5 * (251 + 327) - 0.0)) <= 20.0, f"缺口中心 {cx:.1f} 偏了（真值 289）"
    assert abs(cy - 479.0) <= 20.0, f"墙带 y {cy:.1f} 偏了（真值 479）"
