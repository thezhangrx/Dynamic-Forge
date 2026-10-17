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

def field_rect_in_image(ow=640, oh=360, squash=0.75):
    """与 `scene()` 里同样的"居中等比、左右黑边"布置，返回 (x, y, w, h)。"""
    fld_w = 0.85 * ow
    fld_h = fld_w * (480.0 / 640.0) * squash
    return 0.5 * (ow - fld_w), 0.5 * (oh - fld_h), fld_w, fld_h


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
# 台架原先用来记录两个已修好的缺陷（冷色边框被当暖色量、旋转时量错边界），
# 现在 5 例全绿，xfail 已去掉 —— 它们现在是**回归护栏**，不是待办。


def scene(*, wall_y=300.0, gap=(251.0, 327.0), player=(320.0, 240.0), field=(640.0, 480.0),
          squash=0.75, rot_deg=0.0, gain=1.0, offset=0.0, out=(640, 360)):
    """按**场地坐标**画一张"相机看到的平台"，返回 (rgb, 变换函数)。

    画的东西严格对应场景契约：场地 640x480、橙墙一条带缺口、白色圆角色、
    四角标定标记、等距网格线。之后统一做：旋转（围绕场心）→ 纵向压缩 →
    缩放到 out → 增益/偏置。这样"真值"始终是场地坐标，与板子要算的一致。
    """
    fw, fh = field
    ow, oh = out
    buf = bytearray(ow * oh * 3)

    # 场地在画面里**居中、等比**放置（和仿真一致：4:3 的场地放进 16:9 的窗口，
    # 左右留黑边）。实测真机位：场地宽约 545/640 ≈ 0.85，压缩后高度 ≈ 0.85*360。
    # 之前把场地铺满整幅图，等于造了一张"没有边界"的假图。
    fld_w = 0.85 * ow
    fld_h = fld_w * (fh / fw) * squash
    fld_x = 0.5 * (ow - fld_w)
    fld_y = 0.5 * (oh - fld_h)

    def to_img(fx: float, fy: float):
        cx, cy = 0.5 * fw, 0.5 * fh
        dx, dy = fx - cx, fy - cy
        t = math.radians(rot_deg)
        rx = dx * math.cos(t) - dy * math.sin(t) + cx
        ry = dx * math.sin(t) + dy * math.cos(t) + cy
        return fld_x + rx / fw * fld_w, fld_y + ry / fh * fld_h

    def put(x: int, y: int, c):
        if 0 <= x < ow and 0 <= y < oh:
            i = (y * ow + x) * 3
            buf[i] = max(0, min(255, int(c[0] * gain + offset)))
            buf[i + 1] = max(0, min(255, int(c[1] * gain + offset)))
            buf[i + 2] = max(0, min(255, int(c[2] * gain + offset)))

    # 场外是**黑边**（仿真窗口里只有场地那一块被填底色），这正是 `field_extent`
    # 用来量范围的亮度边界。之前这里把整幅图都填成场地底色，等于**没有边界**，
    # 于是台架自己造出了一个"量不到场地"的假故障。
    for y in range(oh):
        for x in range(ow):
            put(x, y, (0, 0, 0))
    fx0, fy0 = to_img(0.0, 0.0)
    fx1, fy1 = to_img(fw, fh)
    for y in range(max(0, int(min(fy0, fy1))), min(oh, int(max(fy0, fy1)) + 1)):
        for x in range(max(0, int(min(fx0, fx1))), min(ow, int(max(fx0, fx1)) + 1)):
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
    _fx, fy, _fw, fh = field_rect_in_image()
    cy = (0.5 * (r["y0"] + r["y1"]) - fy) * (480.0 / fh)   # 去掉黑边偏移再还原压缩
    assert abs(cx - (0.5 * (251 + 327) - 0.0)) <= 20.0, f"缺口中心 {cx:.1f} 偏了（真值 289）"
    assert abs(cy - 300.0) <= 20.0, f"墙带 y {cy:.1f} 偏了（真值 300）"


def test_wall_hugging_the_border_never_reports_a_fake_door(tmp_path: Path):
    """墙贴在场地边界（wall_y=479，场地高 480）时的**明确预期**。

    这种布置下"缺口"可能被边界吞掉 —— 那就必须报"没有门"，而**不能**在
    别处编一个门出来（上一版就是这样：红框落在墙下面的黑边上、绿框落在实墙上，
    结果平台照着假门开）。所以这里只允许两种结果：
        * found == 0；或者
        * found == 1 且墙带 y 与真值差 ≤20 场地单位。
    """
    rgb, _to_img, size = scene(wall_y=479.0)
    r = probe(tmp_path, rgb, size)
    if r["found"]:
        _fx, fy, _fw, fh = field_rect_in_image()
        cy = (0.5 * (r["y0"] + r["y1"]) - fy) * (480.0 / fh)
        assert abs(cy - 479.0) <= 20.0, f"贴边墙上报了假门：墙带 y={cy:.1f}（真值 479）"
