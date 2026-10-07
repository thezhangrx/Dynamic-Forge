"""C 参考模型与 Python 契约的**跨语言核对**。

验证两件不需要真实相机、也不需要 Verilator 的事：

1. **线格式一致**：C 用自己手写的 CRC32 打包，Python 用 zlib 解 —— 两边
   魔术、版本、长度、CRC 必须全都过。这是 P2（RTL 逐位比对）的前提：
   如果 C 模型的输出 Python 都认不出，RTL 比对就无从谈起。
2. **矩/质心算术正确**：给一个位置已知的亮方块，C 模型算出的质心与
   半径必须等于解析值。这条能抓住定标、整除方向、四舍五入这几类错。

需要 `gcc`；没有就跳过（不假装通过）。
"""

from __future__ import annotations

import pathlib
import shutil
import subprocess

import pytest

from standard.vision.vision_frame import VISION_Q, decode_vision_frame, vision_frame_size

_REF_C = pathlib.Path(__file__).resolve().parents[1] / "ref" / "vision_ref.c"

pytestmark = pytest.mark.skipif(
    shutil.which("gcc") is None or not _REF_C.exists(),
    reason="需要 gcc 与 core/fpga/ref/vision_ref.c")


def _write_pgm(path: pathlib.Path, w: int, h: int, bright_box=None) -> None:
    """P5 PGM，全黑底；可选一个纯白矩形（bright 判据要 min>195）。"""
    img = bytearray(w * h)
    if bright_box is not None:
        x0, y0, x1, y1 = bright_box
        for y in range(y0, y1):
            for x in range(x0, x1):
                img[y * w + x] = 255
    with open(path, "wb") as fp:
        fp.write(b"P5\n%d %d\n255\n" % (w, h))
        fp.write(bytes(img))


@pytest.fixture(scope="module")
def ref_bin(tmp_path_factory) -> pathlib.Path:
    out = tmp_path_factory.mktemp("refbuild") / "vision_ref"
    r = subprocess.run(["gcc", "-O2", "-Wall", "-Wextra", "-std=c11",
                        "-o", str(out), str(_REF_C)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        pytest.fail(f"C 参考模型编译失败：\n{r.stderr}")
    assert "warning" not in r.stderr, f"编译有警告（按本项目的标准不算通过）：\n{r.stderr}"
    return out


def test_c_model_output_is_decodable_by_python(ref_bin, tmp_path) -> None:
    """C 打包 -> Python 解码：魔术/版本/长度/CRC 必须全过。"""
    pgm = tmp_path / "f.pgm"
    _write_pgm(pgm, 64, 48, bright_box=(20, 10, 40, 30))
    out = tmp_path / "f.bin"
    r = subprocess.run([str(ref_bin), str(pgm), str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr

    raw = out.read_bytes()
    assert len(raw) == vision_frame_size(0, 0), "长度公式与 Python 不一致"
    frame = decode_vision_frame(raw)          # 任何不一致都会抛
    assert frame.seq == 0
    assert frame.image_width == 64 and frame.image_height == 48


def test_c_model_centroid_matches_the_analytic_answer(ref_bin, tmp_path) -> None:
    """已知位置的亮方块 -> 质心与半径必须等于解析值。

    方块 x∈[20,40) y∈[10,30)：质心 (29.5, 19.5)，外接框 20×20 ->
    半径 (w+h)/4 = 10。定标 Q=64，编码后解码应当回到这几个数。
    """
    pgm = tmp_path / "box.pgm"
    _write_pgm(pgm, 64, 48, bright_box=(20, 10, 40, 30))
    out = tmp_path / "box.bin"
    assert subprocess.run([str(ref_bin), str(pgm), str(out)],
                          capture_output=True).returncode == 0
    frame = decode_vision_frame(out.read_bytes())
    assert frame.player is not None, "整块亮方块应当被当成一个角色候选"
    assert frame.player.x == pytest.approx(29.5, abs=1.0 / VISION_Q)
    assert frame.player.y == pytest.approx(19.5, abs=1.0 / VISION_Q)
    assert frame.player.radius == pytest.approx(10.0, abs=2.0 / VISION_Q)


def test_c_model_ignores_an_empty_frame(ref_bin, tmp_path) -> None:
    """全黑帧：没有亮块 -> 不该报出角色（宁可少报，不要编一个）。"""
    pgm = tmp_path / "dark.pgm"
    _write_pgm(pgm, 64, 48, bright_box=None)
    out = tmp_path / "dark.bin"
    assert subprocess.run([str(ref_bin), str(pgm), str(out)],
                          capture_output=True).returncode == 0
    frame = decode_vision_frame(out.read_bytes())
    assert frame.player is None
    assert frame.obstacles == [] and frame.gaps == []
