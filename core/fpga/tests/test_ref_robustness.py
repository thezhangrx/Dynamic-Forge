#!/usr/bin/env python3
"""C 参考模型的**健壮性**：不许崩，也不许吐出违反协议不变量的帧。

为什么单开一个文件
------------------
`core/fpga/tests/test_vision_ref.py` 管的是"算得对不对"（与 Python 语义真值对账），
用的是**合成/录像**语料。这批语料有个共同的盲区：**它永远产出结构良好的输入**。

真实照片不是。它一喂就露出两个**既有**缺陷：

1. **越界读导致段错误**（`vr_extract_player`）
   亮掩膜一大，连通域标签表就被打满；被打满的游程 `label` 保持 `-1`，
   却被 `memcpy` 带进 `prev[]`，下一行就 `uf_find(-1)` → 读 `uf_parent[-1]`。
   在 `-fPIC` 的 `.so` 里侥幸不崩、在 `-O2` 独立二进制下**段错误** —— 典型 UB。
   实测：`data/vision/_flash_after.jpg`、`_flash_during.jpg` 触发。

2. **不变量 `half_w >= half_h` 被破坏**
   检测器为了找缺口会把墙切成碎片；**短于厚度**的段会让 PS 侧
   `decode_vision_frame().validate()` 当场抛异常 —— **整条链断**。
   实测：`_capture.jpg`、`_capture_big.jpg`、`calib_20261006_1441/1442*.jpg`。

两条都在 `vision_ref.c` 里修掉了。本文件是它们的守卫。

两类语料，各有分工
------------------
* **对抗图**（本文件自己生成，**永远跑**）：确定性触发第 1 条。
  旧代码在 `grid_bright` / `noise_*` 上**必崩**。
* **真实照片** `data/vision/*.jpg`（**有就跑，没有就 skip**）：覆盖第 2 条。
  `data/` 是 gitignore 的，队友那边没有 —— 所以这条是 skip 而不是失败，
  但**只要照片在，它就守着**。
"""

from __future__ import annotations

import pathlib
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
REF_C = ROOT / "core" / "fpga" / "ref" / "vision_ref.c"
PHOTOS = ROOT / "data" / "vision"


# ---------------------------------------------------------------------------
# 语料
# ---------------------------------------------------------------------------
def _adversarial(dirpath: pathlib.Path) -> list[pathlib.Path]:
    """造一组对抗图。**自己生成**，不依赖任何数据文件。

    覆盖的是"现实里不可能渲染出来、但相机一抖就有"的输入：
    纯色、随机噪声、密集亮点网格、横向条纹。
    """
    import cv2
    import numpy as np

    dirpath.mkdir(parents=True, exist_ok=True)
    H, W = 480, 640
    rng = np.random.default_rng(7)
    made: list[pathlib.Path] = []

    def save(name: str, img) -> None:
        p = dirpath / f"{name}.ppm"
        cv2.imwrite(str(p), img)
        made.append(p)

    save("white", np.full((H, W, 3), 255, np.uint8))
    save("black", np.zeros((H, W, 3), np.uint8))
    wall = np.zeros((H, W, 3), np.uint8)
    wall[:, :, 0], wall[:, :, 1], wall[:, :, 2] = 48, 160, 240   # BGR 橙
    save("wall_full", wall)
    # 密集亮点网格：亮掩膜极大 → 连通域标签表被打满（这正是段错误的触发条件）
    grid = np.zeros((H, W, 3), np.uint8)
    grid[:, :, 0], grid[:, :, 1], grid[:, :, 2] = 48, 160, 240
    for y in range(0, H, 12):
        for x in range(0, W, 12):
            grid[y:y + 8, x:x + 8] = 255
    save("grid_bright", grid)
    # 随机噪声：不同密度都来一份
    for d in (0.10, 0.25, 0.50, 0.80):
        save(f"noise_{int(d * 100)}", (rng.random((H, W, 3)) < d).astype(np.uint8) * 255)
    save("stripes_h", np.tile(
        np.array([[0, 0, 0], [255, 255, 255]], np.uint8).reshape(2, 1, 3), (H // 2, W, 1))[:H])
    return made


@pytest.fixture(scope="module")
def ref_exe(tmp_path_factory: pytest.TempPathFactory) -> pathlib.Path:
    out = tmp_path_factory.mktemp("refrobust") / "vision_ref"
    r = subprocess.run(["gcc", "-O2", "-Wall", "-o", str(out), str(REF_C), "-lm"],
                       capture_output=True, text=True)
    assert r.returncode == 0, f"编译 C 参考模型失败：\n{r.stderr}"
    return out


def _run_ppm(exe: pathlib.Path, ppm: pathlib.Path, out_bin: pathlib.Path):
    """跑一帧，返回 (退出码, 帧字节)。"""
    r = subprocess.run([str(exe), str(ppm), str(out_bin)], capture_output=True)
    data = out_bin.read_bytes() if out_bin.exists() else b""
    return r.returncode, data


# ---------------------------------------------------------------------------
# 1) 对抗图：**不许崩**
# ---------------------------------------------------------------------------
def test_adversarial_images_do_not_crash(ref_exe, tmp_path):
    """对抗图上一个都不许崩。

    旧代码在 `grid_bright` 与 `noise_25/50/80` 上 **SIGSEGV（退出码 139）** ——
    亮掩膜把连通域标签表打满，`label=-1` 的游程被带进下一行，`uf_find(-1)`
    越界读 `uf_parent[-1]`。这是**必须用退出码判的**：不崩才谈得上后面的校验。
    """
    bad = []
    for ppm in _adversarial(tmp_path / "adv"):
        code, _ = _run_ppm(ref_exe, ppm, tmp_path / "o.bin")
        if code != 0:
            bad.append(f"{ppm.stem}: 退出码 {code}"
                       + ("（SIGSEGV，越界读）" if code == -11 or code == 139 else ""))
    assert not bad, "对抗图让 C 参考模型崩了：\n  " + "\n  ".join(bad)


# ---------------------------------------------------------------------------
# 2) 对抗图 + 真实照片：**产出的帧必须过协议校验**
# ---------------------------------------------------------------------------
def _assert_frames_valid(ref_exe, ppts: list[pathlib.Path], tmp_path) -> None:
    from standard.vision.vision_frame import decode_vision_frame

    bad = []
    for ppm in ppts:
        code, data = _run_ppm(ref_exe, ppm, tmp_path / "o.bin")
        if code != 0:
            bad.append(f"{ppm.stem}: 退出码 {code}")
            continue
        try:
            decode_vision_frame(data)
        except Exception as exc:                       # noqa: BLE001
            bad.append(f"{ppm.stem}: {exc}")
    assert not bad, (
        "C 参考模型吐出了 PS 侧解不出的帧 —— 板上这条链会直接断：\n  "
        + "\n  ".join(bad))


def test_adversarial_frames_pass_protocol_validation(ref_exe, tmp_path):
    _assert_frames_valid(ref_exe, _adversarial(tmp_path / "adv"), tmp_path)


def test_real_photos_pass_protocol_validation(ref_exe, tmp_path):
    """真实照片（`data/vision/*.jpg`）产出的帧必须过校验。

    **没有照片就 skip** —— `data/` 是 gitignore 的，队友那边拿不到。
    但只要有照片，这条就守着；实测旧代码在这里挂 6 张
    （2 张段错误 + 4 张 `half_w < half_h`）。
    """
    import cv2

    photos = sorted(PHOTOS.glob("*.jpg"))
    if not photos:
        pytest.skip(f"没有真实照片（{PHOTOS} 为空；data/ 是 gitignore 的）")

    converted: list[pathlib.Path] = []
    for i, p in enumerate(photos):
        im = cv2.imread(str(p))
        if im is None:
            continue
        h, w = im.shape[:2]
        if w > 640:                                  # C 模型上限 640×480
            s = 640.0 / w
            im = cv2.resize(im, (640, int(h * s)), interpolation=cv2.INTER_AREA)
        q = tmp_path / f"photo_{i:03d}.ppm"
        cv2.imwrite(str(q), im)
        converted.append(q)
    assert converted, "照片一张都没读进来"
    _assert_frames_valid(ref_exe, converted, tmp_path)
