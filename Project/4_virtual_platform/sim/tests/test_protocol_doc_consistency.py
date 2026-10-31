"""`docs/PROTOCOL.md` 必须与代码一致。

为什么单开一个文件
------------------
* `test_protocol.py` 钉的是**代码**（`version == 3`、`BULLET_STRIDE == 17`、
  `frame_size = 88 + stride*4*n`）；
* `test_readme_consistency.py` 钉的是 **README**；
* 而 `docs/PROTOCOL.md` 开篇自称"模拟器与硬件/外部进程之间的**唯一契约**"，
  **却一条测试都没有**。

后果是实打实的：v3（stride 13 → 17、帧长 `88+52n` → `88+68n`）上线很久之后，
这份文档的标题还写着 "(v2)"、bullets 行还写着 `float32[n][13]`、
总长度还写着 `88 + 52 × n`。**一份错的"唯一契约"比没有更糟** ——
组员会照着它写出错的解析器，而且因为文档说得斩钉截铁，很难被怀疑。

这个文件把文档里**可机检的部分**钉住：版本号、stride、帧长、字段顺序。
"""
from __future__ import annotations

import pathlib
import re
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
if str(REPO_ROOT / "core") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "core"))

DOC = REPO_ROOT / "core" / "bullet_sim" / "docs" / "PROTOCOL.md"


@pytest.fixture(scope="module")
def text() -> str:
    return DOC.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def proto():
    from bullet_sim.interface import protocol as m

    return m


# ---------------------------------------------------------------------------
# 1) 标题与 header 表里的版本号
# ---------------------------------------------------------------------------
def test_title_announces_the_current_version(text: str, proto) -> None:
    title = text.splitlines()[0]
    m = re.search(r"\(v(\d+)\)", title)
    assert m, f"标题里没有 (vN)：{title!r}"
    assert int(m.group(1)) == proto.STATE_PROTOCOL_VERSION, (
        f"标题写着 {title!r}，而 STATE_PROTOCOL_VERSION = "
        f"{proto.STATE_PROTOCOL_VERSION}"
    )


def test_header_table_version_field_matches_code(text: str, proto) -> None:
    m = re.search(r"\|\s*4\s*\|\s*2\s*\|\s*uint16\s*\|\s*version\s*=\s*(\d+)", text)
    assert m, "header 表里找不到 `version = N` 那一行"
    assert int(m.group(1)) == proto.STATE_PROTOCOL_VERSION


# ---------------------------------------------------------------------------
# 2) 字段顺序（v3 那一块代码围栏）
# ---------------------------------------------------------------------------
def test_v3_field_order_matches_code(text: str, proto) -> None:
    m = re.search(r"弹幕行字段顺序（`stride = (\d+)`）：\s*```\s*(.*?)```",
                  text, re.S)
    assert m, "找不到「弹幕行字段顺序」那段代码围栏"
    assert int(m.group(1)) == proto.BULLET_STRIDE, (
        f"文档说 stride = {m.group(1)}，代码是 {proto.BULLET_STRIDE}"
    )
    documented = tuple(
        w.strip() for w in m.group(2).replace("\n", " ").split(",") if w.strip()
    )
    assert documented == proto.fields_for(proto.STATE_PROTOCOL_VERSION), (
        "文档里的 v3 字段顺序与 PROTOCOL_FIELDS 不一致：\n"
        f"  文档：{documented}\n  代码：{proto.fields_for(proto.STATE_PROTOCOL_VERSION)}"
    )


# ---------------------------------------------------------------------------
# 3) 版本兼容表：stride 与帧长
# ---------------------------------------------------------------------------
def _version_rows(text: str) -> dict[int, tuple[int, int]]:
    """从「版本兼容表」里抽出 {version: (stride_floats, bytes_per_bullet)}。

    行形如：``| 3 | **17 floats (68 B)** | **`88 + 68n`** | ... |``
    """
    assert "版本兼容表" in text, "找不到「版本兼容表」"
    tail = text.split("版本兼容表", 1)[1]
    rows: dict[int, tuple[int, int]] = {}
    for line in tail.splitlines():
        if not line.startswith("|"):
            continue
        cells = [c.strip().strip("*").strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 3 or not cells[0].isdigit():
            continue
        ver = int(cells[0])
        sm = re.search(r"(\d+)\s*floats?\s*\(\s*(\d+)\s*B\s*\)", cells[1])
        fm = re.search(r"88\s*\+\s*(\d+)\s*n", cells[2])
        if sm and fm:
            rows[ver] = (int(sm.group(1)), int(fm.group(1)))
    return rows


def test_version_table_matches_code(text: str, proto) -> None:
    rows = _version_rows(text)
    assert rows, "版本兼容表一行都没解析出来（表格格式变了？）"

    version = proto.STATE_PROTOCOL_VERSION
    assert version in rows, f"版本兼容表里没有当前版本 v{version} 那一行"
    # 至少要把 v1..当前 都列全，否则"兼容表"名不副实
    assert set(range(1, version + 1)) <= set(rows), (
        f"版本兼容表缺行：{sorted(set(range(1, version + 1)) - set(rows))}"
    )

    for ver, (stride_floats, per_bullet) in sorted(rows.items()):
        want_stride = proto.stride_for(ver)
        assert stride_floats == want_stride, (
            f"v{ver}：文档写 stride {stride_floats} floats，代码是 {want_stride}"
        )
        assert per_bullet == want_stride * 4, (
            f"v{ver}：文档写每条弹幕 {per_bullet} 字节，代码是 {want_stride * 4}"
        )
        # 帧长公式：空帧 88 字节（84 头 + 4 CRC）
        assert proto.frame_size(0, ver) == 88
        assert proto.frame_size(7, ver) == 88 + per_bullet * 7


# ---------------------------------------------------------------------------
# 4) 布局表里的关键常量
# ---------------------------------------------------------------------------
def test_layout_table_sizes_match_code(text: str, proto) -> None:
    # 头 84 字节（32 定长头 + player 7 + target 6 个 float32）
    assert proto.HEADER_BYTES == 84
    assert re.search(r"\|\s*84\s*\|", text), "布局表里没有 offset 84 那一行"
    # CRC 尾 4 字节
    assert re.search(r"\|\s*84\s*\+\s*68n\s*\|\s*4\s*\|", text), \
        "布局表里没有 `84+68n | 4 | uint32 | crc32` 那一行"

    stride = proto.BULLET_STRIDE
    # 布局表的 `size` 列是**字节**（68·n），`type` 列才是浮点数个数（float32[n][17]）。
    assert re.search(
        rf"\|\s*84\s*\|\s*{stride * 4}\s*·\s*n\s*\|\s*float32\[n\]\[{stride}\]", text
    ), f"布局表的 bullets 行没写成 `{stride * 4}·n` / `float32[n][{stride}]`"

    total = f"88 + {stride * 4} × n_bullets"
    assert total in text, f"总长度没写成 `{total} 字节`"


def test_magic_matches_code(text: str, proto) -> None:
    assert f"{proto.MAGIC:08X}" in text.upper(), (
        f"文档里找不到 magic {proto.MAGIC:#010x}"
    )


# ---------------------------------------------------------------------------
# 5) DATASET.md 的弹幕列数 / 列顺序
#
# `dataset/schema.py` 的 `BULLET_COLUMNS = tuple(PROTOCOL_FIELDS)` 是跟着协议走的，
# 文档却停在 v1 的 10 列（`x,y,vx,vy,ax,ay,radius,ttl,type_id,group_id`）。
# 协议扩容两次（v2 +3、v3 +4）而文档没跟 —— 和本文档开头那件事是同一类。
# ---------------------------------------------------------------------------
DATASET_DOC = REPO_ROOT / "core" / "bullet_sim" / "docs" / "DATASET.md"


def test_dataset_doc_bullet_columns_match_code(proto) -> None:
    text = DATASET_DOC.read_text(encoding="utf-8")

    m = re.search(r"\|\s*`bullets`\s*\|\s*\(Σn_t,\s*\**\s*(\d+)\s*\**\s*\)", text)
    assert m, "DATASET.md 里找不到 `bullets` 的形状那一行"
    assert int(m.group(1)) == proto.BULLET_STRIDE, (
        f"DATASET.md 写 bullets 有 {m.group(1)} 列，代码是 {proto.BULLET_STRIDE}"
    )

    # 列顺序那段代码围栏：从「弹幕行」起找**第一个**围栏里的内容。
    # 不能写成 `[^\n]*?：` —— 冒号可能落在下一行（DATASET.md 里就是）。
    m2 = re.search(r"弹幕行[\s\S]*?```\s*\n(.*?)\n```", text, re.S)
    assert m2, "DATASET.md 里找不到弹幕列顺序那段代码围栏"
    documented = tuple(
        w.strip() for w in m2.group(1).replace("\n", " ").split(",") if w.strip()
    )
    assert documented == proto.fields_for(proto.STATE_PROTOCOL_VERSION), (
        "DATASET.md 的列顺序与 PROTOCOL_FIELDS 不一致：\n"
        f"  文档：{documented}\n  代码：{proto.fields_for(proto.STATE_PROTOCOL_VERSION)}"
    )


# ---------------------------------------------------------------------------
# 6) DESIGN.md 的 SPAWN_REGIONS 名单
#
# 文档一度写的是 `left/right/top/bottom/center_edge/...` —— `center_edge` 这个
# 名字在代码里**根本不存在**，而真正存在的 `farthest_side`/`nearest_side` 没写。
# 照文档去配 `--spawn-region` 会直接报非法值。
# ---------------------------------------------------------------------------
DESIGN_DOC = REPO_ROOT / "core" / "bullet_sim" / "docs" / "DESIGN.md"


def test_design_doc_spawn_regions_match_code() -> None:
    from bullet_sim.generators.patterns import SPAWN_REGIONS

    text = DESIGN_DOC.read_text(encoding="utf-8")
    m = re.search(r"SPAWN_REGIONS`?\s*（`([^`]+)`", text)
    assert m, "DESIGN.md 里找不到 SPAWN_REGIONS 的取值列表"
    documented = tuple(p.strip() for p in m.group(1).split("/") if p.strip())
    assert set(documented) == set(SPAWN_REGIONS), (
        "DESIGN.md 的 SPAWN_REGIONS 与代码不一致：\n"
        f"  文档：{sorted(documented)}\n  代码：{sorted(SPAWN_REGIONS)}"
    )
