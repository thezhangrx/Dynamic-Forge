"""仓库**布局契约**的守卫：根目录只放调用代码。

为什么需要它
------------
仓库约定是"``core/<模块>/`` 放源码，根目录只放 ``<模块>_*.py`` 调用脚本"。
这条约定被无声破坏过一次，而且破坏得很难看：``vision_reconcile.py`` 长到
1195 行、其中 78% 是非 main 逻辑（内容对齐、仿射漂移拟合、流式取帧、标定
全在里面），``cpu_vision_loop.py`` 578 行、58%。根脚本装成模块有三个后果：

* 别的模块想复用那些逻辑，只能 ``from vision_reconcile import ...``
  —— 根脚本之间互相 import，实测直接导致过一次**循环导入 ImportError**；
* ``pip install`` 装出来的包**不含根脚本**，于是"能用的逻辑"跟着脚本一起
  留在仓库外，可迁移性变差；
* 没有测试愿意去 import 一个脚本，逻辑因此长期没有单测。

守卫方式：**棘轮（ratchet）**。每个根脚本的"非 main 逻辑行数"记在
``_RATCHET`` 里，只许减不许增；不在表里的根脚本必须本来就薄。
这样"新写的根脚本"立刻受约束，"历史上就厚的"至少不会继续恶化，
而且剩下的欠账在表里是**看得见**的。

为什么放在 ``core/standard/tests/``
----------------------------------
根目录按同一条契约不能放 ``tests/``，所以这个守卫得住在某个模块里。
``core/standard/`` 是共享契约层 —— 而"仓库怎么摆"正是所有模块共同遵守的
契约，放这里比塞进任何一个业务模块都合适。
"""

from __future__ import annotations

import ast
import pathlib

import pytest

_ROOT = pathlib.Path(__file__).resolve().parents[3]

#: 每个根脚本允许的**非 main 函数/类**总行数上限（棘轮，只许减）。
#:
#: 目标统一是 **≤ 150 行**（那时根脚本就真的只剩调用+打印了）。
#: 表里这几项是历史欠账，降到目标之前不许再涨：
#:   * ``vision_reconcile.py`` 的 ``_run_real``（报告生成）+ ``run``（合成模式）
#:   * ``bullet_sim_run_demo.py`` 是平台侧自己的单文件入口，归平台模块管
#:   * ``vision_calibrate.py`` / ``vision_track.py`` 还带着旧的演示逻辑
_RATCHET: dict[str, int] = {
    "vision_reconcile.py": 424,
    "bullet_sim_run_demo.py": 273,
    "vision_calibrate.py": 111,
    "vision_track.py": 90,
}
#: 不在 ``_RATCHET`` 里的根脚本不得超过这个数（新脚本必须一开始就薄）
_DEFAULT_MAX = 150


def _nonmain_logic_lines(path: pathlib.Path) -> int:
    """非 ``main`` 的函数/类共占多少行 —— "这个脚本里装了多少模块"。"""
    tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
    return sum(
        n.end_lineno - n.lineno + 1
        for n in tree.body
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
        and n.name != "main"
    )


def _root_scripts() -> list[pathlib.Path]:
    return sorted(p for p in _ROOT.glob("*.py"))


def test_root_scripts_do_not_grow_into_modules() -> None:
    """根脚本的非 main 逻辑不得增长（棘轮）。"""
    over = []
    for p in _root_scripts():
        n = _nonmain_logic_lines(p)
        cap = _RATCHET.get(p.name, _DEFAULT_MAX)
        if n > cap:
            over.append(f"{p.name}: {n} 行 > 上限 {cap} 行")
    assert not over, (
        "根目录只放调用代码，但下面这些脚本里装了模块级逻辑：\n  "
        + "\n  ".join(over)
        + "\n\n正确做法：把逻辑移到 core/<模块>/ 下，根脚本只留 argparse + 调用 + 打印。\n"
        "如果是从脚本里搬走了逻辑，请顺手把 _RATCHET 里的数字调小（棘轮只许减）。"
    )


def test_root_scripts_never_import_each_other() -> None:
    """根脚本之间不许互相 import。

    它们不是包，没有稳定的导入名（安装后根本不存在），互相 import 会
    * 让"哪个脚本是入口"变得含糊；
    * 埋下**循环导入**（``cpu_vision_loop`` ↔ ``vision_reconcile`` 真的炸过）。

    共享的东西应当下沉到 ``core/`` 里的某个模块，两边都从那里取。
    """
    names = {p.stem for p in _root_scripts()}
    bad = []
    for p in _root_scripts():
        tree = ast.parse(p.read_text(encoding="utf-8", errors="replace"))
        for n in ast.walk(tree):
            mods: set[str] = set()
            if isinstance(n, ast.Import):
                mods = {a.name for a in n.names}
            elif isinstance(n, ast.ImportFrom) and n.module:
                mods = {n.module}
            for m in mods:
                if m.split(".")[0] in names and m.split(".")[0] != p.stem:
                    bad.append(f"{p.name} -> {m}")
    assert not bad, (
        "根脚本之间出现了 import：\n  " + "\n  ".join(sorted(set(bad)))
        + "\n\n把被复用的逻辑下沉到 core/<模块>/，两边都从那里导入。"
    )


@pytest.mark.parametrize("name", ["cpu_vision_loop.py", "vision_capture_rig.py"])
def test_refactored_entries_stay_thin(name: str) -> None:
    """已经瘦下来的入口**必须保持薄** —— 不靠棘轮，直接卡死。

    这两个是本轮收敛的成果（逻辑分别在 ``core/harness/loop.py`` 与
    ``core/vision/sources.py``、``core/bullet_sim/simulator/truth.py``），
    退回脚本里装模块是不可接受的。
    """
    p = _ROOT / name
    assert p.exists(), f"{name} 不见了"
    tree = ast.parse(p.read_text(encoding="utf-8", errors="replace"))
    classes = [n for n in tree.body if isinstance(n, ast.ClassDef)]
    assert not classes, (
        f"{name} 里又出现了类定义 {[c.name for c in classes]} —— "
        "入口脚本不该装实现，把它移回 core/。"
    )
    assert _nonmain_logic_lines(p) <= 40, (
        f"{name} 的非 main 逻辑涨到 {_nonmain_logic_lines(p)} 行；它应当只有 "
        "argparse + 调用 + 打印。"
    )
