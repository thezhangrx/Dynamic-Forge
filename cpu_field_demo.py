#!/usr/bin/env python3
"""势场方案可视化入口 —— 和 ``core/cpu/`` 同名的根目录入口（约定：前缀 = 模块名）。

    python cpu_field_demo.py                       # 交互窗口，目标=plan() 的缺口
    python cpu_field_demo.py --goal center         # 目标=场中央（平台 target 默认）
    python cpu_field_demo.py --goal none           # 不给目标（只用风险场）
    python cpu_field_demo.py --save out.png        # 离屏渲一帧存 PNG
    python cpu_field_demo.py --cell 8 --lam 0      # 调参：网格边长 / 排斥权重
    python cpu_field_demo.py --help                # 全部参数

    python core/cpu/field_demo.py --save out.png   # 等价（同一份 argparse）

实现全在 ``core/cpu/``：

    potential_field.py   算法本体（网格势场 + 波前测地距离）
    field_demo.py        决策场可视化（``main`` / ``run_demo`` 都在这里）

**本文件只做转发，不含任何算法。** argparse 在 ``field_demo`` 里 ——
这样两条入口（根脚本 / 模块文件）走同一份代码和同一份参数表，不会漂移。
"""

from __future__ import annotations

import pathlib
import sys

_CORE = pathlib.Path(__file__).resolve().parent / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))


def main(argv: list[str] | None = None) -> int:
    from cpu.field_demo import main as field_main
    return field_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
