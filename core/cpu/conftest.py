"""让 ``python -m pytest core/cpu/tests`` 在**裸源码检出**下就能跑。

仓库约定是 ``core/<模块>/`` 放源码，所以本包的导入名是 ``cpu``（不是
``bullet_sim.cpu``），并且它依赖同级的 ``bullet_sim``。两者都只有把 ``core/``
放进 ``sys.path`` 才可导入：安装过项目（``pip install -e .``）时天然满足，
没安装时由这个 conftest 补上。

放在 ``core/cpu/`` 而不是仓库根，是为了让本模块自洽 —— 队友把 ``core/cpu/``
整个目录搬走也能跑测试。
"""

from __future__ import annotations

import pathlib
import sys

#: core/cpu/ 本模块目录：``core/cpu/tests/`` 与 ``core/cpu/benchmark_decision.py``
#: 都有 ``__init__.py``/是同级模块，pytest 的 prepend 导入模式会把 ``core/``
#: 插进来而**不插** ``core/cpu/``，于是 ``import benchmark_decision`` 找不到。
_HERE = pathlib.Path(__file__).resolve().parent          # core/cpu/
_CORE = _HERE.parent                                     # core/（cpu / bullet_sim 在它下面）

for _path in (str(_CORE), str(_HERE)):
    if _path not in sys.path:
        sys.path.insert(0, _path)
