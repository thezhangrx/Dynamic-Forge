"""让 ``python -m pytest core/fpga/tests`` 在裸源码检出下能跑。

和 ``core/cpu/conftest.py`` / ``core/harness/conftest.py`` 同一个理由：
源码在 ``core/<模块>/`` 下，本目录的测试要 import ``standard.*``，
只有把 ``core/`` 放进 ``sys.path`` 才可导入（安装过项目时天然满足）。
"""

from __future__ import annotations

import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve().parent          # core/fpga/
_CORE = _HERE.parent                                     # core/

if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))
