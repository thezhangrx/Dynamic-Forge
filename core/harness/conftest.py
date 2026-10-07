"""让 ``python -m pytest core/harness/tests`` 在**裸源码检出**下就能跑。

和 ``core/cpu/conftest.py`` 同一个理由：仓库约定源码在 ``core/<模块>/`` 下，
所以导入名是 ``harness``（不是 ``core.harness``），它依赖同级的
``bullet_sim`` / ``cpu`` / ``vision`` / ``standard``。这些只有把 ``core/``
放进 ``sys.path`` 才可导入：安装过项目时天然满足，没安装时由这个 conftest 补上。

放在 ``core/harness/`` 而不是仓库根，是为了让本模块自洽。
"""

from __future__ import annotations

import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve().parent          # core/harness/
_CORE = _HERE.parent                                     # core/

if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))
