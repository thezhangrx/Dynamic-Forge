"""跨帧身份指标 —— 给"墙/缺口是不是同一个"一把尺子。

为什么需要它
------------
``wall_with_gap`` 的缺口是要**跨帧跟着走**的：这一帧的缺口和上一帧的缺口是不是同一个，
决定了"缺口在往哪边走"。现在这个问题的状态是**无法度量** ——
``core/vision/motion.py`` 的 ``Tracker`` 只做角色，墙/缺口没有身份。
无法度量就无法改进，所以先把尺子建起来。

指标定义取自 TrackEval（``trackeval/metrics/``），逐条对齐：

============  ================================================================
指标          定义
============  ================================================================
``IDSW``      身份切换数。判定用**"该真值上一次出现时"的 tracker id**，
              **不是"上一帧"**（``clear.py:62-64, 93-96``）。墙上常态是
              在画面边缘进出：中间断几帧再回来只要 id 没变就不算切换，
              用逐帧比会把正常进出误判成切换。
``Frag``      碎片化：真值从"未被跟踪"变为"被跟踪"的次数，
              最后对每个真值减 1（``clear.py:105-106, 123``）。
              **与 TrackEval 有意偏离**：它在"本帧估计为空"时直接 ``continue``，
              跳过了帧末的重置，于是 ``prev_timestep_tracker_id`` 保留两帧前的值，
              **跨整帧漏检的那次断裂不计入 Frag**（``clear.py:75-78``）。
              我们按语义本意实现：空帧 = 这一帧什么都没跟踪到，下次再出现算断裂。
``MT/PT/ML``  按``被跟踪帧数/存在帧数``分：``>0.8`` 为 MT、
              ``>=0.2`` 为 PT，其余 ML（``clear.py:118-122``）。
``IDF1``      用**全序列全局指派**求 IDTP，而不是逐帧贪心
              （``identity.py:46-85``）。与 ``IDSW`` 的局部判法结论可能不同，
              **所以两个必须并列报**，否则"身份保持得好不好"取决于选了哪个指标。
============  ================================================================

逐帧匹配与 TrackEval 一致（``clear.py:79-88``）：先给"与上一帧同一个 tracker id"
一个巨大加成（1000），再加相似度；相似度低于门限的整格清零；然后匈牙利最大化，
只有得分 > 0 的才算配上。

为什么放在 ``core/vision/`` 且**不依赖第三方库**
------------------------------------------------
匈牙利算法（``hungarian``）与指标累计都只有加减比大小，纯 Python 几十行，
和 ``primitives`` 一样可以往 Verilog 翻。实测依赖 numpy 的代价见 ``adaptive.py`` 的记录。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

#: 相似度门限：低于它不算匹配（TrackEval 的 ``THRESHOLD`` 默认 0.5）。
DEFAULT_THRESHOLD = 0.5
#: "与上一帧同一个 tracker id"的加成（TrackEval 用 1000，``clear.py:81``）。
ID_CONSISTENCY_BONUS = 1000.0
#: MT/ML 的比率阈值（``clear.py:119-121``）。
MT_RATIO = 0.8
ML_RATIO = 0.2

#: 匈牙利里代表"禁止"的大数。够大压过任何合法代价，又不至于让浮点累加失真。
_BIG = 1.0e9


class IdentityError(ValueError):
    """身份指标上的输入问题。**不静默算出一个看起来合理的数。**"""


# ---------------------------------------------------------------------------
# 匈牙利算法（最小化方阵/矩形阵的指派）
# ---------------------------------------------------------------------------
def hungarian(cost: Sequence[Sequence[float]]) -> list[tuple[int, int]]:
    """最小化代价的指派。返回 ``[(row, col), ...]``，行全部被指派到互不相同的列。

    用带势的 O(n^3) 实现（e-maxx 的经典形式），``n <= m``；``n > m`` 时自动转置。
    纯 Python、只用加减比大小 —— 可以往 Verilog 翻。

    为什么不用贪心：``IDF1`` 的价值就在于"全局指派"。
    逐帧贪心会因为某一帧的局部最优而丢掉整条序列上的最优对应，
    于是同一个身份问题在 ``IDSW`` 和 ``IDF1`` 上给出**相反**的结论。
    """
    if not cost:
        return []
    n, m = len(cost), len(cost[0])
    if any(len(r) != m for r in cost):
        raise IdentityError("代价矩阵各行列数不一致")
    if n == 0 or m == 0:
        return []
    transposed = n > m
    if transposed:
        cost = [[cost[i][j] for i in range(n)] for j in range(m)]
        n, m = m, n

    inf = float("inf")
    u = [0.0] * (n + 1)
    v = [0.0] * (m + 1)
    p = [0] * (m + 1)      # p[j] = 被指派到列 j 的行（1-based）
    way = [0] * (m + 1)

    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [inf] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = inf
            j1 = -1
            row = cost[i0 - 1]
            for j in range(1, m + 1):
                if used[j]:
                    continue
                cur = row[j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            if j1 < 0:
                break                      # 理论上不会发生；防死循环
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1

    pairs = [(p[j] - 1, j - 1) for j in range(1, m + 1) if p[j] != 0]
    if transposed:
        pairs = [(c, r) for r, c in pairs]
    return sorted(pairs)


# ---------------------------------------------------------------------------
# 逐帧匹配
# ---------------------------------------------------------------------------
def match_frame(similarity: Sequence[Sequence[float]], *,
                threshold: float = DEFAULT_THRESHOLD,
                prev_id_of_truth: Sequence[int | None] | None = None,
                est_ids: Sequence[int] = ()) -> list[tuple[int, int]]:
    """一帧内的真值↔估计匹配。返回 ``[(i_truth, j_est), ...]``。

    与 TrackEval ``clear.py:79-88`` 一致：

    1. 得分 = ``1000 * (本帧估计 id == 上一帧该真值配到的估计 id) + 相似度``；
    2. **相似度低于门限的整格清零**（连那 1000 加成一起去掉）；
    3. 匈牙利**最大化**；只有得分 > 0 的算配上。
    """
    n = len(similarity)
    if n == 0:
        return []
    m = len(similarity[0])
    if m == 0:
        return []
    if prev_id_of_truth is not None and len(prev_id_of_truth) != n:
        raise IdentityError("prev_id_of_truth 长度必须等于相似度行数")

    score: list[list[float]] = []
    for i in range(n):
        row: list[float] = []
        prev = prev_id_of_truth[i] if prev_id_of_truth is not None else None
        for j in range(m):
            s = float(similarity[i][j])
            if s < threshold:
                row.append(0.0)            # 门限以下整格清零（含加成）
                continue
            bonus = ID_CONSISTENCY_BONUS if (prev is not None
                                             and est_ids[j] == prev) else 0.0
            row.append(bonus + s)
        score.append(row)

    pairs = hungarian([[-x for x in r] for r in score])     # 取负 => 最大化
    return [(i, j) for i, j in pairs if score[i][j] > 0.0]


# ---------------------------------------------------------------------------
# 指标
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class IdentityReport:
    """身份指标汇总。字段名与 TrackEval 对齐，便于交叉核对。"""

    frames: int
    num_truth_ids: int
    num_est_ids: int
    CLR_TP: int = 0
    CLR_FN: int = 0
    CLR_FP: int = 0
    IDSW: int = 0
    Frag: int = 0
    MT: int = 0
    PT: int = 0
    ML: int = 0
    IDTP: int = 0
    IDFN: int = 0
    IDFP: int = 0
    IDF1: float = 0.0
    IDP: float = 0.0
    IDR: float = 0.0
    MOTA: float = 0.0
    MOTP: float = 0.0

    def to_dict(self) -> dict:
        return {
            "frames": self.frames,
            "num_truth_ids": self.num_truth_ids,
            "num_est_ids": self.num_est_ids,
            "CLR_TP": self.CLR_TP, "CLR_FN": self.CLR_FN, "CLR_FP": self.CLR_FP,
            "IDSW": self.IDSW, "Frag": self.Frag,
            "MT": self.MT, "PT": self.PT, "ML": self.ML,
            "IDTP": self.IDTP, "IDFN": self.IDFN, "IDFP": self.IDFP,
            "IDF1": self.IDF1, "IDP": self.IDP, "IDR": self.IDR,
            "MOTA": self.MOTA, "MOTP": self.MOTP,
        }

    def summary(self) -> str:
        return (f"身份: IDF1={self.IDF1:.3f} (P={self.IDP:.3f} R={self.IDR:.3f})  "
                f"IDSW={self.IDSW}  碎片={self.Frag}  "
                f"MT/PT/ML={self.MT}/{self.PT}/{self.ML}  "
                f"检出 TP/FN/FP={self.CLR_TP}/{self.CLR_FN}/{self.CLR_FP}")


def evaluate_identity(
    truth_ids: Sequence[Sequence[int]],
    est_ids: Sequence[Sequence[int]],
    similarities: Sequence[Sequence[Sequence[float]]],
    *,
    threshold: float = DEFAULT_THRESHOLD,
) -> IdentityReport:
    """整段序列的身份指标。

    三个输入都是**逐帧等长**的列表（nuscenes 的"逐帧位置化"约定，
    见 ``docs/reference/nuscenes-devkit.md``）：第 t 项是该帧的
    真值 id / 估计 id / 相似度矩阵（行=真值，列=估计）。
    **某一帧没有目标就是空列表，而不是把这一帧删掉** ——
    删掉会让后续所有帧错位一帧，而错位会伪装成"身份变差"。
    """
    nt = len(truth_ids)
    if len(est_ids) != nt or len(similarities) != nt:
        raise IdentityError(
            f"三个输入必须等长（逐帧位置化）：{nt} / {len(est_ids)} / {len(similarities)}")

    # -- id 去重并建立索引 --
    truth_index: dict[int, int] = {}
    est_index: dict[int, int] = {}
    for ids in truth_ids:
        for i in ids:
            truth_index.setdefault(i, len(truth_index))
    for ids in est_ids:
        for i in ids:
            est_index.setdefault(i, len(est_index))
    g, t = len(truth_index), len(est_index)

    if g == 0 and t == 0:
        return IdentityReport(frames=nt, num_truth_ids=0, num_est_ids=0)

    gt_count = [0] * g
    est_count = [0] * t
    potential = [[0] * t for _ in range(g)]
    gt_matched = [0] * g

    prev_id_of_truth: dict[int, int | None] = {i: None for i in truth_index}
    prev_step_id: dict[int, int | None] = {i: None for i in truth_index}
    frag = [0] * g

    tp = fn = fp = idsw = 0
    motp_sum = 0.0

    for ti in range(nt):
        tid, eid, sim = truth_ids[ti], est_ids[ti], similarities[ti]
        if len(sim) != len(tid):
            raise IdentityError(f"第 {ti} 帧相似度行数 {len(sim)} != 真值数 {len(tid)}")
        if sim and len(sim[0]) != len(eid):
            raise IdentityError(f"第 {ti} 帧相似度列数 {len(sim[0])} != 估计数 {len(eid)}")

        for i in tid:
            gt_count[truth_index[i]] += 1
        for j in eid:
            est_count[est_index[j]] += 1

        if not tid:
            fp += len(eid)
            prev_step_id = {}          # 这一帧什么都没跟踪到
            continue
        if not eid:
            fn += len(tid)
            prev_step_id = {}          # 同上
            continue

        # ``prev_step_id`` 是**上一帧**的结果，本帧的新结果先写进 ``this_step_id``，
        # 帧末才替换。早先把两者混成一个变量（在帧首清空），于是 Frag 退化成
        # "帧数-1" —— 完美跟踪也报 Frag=2。
        prev_row = [prev_step_id.get(i) for i in tid]
        pairs = match_frame(sim, threshold=threshold,
                            prev_id_of_truth=prev_row, est_ids=eid)

        # IDF1 的全局指派输入：**统计所有过门限的潜在配对**，
        # 不是只统计本帧被选中的那一对（identity.py:53-56）
        for i in range(len(tid)):
            for j in range(len(eid)):
                if sim[i][j] >= threshold:
                    potential[truth_index[tid[i]]][est_index[eid[j]]] += 1

        matched_truth = {i for i, _ in pairs}
        tp += len(pairs)
        fn += len(tid) - len(pairs)
        fp += len(eid) - len(pairs)
        for i in range(len(tid)):
            if i not in matched_truth:
                continue
            j = next(jj for ii, jj in pairs if ii == i)
            gi = truth_index[tid[i]]
            gt_matched[gi] += 1
            motp_sum += float(sim[i][j]) if sim[i][j] > 0 else 0.0
            # IDSW：跟**上一次出现时**配到的 estimation id 比（clear.py:93-96）
            last = prev_id_of_truth[tid[i]]
            if last is not None and eid[j] != last:
                idsw += 1
            # Frag：从"未被跟踪"变为"被跟踪"
            if prev_step_id.get(tid[i]) is None:
                frag[gi] += 1
            prev_id_of_truth[tid[i]] = eid[j]

        this_step_id: dict[int, int] = {}
        for i, j in pairs:
            this_step_id[tid[i]] = eid[j]
        prev_step_id = this_step_id

    # -- MT/PT/ML（逐条对齐 clear.py:118-122）--
    #   MT = 跟踪率 > 0.8 的个数
    #   PT = 跟踪率 >= 0.2 的个数 - MT
    #   ML = num_gt_ids - MT - PT      ← TrackEval 用的是 num_gt_ids 而不是
    #                                     len(ratios)：跟踪率为 0 的真值 id
    #                                     不进 ratios，但仍要计入 ML。
    ratios = [gt_matched[i] / gt_count[i] for i in range(g) if gt_count[i] > 0]
    mt = sum(1 for r in ratios if r > MT_RATIO)
    pt = sum(1 for r in ratios if r >= ML_RATIO) - mt
    ml = g - mt - pt
    frag_total = sum(c - 1 for c in frag if c > 0)

    # -- IDF1：全序列全局指派（identity.py:58-85）--
    idtp, idfn, idfp = _global_id_assignment(potential, gt_count, est_count)
    denom = 2 * idtp + idfp + idfn
    idf1 = 2.0 * idtp / denom if denom > 0 else 0.0
    idp = idtp / (idtp + idfp) if (idtp + idfp) > 0 else 0.0
    idr = idtp / (idtp + idfn) if (idtp + idfn) > 0 else 0.0

    mota = 1.0 - (fn + fp + idsw) / max(1, tp + fn)
    motp = motp_sum / max(1, tp)

    return IdentityReport(
        frames=nt, num_truth_ids=g, num_est_ids=t,
        CLR_TP=tp, CLR_FN=fn, CLR_FP=fp, IDSW=idsw, Frag=frag_total,
        MT=mt, PT=pt, ML=ml, IDTP=idtp, IDFN=idfn, IDFP=idfp,
        IDF1=idf1, IDP=idp, IDR=idr, MOTA=mota, MOTP=motp)


def _global_id_assignment(potential: list[list[int]], gt_count: list[int],
                          est_count: list[int]) -> tuple[int, int, int]:
    """按 TrackEval ``identity.py:58-85`` 构造 ``fn+fp`` 代价阵并全局指派。

    增广成 ``(G+T) x (G+T)`` 方阵：真值可以配到某个估计列
    （代价 ``gt_count + est_count - 2*potential``），
    或者配到**自己的哑列**（代价 ``gt_count``，即完全没配上）；
    估计列同理。于是指派一次就同时决定"配哪些"和"放掉哪些"。
    """
    g, t = len(gt_count), len(est_count)
    if g == 0 and t == 0:
        return 0, 0, 0
    n = g + t
    fn_mat = [[0.0] * n for _ in range(n)]
    fp_mat = [[0.0] * n for _ in range(n)]

    # 顺序严格照 TrackEval identity.py:63-74，**顺序本身是有意义的**：
    # ① 基础代价（真值行 × 估计列；
    for i in range(g):
        for j in range(t):
            fn_mat[i][j] = float(gt_count[i])
            fp_mat[i][j] = float(est_count[j])
    # ② **先把禁止块整块置大数**。哑列只有 g 列（t..t+g-1），哑行只有 t 行
    #    （g..g+t-1）—— 早先按 range(t)/range(g) 双循环写，t>g 时列号越界。
    for i in range(g):
        for j in range(g):
            fn_mat[i][t + j] = _BIG
    for i in range(t):
        for j in range(t):
            fp_mat[g + i][j] = _BIG
    # ③ **再**覆盖对角线：真值配到自己的哑列 = 完全没配上。
    #    这两步顺序写反过一次（对角线被 ② 的大数盖掉），
    #    结果是"任何配对都被禁止"，IDF1 恒为 0 —— 而且看起来像个合理的小数。
    for i in range(g):
        fn_mat[i][t + i] = float(gt_count[i])
    for j in range(t):
        fp_mat[g + j][j] = float(est_count[j])
    # ④ 配上就能省下的代价：每个成功配对省 2*potential
    for i in range(g):
        for j in range(t):
            fn_mat[i][j] -= potential[i][j]
            fp_mat[i][j] -= potential[i][j]

    cost = [[fn_mat[i][j] + fp_mat[i][j] for j in range(n)] for i in range(n)]
    pairs = hungarian(cost)
    idfn = int(round(sum(fn_mat[i][j] for i, j in pairs)))
    idfp = int(round(sum(fp_mat[i][j] for i, j in pairs)))
    idtp = sum(gt_count) - idfn
    return idtp, idfn, idfp


def similarity_from_points(truth: Sequence[tuple[float, float]],
                           est: Sequence[tuple[float, float]],
                           *, gate: float) -> list[list[float]]:
    """把"点与点"变成相似度矩阵：``1 - d/gate``，超出 ``gate`` 记 0。

    相似度用距离而不是 IoU（TrackEval 用 IoU），因为我们的缺口/墙段在
    标准视觉结构里是**中心点 + 宽度**。**这是与 TrackEval 的一处有意偏离**，
    已在 ``docs/vision/identity.md`` 里记录。
    """
    out: list[list[float]] = []
    for (tx, ty) in truth:
        row: list[float] = []
        for (ex, ey) in est:
            d = math.hypot(ex - tx, ey - ty)
            row.append(max(0.0, 1.0 - d / gate) if gate > 0 else 0.0)
        out.append(row)
    return out


__all__ = [
    "DEFAULT_THRESHOLD", "ID_CONSISTENCY_BONUS", "MT_RATIO", "ML_RATIO",
    "IdentityError", "IdentityReport",
    "hungarian", "match_frame", "evaluate_identity", "similarity_from_points",
]
