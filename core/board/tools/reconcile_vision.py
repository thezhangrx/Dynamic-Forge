#!/usr/bin/env python3
"""板子诊断 ↔ 仿真真值 的对账（判据③ 的两个半边 + 判据④ 的碰撞上下文）。

三件事，按顺序做：

1. **时间对齐**：用**速度方向序列**（板子 `decision` 的符号 ↔ 仿真 `applied` 的符号）
   做互相关。为什么不用位置：角色被墙遮住时板子走的是预测值、仿真走的是真实值，
   两条位置轨迹会持续分叉（实测残差 197 场地单位，等于没对齐）；而"往左还是往右"
   是每帧都独立可比的强信号，峰值尖锐得多。
2. **误差统计**（判据③）：对齐后逐帧算
      · 角色：板子 `player_field` 与真值 `player` 的距离（场地单位）
      · 缺口：板子 `gap_units` 与真值缺口中心（按 P3 同一规则挑"前方最近的带缺口墙"）
   分母**不删帧**：无效帧（无有效坐标）单独计数并报出来（协议 §5.3）。
3. **碰撞上下文**（判据④）：每个碰撞事件打印前 1.5 s 的动作/缺口/角色/真值。

用法：
    python3 core/board/tools/reconcile_vision.py --dir /tmp/crit4
"""
import argparse
import json
import os
import statistics as st
import sys


def load(path):
    out = []
    try:
        f = open(path)
    except FileNotFoundError:
        return out
    with f:
        for line in f:
            line = line.strip()
            if not line or not line.startswith('{'):
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def sign(v, eps=1e-6):
    if v > eps:
        return 1
    if v < -eps:
        return -1
    return 0


def truth_series(truth, comp):
    """每个仿真帧的速度方向（-1/0/1）。"""
    return [sign(r['applied'][comp]) if r.get('applied') else 0 for r in truth]


def board_series(diag, comp):
    return [sign(r['decision'][comp]) for r in diag]


def align_by_clock(diag, truth, sim_start_epoch, dt_truth):
    """用共享墙钟对齐：板子 diag 的 unix_ms ↔ 仿真 start_epoch + t。

    为什么优先用这个：板子和仿真机都是 CLOCK_REALTIME（NTP 同源），偏差在毫秒级；
    而"按速度序列互相关"在长段恒定动作上会给出多个几乎同样好的偏移（实测漂到
    55 s），按位置对齐又会被遮挡期的预测值带偏（残差 197 场地单位）。
    """
    if not sim_start_epoch or 'unix_ms' not in diag[0]:
        return None
    offsets = []
    for r in diag:
        if 'unix_ms' not in r:
            continue
        offsets.append(r['unix_ms'] / 1000.0 - sim_start_epoch)
    if len(offsets) < 10:
        return None
    offsets.sort()
    return offsets[len(offsets) // 2]


def align(diag, truth, dt_truth):
    """返回(偏移秒, x 一致率, y 一致率, 参与比较的帧数)。"""
    bx = board_series(diag, 0)
    tx = truth_series(truth, 0)
    by = board_series(diag, 1)
    ty = truth_series(truth, 1)
    t0 = diag[0]['t_ns']
    dts = [(r['t_ns'] - t0) * 1e-9 for r in diag]

    def agreement(bo, tr, off):
        n = ok = 0
        for i, b in enumerate(bo):
            if b == 0:
                continue
            k = int(round((dts[i] + off) / dt_truth))
            if k < 0 or k >= len(tr):
                continue
            if tr[k] == 0:
                continue
            n += 1
            if b == tr[k]:
                ok += 1
        return (ok / n if n else 0.0), n

    best = None
    off = 0.0
    span = min(truth[-1]['t'] if truth else 0.0, 200.0)
    while off <= span:
        ax, nx = agreement(bx, tx, off)
        if nx >= 40 and (best is None or ax > best[1]):
            best = (off, ax, nx)
        off += dt_truth
    if best is None:
        return None
    off, ax, nx = best
    ay, _ny = agreement(by, ty, off)
    return off, ax, ay, nx


def nearest_gap_ahead(obstacles, player_y):
    """真值里"角色前方最近的带缺口墙"的缺口中心（场地单位，y 朝上）。

    仿真把一道带缺口的墙存成**两条横段**（同 y、同 half_h），缺口在中间。
    规则与 `wall_gap::select_threat_wall` 一致：只看 y > 角色 y 的墙，取最近的。
    """
    if not obstacles or player_y is None:
        return None
    by_y = {}
    for o in obstacles:
        if len(o) < 5:
            continue
        x, y, hw, hh, rot = o[0], o[1], o[2], o[3], o[4]
        if abs(hh) > 60 or abs(hw) < 1:      # 只看"横条"（墙段）
            continue
        by_y.setdefault(round(y, 1), []).append((x - abs(hw), x + abs(hw), y))
    best = None
    for y, segs in by_y.items():
        if len(segs) != 2:
            continue
        if y <= player_y:
            continue
        segs.sort()
        gap = 0.5 * (segs[0][1] + segs[1][0])
        d = y - player_y
        if best is None or d < best[0]:
            best = (d, gap, y)
    return None if best is None else (best[1], best[2])


def pct(v, p):
    if not v:
        return float('nan')
    v = sorted(v)
    return v[min(len(v) - 1, int(p * (len(v) - 1)))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='/tmp/crit4')
    ap.add_argument('--dt', type=float, default=1.0 / 60.0)
    ap.add_argument('--window', type=float, default=1.5)
    a = ap.parse_args()

    diag = load(os.path.join(a.dir, 'diag_crit4.jsonl'))
    truth = load(os.path.join(a.dir, 'truth.jsonl'))
    coll = load(os.path.join(a.dir, 'collision.jsonl'))
    if not diag or not truth:
        print('缺 diag 或 truth', file=sys.stderr)
        return 2
    # 优先用共享墙钟；没有就退回速度序列互相关
    sim_start = None
    try:
        sim_start = float(open(os.path.join(a.dir, 'sim_start_epoch.txt')).read().strip())
    except (FileNotFoundError, ValueError):
        pass
    off_clock = align_by_clock(diag, truth, sim_start, a.dt) if sim_start else None
    r = align(diag, truth, a.dt)
    ax = ay = 0.0
    nx = 0
    # 优先用"主机侧两个启动时刻 + 板子自身的单调时基"：
    #   sim_t(frame) = (board_start - sim_start) + (t_ns(frame) - t_ns(0))
    # 这样完全不依赖板子墙钟（实测它与主机差 1.5 s，而且会漂）。
    board_start = None
    try:
        board_start = float(open(os.path.join(a.dir, 'board_start_epoch.txt')).read().strip())
    except (FileNotFoundError, ValueError):
        pass
    if board_start is not None and sim_start is not None:
        off = board_start - sim_start
        off_clock = off
        # 校验：x/y 速度方向一致率
        bx = board_series(diag, 0)
        tx = truth_series(truth, 0)
        by = board_series(diag, 1)
        ty = truth_series(truth, 1)
        t0c = diag[0]['t_ns']
        dtsc = [(q['t_ns'] - t0c) * 1e-9 for q in diag]
        def agr(bo, tr):
            n = ok = 0
            for i, b in enumerate(bo):
                if b == 0:
                    continue
                k = int(round((dtsc[i] + off) / a.dt))
                if 0 <= k < len(tr) and tr[k] != 0:
                    n += 1
                    ok += (b == tr[k])
            return (ok / n if n else 0.0), n
        ax, nx = agr(bx, tx)
        ay, _ = agr(by, ty)
        print(f'对齐（主机侧启动时刻 + 板子单调时基）：偏移 {off:.2f}s；'
              f'校验 x 一致率 {100*ax:.0f}%（{nx} 帧）、y 一致率 {100*ay:.0f}%')
    elif off_clock is not None:
        off = off_clock
        # 用速度方向一致率做**校验**（不参与决定）
        _, ax, ay, nx = align(diag, truth, a.dt) and (0, 0, 0, 0) or (0, 0, 0, 0)
        bx = board_series(diag, 0)
        tx = truth_series(truth, 0)
        by = board_series(diag, 1)
        ty = truth_series(truth, 1)
        t0c = diag[0]['t_ns']
        dtsc = [(q['t_ns'] - t0c) * 1e-9 for q in diag]
        def agr(bo, tr):
            n = ok = 0
            for i, b in enumerate(bo):
                if b == 0:
                    continue
                k = int(round((dtsc[i] + off) / a.dt))
                if 0 <= k < len(tr) and tr[k] != 0:
                    n += 1
                    ok += (b == tr[k])
            return (ok / n if n else 0.0), n
        ax, nx = agr(bx, tx)
        ay, _ = agr(by, ty)
        print(f'对齐（共享墙钟）：板子相对仿真起点 {off:.3f}s；'
              f'校验 x 一致率 {100*ax:.0f}%（{nx} 帧）、y 一致率 {100*ay:.0f}%')
    elif r is not None:
        off, ax, ay, nx = r
        print(f'对齐（速度互相关）：偏移 {off:.3f}s，x 一致率 {100*ax:.0f}%（{nx} 帧），'
              f'y 一致率 {100*ay:.0f}%')
    else:
        print('对齐失败', file=sys.stderr)
        return 2
    if ax < 0.6:
        print('⚠ x 一致率偏低：这一轮可能含大量无效帧，结论要谨慎')

    t0 = diag[0]['t_ns']
    def sim_k(i):
        return int(round(((diag[i]['t_ns'] - t0) * 1e-9 + off) / a.dt))

    # ---- 判据③：角色与缺口误差 ----
    pe, pe_pred, gap_err = [], [], []
    n_valid_player = n_valid_gap = 0
    for i, row in enumerate(diag):
        k = sim_k(i)
        if k < 0 or k >= len(truth):
            continue
        th = truth[k]
        pl = th['player']
        if row.get('player_field_valid'):
            n_valid_player += 1
            e = ((row['player_field'][0] - pl[0]) ** 2 +
                 (row['player_field'][1] - pl[1]) ** 2) ** 0.5
            if row.get('player_src') == 'ps_pred':
                pe_pred.append(e)
            else:
                pe.append(e)
        g = nearest_gap_ahead(th.get('obstacles'), pl[1])
        if g is not None and row.get('gap_units', 0) > 0:
            n_valid_gap += 1
            gap_err.append(abs(row['gap_units'] - g[0]))

    print(f'\n判据③（对齐后逐帧，分母含无效帧）')
    print(f'  角色：{len(pe)+len(pe_pred)}/{len(diag)} 帧有坐标'
          f'（其中预测 {len(pe_pred)} 帧）')
    if pe:
        ok = sum(1 for e in pe if e <= 20)
        print(f'  角色误差（实测帧）：中位 {st.median(pe):.1f}，p90 {pct(pe,0.9):.1f}，'
              f'≤20 单位 {ok}/{len(pe)} = {100*ok/len(pe):.0f}%')
    if pe_pred:
        ok = sum(1 for e in pe_pred if e <= 20)
        print(f'  角色误差（预测帧）：中位 {st.median(pe_pred):.1f}，'
              f'≤20 单位 {ok}/{len(pe_pred)} = {100*ok/len(pe_pred):.0f}%')
    if gap_err:
        ok = sum(1 for e in gap_err if e <= 20)
        print(f'  缺口中心误差：{len(gap_err)} 帧有真值可对，中位 {st.median(gap_err):.1f}，'
              f'p90 {pct(gap_err,0.9):.1f}，≤20 单位 {ok}/{len(gap_err)} = '
              f'{100*ok/len(gap_err):.0f}%')

    # ---- 判据④：碰撞上下文 ----
    ev = [c for c in coll if c.get('collision_event')]
    print(f'\n判据④：碰撞事件 {len(ev)} 个，逐个看前 {a.window:.1f}s')
    for c in ev[:8]:
        kc = int(round(c['t'] / a.dt))
        cand = [i for i in range(len(diag)) if sim_k(i) <= kc]
        if not cand:
            continue
        i = cand[-1]
        row = diag[i]
        w = row['walls'][0] if row['walls'] else None
        pl = truth[kc]['player'] if 0 <= kc < len(truth) else None
        g = nearest_gap_ahead(truth[kc].get('obstacles'), pl[1]) if pl else None
        print(f'  t={c["t"]:.2f}s clr={c.get("clearance")} | 板子 角色={row.get("player_field")} '
              f'src={row.get("player_src")} 缺口={row.get("gap_units"):.0f} '
              f'动作={row["action"]} 选择={row.get("select_reason")}')
        if pl:
            print(f'      真值 角色=({pl[0]:.0f},{pl[1]:.0f}) '
                  f'缺口={(0 if g is None else round(g[0]))} 墙 y={(0 if g is None else round(g[1]))}')
        near = [(j, diag[j]) for j in cand if sim_k(j) >= kc - a.window]
        print(f'      前 {a.window:.1f}s 动作：' + ''.join(str(x[1]['action']) for x in near))
    return 0


if __name__ == '__main__':
    sys.exit(main())
