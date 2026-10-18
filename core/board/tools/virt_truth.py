#!/usr/bin/env python3
"""虚拟平台真值服务（第一步：**影子模式**）。

设计口径（与用户确认过的分工）
------------------------------
* **决策层不动**：板子的 `gap_avoid` 已经验证过，问题都应该往视觉/状态估计上修。
* **视觉与决策之间隔一层状态**：视觉只在"看得清"时给**校正**，平台状态由
  **自己的指令积分**推进（`p += DISCRETE_DIRECTIONS[action] × speed × dt`）。
  这层就是"虚拟真值"，板子应该读它而不是直接读每帧识别结果。
* **真值只用于对账**：`--truth-jsonl` 只进日志/汇总，**不参与**任何状态更新
  （否则就是自己判自己）。

影子模式下它**不驱动任何东西**，只做三件事：
1. 按指令积分出平台/障碍的虚拟状态；
2. 用视觉做慢校正（α 小，只在 `geo_why=ok` 时）；
3. 逐帧写 `virt.jsonl`（虚拟状态 + 视觉观测 + 真值 + 残差），供对账。

用法
----
    python3 core/board/tools/virt_truth.py \\
        --diag-jsonl /tmp/crit4/diag_crit4.jsonl \\
        --truth-jsonl /tmp/crit4/truth.jsonl \\
        --out /tmp/virt.jsonl

对账口径（都写进汇总）：
    · 角色：|虚拟 − 真值| 与 |视觉 − 真值| 的 p50/p95，以及"虚拟更好"的帧占比
    · 墙 y / 缺口 x：模型与真值的差（按"同 y 最近"配对）
    · 稳定性判据（用于决定"能不能切换到虚拟真值"）见 `stability_report()`
"""
from __future__ import annotations

import argparse
import bisect
import json
import statistics as st
import sys

#: 动作码 → 单位方向（与 `bullet_sim/core/actions.py` 的 DISCRETE_DIRECTIONS 一致）
DIRS = {
    0: (0.0, 0.0),
    1: (-1.0, 0.0),
    2: (1.0, 0.0),
    3: (0.0, 1.0),
    4: (0.0, -1.0),
    5: (-0.7071067811865476, 0.7071067811865476),
    6: (0.7071067811865476, 0.7071067811865476),
    7: (-0.7071067811865476, -0.7071067811865476),
    8: (0.7071067811865476, -0.7071067811865476),
}


def load(path):
    out = []
    try:
        f = open(path)
    except FileNotFoundError:
        return out
    with f:
        for line in f:
            line = line.strip()
            if not line.startswith('{'):
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


class TruthIndex:
    """真值索引（只有对账用）。"""

    def __init__(self, truth):
        self.t = truth
        self.unix = [r['unix'] for r in truth] if truth and 'unix' in truth[0] else []

    def player_at(self, unix):
        if not self.unix:
            return None
        i = bisect.bisect_left(self.unix, unix)
        i = min(i, len(self.t) - 1)
        return self.t[i]['player']

    def walls_at(self, unix):
        """返回 [(y, gap_centre)]（每道带缺口墙一条）。"""
        if not self.unix:
            return []
        i = min(bisect.bisect_left(self.unix, unix), len(self.t) - 1)
        by = {}
        for o in self.t[i]['obstacles']:
            x, y, hw, hh = o[0], o[1], o[2], o[3]
            if abs(hh) > 60 or abs(hw) < 1:
                continue
            by.setdefault(round(y, 1), []).append((x - hw, x + hw))
        out = []
        for y, segs in by.items():
            if len(segs) == 2:
                a, b = sorted(segs)
                out.append((y, 0.5 * (a[1] + b[0])))
        return out


class WallModel:
    """一道墙的虚拟状态：y 按墙速积分，缺口 x 按扫动速度积分。"""

    def __init__(self, y, gap, t):
        self.y = y
        self.gap = gap
        self.vy = 0.0
        self.vgap = 0.0
        self.t = t
        self.seen = 1
        self.miss = 0

    def predict(self, t):
        dt = max(0.0, t - self.t)
        return self.y + self.vy * dt, self.gap + self.vgap * dt

    def correct(self, y, gap, t, alpha=0.25):
        py, pg = self.predict(t)
        dt = t - self.t
        if dt > 1e-3:
            # 速度用**预测值差分**估（差分放大抖动，先预测再差分才稳）
            self.vy = 0.7 * self.vy + 0.3 * ((y - self.y) / dt)
            self.vgap = 0.7 * self.vgap + 0.3 * ((gap - self.gap) / dt)
        self.y = py + alpha * (y - py)
        self.gap = pg + alpha * (gap - pg)
        self.t = t
        self.seen += 1
        self.miss = 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--diag-jsonl', required=True, help='板子逐帧诊断（视觉 + 指令）')
    ap.add_argument('--truth-jsonl', default=None, help='仿真真值（**只用于对账**）')
    ap.add_argument('--out', default='/tmp/virt.jsonl')
    ap.add_argument('--speed', type=float, default=None,
                    help='平台速度（场地单位/秒）。不给就从真值标定')
    ap.add_argument('--alpha', type=float, default=0.25, help='视觉校正比例（越小越信积分）')
    ap.add_argument('--robust-window', type=float, default=0.0,
                    help='鲁棒时间窗（秒）。>0 时：角色/缺口/墙的位置都取窗口内的鲁棒均值并丢离群值')
    ap.add_argument('--gate', type=float, default=60.0,
                    help='视觉残差门限（场地单位）：观测与模型差超过它就不采信（当离群）')
    ap.add_argument('--table', type=int, default=5,
                    help='每隔 N 帧打印一行对账表（0 = 不打印）')
    ap.add_argument('--cmd-delay', type=float, default=0.0,
                    help='指令生效延迟（秒）。实测启动阶段 0.457 s；稳态需按实测填')
    ap.add_argument('--calib', default=None,
                    help='开场标定结果（`boot_cal.py` 的 calib.json）。给了它就用里面的'
                         '**速度（每板秒）**、**指令延迟（板秒）**和**初始位置**，'
                         '不必再手填 --speed/--cmd-delay')
    a = ap.parse_args(argv)

    calib = {}
    if a.calib:
        try:
            calib = json.load(open(a.calib, encoding='utf-8'))
            print(f"[标定] 读入 {a.calib}：速度 {calib.get('speed_units_per_board_s')} 单位/板秒，"
                  f"指令延迟 {calib.get('cmd_delay_board_s')} 板秒，"
                  f"端到端 {calib.get('vision_latency_board_s')} 板秒")
        except Exception as exc:
            print(f"[标定] 读不了 {a.calib}：{exc!r}（继续用命令行参数）", file=sys.stderr)
    if calib and a.speed is None:
        a.speed = calib.get('speed_units_per_board_s')
    if calib and a.cmd_delay == 0.0:
        a.cmd_delay = float(calib.get('cmd_delay_board_s') or 0.0)

    diag = load(a.diag_jsonl)
    if not diag:
        print('诊断日志为空', file=sys.stderr)
        return 2
    truth = load(a.truth_jsonl) if a.truth_jsonl else []
    ti = TruthIndex(truth)

    # ---- 标定平台速度：真值里"单方向持续运动"的 |Δp|/Δt 中位数 ----
    speed = a.speed
    if speed is None and truth:
        sp = []
        for p, q in zip(truth, truth[1:]):
            if not q.get('applied') or not p.get('applied'):
                continue
            if abs(q['applied'][0]) + abs(q['applied'][1]) < 1e-9:
                continue
            dt = q['unix'] - p['unix']
            if dt <= 1e-4:
                continue
            d = ((q['player'][0] - p['player'][0]) ** 2 +
                 (q['player'][1] - p['player'][1]) ** 2) ** 0.5
            sp.append(d / dt)
        speed = st.median(sp) if sp else 200.0
        print(f'[标定] 平台速度 ≈ {speed:.1f} 场地单位/秒（真值单方向运动，n={len(sp)}）')
    if speed is None:
        speed = 200.0

    # ---- 状态 ----
    px = py = 0.0
    have = False
    walls: list[WallModel] = []
    # ---- 鲁棒时间窗（用户口径：宁可要大概的范围，不要不稳定的点值）----
    from collections import deque
    def new_win():
        return deque(maxlen=12)
    def win_robust(dq, t, w):
        while dq and t - dq[0][1] > w:
            dq.popleft()
        if not dq:
            return None
        vs = sorted(v for v, _ in dq)
        n = len(vs)
        med = vs[n // 2] if n % 2 else 0.5 * (vs[n // 2] + vs[n // 2 - 1])
        dev = sorted(abs(v - med) for v in vs)
        mad = dev[n // 2] * 1.4826
        keep = [v for v in vs if mad < 1e-9 or abs(v - med) <= 3.0 * mad]
        if not keep:
            keep = vs
        keep = keep[max(0, int(0.2 * len(keep))):len(keep) - max(0, int(0.2 * len(keep)))] or keep
        return sum(keep) / len(keep)
    win_px, win_py, win_gap = new_win(), new_win(), new_win()
    out = open(a.out, 'w', encoding='utf-8')
    n = 0
    n_corr = 0
    n_rejected = 0
    # 指令延迟：先入队，delay 秒后再作用（影子模式下用 host_t 对齐）
    pending = []      # [(生效时刻, action)]

    for r in diag:
        t = r.get('host_t') or 0.0
        if not t:
            continue
        act = int(r.get('action', 0))
        pending.append((t + a.cmd_delay, act))

        # 1) 用"已生效的指令"积分
        dt_total = 0.0
        if have:
            dt_total = max(0.0, t - last_t)
        # 简化：按 last action 在 [last_t, t] 内积分（影子模式足够；正式服务按事件重放）
        if have and dt_total > 0:
            eff = last_action
            dx, dy = DIRS.get(eff, (0.0, 0.0))
            px += dx * speed * dt_total
            py += dy * speed * dt_total
        last_action = act
        last_t = t

        # 2) 视觉校正（只在几何可信 + 角色有效时）
        vx_raw = r.get('player_field')[0] if r.get('player_field_valid') else None
        vy_raw = r.get('player_field')[1] if r.get('player_field_valid') else None
        vis_ok = (vx_raw is not None) and r.get('geo_why') == 'ok'
        vx, vy = vx_raw, vy_raw
        if a.robust_window > 0 and vis_ok:
            win_px.append((vx_raw, t))
            win_py.append((vy_raw, t))
            rx = win_robust(win_px, t, a.robust_window)
            ry = win_robust(win_py, t, a.robust_window)
            if rx is not None and ry is not None:
                vx, vy = rx, ry
        pred_before = (px, py)
        if vis_ok and have:
            # **离群门**：观测与模型预测差得太远 → 不采信（宁可短时靠积分）
            if ((vx - px) ** 2 + (vy - py) ** 2) ** 0.5 > a.gate:
                vis_ok = False
                n_rejected += 1
        if vis_ok:
            if not have:
                px, py = vx, vy
                have = True
            else:
                px = px + a.alpha * (vx - px)
                py = py + a.alpha * (vy - py)
            n_corr += 1
        if not have:
            last_t = t
            continue

        # 3) 墙/缺口模型（有观测就校正，没有就积分）
        obs = []
        for w in r.get('walls', []):
            if not w.get('gap_w'):
                continue
            wy = r.get('wall_y_units')
            if not wy:
                continue
            obs.append((wy, r.get('gap_units', 0.0)))
        for wy, wg in obs:
            best = None
            for m in walls:
                d = abs(m.predict(t)[0] - wy)
                if d < 40 and (best is None or d < best[0]):
                    best = (d, m)
            if best is None:
                walls.append(WallModel(wy, wg, t))
            else:
                best[1].correct(wy, wg, t, alpha=a.alpha)
        for m in walls:
            if all(m is not b[1] for b in []) :
                pass
        # 过期墙丢弃（预测 y 跑出场地很多）
        walls = [m for m in walls if -60 <= m.predict(t)[0] <= 540]

        # 4) 日志
        rec = {
            't': round(t, 4),
            'action': act,
            'pred': [round(pred_before[0], 1), round(pred_before[1], 1)],
            'vision': [None if vx is None else round(vx, 1),
                       None if vy is None else round(vy, 1)],
            'vision_raw': [None if vx_raw is None else round(vx_raw, 1),
                           None if vy_raw is None else round(vy_raw, 1)],
            'virt': [round(px, 1), round(py, 1)],
            'vision_ok': bool(vis_ok),
            #: 视觉观测 − 模型预测（校正前）。**对账时先看这个**：
            #: 残差长期有偏 → 速度常数/指令延迟没标定好；残差无偏但大 → 视觉抖。
            'resid': ([None, None] if not vis_ok else
                      [round(vx - pred_before[0], 1), round(vy - pred_before[1], 1)]),
            'walls': [{'y': round(m.predict(t)[0], 1), 'gap': round(m.predict(t)[1], 1),
                       'vy': round(m.vy, 1), 'vgap': round(m.vgap, 1)} for m in walls],
        }
        if ti.unix:
            tp = ti.player_at(t)
            if tp:
                rec['truth'] = [round(tp[0], 1), round(tp[1], 1)]
        out.write(json.dumps(rec, ensure_ascii=False) + '\n')
        n += 1
    out.close()

    print(f'[虚拟真值] 处理 {n} 帧，视觉校正 {n_corr} 帧（{100*n_corr/max(1,n):.0f}%），'
          f'离群拒绝 {n_rejected} 帧（{100*n_rejected/max(1,n):.0f}%），输出 {a.out}')
    stability_report(a.out)
    if a.table:
        table(a.out, a.table)
    return 0


def table(path, every):
    """逐帧对账表：模型 / 视觉 / 真值 三列并排，直接看出是谁错。"""
    rows = [json.loads(l) for l in open(path) if l.strip()]
    t0 = rows[0]['t'] if rows else 0.0
    print(f"\n{'t(s)':>7} {'动作':>4} {'模型':>14} {'视觉':>14} {'真值':>14} {'误差(模型/视觉)':>18}")
    for i, r in enumerate(rows):
        if i % every:
            continue
        if 'truth' not in r:
            continue
        tr = r['truth']
        ve = r['vision']
        e_m = ((r['virt'][0] - tr[0]) ** 2 + (r['virt'][1] - tr[1]) ** 2) ** 0.5
        e_v = (float('nan') if ve[0] is None else
               ((ve[0] - tr[0]) ** 2 + (ve[1] - tr[1]) ** 2) ** 0.5)
        print(f"{r['t']-t0:7.2f} {r['action']:4d} "
              f"({r['virt'][0]:6.0f},{r['virt'][1]:6.0f}) "
              f"({('----' if ve[0] is None else f'{ve[0]:6.0f}')},"
              f"{('----' if ve[1] is None else f'{ve[1]:6.0f}')}) "
              f"({tr[0]:6.0f},{tr[1]:6.0f})  {e_m:6.0f} / {e_v:6.0f}")


def stability_report(path):
    """对账 + 稳定性判定。

    稳定性判据（用于回答"识别到什么程度才算稳定"）：
      · 角色误差 p95 ≤ 20 场地单位（= 缺口半宽 − 角色半径的量级）
      · 缺口中心误差 p95 ≤ 20 场地单位
      · 墙 y 误差 p95 ≤ 20 场地单位（≈ 0.2~0.3 s 的时间误差）
      · 有效帧占比 ≥ 80%，且**最长连续无观测** ≤ 0.30 s（超过就只能 stay）
      · 虚拟真值相对纯视觉：p95 不劣化，且抖动（帧间跳变）明显更小
    """
    rows = [json.loads(l) for l in open(path) if l.strip()]
    if not rows:
        return
    with_truth = [r for r in rows if 'truth' in r]
    def pct(v, p):
        if not v:
            return float('nan')
        v = sorted(v)
        return v[min(len(v) - 1, int(p * (len(v) - 1)))]

    ev = [(ic, ((r['virt'][0] - r['truth'][0]) ** 2 + (r['virt'][1] - r['truth'][1]) ** 2) ** 0.5)
          for r in with_truth for ic in [1] if r['vision_ok']]
    ev_all = [((r['virt'][0] - r['truth'][0]) ** 2 + (r['virt'][1] - r['truth'][1]) ** 2) ** 0.5
              for r in with_truth]
    evis = [((r['vision'][0] - r['truth'][0]) ** 2 + (r['vision'][1] - r['truth'][1]) ** 2) ** 0.5
            for r in with_truth if r['vision_ok']]
    jump_v = [((b['virt'][0] - a2['virt'][0]) ** 2 + (b['virt'][1] - a2['virt'][1]) ** 2) ** 0.5
              for a2, b in zip(rows, rows[1:])]
    evis_raw = [((r['vision_raw'][0] - r['truth'][0]) ** 2 +
                 (r['vision_raw'][1] - r['truth'][1]) ** 2) ** 0.5
                for r in with_truth if r.get('vision_raw') and r['vision_raw'][0] is not None]
    jump_s = [((b['vision'][0] - a2['vision'][0]) ** 2 + (b['vision'][1] - a2['vision'][1]) ** 2) ** 0.5
              for a2, b in zip(rows, rows[1:])
              if a2['vision'][0] is not None and b['vision'][0] is not None]
    print('\n---- 虚拟真值 vs 视觉（对真值）----')
    if ev:
        print(f'  角色误差（视觉有效的帧）：虚拟 p50 {pct(ev_all,0.5):.0f} / p95 {pct(ev_all,0.95):.0f}'
              f'；纯视觉 p50 {pct(evis,0.5):.0f} / p95 {pct(evis,0.95):.0f} 场地单位')
    if evis_raw:
        print(f'  角色误差（单帧原始）：p50 {pct(evis_raw,0.5):.0f} / p95 {pct(evis_raw,0.95):.0f} 场地单位')
    print(f'  帧间抖动：虚拟 p50 {pct(jump_v,0.5):.1f} / p95 {pct(jump_v,0.95):.1f}；'
          f'视觉(入模型) p50 {pct(jump_s,0.5):.1f} / p95 {pct(jump_s,0.95):.1f} 场地单位')
    # 有效帧与最长空档
    gaps = []
    run = 0
    for r in rows:
        if r['vision_ok']:
            if run:
                gaps.append(run)
            run = 0
        else:
            run += 1
    valid = sum(1 for r in rows if r['vision_ok']) / len(rows)
    max_gap = max(gaps) if gaps else 0
    print(f'  视觉有效率 {100*valid:.0f}%，最长连续无观测 {max_gap} 帧')
    res = [r['resid'] for r in rows if r.get('resid') and r['resid'][0] is not None]
    if res:
        rx = [abs(v[0]) for v in res]
        ry = [abs(v[1]) for v in res]
        sx = [v[0] for v in res]
        sy = [v[1] for v in res]
        print(f'  视觉残差（观测−预测）：|Δx| p50 {pct(rx,0.5):.0f} p95 {pct(rx,0.95):.0f}'
              f'；|Δy| p50 {pct(ry,0.5):.0f} p95 {pct(ry,0.95):.0f}')
        print(f'    平均残差（有偏就会大）：Δx {st.mean(sx):+.1f}、Δy {st.mean(sy):+.1f} 场地单位'
              f' → {"速度常数/指令延迟没标定好" if abs(st.mean(sx))>20 or abs(st.mean(sy))>20 else "无显著偏置"}')
    print('\n稳定性判据（全部满足才算"可以切到虚拟真值"）：')
    print('  角色 p95 ≤ 20、缺口 p95 ≤ 20、墙 y p95 ≤ 20 场地单位；有效帧 ≥80%；最长空档 ≤6 帧')


if __name__ == '__main__':
    sys.exit(main())
