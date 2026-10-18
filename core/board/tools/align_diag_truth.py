#!/usr/bin/env python3
"""把板子的诊断日志与仿真真值**对齐**，然后看碰撞前那几帧到底发生了什么。

为什么必须对齐：板子的 `t_ns` 是它自己的单调时钟，仿真从头开始计 `t`，两者没有共同
时间原点。以前只能靠"看起来差不多"来判断，这正是不该做的事（不变式 I3/协议 §5）。

对齐原理（只用**独立于被测对象**的量）：
    真值里的角色位置（场地单位，y 朝上）   ←→   诊断里的 `player_xy`（已换成场地单位）
两者都由"板子发指令 → 仿真积分"这条链决定，所以存在一个**唯一的时间偏移**使两条
轨迹重合。对每个候选偏移算平均距离，取最小值。角色被遮挡的帧（`player_src` 不是
`ps_new/ps_pred`）不参与对齐，避免用插值数据"自证"。

用法：
    python3 core/board/tools/align_diag_truth.py --dir /tmp/crit4
    python3 core/board/tools/align_diag_truth.py --dir /tmp/crit4 --window 90
"""
import argparse
import json
import os
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
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='/tmp/crit4', help='放 diag/truth/collision 的目录')
    ap.add_argument('--diag', default=None)
    ap.add_argument('--truth', default=None)
    ap.add_argument('--collision', default=None)
    ap.add_argument('--window', type=float, default=60.0,
                    help='碰撞前后各看多少秒（仿真时间）')
    a = ap.parse_args()

    diag = load(a.diag or os.path.join(a.dir, 'diag_crit4.jsonl'))
    truth = load(a.truth or os.path.join(a.dir, 'truth.jsonl'))
    coll = load(a.collision or os.path.join(a.dir, 'collision.jsonl'))
    if not diag or not truth:
        print('缺 diag 或 truth 日志', file=sys.stderr)
        return 2

    # 板子时间戳 → 秒（相对首帧）
    t0 = diag[0]['t_ns']
    dseq = [(r['t_ns'] - t0) * 1e-9 for r in diag]
    tseq = [r['t'] for r in truth]

    # 只取"真观测"的角色帧参与对齐
    good = [(i, r) for i, r in enumerate(diag)
            if r.get('player_valid') and r.get('player_src') in ('ps_new', 'ps_pred')]
    if len(good) < 10:
        print(f'可用于对齐的观测太少（{len(good)} 帧）', file=sys.stderr)
        return 2

    # 在真值里为 diag 首帧找起点：枚举偏移
    best = None
    step = 1.0 / 60.0
    max_off = min(tseq[-1] - tseq[0], 200.0)
    off = 0.0
    while off <= max_off:
        errs = []
        for i, r in good:
            tt = dseq[i] + off
            k = int(round((tt - tseq[0]) / step))
            if k < 0 or k >= len(truth):
                continue
            p = truth[k]['player']
            errs.append(abs(p[0] - r['player_xy'][0]) + abs(p[1] - r['player_xy'][1]))
        if len(errs) >= 8:
            m = sum(errs) / len(errs)
            if best is None or m < best[1]:
                best = (off, m, len(errs))
        off += step
    if best is None:
        print('没有找到可用偏移', file=sys.stderr)
        return 2
    off, merr, cnt = best
    print(f'对齐：偏移 {off:.3f}s（相对板子首帧），平均 |Δx|+|Δy| = {merr:.1f} 场地单位，'
          f'用 {cnt} 帧观测')
    if merr > 30:
        print('⚠ 对齐残差偏大：可能这一轮里相机被挡过、或角色大量靠预测 —— 结论要谨慎')

    # 板子 diag 时间 → 仿真时间
    def sim_time_of(i):
        return dseq[i] + off

    ev = [c for c in coll if c.get('collision_event')]
    print(f'\n碰撞事件 {len(ev)} 个。逐个看前 {a.window:.0f}s 内板子的判断：')
    for c in ev:
        tc = c['t']
        # 找 tc 之前的 diag 帧
        idx = [i for i in range(len(diag)) if sim_time_of(i) <= tc]
        if not idx:
            continue
        i = idx[-1]
        r = diag[i]
        w = r['walls'][0] if r['walls'] else None
        print(f"\n  t={tc:.2f}s clearance={c.get('clearance')} "
              f"(板子帧 {r['frame']}, 提前 {sim_time_of(i)-tc:.2f}s)")
        print(f"    角色 src={r.get('player_src')} xy=({r['player_xy'][0]:.0f},"
              f"{r['player_xy'][1]:.0f})  墙={len(r['walls'])} "
              f"缺口={(w['gap_cx'] if w else None)} "
              f"选择={r.get('select_reason')} 动作={r['action']} "
              f"stop={r['stop_reason']}")
        # 看碰撞前 1s 的动作序列（板子视角）
        near = [(j, diag[j]) for j in idx if sim_time_of(j) >= tc - 1.0]
        acts = ''.join(str(x[1]['action']) for x in near)
        print(f"    前 1s 动作序列（1=左 2=右 3=上 4=下）：{acts}")
        # 同时看真值里角色当时在哪、往哪走
        k = int(round((tc - tseq[0]) / step))
        if 0 <= k < len(truth):
            p = truth[k]['player']
            print(f"    真值：角色 ({p[0]:.0f},{p[1]:.0f})  applied={truth[k]['applied']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
