#!/usr/bin/env python3
# claude: 2026-10-27 作成
"""シングル/デュアルエコーで櫛の偽点を比べる + デュアル時の「相方エコー」検査.

背景: docs/issue/2026-10-25_rfans_range_comb_ghost.md §7.7 (critic 推奨の次の実験)。
デュアル (data_level 3, CMD_LEVEL3_DUAL_ECHO) では group (0,1), (2,3), ... が同じ方位の 2 エコー
(bufferDecode.cpp の echo_flag==2 の扱い)。偶数 group = エコー a、奇数 group = エコー b として分け、
それぞれに rfans_comb_check.py と同じ判定をかける。デュアルはパケット数が倍 (1565/s) になり、
1 エコーあたりの点間隔はシングルと同じなので、シングルはネイティブのまま比べる (10-27 試し録りで確認)。
エコー b は 2 つ目の反射が無いと 0 (室内で約 93%)。相方検査は相方 > 0 の点だけで行う。

相方検査: エコー a の偽点と同じ (レーザ, 列) のエコー b (> 0 のときだけ) が
  truth = 近傍の直線 (mean) から 15 LSB 以内 / same = 偽点と 15 LSB 以内 / other のどれか。

例 (main コンテナ内):
  python3 rfans_comb_dual.py S1.pcap D1.pcap S2.pcap D2.pcap
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rfans_comb_check as rc  # noqa: E402


def load(path):
    a = np.frombuffer(b"".join(p[1] for p in rc.read_pcap(path)), dtype=rc.PKT)
    az = a["g"]["az"].astype(np.int32)  # (npkt, 12)
    R = a["g"]["pts"]["r"].astype(np.int32)  # (npkt, 12, 32)
    same = float((az[:, 0::2] == az[:, 1::2]).mean())
    return R, az, same


def to_cols(Rg):
    """(npkt, ngrp, 32) → 時刻順の列 (ncol, 16)."""
    R = Rg.reshape(-1, 32)
    return np.stack([R[:, :16], R[:, 16:]], 1).reshape(-1, 16)


def judge(cols):
    m2, m1, c, p1, p2 = cols[:-4], cols[1:-3], cols[2:-2], cols[3:-1], cols[4:]
    slope = (2 * (p2 - m2) + (p1 - m1)) / 10.0
    mean = (m2 + m1 + p1 + p2) / 4.0
    res = np.max(np.abs(np.stack([m2 - (mean - 2 * slope), m1 - (mean - slope),
                                  p1 - (mean + slope), p2 - (mean + 2 * slope)])), axis=0)
    valid = (res < 10) & (np.minimum.reduce([m2, m1, p1, p2]) > 125) & (c > 0)
    off = c - mean
    k = np.round(off / rc.P_LSB)
    comb = valid & (np.abs(k) >= 1) & (np.abs(k) <= 5) & (np.abs(off - k * rc.P_LSB) < 15)
    return valid, comb, mean, k


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inputs", nargs="+")
    args = ap.parse_args()
    for path in args.inputs:
        R, az, same = load(path)
        name = os.path.basename(path)
        dual = same > 0.9
        print(f"== {name}: 同方位の group 対 {100 * same:.1f}% → {'デュアル' if dual else 'シングル'}")
        if not dual:
            v, c, _, _ = judge(to_cols(R))
            print(f"  comb% {100 * c.sum() / v.sum():.3f} (偽点 {c.sum()}, 母数 {v.sum()})")
            continue
        ca, cb = to_cols(R[:, 0::2]), to_cols(R[:, 1::2])
        diff = ca != cb
        print(f"  a と b が違う点 {100 * diff.mean():.2f}%  (a>b {100 * (ca > cb).mean():.2f}%, a<b {100 * (ca < cb).mean():.2f}%,"
              f" b=0 {100 * (cb == 0).mean():.2f}%, a=0 {100 * (ca == 0).mean():.2f}%)")
        for lab, me, ot in (("a", ca, cb), ("b", cb, ca)):
            v, c, mean, k = judge(me)
            o = ot[2:-2]
            m = me[2:-2]
            has = o > 0
            cn = c & has
            truth = cn & (np.abs(o - mean) < 15)
            samev = cn & (np.abs(o - m) < 15)
            other = cn & ~truth & ~samev
            n = max(cn.sum(), 1)
            base = v & ~c & has
            print(f"  echo {lab}: comb% {100 * c.sum() / v.sum():.3f} (偽点 {c.sum()}, 母数 {v.sum()})")
            print(f"    相方 > 0: 偽点 {100 * cn.sum() / max(c.sum(), 1):.1f}% vs 正常点 {100 * base.sum() / max((v & ~c).sum(), 1):.1f}%")
            print(f"    相方 > 0 の偽点 {cn.sum()} 個の相方: 真値 {100 * truth.sum() / n:.1f}% / 偽点と同値 {100 * samev.sum() / n:.1f}% / その他 {100 * other.sum() / n:.1f}%")
            kk = {int(x): int(((k == x) & c).sum()) for x in range(-5, 6) if x}
            kt = {int(x): int(((k == x) & truth).sum()) for x in range(-5, 6) if x}
            print(f"    k 内訳 {kk}\n    うち相方=真値 {kt}")


if __name__ == "__main__":
    main()
