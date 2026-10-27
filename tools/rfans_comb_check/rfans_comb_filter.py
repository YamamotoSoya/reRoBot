#!/usr/bin/env python3
# claude: 2026-10-27 作成
"""R-Fans の櫛状距離偽点 (k x 0.686 m ずれ) を点群から消すフィルタの試作.

背景: docs/issue/2026-10-25_rfans_range_comb_ghost.md (§6, §7.7)。偽点は
  - 真の面の距離から k x 0.686 m (k=±1..±5) ずれる
  - 角度方向に約 0.4° の塊で出る (細かい点間隔では隣も偽点になる)
  - 輪郭 (細いポールの後ろ等) にも出る (両隣が同じ面に乗らない)

判定 (同じレーザを時刻順に並べ、左右それぞれ n 列を見る):
  予測 = 片側の連続 2 点 (j, j+1) を直線で中心まで延ばした距離 (左右で最大 2(n-1) 個)
  m0 = 予測のうち、その点の距離に tol (+ 延ばした列数 x tol_step) 以内のものの数
  mk = 予測のうち、その点の距離 - k x P に同じ幅で一致するものの数 (k ごと, 最大を取る)
  偽点 = m0 <= own_max かつ mk >= sup_min かつ mk >= m0 + margin
斜めの面・輪郭の本物は片側の予測に乗るので m0 が立つ。塊の偽点は予測の多数決で真の面側が勝つ。

入力は driver の PointCloud2 と同じ並び (x, y, z, laserid, time)。1 フレーム単位で処理する。
"""
import numpy as np

P = 0.686  # 櫛の間隔 [m]


def _predictions(rr, n):
    """各点について、左右の連続 2 点からの直線外挿 (m, 2(n-1)) と、外挿距離 (列数)."""
    m = len(rr)
    pad = np.concatenate([np.full(n, np.nan), rr, np.full(n, np.nan)])
    preds, dist = [], []
    for side in (-1, 1):
        for j in range(1, n):
            a = pad[n + side * j:n + side * j + m]          # 中心から j 列
            b = pad[n + side * (j + 1):n + side * (j + 1) + m]  # j+1 列
            preds.append(a + (a - b) * j)                  # 中心 (0 列) まで延ばす
            dist.append(j)
    return np.stack(preds, 1), np.array(dist)


KS = (-5, -3, -2, -1, 1, 2, 3)  # 実測で偽点が出る k (±4, +5 はほぼ出ない)


def ghost_mask(xyz, laserid, t, n=5, tol=0.05, tol_step=0.01, own_max=0, sup_min=2, margin=2,
               ks=KS, vtol=0.10, vcomb=True, ktol=0.05):
    """1 フレームの偽点フラグ (bool, 点数) を返す.

    水平の判定 (同じレーザの前後) で候補を出したあと、上下の隣のレーザ (仰角順) で
    同じ方位に同じ距離 (±vtol) の点があれば本物として残す (壁・幹は縦に続く。偽点は単発)。
    vcomb: 同じレーザで支えが無い点 (m0=0) のうち、上下両方のレーザの同じ方位に
    「この点 - k x P」(±ktol) があり同じ距離 (±vtol) が無いものも偽点にする (2026-10-27 追加)。
    細いポールの奥の偽点は左右が奥の壁で、水平の判定では拾えないため。
    """
    r = np.linalg.norm(xyz, axis=1)
    cand = np.zeros(len(r), bool)
    own = np.full(len(r), 99)  # 同じレーザの予測で支えられた数 (m0)
    lids = np.unique(laserid)
    for lid in lids:
        idx = np.where(laserid == lid)[0]
        idx = idx[np.argsort(t[idx], kind="stable")]
        rr = r[idx]
        if len(rr) < 2 * n + 1:
            continue
        pr, dj = _predictions(rr, n)
        w = tol + tol_step * dj[None, :]
        m0 = np.sum(np.abs(pr - rr[:, None]) < w, 1)
        own[idx] = m0
        mk = np.zeros(len(rr), int)
        for k in ks:
            tgt = rr - k * P
            s = np.sum(np.abs(pr - tgt[:, None]) < w, 1)
            s[tgt < 0.3] = 0
            mk = np.maximum(mk, s)
        cand[idx] = (rr > 0.3) & (m0 <= own_max) & (mk >= sup_min) & (mk >= m0 + margin)
    if vtol is None or not (cand.any() or vcomb):
        return cand
    # 上下の隣のレーザ: 仰角の中央値で並べる
    elev = np.arctan2(xyz[:, 2], np.hypot(xyz[:, 0], xyz[:, 1]))
    order = sorted(lids, key=lambda l: np.median(elev[laserid == l]))
    az = np.arctan2(xyz[:, 1], xyz[:, 0])
    per = {}
    for l in lids:
        ii = np.where(laserid == l)[0]
        ii = ii[np.argsort(az[ii])]
        per[l] = (az[ii], r[ii])
    out = cand.copy()
    for pos, l in enumerate(order):
        # 水平候補 + (vcomb なら) 支え無しの点をまとめて上下と照合する
        ci = np.where((cand | ((own == 0) & (r > 0.3) & vcomb)) & (laserid == l))[0]
        if len(ci) == 0:
            continue
        sup = np.zeros(len(ci), bool)
        hits = []  # 上下それぞれで「k x P 手前に本物」があるか
        for q in (pos - 1, pos + 1):
            hk = np.zeros(len(ci), bool)
            hits.append(hk)
            if not 0 <= q < len(order):
                continue
            a2, r2 = per[order[q]]
            if len(a2) < 2:
                continue
            j = np.clip(np.searchsorted(a2, az[ci]), 1, len(a2) - 1)
            for jj in (j - 1, j):  # 方位が最も近い 2 点
                near = np.abs(a2[jj] - az[ci]) < np.radians(0.6)
                sup |= near & (np.abs(r2[jj] - r[ci]) < vtol)
                for k in ks:
                    hk |= near & (np.abs(r2[jj] - (r[ci] - k * P)) < ktol) & (r[ci] - k * P > 0.3)
        out[ci] = (cand[ci] | (vcomb & hits[0] & hits[1])) & ~sup
    return out
