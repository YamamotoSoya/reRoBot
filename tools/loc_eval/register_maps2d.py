#!/usr/bin/env python3
# claude: 2026-10-11 新設 — 別 bag の GLIM 座標を地図の座標へ合わせる剛体変換 (x, y, yaw) を 2D 地図同士の ICP で求める。
#
#   用途: 地図を作った bag (例 1643) と別の bag (例 1611) で評価するとき、1611 の GLIM 軌跡は 1611 自身の
#   始点基準なので、そのままでは地図座標と比べられない。両 bag の 2D 地図 (base_link 基準の同じ高さ帯で作った
#   raw_dump/<帯>/map.yaml) の占有セルを ICP で合わせ、得た変換を make_reference.py --transform に渡す。
#   2D 地図を使うのは、GLIM の z ドリフトが run ごとに違っても base_link 相対の帯なら影響しないため。
#
#   出力: <out_dir>/T_map_from_src.txt (4x4、z 成分なし) + residual.json (最終の最近傍距離の分布 = 測れる誤差の下限)
#         + overlay.png (黒 = 両方、赤 = src のみ、青 = 地図のみ)
#
# 使い方 (コンテナ内):
#   python3 register_maps2d.py <src map.yaml (別 bag)> <dst map.yaml (地図の bag)> <out_dir>
#       [--init x y yaw_deg] [--coarse] [--max_dist 3.0] [--iters 60]
#   --coarse: 初期値が不明なとき、yaw ±60° × 平行移動を位相相関で粗探索してから ICP
import argparse
import json
import os

import cv2
import numpy as np
import yaml
from scipy.spatial import cKDTree


def load_occ(map_yaml):
    y = yaml.safe_load(open(map_yaml))
    img = cv2.imread(os.path.join(os.path.dirname(map_yaml), y["image"]), cv2.IMREAD_GRAYSCALE)
    p = (255 - img.astype(np.float32)) / 255.0
    occ = p > y["occupied_thresh"]
    rows, cols = np.nonzero(occ)
    res, ox, oy = y["resolution"], y["origin"][0], y["origin"][1]
    H = img.shape[0]
    x = ox + (cols + 0.5) * res
    yy = oy + (H - rows - 0.5) * res
    return np.c_[x, yy], res


def voxel(pts, v):
    key = np.floor(pts / v).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return pts[idx]


def rigid2d(src, dst):
    """src→dst の最小二乗剛体 (R, t)"""
    cs, cd = src.mean(0), dst.mean(0)
    Hm = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(Hm)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, d]) @ U.T
    return R, cd - R @ cs


def apply(T, pts):
    return pts @ T[0].T + T[1]


def icp(src, dst, T, max_dist, iters, trim=0.8):
    tree = cKDTree(dst)
    hist = []
    for it in range(iters):
        s = apply(T, src)
        d, j = tree.query(s, distance_upper_bound=max_dist)
        ok = np.isfinite(d)
        if ok.sum() < 100:
            break
        # trimmed: 近い方 trim 割だけ使う (重ならない部分 = 別経路や駐車車両の影響を減らす)
        thr = np.quantile(d[ok], trim)
        use = ok & (d <= thr)
        R, t = rigid2d(src[use], dst[j[use]])
        T = (R, t)
        hist.append((it, float(np.median(d[ok])), int(use.sum())))
        if it > 3 and abs(hist[-1][1] - hist[-2][1]) < 1e-4:
            break
        max_dist = max(0.5, max_dist * 0.85)
    return T, hist


def trunc_mean_nn(tree, pts, trunc=2.0):
    """最近傍距離の打ち切り平均 (yaw の評価値)。
    claude: 2026-10-11 「1 m 打ち切りの中央値」は密なまだら占有地図では yaw に盲目だった (0° 0.085 / −1° 0.091 とほぼ不変で、
    真の −1.05° を見逃した — critic 検算)。打ち切り平均 (2 m) は単調 (0° 0.160 → −1° 0.114) なのでこちらを使う"""
    d, _ = tree.query(pts, distance_upper_bound=trunc)
    return float(np.where(np.isfinite(d), d, trunc).mean())


def yaw_scan(src, dst, T, half_deg=3.0, step_deg=0.1, inlier=0.5):
    """ICP は回転に鈍い (まだら占有の地図では最近傍の引きが対称になり yaw が初期値のまま止まる — 10-11 実測) ので、
    yaw を総当たりし、各 yaw で平行移動だけ数回合わせて打ち切り平均 NN が最小のものを採る"""
    tree = cKDTree(dst)
    R0, t0 = T
    yaw0 = np.arctan2(R0[1, 0], R0[0, 0])
    c = src.mean(0)
    best = None
    for dy in np.arange(-half_deg, half_deg + 1e-9, step_deg):
        a = yaw0 + np.radians(dy)
        R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        t = R0 @ c + t0 - R @ c  # 重心の位置を保ったまま回す
        for _ in range(6):
            s = src @ R.T + t
            d, j = tree.query(s, distance_upper_bound=inlier)
            ok = np.isfinite(d)
            if ok.sum() < 100:
                break
            t = t + (dst[j[ok]] - s[ok]).mean(0)
        score = trunc_mean_nn(tree, src @ R.T + t)
        if best is None or score < best[0]:
            best = (score, dy, R, t)
    print(f"yaw_scan: best {best[1]:+.2f} deg from ICP result, truncated-mean NN {best[0]:.3f} m")
    return (best[2], best[3])


def raster(pts, res, x0, y0, W, H):
    img = np.zeros((H, W), np.float32)
    c = ((pts[:, 0] - x0) / res).astype(int); r = ((pts[:, 1] - y0) / res).astype(int)
    ok = (c >= 0) & (c < W) & (r >= 0) & (r < H)
    img[r[ok], c[ok]] = 1.0
    return cv2.GaussianBlur(img, (5, 5), 1.0)


def coarse_search(src, dst, res=0.5, yaw_range=60, yaw_step=3):
    lo = np.minimum(src.min(0), dst.min(0)) - 50; hi = np.maximum(src.max(0), dst.max(0)) + 50
    W, H = int((hi[0] - lo[0]) / res), int((hi[1] - lo[1]) / res)
    D = raster(dst, res, lo[0], lo[1], W, H)
    best = None
    cs, cd = src.mean(0), dst.mean(0)
    for yaw in np.arange(-yaw_range, yaw_range + 0.1, yaw_step):
        a = np.radians(yaw)
        R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        s = (src - cs) @ R.T + cd  # 重心を合わせて回す
        S = raster(s, res, lo[0], lo[1], W, H)
        (dx, dy), resp = cv2.phaseCorrelate(S, D)
        if best is None or resp > best[0]:
            best = (resp, yaw, R, cd - R @ cs + np.array([dx * res, dy * res]))
    print(f"coarse: yaw {best[1]:+.0f} deg, response {best[0]:.3f}, t = ({best[3][0]:.1f}, {best[3][1]:.1f})")
    return (best[2], best[3])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src_yaml"); ap.add_argument("dst_yaml"); ap.add_argument("out_dir")
    ap.add_argument("--init", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"), default=None)
    ap.add_argument("--coarse", action="store_true")
    ap.add_argument("--voxel", type=float, default=0.2)
    ap.add_argument("--max_dist", type=float, default=3.0)
    ap.add_argument("--iters", type=int, default=60)
    ap.add_argument("--yaw_half", type=float, default=3.0, help="yaw 総当たりの範囲 ±[deg]")
    ap.add_argument("--yaw_step", type=float, default=0.1, help="yaw 総当たりの刻み [deg]")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    src, _ = load_occ(args.src_yaml); dst, _ = load_occ(args.dst_yaml)
    src, dst = voxel(src, args.voxel), voxel(dst, args.voxel)
    print(f"src {len(src)} cells, dst {len(dst)} cells (voxel {args.voxel} m)")
    if args.coarse:
        T = coarse_search(src, dst)
    else:
        x, y, yd = args.init or (0.0, 0.0, 0.0)
        a = np.radians(yd)
        T = (np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]), np.array([x, y]))
    T, hist = icp(src, dst, T, args.max_dist, args.iters)
    for it, med, n in hist[-3:]:
        print(f"  iter {it}: median NN {med:.3f} m, used {n}")
    T = yaw_scan(src, dst, T, args.yaw_half, args.yaw_step)
    T, hist = icp(src, dst, T, 0.5, 10)  # 仕上げ
    R, t = T
    yaw = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    T4 = np.eye(4); T4[:2, :2] = R; T4[:2, 3] = t
    np.savetxt(os.path.join(args.out_dir, "T_map_from_src.txt"), T4, fmt="%.9f")
    # 残差: 変換後 src の各セルから dst 最近傍までの距離 (1 m 以内のものの分布)
    s = apply(T, src)
    tree = cKDTree(dst)
    d, _ = tree.query(s, distance_upper_bound=1.0)
    ok = np.isfinite(d)
    # nn_median_m / matched_within_1m は密な地図では yaw に盲目 (10-11) — 位置合わせの良さは within_0p2m / trunc_mean_nn_m で見る
    res = {"x": float(t[0]), "y": float(t[1]), "yaw_deg": yaw, "matched_within_1m": float(ok.mean()),
           "within_0p2m": float((d < 0.2).mean()), "trunc_mean_nn_m": trunc_mean_nn(tree, s),
           "nn_median_m": float(np.median(d[ok])), "nn_p95_m": float(np.percentile(d[ok], 95)), "iters": len(hist)}
    json.dump(res, open(os.path.join(args.out_dir, "residual.json"), "w"), indent=1)
    print("result:", json.dumps(res))
    # overlay
    resv = 0.2
    lo = np.minimum(s.min(0), dst.min(0)) - 5; hi = np.maximum(s.max(0), dst.max(0)) + 5
    W, H = int((hi[0] - lo[0]) / resv), int((hi[1] - lo[1]) / resv)
    img = np.full((H, W, 3), 205, np.uint8)
    def cells(p):
        c = ((p[:, 0] - lo[0]) / resv).astype(int); r = H - 1 - ((p[:, 1] - lo[1]) / resv).astype(int)
        return r.clip(0, H - 1), c.clip(0, W - 1)
    A = np.zeros((H, W), bool); B = np.zeros((H, W), bool)
    A[cells(s)] = True; B[cells(dst)] = True
    img[A & B] = (0, 0, 0); img[A & ~B] = (0, 0, 230); img[~A & B] = (230, 120, 0)
    cv2.putText(img, f"black=both red=src-only blue=map-only  t=({t[0]:.2f},{t[1]:.2f}) yaw={yaw:.2f}deg  NN median {res['nn_median_m']:.2f} m",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.imwrite(os.path.join(args.out_dir, "overlay.png"), img)


if __name__ == "__main__":
    main()
