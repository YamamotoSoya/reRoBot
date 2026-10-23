#!/usr/bin/env python3
# claude: 2026-10-15 作成。GLIM dump の点群の「鮮鋭度」を数値化する。
#
# 目的: 「建物のエッジが鮮明か」を目視でなく数値で比べる。設定を変えた dump どうしの
#       比較に使う (このプロジェクトは実行間のばらつきが大きく、目視と単発指標では
#       設定の効果とノイズを分離しにくいため — docs/issue/2026-10-10 §6(b)/§7)。
#
# 原理: 点群をボクセルに分け、各ボクセル内の点の共分散を固有値分解する。
#       最小固有値の平方根 = その局所パッチの「面に垂直な方向の広がり」= 厚み。
#       よく合った壁は薄く (数 mm〜cm)、ずれて二重化した壁は厚くなる。
#       法線 (最小固有値の固有ベクトル) が水平に近いボクセルだけを選べば壁だけを測れる。
#
# 使い方 (コンテナ内に numpy があるので docker exec で回す):
#   docker exec glim_gpu_env python3 /workspace/tools/glim_map_sharpness.py \
#     /workspace/bags/glim/<bag>_dump/<タグ> [--stride 10] [--voxel 0.3]
#
#   --mode submap (既定) : submap ごとに自分の点だけで測る = sub_mapping の合成品質
#   --mode merged        : 近接する submap をまとめて測る = submap 間の整合も含む
import argparse, os, sys, struct
import numpy as np


def read_submap(d):
    """dump の 1 submap を読む。points_compact.bin は float32 x,y,z (12 B/点)。"""
    pf = os.path.join(d, "points_compact.bin")
    if not os.path.exists(pf):
        return None, None
    pts = np.fromfile(pf, dtype=np.float32).reshape(-1, 3).astype(np.float64)
    # data.txt の T_world_origin (4x4) を読む
    T = np.eye(4)
    with open(os.path.join(d, "data.txt")) as f:
        lines = f.readlines()
    for i, ln in enumerate(lines):
        if ln.startswith("T_world_origin:"):
            T = np.array([[float(v) for v in lines[i + 1 + r].split()] for r in range(4)])
            break
    return pts, T


def voxel_thickness(pts, vres, min_pts, wall_only):
    """ボクセルごとに共分散の最小固有値 → 厚み [m] を返す。"""
    if len(pts) < min_pts:
        return np.array([])
    idx = np.floor(pts / vres).astype(np.int64)
    # ボクセル id を 1 本のキーにまとめる
    key = (idx[:, 0] * 1000003) ^ (idx[:, 1] * 10000019) ^ (idx[:, 2] * 100000007)
    order = np.argsort(key, kind="stable")
    key_s, pts_s = key[order], pts[order]
    bounds = np.flatnonzero(np.diff(key_s)) + 1
    groups = np.split(np.arange(len(key_s)), bounds)

    # claude: ★密度バイアス対策★ ボクセルごとに必ず min_pts 点だけを無作為に使う。
    #   そのまま全点を使うと、点が濃い dump ほど共分散の推定が安定して厚みが小さく出るうえ、
    #   「最低点数を満たすボクセル」の数自体が変わって母集団がずれる。
    #   同数に揃えることで、測っているのが密度ではなく合わせの精度になる。
    rng = np.random.default_rng(0)
    out = []
    for g in groups:
        if len(g) < min_pts:
            continue
        if len(g) > min_pts:
            g = rng.choice(g, min_pts, replace=False)
        P = pts_s[g]
        C = np.cov((P - P.mean(0)).T)
        if not np.all(np.isfinite(C)):
            continue
        w, v = np.linalg.eigh(C)
        if w[0] < 0:
            w[0] = 0.0
        # 平面的でないボクセル (角・植生・ノイズ) は除く: 最小/中間 の比が大きいものを除外
        if w[1] <= 0 or w[0] / w[1] > 0.1:
            continue
        if wall_only:
            n = v[:, 0]           # 最小固有値の固有ベクトル = 面法線
            if abs(n[2]) > 0.3:   # 水平面 (地面・屋根) を除外し垂直面だけ残す
                continue
        out.append(np.sqrt(w[0]))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("--stride", type=int, default=10, help="何個おきに submap を見るか")
    ap.add_argument("--voxel", type=float, default=0.3, help="ボクセル辺長 [m]")
    ap.add_argument("--min_pts", type=int, default=20, help="評価する最小点数/ボクセル")
    ap.add_argument("--mode", choices=["submap", "merged"], default="submap")
    ap.add_argument("--all_surfaces", action="store_true", help="垂直面に限定しない")
    a = ap.parse_args()

    subs = sorted(d for d in os.listdir(a.dump) if d.isdigit())
    if not subs:
        print("submap が見つかりません", file=sys.stderr)
        sys.exit(1)
    picked = subs[:: a.stride]

    th = []
    if a.mode == "submap":
        for s in picked:
            pts, _ = read_submap(os.path.join(a.dump, s))
            if pts is None:
                continue
            th.append(voxel_thickness(pts, a.voxel, a.min_pts, not a.all_surfaces))
    else:
        # 連続する 5 submap をワールド座標で重ねて測る (submap 間の整合も含む)
        for i in range(0, len(picked) - 4, 5):
            acc = []
            for s in picked[i : i + 5]:
                pts, T = read_submap(os.path.join(a.dump, s))
                if pts is None:
                    continue
                acc.append(pts @ T[:3, :3].T + T[:3, 3])
            if acc:
                th.append(voxel_thickness(np.vstack(acc), a.voxel, a.min_pts, not a.all_surfaces))

    th = np.concatenate([t for t in th if len(t)]) if any(len(t) for t in th) else np.array([])
    if len(th) == 0:
        print("評価できるボクセルがありませんでした")
        return
    name = os.path.basename(a.dump.rstrip("/"))
    print(
        f"[{name:16s}] mode={a.mode} 壁ボクセル {len(th):6d} 個 / "
        f"厚み 中央値 {np.median(th)*1000:6.1f} mm  平均 {th.mean()*1000:6.1f} mm  "
        f"25%点 {np.percentile(th,25)*1000:6.1f} mm  75%点 {np.percentile(th,75)*1000:6.1f} mm"
    )


if __name__ == "__main__":
    main()
