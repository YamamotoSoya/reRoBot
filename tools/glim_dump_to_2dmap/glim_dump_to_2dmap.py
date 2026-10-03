#!/usr/bin/env python3
# claude: 2026-08-20 作成
# GLIM dump (submap 群) を直接読んで Nav2 用 2D 占有格子 (map.pgm + map.yaml) を作る。
#
# 既製 pointcloud_to_2dmap との違い:
#   - 入力がマージ済み PCD ではなく GLIM dump そのもの。各点が「どの submap
#     (= どのセンサ姿勢) から観測されたか」の対応が残っているので、高さスライスを
#     世界座標の絶対 z ではなく「その submap のセンサ z からの相対高さ」で行える
#     (--height_mode sensor, 既定)。地図全体で z がドリフトしていても、スライス帯が
#     センサと一緒に上下するため場所によらず同じ高さ帯の壁が取れる。
#   - --export_pcd で全点をマージした世界座標 PCD も書ける (既製ツールとの比較用)。
#
# 依存: numpy のみ (glim / rerobot_env コンテナに導入済み)。出力は PGM
# (map_server は png/pgm どちらも可)。
#
# 使い方例:
#   python3 glim_dump_to_2dmap.py <dump_dir> <dest_dir> \
#       -r 0.05 --map_width 6144 --map_height 6144 \
#       --min_height -0.5 --max_height 0.7        # センサ z 基準の帯

import argparse
import os
import re
import sys

import numpy as np


def load_submap(dump_dir, idx):
    """submap の T_world_origin (4x4) と点群 (N,3 float32, submap 原点フレーム) を返す"""
    d = os.path.join(dump_dir, f"{idx:06d}")
    txt = open(os.path.join(d, "data.txt")).read()
    m = re.search(r"T_world_origin:\s*\n((?:.+\n){4})", txt)
    if m is None:
        raise RuntimeError(f"T_world_origin not found in {d}/data.txt")
    T = np.array([[float(v) for v in line.split()] for line in m.group(1).strip().split("\n")])
    pts = np.fromfile(os.path.join(d, "points_compact.bin"), dtype=np.float32).reshape(-1, 3)
    return T, pts


def load_viewpoints(dump_dir, idx):
    """claude: 2026-10-02 submap 内の各スキャンのセンサ位置 (T_world_lidar の並進, (F,3)) を返す"""
    txt = open(os.path.join(dump_dir, f"{idx:06d}", "data.txt")).read()
    blocks = re.findall(r"T_world_lidar:\s*\n((?:.+\n){4})", txt)
    return np.array([[float(line.split()[3]) for line in b.strip().split("\n")[:3]] for b in blocks])


def load_frame_stamps(dump_dir, idx):
    """claude: 2026-10-02 submap 内の各スキャンの stamp (F,) を返す"""
    txt = open(os.path.join(dump_dir, f"{idx:06d}", "data.txt")).read()
    return np.array([float(v) for v in re.findall(r"^stamp:\s*([\d.]+)", txt, re.M)])


def load_traj_tum(path):
    """claude: 2026-10-02 TUM 形式 (stamp x y z qx qy qz qw) → (stamps, R[N,3,3], t[N,3])、stamp 昇順"""
    a = np.loadtxt(path, ndmin=2)
    a = a[np.argsort(a[:, 0])]
    x, y, z, w = a[:, 4], a[:, 5], a[:, 6], a[:, 7]
    R = np.empty((len(a), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - z * w); R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w); R[:, 2, 1] = 2 * (y * z + x * w); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return a[:, 0], R, a[:, 1:4]


def base_link_height(w, stamps, traj, h_sensor, tol=0.02):
    """claude: 2026-10-02 世界座標の点 w (N,3) の base_link 基準の高さを返す。
    submap 内スキャンの最適化後 LiDAR 姿勢 (traj_lidar.txt を stamp で引く) のうち水平距離で最寄りの
    ものを選び、点をその LiDAR 座標に戻した z + h_sensor (base_link → LiDAR の高さ、取付 rpy=0 前提)。
    実機 rfans_scan.launch.py (pointcloud_to_laserscan, target_frame base_link) と同じ基準"""
    st, R, t = traj
    k = np.clip(np.searchsorted(st, stamps), 1, len(st) - 1)
    k = np.where(np.abs(st[k - 1] - stamps) < np.abs(st[k] - stamps), k - 1, k)
    k = k[np.abs(st[k] - stamps) <= tol]
    if len(k) == 0:
        return None
    pos = t[k]
    d2 = (w[:, None, 0] - pos[None, :, 0]) ** 2 + (w[:, None, 1] - pos[None, :, 1]) ** 2
    j = k[np.argmin(d2, axis=1)]
    return np.einsum("ni,ni->n", w - t[j], R[j][:, :, 2]) + h_sensor


def add_free_fan(free, sx, sy, wx, wy, obst, cx, cy, res, rmax, n_bins):
    """claude: 2026-10-02 センサ (sx,sy) から見えた範囲を空きとして free (H,W uint8) に塗る。
    方位を n_bins に分け、各方位で「帯内の点 (障害物) の最近距離」と「全点の最遠距離」の小さい方
    までを空きとする (= 光線が障害物で止まる。帯の外の点 = 地面などは「そこまで何も無かった」証拠)。
    光線を 1 本ずつ引く代わりに、方位ごとの端点をつないだ星形多角形を塗る"""
    import cv2
    H, W = free.shape
    dx, dy = wx - sx, wy - sy
    r = np.hypot(dx, dy)
    m = (r > 0.05) & (r <= rmax)
    if not m.any():
        return
    b = ((np.arctan2(dy[m], dx[m]) + np.pi) / (2 * np.pi) * n_bins).astype(np.int64) % n_bins
    rm = r[m]
    rfar = np.zeros(n_bins)
    np.maximum.at(rfar, b, rm)
    robs = np.full(n_bins, np.inf)
    om = obst[m]
    np.minimum.at(robs, b[om], rm[om])
    rf = np.minimum(rfar, robs)
    ang = (np.arange(n_bins) + 0.5) / n_bins * 2 * np.pi - np.pi
    ex = sx + rf * np.cos(ang)
    ey = sy + rf * np.sin(ang)
    # 画素系は地図本体と同じ (x 右, 画像行は -y)。shift=2 で 1/4 画素精度
    px = ((ex - cx) / res + W // 2) * 4
    py = (-(ey - cy) / res + H // 2) * 4
    cv2.fillPoly(free, [np.stack([px, py], axis=1).round().astype(np.int32)], 1, lineType=cv2.LINE_8, shift=2)


def list_submaps(dump_dir):
    ids = sorted(int(n) for n in os.listdir(dump_dir) if n.isdigit() and len(n) == 6)
    if not ids:
        raise RuntimeError(f"no submap directories under {dump_dir}")
    return ids


def write_pcd(path, points):
    """世界座標の全点を binary PCD (x y z float32) で書き出す"""
    n = len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(points.astype(np.float32).tobytes())


def write_params(path, tool, args, summary, result):
    """claude: 2026-10-02 生成条件の記録 (map_params.yaml)。値は JSON 表記 (YAML として読める、PyYAML 不要)。
    summary = 高さ基準・帯・距離などの要点、result = 実行結果の要約"""
    import datetime
    import json
    import shlex

    def dump(d, ind=""):
        return "".join(f"{ind}{k}: {json.dumps(v, ensure_ascii=False)}\n" for k, v in d.items())

    with open(path, "w") as f:
        f.write(f"# {tool} の生成条件 (自動生成。再実行は command をそのまま使う)\n")
        f.write(dump({"tool": tool, "created": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
                      "command": "python3 " + " ".join(shlex.quote(a) for a in sys.argv)}))
        f.write("summary:\n" + dump(summary, "  "))
        f.write("args:\n" + dump({k: (os.path.abspath(v) if isinstance(v, str) and os.path.exists(v) else v)
                                  for k, v in vars(args).items()}, "  "))
        f.write("result:\n" + dump(result, "  "))


def match_owner(dest, ref):
    """claude: 2026-10-02 root (コンテナ) で書いた出力の所有者を ref (実行前から在った親ディレクトリ) に合わせる。
    ホストのユーザが GIMP 等でそのまま編集できるようにするため。root 以外で実行したときは何もしない"""
    if not hasattr(os, "geteuid") or os.geteuid() != 0:
        return
    st = os.stat(ref)
    d = os.path.abspath(dest)
    while d != os.path.abspath(ref) and d != os.path.dirname(d):  # dest から ref の手前まで遡って作ったディレクトリ
        os.chown(d, st.st_uid, st.st_gid)
        d = os.path.dirname(d)
    for root, dirs, files in os.walk(dest):
        for n in dirs + files:
            os.chown(os.path.join(root, n), st.st_uid, st.st_gid)


def existing_parent(path):
    """path の最も近い既存の祖先ディレクトリ"""
    d = os.path.dirname(os.path.abspath(path))
    while not os.path.isdir(d):
        d = os.path.dirname(d)
    return d


def run_keepout(dest_dir, map_dir, traj_path, args):
    """claude: 2026-10-02 --with_keepout: raw/ の地図から keep_out/ と nav2/ (clean map) を作る"""
    sys.dont_write_bytecode = True  # tools/ 配下に root 所有の __pycache__ を作らない
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "map_to_keepout"))
    import map_to_keepout
    kargv = [os.path.join(map_dir, "map.yaml"), os.path.join(dest_dir, "keep_out"),
             "--unknown_value", str(args.unknown_value), "--min_unknown_area", str(args.min_unknown_area),
             "--path_clear_radius", str(args.path_clear_radius), "--clean_map", os.path.join(dest_dir, "nav2")]
    if traj_path and os.path.exists(traj_path):
        kargv += ["--traj", traj_path]
    else:
        print("warning: 軌跡が無いので走路周りの除外 (操作者跡の掃除) をしない")
    map_to_keepout.main(kargv)


def main():
    ap = argparse.ArgumentParser(description="GLIM dump -> Nav2 2D occupancy grid")
    ap.add_argument("dump_dir")
    ap.add_argument("dest_dir")
    ap.add_argument("-r", "--resolution", type=float, default=0.05, help="m / pixel")
    ap.add_argument("--map_width", type=int, default=0, help="pixels; 0 = auto fit to points")
    ap.add_argument("--map_height", type=int, default=0, help="pixels; 0 = auto fit to points")
    ap.add_argument("--center", choices=["world", "auto"], default="world",
                    help="world: 既製ツール互換 (world 原点中心) / auto: 点群 bbox 中心")
    ap.add_argument("--height_mode", choices=["sensor", "absolute", "base_link"], default="sensor",
                    help="sensor: submap 原点 z 基準の相対高さ (LiDAR 高ではない) / absolute: 世界座標 z / "
                         "base_link: 実機 /scan と同じ車体基準の地上高 (要 --base_to_sensor_z)")
    # claude: 2026-10-02 base_link モード用
    ap.add_argument("--base_to_sensor_z", type=float, default=None,
                    help="base_link → rfans の高さ [m] (URDF rfans_joint z。〜09-30 の bag は 0.80246、10-01 以降 0.79396)")
    ap.add_argument("--traj", default=None,
                    help="最適化後 LiDAR 軌跡 (既定 <dump_dir>/traj_lidar.txt)。base_link / --range_max / --mark_unknown の"
                         "スキャン位置に使う (data.txt の T_world_lidar は最適化前で最大数 m ずれる)")
    ap.add_argument("--range_max", type=float, default=0.0,
                    help="submap 内の最寄りスキャン位置から水平にこの距離 [m] を超える点を捨てる (0 = 無制限。"
                         "実機 rfans_scan / traj ツールに合わせるなら 30)")
    ap.add_argument("--min_height", type=float, default=-0.5)
    ap.add_argument("--max_height", type=float, default=0.7)
    ap.add_argument("--min_points_in_pix", type=int, default=2)
    ap.add_argument("--max_points_in_pix", type=int, default=5)
    ap.add_argument("--export_pcd", default=None,
                    help="全点 (高さフィルタ前) の世界座標マージ PCD をこのパスへ書き出す")
    # claude: 2026-10-02 未観測域を灰 (map_server で unknown) にする
    ap.add_argument("--mark_unknown", action="store_true",
                    help="センサから見えた範囲を空き、見えていない範囲を --unknown_value で塗る (要 cv2)")
    ap.add_argument("--unknown_value", type=int, default=180,
                    help="未観測画素の値。occupied/free_thresh 0.5/0.2 では 128〜204 が unknown")
    ap.add_argument("--free_range_max", type=float, default=30.0, help="空きを伸ばす最大水平距離 [m]")
    ap.add_argument("--view_stride", type=int, default=10,
                    help="submap 内のスキャン姿勢を N 個に 1 個だけ視点に使う (点とスキャンの対応は dump に無いので近似)")
    ap.add_argument("--free_bins", type=int, default=720, help="空き判定の方位分割数 (720 = 0.5°)")
    # claude: 2026-10-02 1 コマンドで Nav2 用一式 (raw/ nav2/ keep_out/) を作る
    # 既定 = 一式出力 (2026-10-02 ユーザ指示)。地図だけ欲しいときは --map_only
    ap.add_argument("--map_only", dest="with_keepout", action="store_false",
                    help="地図 (map.pgm / map.yaml / map_params.yaml) だけを dest_dir に書く (従来動作)。"
                         "既定は dest_dir を地図一式の親とし、raw/ (未観測マーク付き原本)・nav2/ (走路掃除済み本体 + "
                         "map.yaml)・keep_out/ (未観測 = 進入禁止) を書く (--mark_unknown を自動で有効化)")
    ap.add_argument("--with_keepout", dest="with_keepout", action="store_true", default=True,
                    help="一式出力 (既定。互換のため残しているだけ)")
    ap.add_argument("--min_unknown_area", type=float, default=1.0, help="--with_keepout: この面積 [m^2] 未満の未観測塊は keepout にしない")
    ap.add_argument("--path_clear_radius", type=float, default=0.4, help="--with_keepout: 軌跡からこの半径 [m] を keepout・壁から外す")
    args = ap.parse_args()
    if args.with_keepout:
        args.mark_unknown = True
    out_dir = os.path.join(args.dest_dir, "raw") if args.with_keepout else args.dest_dir
    owner_ref = existing_parent(args.dest_dir)

    ids = list_submaps(args.dump_dir)
    traj = None
    n_fallback = 0
    if args.height_mode == "base_link" and args.base_to_sensor_z is None:
        raise SystemExit("--height_mode base_link には --base_to_sensor_z が必要 (URDF rfans_joint の z)")
    if args.height_mode == "base_link" or args.range_max > 0 or args.mark_unknown:
        # claude: 2026-10-02 スキャン位置は最適化後の traj_lidar.txt から引く
        tp = args.traj or os.path.join(args.dump_dir, "traj_lidar.txt")
        if os.path.exists(tp):
            traj = load_traj_tum(tp)
        elif args.height_mode == "base_link":
            raise SystemExit(f"{tp} が無い (base_link モードには最適化後軌跡が必要)")
        else:
            print(f"warning: {tp} が無いので data.txt の T_world_lidar (最適化前) をスキャン位置に使う")
    world_pts = []      # スライス帯を通過した点 (地図用)
    all_pts = []        # 全点 (--export_pcd 用)
    views = []          # (視点 (F,3), 全点 xy, 帯内マスク) — --mark_unknown 用
    for i in ids:
        T, pts = load_submap(args.dump_dir, i)
        w = pts @ T[:3, :3].T + T[:3, 3]
        if args.export_pcd:
            all_pts.append(w)
        # submap 内スキャン位置 (traj があれば最適化後、無ければ data.txt)
        vp = None
        if traj is not None:
            stamps = load_frame_stamps(args.dump_dir, i)
            k = np.clip(np.searchsorted(traj[0], stamps), 1, len(traj[0]) - 1)
            k = np.where(np.abs(traj[0][k - 1] - stamps) < np.abs(traj[0][k] - stamps), k - 1, k)
            k = k[np.abs(traj[0][k] - stamps) <= 0.02]
            if len(k):
                vp = traj[2][k]
            else:
                n_fallback += 1
        if vp is None and args.height_mode != "base_link":
            vp = load_viewpoints(args.dump_dir, i)
            if len(vp) == 0:
                vp = T[None, :3, 3]
        if args.height_mode == "base_link":
            rel = base_link_height(w, stamps, traj, args.base_to_sensor_z) if vp is not None else None
            if rel is None:  # traj に対応するスキャンが無い submap は捨てる
                rel = np.full(len(w), np.nan)
        else:
            z_ref = T[2, 3] if args.height_mode == "sensor" else 0.0
            rel = w[:, 2] - z_ref
        keep = (rel >= args.min_height) & (rel <= args.max_height)
        if args.range_max > 0 and vp is not None:
            # claude: 2026-10-02 遠方の点を捨てる。30 m 超の点は base_link 高さが距離に比例して持ち上がり
            # 地面が帯に入る (09-18 5号館: 30 m 以上でノイズ率 31%、未満は 1〜2%)
            d2 = ((w[:, None, 0] - vp[None, :, 0]) ** 2 + (w[:, None, 1] - vp[None, :, 1]) ** 2).min(axis=1)
            keep &= d2 <= args.range_max ** 2
        world_pts.append(w[keep, :2])
        if args.mark_unknown:
            views.append((vp[::max(args.view_stride, 1)], w[:, :2].copy(), keep))
    sel = np.concatenate(world_pts)
    print(f"submaps: {len(ids)}, points in height band: {len(sel)}")
    if traj is not None and n_fallback:
        print(f"warning: {n_fallback} submaps had no matching traj stamps "
              f"({'skipped' if args.height_mode == 'base_link' else 'used data.txt poses'})")

    if args.export_pcd:
        merged = np.concatenate(all_pts)
        write_pcd(args.export_pcd, merged)
        print(f"exported merged PCD: {args.export_pcd} ({len(merged)} points, "
              f"x[{merged[:,0].min():.1f},{merged[:,0].max():.1f}] "
              f"y[{merged[:,1].min():.1f},{merged[:,1].max():.1f}] "
              f"z[{merged[:,2].min():.1f},{merged[:,2].max():.1f}])")

    res = args.resolution
    if args.center == "world":
        cx, cy = 0.0, 0.0
    else:
        cx = (sel[:, 0].min() + sel[:, 0].max()) / 2
        cy = (sel[:, 1].min() + sel[:, 1].max()) / 2
    W, H = args.map_width, args.map_height
    if W == 0 or H == 0:
        # bbox が入る最小サイズ + 2 m マージン (64 px 単位に切り上げ)
        need_w = int(np.ceil((2 * np.abs(sel[:, 0] - cx).max() + 4.0) / res / 64) * 64)
        need_h = int(np.ceil((2 * np.abs(sel[:, 1] - cy).max() + 4.0) / res / 64) * 64)
        W = W or need_w
        H = H or need_h
    print(f"map: {W}x{H} px @ {res} m/px, center=({cx:.2f},{cy:.2f})")

    # 既製ツールと同じ画素系: x 右向き、y は上向き (画像行は -y)。中心画素 = (cx,cy)
    px = np.floor((sel[:, 0] - cx) / res).astype(np.int64) + W // 2
    py = np.floor(-(sel[:, 1] - cy) / res).astype(np.int64) + H // 2
    ok = (px >= 0) & (px < W) & (py >= 0) & (py < H)
    dropped = np.count_nonzero(~ok)
    if dropped:
        print(f"warning: {dropped} points fell outside the map ({100*dropped/len(sel):.1f}%)")
    counts = np.bincount(py[ok] * W + px[ok], minlength=W * H).reshape(H, W)

    # 既製ツールと同じ濃度変換: count<=min → 白 (自由), count>=max → 黒 (占有)
    lo, hi = args.min_points_in_pix, args.max_points_in_pix
    img = np.clip(255.0 - 255.0 * (counts - lo) / (hi - lo), 0, 255).astype(np.uint8)

    if args.mark_unknown:
        # claude: 2026-10-02 白 (点数 <= lo) のうち、どの視点からも見えていない画素を unknown に
        free = np.zeros((H, W), np.uint8)
        nv = 0
        for vp, wxy, keep in views:
            for v in vp:
                add_free_fan(free, v[0], v[1], wxy[:, 0], wxy[:, 1], keep, cx, cy, res,
                             args.free_range_max, args.free_bins)
                nv += 1
        unk = (img == 255) & (free == 0)
        img[unk] = args.unknown_value
        print(f"mark_unknown: {nv} viewpoints, free {int(free.sum())} px, unknown {int(unk.sum())} px")

    os.makedirs(out_dir, exist_ok=True)
    pgm = os.path.join(out_dir, "map.pgm")
    with open(pgm, "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode())
        f.write(img.tobytes())
    origin_x = cx - res * W / 2
    origin_y = cy - res * H / 2
    with open(os.path.join(out_dir, "map.yaml"), "w") as f:
        f.write(f"image: map.pgm\nresolution: {res}\n"
                f"origin: [{origin_x}, {origin_y}, 0.0]\n"
                "occupied_thresh: 0.5\nfree_thresh: 0.2\nnegate: 0\n")
    occ = int((img < 128).sum())
    write_params(os.path.join(out_dir, "map_params.yaml"), "glim_dump_to_2dmap", args, {
        "height_reference": {"sensor": "submap 原点 z (LiDAR 高ではない)", "absolute": "世界座標 z",
                             "base_link": "base_link (実機 rfans_scan と同じ)"}[args.height_mode],
        "height_band_m": [args.min_height, args.max_height],
        "base_to_sensor_z_m": args.base_to_sensor_z,
        "range_max_m": args.range_max if args.range_max > 0 else None,
        "points_in_pix_lo_hi": [lo, hi],
        "resolution_m": res,
        "mark_unknown": args.mark_unknown,
    }, {
        "submaps": len(ids), "points_in_band": int(len(sel)), "map_px": [W, H],
        "origin": [origin_x, origin_y], "occupied_m2": round(occ * res * res, 1),
        "unknown_m2": round(int((img == args.unknown_value).sum()) * res * res, 1) if args.mark_unknown else None,
        "traj_used": (args.traj or os.path.join(args.dump_dir, "traj_lidar.txt")) if traj is not None else None,
    })
    print(f"wrote {pgm}, map.yaml, map_params.yaml (origin=[{origin_x:.2f},{origin_y:.2f}]); occupied {occ*res*res:.1f} m^2")
    if args.with_keepout:
        run_keepout(args.dest_dir, out_dir, args.traj or os.path.join(args.dump_dir, "traj_lidar.txt"), args)
    match_owner(args.dest_dir, owner_ref)
    return 0


if __name__ == "__main__":
    sys.exit(main())
