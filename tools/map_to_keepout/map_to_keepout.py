#!/usr/bin/env python3
# claude: 2026-10-02 作成
# --mark_unknown 付きで作った 2D 地図 (glim_dump_to_2dmap / glim_traj_to_2dmap の出力) から
# Nav2 KeepoutFilter 用マスク (keep_out.pgm + keep_out.yaml) を作る。
#
#   keepout (黒) = 未観測 (画素値 == --unknown_value)  [--include_walls で 壁 (画素値 < 128) も]
#                  − 走行軌跡から --path_clear_radius 以内 (実際に走れた場所)
#   それ以外 (白) = 空き、および点が少数あるだけの灰 (lo〜hi 点の濃淡) → 進入可
#
# 走行軌跡の除外が要る理由 (2026-10-02 実測): ロボットの 1.6 m 後ろを歩く操作者が毎スキャン
# 帯に入り、走路そのものが「壁」として焼き付く (09-18 5号館 traj 版で軌跡上の 94% が占有)。
# --clean_map を付けると、同じ範囲の壁/未観測を白に戻した本体地図も書き出す。
#
# マスクは白黒 2 値で書く (filter_mask_server は trinary で読み、黒 = 100 → 進入禁止。
# 灰 = unknown (-1) は KeepoutFilter が無視するので、灰を残すと禁止にならない)。
#
# 依存: numpy (+ --min_unknown_area > 0 のとき cv2)。glim_env にある。
#
# 使い方例:
#   python3 map_to_keepout.py <map.yaml> <keep_out_dir> \
#       --min_unknown_area 1.0 --traj <dump>/traj_lidar.txt --clean_map <map_dir>/nav2

import argparse
import json
import os
import re
import sys

import numpy as np


def read_pgm(path):
    b = open(path, "rb").read()
    m = re.match(rb"P5\s+(?:#.*\s+)*(\d+)\s+(\d+)\s+(\d+)\s", b)
    if m is None:
        raise SystemExit(f"{path}: binary PGM (P5) ではない")
    w, h = int(m.group(1)), int(m.group(2))
    return np.frombuffer(b[m.end():m.end() + w * h], np.uint8).reshape(h, w)


def read_map_yaml(path):
    txt = open(path).read()
    image = re.search(r"^image:\s*(\S+)", txt, re.M).group(1)
    res = float(re.search(r"^resolution:\s*([-\d.eE+]+)", txt, re.M).group(1))
    org = [float(v) for v in re.search(r"^origin:\s*\[([^\]]+)\]", txt, re.M).group(1).split(",")]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(path)), image)
    return image, res, org


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


def main(argv=None):
    ap = argparse.ArgumentParser(description="--mark_unknown 付き 2D 地図 -> Nav2 keepout マスク")
    ap.add_argument("map_yaml")
    ap.add_argument("dest_dir", help="keep_out.pgm / keep_out.yaml の出力先 (nav2.launch.py 規約では <map_dir>/keep_out)")
    ap.add_argument("--unknown_value", type=int, default=180, help="地図の未観測画素の値 (--mark_unknown の既定 180)")
    # 2026-10-02 ユーザ指示で既定は「壁を入れない」(壁は static_layer + inflation が担う。keepout 側は inflation されない)
    ap.add_argument("--include_walls", action="store_true", help="壁 (占有) も keepout に含める")
    ap.add_argument("--min_unknown_area", type=float, default=0.0,
                    help="この面積 [m^2] 未満の未観測の塊は keepout にしない (空き域内の小穴が通路を塞ぐのを防ぐ)")
    ap.add_argument("--traj", default=None,
                    help="TUM 形式の軌跡 (traj_lidar.txt 等)。走行位置の周りを keepout から外す")
    ap.add_argument("--path_clear_radius", type=float, default=0.4,
                    help="軌跡の各姿勢からこの半径 [m] を keepout から外す (0 で無効。Nav2 robot_radius 0.35 + 余裕)")
    ap.add_argument("--clean_map", default=None,
                    help="走路の除外範囲を白にした本体地図 (map.pgm + map.yaml + my_map.yaml) の出力先")
    args = ap.parse_args(argv)

    image, res, org = read_map_yaml(args.map_yaml)
    img = read_pgm(image)
    H, W = img.shape
    unk = img == args.unknown_value
    if not unk.any():
        print(f"warning: 画素値 {args.unknown_value} が 1 つも無い (--mark_unknown 無しの地図?)")
    n_small = 0
    if args.min_unknown_area > 0:
        import cv2
        n, lab, st, _ = cv2.connectedComponentsWithStats(unk.astype(np.uint8), connectivity=8)
        small = np.nonzero(st[:, cv2.CC_STAT_AREA] * res * res < args.min_unknown_area)[0]
        small = small[small != 0]
        n_small = len(small)
        unk &= ~np.isin(lab, small)
    occ = (img < 128) if args.include_walls else np.zeros_like(unk)
    keep = unk | occ

    path = np.zeros_like(keep)
    if args.traj:
        t = np.loadtxt(args.traj, ndmin=2)[:, 1:3]
        px = np.floor((t[:, 0] - org[0]) / res).astype(np.int64)
        py = H - 1 - np.floor((t[:, 1] - org[1]) / res).astype(np.int64)
        ok = (px >= 0) & (px < W) & (py >= 0) & (py < H)
        before = keep[py[ok], px[ok]]
        print(f"traj: {len(t)} poses ({(~ok).sum()} outside map); before clearing, on keepout {before.sum()} "
              f"({100*before.mean():.1f}%: walls {(occ[py[ok], px[ok]]).sum()}, unknown {(unk[py[ok], px[ok]]).sum()})")
        if args.path_clear_radius > 0:
            import cv2
            r = max(int(round(args.path_clear_radius / res)), 1)
            pm = np.zeros((H, W), np.uint8)
            pm[py[ok], px[ok]] = 1
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
            path = cv2.dilate(pm, k) > 0
            keep &= ~path
    mask = np.where(keep, 0, 255).astype(np.uint8)

    os.makedirs(args.dest_dir, exist_ok=True)
    with open(os.path.join(args.dest_dir, "keep_out.pgm"), "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode())
        f.write(mask.tobytes())
    with open(os.path.join(args.dest_dir, "keep_out.yaml"), "w") as f:
        f.write(f"image: keep_out.pgm\nmode: trinary\nresolution: {res}\n"
                f"origin: [{org[0]}, {org[1]}, {org[2] if len(org) > 2 else 0.0}]\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.25\nnegate: 0\n")
    a = res * res
    print(f"keepout: {keep.sum()*a:.0f} m^2 (unknown {(unk & keep).sum()*a:.0f}, walls {(occ & keep).sum()*a:.0f}); "
          f"free {(~keep).sum()*a:.0f} m^2; small unknown blobs dropped {n_small}; path cleared {path.sum()*a:.0f} m^2")

    if args.clean_map:
        # 本体地図: 走路の除外範囲にある壁/未観測を白へ (濃淡の灰はそのまま)
        cm = img.copy()
        fix = path & ((cm < 128) | (cm == args.unknown_value))
        cm[fix] = 255
        os.makedirs(args.clean_map, exist_ok=True)
        with open(os.path.join(args.clean_map, "map.pgm"), "wb") as f:
            f.write(f"P5\n{W} {H}\n255\n".encode())
            f.write(cm.tobytes())
        y = (f"image: map.pgm\nresolution: {res}\norigin: [{org[0]}, {org[1]}, {org[2] if len(org) > 2 else 0.0}]\n"
             "occupied_thresh: 0.5\nfree_thresh: 0.2\nnegate: 0\n")
        for n in ("map.yaml", "my_map.yaml"):  # my_map.yaml = nav2.launch.py の map_dir 規約名
            open(os.path.join(args.clean_map, n), "w").write(y)
        print(f"clean map: {fix.sum()} px set to free -> {args.clean_map}")
        # 元地図の生成条件を引き継ぎ、clean の内容を追記
        src = os.path.join(os.path.dirname(os.path.abspath(image)), "map_params.yaml")
        dst = os.path.join(args.clean_map, "map_params.yaml")
        if os.path.exists(src) and os.path.abspath(src) != os.path.abspath(dst):
            body = open(src).read()
            with open(dst, "w") as f:
                f.write(body + f"cleaned_by_map_to_keepout:\n  source_map: {json.dumps(os.path.abspath(args.map_yaml))}\n"
                        f"  path_clear_radius_m: {args.path_clear_radius}\n  traj: {json.dumps(args.traj)}\n"
                        f"  px_set_free: {int(fix.sum())}\n")
    write_params(os.path.join(args.dest_dir, "keepout_params.yaml"), "map_to_keepout", args, {
        "keepout": "未観測" + (" + 壁" if args.include_walls else "") + " − 小さい未観測塊 − 軌跡周り",
        "min_unknown_area_m2": args.min_unknown_area, "path_clear_radius_m": args.path_clear_radius if args.traj else None,
    }, {"keepout_m2": round(float(keep.sum()) * a, 1), "free_m2": round(float((~keep).sum()) * a, 1),
        "small_unknown_blobs_dropped": n_small})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
