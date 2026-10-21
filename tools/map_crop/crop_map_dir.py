#!/usr/bin/env python3
# claude: 2026-10-17 — 地図ディレクトリ一式を世界座標の矩形で切り抜いて別ディレクトリへ書き出す。
#   目的: つくば composite 地図 (9984×4736 = 47 M セル) だと NavFn が 1 区間 0.55〜0.8 s かかり、
#   11 点の区間で計画 6〜9 s → 古い始点の経路で首振り / "0 poses" abort (bags/exp/2026-10-17_tsukuba_wander/)。
#   NavFn は区間ごとに地図全体をコピー・初期化するので、走らない範囲を落とすのが効く。
#
#   対象: <src>/{nav2,keep_out,raw_traj/*,raw_dump/*}/ の map 系 yaml (image: を持つもの) と waypoints/course.yaml。
#   同じ解像度・同じ origin の地図 (nav2 と keep_out) は同じ画素で切れるので、切り抜き後もサイズが一致する
#   (global costmap の StaticLayer 2 枚が同サイズである前提を保つ)。
#
#   使い方 (範囲は course.yaml の点 + --extra の点の外接矩形 + --margin):
#     python3 tools/map_crop/crop_map_dir.py <src_map_dir> <dst_map_dir> [--until waypoint11] \
#         [--extra=-21.6,-5.5 --extra=6.0,1.2] [--margin 30] [--bbox=xmin,xmax,ymin,ymax]
#     --until : course.yaml をこの点まで残す (以降は捨てる)。範囲計算にもこの点までを使う
#     --bbox  : 範囲を直接指定 (--extra / --margin より優先)
import argparse, glob, math, os, shutil, sys
import numpy as np
import yaml
from PIL import Image

Image.MAX_IMAGE_PIXELS = None  # raw_traj の 0.05 m 地図は 2.8 億画素あり、PIL の既定上限を超える


def load_course(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    return d, list(d["waypoints"].keys())


def crop_map(yaml_path, dst_yaml, bbox, note):
    with open(yaml_path) as f:
        m = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(yaml_path), m["image"])
    im = Image.open(img_path)
    a = np.array(im)
    H, W = a.shape[:2]
    res = float(m["resolution"])
    ox, oy = float(m["origin"][0]), float(m["origin"][1])
    xmin, xmax, ymin, ymax = bbox
    # 画素の列 c / 下からの行 r (origin = 左下画素の左下角)
    c0 = max(0, math.floor((xmin - ox) / res)); c1 = min(W, math.ceil((xmax - ox) / res))
    r0 = max(0, math.floor((ymin - oy) / res)); r1 = min(H, math.ceil((ymax - oy) / res))
    if c1 <= c0 or r1 <= r0:
        sys.exit(f"範囲が地図の外: {yaml_path}")
    sub = a[H - r1:H - r0, c0:c1]  # pgm は上の行が先頭
    os.makedirs(os.path.dirname(dst_yaml), exist_ok=True)
    out_img = os.path.join(os.path.dirname(dst_yaml), m["image"])
    Image.fromarray(sub, mode=im.mode).save(out_img)
    m["origin"] = [round(ox + c0 * res, 6), round(oy + r0 * res, 6), float(m["origin"][2]) if len(m["origin"]) > 2 else 0.0]
    with open(dst_yaml, "w") as f:
        f.write(f"# claude: {note}\n")
        yaml.safe_dump(m, f, sort_keys=False, allow_unicode=True, default_flow_style=None)
    print(f"  {os.path.relpath(yaml_path)}: {W}x{H} → {c1 - c0}x{r1 - r0} ({(c1 - c0) * (r1 - r0) / 1e6:.1f} M セル)  origin {m['origin'][:2]}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src"); ap.add_argument("dst")
    ap.add_argument("--until", default="", help="course.yaml をこの waypoint まで残す")
    ap.add_argument("--extra", action="append", default=[], help="範囲に含める追加点 --extra=x,y (複数可。負の値があるので = 必須)")
    ap.add_argument("--margin", type=float, default=30.0)
    ap.add_argument("--bbox", default="", help="xmin,xmax,ymin,ymax (直接指定)")
    a = ap.parse_args()
    src, dst = a.src.rstrip("/"), a.dst.rstrip("/")
    if os.path.exists(dst):
        sys.exit(f"出力先が既にある: {dst}")
    course_path = os.path.join(src, "waypoints", "course.yaml")
    course, names = load_course(course_path) if os.path.exists(course_path) else (None, [])
    if a.until:
        if a.until not in names:
            sys.exit(f"--until {a.until} が course.yaml に無い")
        names = names[:names.index(a.until) + 1]
    if a.bbox:
        bbox = tuple(float(v) for v in a.bbox.split(","))
    else:
        pts = [course["waypoints"][n]["pose"][:2] for n in names] + [[float(v) for v in e.split(",")] for e in a.extra]
        if not pts:
            sys.exit("範囲を決める点が無い (--bbox を指定)")
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        bbox = (min(xs) - a.margin, max(xs) + a.margin, min(ys) - a.margin, max(ys) + a.margin)
    note = (f"crop_map_dir.py で {src} から切り抜き (x {bbox[0]:.1f}〜{bbox[1]:.1f}, y {bbox[2]:.1f}〜{bbox[3]:.1f})")
    print(note)
    targets = [os.path.join(src, "nav2", "map.yaml"), os.path.join(src, "keep_out", "keep_out.yaml")]
    targets += sorted(glob.glob(os.path.join(src, "raw_traj", "*", "map.yaml")) + glob.glob(os.path.join(src, "raw_dump", "*", "map.yaml")))
    for t in targets:
        if os.path.exists(t):
            crop_map(t, os.path.join(dst, os.path.relpath(t, src)), bbox, note)
    # 生成条件の記録 (map_params.yaml / keepout_params.yaml) はそのまま写す (元の地図の出自)
    for p in glob.glob(os.path.join(src, "**", "*_params.yaml"), recursive=True):
        q = os.path.join(dst, os.path.relpath(p, src))
        if os.path.isdir(os.path.dirname(q)):
            shutil.copy2(p, q)
    if course is not None:
        course["waypoints"] = {n: course["waypoints"][n] for n in names}
        q = os.path.join(dst, "waypoints", "course.yaml"); os.makedirs(os.path.dirname(q), exist_ok=True)
        with open(q, "w") as f:
            f.write(f"# claude: {note}。waypoint は {names[0]}〜{names[-1]} の {len(names)} 点\n")
            f.write("# RViz Nav2 パネル「Load WPs」用。orientation は [w, x, y, z] の順。\n")
            yaml.safe_dump(course, f, sort_keys=False, allow_unicode=True, default_flow_style=None)
        print(f"  course.yaml: {len(names)} 点 ({names[0]}〜{names[-1]})")


if __name__ == "__main__":
    main()
