#!/usr/bin/env python3
# claude: 2026-10-11 作成
# bag 撮影中に付けた停止点マーカー (/teach_marker、rerobot_bringup/scripts/teach_marker.py) を、
# GLIM dump の軌跡 (traj_lidar.txt) から map 座標の停止点に変換して waypoint YAML に書く。
#
# 仕組み:
#   - マーカーには「押した時刻 + ラベル」しか入っていない。座標はその時刻の GLIM 姿勢から引く。
#   - glim_dump_to_2dmap は GLIM の世界座標をそのまま 2D 地図の map 座標にしている
#     → 同じ dump の軌跡から取った点は、その dump から作った地図と同じ座標系に乗る。
#     地図を作り直したら (別 dump)、その dump を指定して再実行すれば点も追従する。
#   - traj_lidar.txt は T_world_lidar (TUM 形式: t x y z qx qy qz qw)。URDF の rfans_joint
#     (base_link → rfans) を逆に掛けて T_world_base にし、x, y と yaw (base の x 軸の水平方位) を取る。
#   - 押す前後は静止している約束なので、[t - before, t + after] の軌跡サンプルを平均する。
#     窓内の広がりが --still_tol を超えたら ▲ (動いていた疑い。ラベル確認を)。
#
# 出力 (waypoint_editor / waypoint_runner / RViz Nav2 パネルで読める形式):
#   waypoints:
#     waypoint0: {pose, orientation [w,x,y,z], yaw_manual: true, stop: true, taught: true,
#                 label: teach_00, teach_meta: {...}}
#   → waypoint_editor で開き、Shift + クリックで停止点の間に通過点を足す (taught 点は動かせない)。
#
# 依存: rosbag2_py, numpy, PyYAML (rerobot_env にある)。
# 使い方 (rerobot_env 内):
#   python3 /workspace/tools/teach_from_traj/teach_from_traj.py \
#       /workspace/bags/raw/<bag> /workspace/bags/glim/<bag>_dump/<タグ> \
#       /workspace/maps/2d/glim/<map>/waypoints/course.yaml \
#       --map_yaml /workspace/maps/2d/glim/<map>/nav2/map.yaml
#   ⚠️ dump は「地図を作ったのと同じ dump」を指定する (地図の map_params.yaml の dump_dir)。

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF の rpy = 固定軸 X→Y→Z


def urdf_T_base_child(urdf_path, child):
    """base_link → child の 4x4。joint を親方向にたどって合成する。"""
    root = ET.parse(urdf_path).getroot()
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    T = np.eye(4)
    link = child
    while link != "base_link":
        j = joints.get(link)
        if j is None:
            raise RuntimeError(f"URDF に {link} を子に持つ joint が無い ({urdf_path})")
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        Tj = np.eye(4)
        Tj[:3, :3], Tj[:3, 3] = rpy_to_R(*rpy), xyz
        T = Tj @ T
        link = j.find("parent").get("link")
    return T


def read_markers(bag, topic):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from visualization_msgs.msg import Marker

    storage = "mcap" if any(f.endswith(".mcap") for f in os.listdir(bag)) else "sqlite3"
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=storage), ConverterOptions("", ""))
    r.set_filter(StorageFilter(topics=[topic]))
    out = []
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Marker)
        out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.text))
    return sorted(out)


def main():
    ap = argparse.ArgumentParser(description="bag の /teach_marker + GLIM 軌跡 → 停止点 waypoint YAML")
    ap.add_argument("bag", help="マーカーを記録した bag ディレクトリ")
    ap.add_argument("dump", help="GLIM dump ディレクトリ (traj_lidar.txt を含む。地図を作ったのと同じもの)")
    ap.add_argument("out_yaml", help="出力 waypoint YAML (既存なら上書きしない、--force で上書き)")
    ap.add_argument("--map_yaml", help="対応する地図 yaml (出力のコメントに記録するだけ)")
    ap.add_argument("--urdf", default="/workspace/src/bringup/rerobot_bringup/urdf/rerobot.urdf",
                    help="base_link → LiDAR の取付を読む URDF (撮影時の取付と同じもの)")
    ap.add_argument("--lidar_link", default="rfans")
    ap.add_argument("--topic", default="/teach_marker")
    ap.add_argument("--before", type=float, default=0.5, help="押す前に平均に使う時間 [s]")
    ap.add_argument("--after", type=float, default=1.5, help="押した後に平均に使う時間 [s]")
    ap.add_argument("--still_tol", type=float, default=0.05, help="窓内の広がりがこれ超なら ▲ [m]")
    ap.add_argument("--force", action="store_true", help="out_yaml が既にあっても上書きする")
    args = ap.parse_args()

    if os.path.exists(args.out_yaml) and not args.force:
        sys.exit(f"✘ {args.out_yaml} が既にある (編集済みの通過点を消さないため停止。上書きは --force)")

    traj_path = os.path.join(args.dump, "traj_lidar.txt")
    tr = np.loadtxt(traj_path, comments="#", ndmin=2)  # t x y z qx qy qz qw
    T_base_lidar = urdf_T_base_child(args.urdf, args.lidar_link)
    T_lidar_base = np.linalg.inv(T_base_lidar)
    markers = read_markers(args.bag, args.topic)
    if not markers:
        sys.exit(f"✘ {args.bag} に {args.topic} が無い")
    print(f"bag: {args.bag}  マーカー {len(markers)} 個")
    print(f"traj: {traj_path}  {len(tr)} 行 ({tr[0, 0]:.1f} 〜 {tr[-1, 0]:.1f})")
    print(f"T_base_{args.lidar_link}: xyz={np.round(T_base_lidar[:3, 3], 4).tolist()}")

    wps, bad = [], 0
    print(f"{'#':>3} {'label':22s} {'x':>9} {'y':>9} {'yaw[deg]':>9} {'n':>3} {'広がり[m]':>9}  判定")
    for i, (t, label) in enumerate(markers):
        sel = (tr[:, 0] >= t - args.before) & (tr[:, 0] <= t + args.after)
        if not sel.any():  # 窓に 1 点も無い (軌跡の外 / 欠落) → 最寄りが 0.5 s 以内なら使う
            k = int(np.argmin(np.abs(tr[:, 0] - t)))
            if abs(tr[k, 0] - t) > 0.5:
                print(f"{i:>3} {label:22s} ✘ 軌跡に対応する時刻が無い (最寄り {tr[k, 0] - t:+.1f} s) — 除外")
                bad += 1
                continue
            sel = np.zeros(len(tr), bool)
            sel[k] = True
        xy, yaws = [], []
        for row in tr[sel]:
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = quat_to_R(*row[4:8]), row[1:4]
            Tb = T @ T_lidar_base
            xy.append(Tb[:2, 3])
            yaws.append(math.atan2(Tb[1, 0], Tb[0, 0]))
        xy = np.array(xy)
        c = xy.mean(axis=0)
        yaw = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))  # 円周平均
        spread = float(np.max(np.linalg.norm(xy - c, axis=1)))
        flag = "✔"
        if spread > args.still_tol or "(moving)" in label or "(no-odom)" in label:
            flag = "▲ 動いていた疑い"
            bad += 1
        print(f"{i:>3} {label:22s} {c[0]:>9.3f} {c[1]:>9.3f} {math.degrees(yaw):>9.1f} {len(xy):>3} "
              f"{spread:>9.3f}  {flag}")
        wps.append(dict(x=float(c[0]), y=float(c[1]), yaw=yaw, label=label.split(" ")[0],
                        meta=dict(stamp=round(t, 3), n=len(xy), spread_m=round(spread, 3),
                                  bag=os.path.basename(os.path.normpath(args.bag)),
                                  dump=os.path.normpath(args.dump))))

    out_dir = os.path.dirname(os.path.abspath(args.out_yaml))
    if not os.path.isdir(out_dir):  # 新規 dir はコンテナ (root) 所有になるので一つ上の所有者に合わせる
        os.makedirs(out_dir)
        try:
            st = os.stat(os.path.dirname(out_dir))
            os.chown(out_dir, st.st_uid, st.st_gid)
        except OSError:
            pass
    lines = [f"# claude: teach_from_traj.py が生成 — map: {args.map_yaml or '(未指定)'}",
             f"#   bag: {args.bag}",
             f"#   dump: {args.dump}",
             "# RViz Nav2 パネル「Load WPs」用。orientation は [w, x, y, z] の順。",
             "# taught: true の点は waypoint_editor で動かせない (位置を変えるなら bag マーカーを取り直す)。",
             "waypoints:"]
    for i, w in enumerate(wps):
        meta = yaml.safe_dump(w["meta"], default_flow_style=True, allow_unicode=True,
                              width=10**9).strip()  # 折り返すと flow の継続行インデントが崩れる
        lines += [f"  waypoint{i}:",
                  f"    pose: [{w['x']:.3f}, {w['y']:.3f}, 0.0]",
                  f"    orientation: [{math.cos(w['yaw'] / 2):.6f}, 0.0, 0.0, {math.sin(w['yaw'] / 2):.6f}]",
                  "    yaw_manual: true",
                  "    stop: true",
                  "    taught: true",
                  f"    label: {w['label']}",
                  f"    teach_meta: {meta}"]
    with open(args.out_yaml, "w") as f:
        f.write("\n".join(lines) + "\n")
    try:  # コンテナ (root) で書いても所有者をホスト側の親ディレクトリに合わせる (他 tools と同じ)
        st = os.stat(os.path.dirname(os.path.abspath(args.out_yaml)))
        os.chown(args.out_yaml, st.st_uid, st.st_gid)
    except OSError:
        pass
    print(f"書き出し: {args.out_yaml} ({len(wps)} 点、要確認 {bad})")


if __name__ == "__main__":
    main()
