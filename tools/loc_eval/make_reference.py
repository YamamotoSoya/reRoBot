#!/usr/bin/env python3
# claude: 2026-10-11 新設 — 自己位置推定の評価用「基準軌跡」を GLIM の traj_lidar.txt から作る。
#
#   traj_lidar.txt (TUM: stamp x y z qx qy qz qw、LiDAR 姿勢、地図と同じ GLIM 世界座標)
#     → base_link 姿勢へ (URDF の base_link→rfans は並進 (0,0,lidar_z)・回転なし)
#     → [任意] 別 bag の GLIM 座標から地図座標へ移す 4x4 変換 (register_traj.py の出力) を掛ける
#     → ref_base.tum (基準) + initialpose.yaml (先頭姿勢 = 全手法に同じ初期姿勢を与えるため)
#
# 使い方 (コンテナ内):
#   python3 make_reference.py <traj_lidar.txt> <out_dir> [--transform T_map_from_traj.txt] [--lidar_z 0.79396]
import argparse
import os

import numpy as np
import yaml


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R):
    # 安定版 (Shepperd)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])


def yaw_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("traj", help="GLIM dump の traj_lidar.txt")
    ap.add_argument("out_dir")
    ap.add_argument("--transform", default=None,
                    help="4x4 (行優先テキスト)。traj の座標 → 地図座標。register_traj.py の T_map_from_traj.txt")
    ap.add_argument("--lidar_z", type=float, default=0.79396,
                    help="URDF rfans_joint の z (base_link→rfans 並進)。rerobot.urdf と合わせる")
    args = ap.parse_args()

    traj = np.loadtxt(args.traj, ndmin=2)
    T = np.eye(4) if args.transform is None else np.loadtxt(args.transform).reshape(4, 4)
    t_base_lidar = np.array([0.0, 0.0, args.lidar_z])

    rows = []
    for st, x, y, z, qx, qy, qz, qw in traj[:, :8]:
        R_l = quat_to_rot((qx, qy, qz, qw))
        p_l = np.array([x, y, z])
        # T_map_base = T_map_lidar * inv(T_base_lidar)  (回転は共通、並進だけ戻す)
        p_b = p_l - R_l @ t_base_lidar
        R_b = R_l
        # 別 bag の座標 → 地図座標
        p_b = T[:3, :3] @ p_b + T[:3, 3]
        R_b = T[:3, :3] @ R_b
        q = rot_to_quat(R_b)
        rows.append((st, *p_b, *q))
    rows = np.array(rows)

    os.makedirs(args.out_dir, exist_ok=True)
    np.savetxt(os.path.join(args.out_dir, "ref_base.tum"), rows,
               fmt=["%.9f"] + ["%.6f"] * 3 + ["%.9f"] * 4)
    st, x, y, z = rows[0, :4]
    yaw = yaw_of(quat_to_rot(rows[0, 4:8]))
    init = {"stamp": float(st), "x": float(x), "y": float(y), "z": float(z), "yaw": yaw,
            "source": os.path.abspath(args.traj), "transform": args.transform and os.path.abspath(args.transform)}
    with open(os.path.join(args.out_dir, "initialpose.yaml"), "w") as f:
        yaml.safe_dump(init, f, sort_keys=False, allow_unicode=True)
    n = len(rows)
    print(f"ref_base.tum: {n} rows, {rows[-1, 0] - rows[0, 0]:.1f} s, "
          f"start ({x:.2f}, {y:.2f}, z {z:.2f}, yaw {np.degrees(yaw):.1f} deg), "
          f"end ({rows[-1, 1]:.2f}, {rows[-1, 2]:.2f})")


if __name__ == "__main__":
    main()
