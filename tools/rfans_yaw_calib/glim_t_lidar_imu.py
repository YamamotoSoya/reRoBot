#!/usr/bin/env python3
# claude: 2026-10-16 新設 — URDF の rfans_joint / imu_joint (xyz rpy) から GLIM の T_lidar_imu (TUM [x y z qx qy qz qw]) を計算する。
#   T_lidar_imu = T_base_lidar⁻¹ · T_base_imu。URDF の rpy は固定軸 (R = Rz(yaw) Ry(pitch) Rx(roll))。
#   rfans の yaw を変えるたびに手計算しないための補助。--json を与えると既存 config_sensors.*.json の値との差も出す。
#   使い方:
#     python3 glim_t_lidar_imu.py --rfans "0 0 0.79396 0 0 0.0983" --imu "0 0 0.73346 3.141592653589793 0 -1.625736" [--json ros2_ws_glim/config/config_sensors.flat.json]
#     --urdf <rerobot.urdf> を与えると rfans_joint / imu_joint の origin を URDF から読む
import argparse, json, re, sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def parse6(s):
    v = [float(x) for x in s.split()]
    assert len(v) == 6, "xyz rpy の 6 値"
    return np.array(v[:3]), Rot.from_euler("xyz", v[3:])   # 小文字 xyz = 固定軸 roll, pitch, yaw


def from_urdf(path, joint):
    txt = open(path).read()
    m = re.search(rf'<joint name="{joint}"[^>]*>.*?<origin xyz="([^"]+)" rpy="([^"]+)"', txt, re.S)
    if not m:
        sys.exit(f"{joint} が {path} に無い")
    return parse6(m.group(1) + " " + m.group(2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rfans"); ap.add_argument("--imu"); ap.add_argument("--urdf"); ap.add_argument("--json")
    a = ap.parse_args()
    if a.urdf:
        tl, Rl = from_urdf(a.urdf, "rfans_joint"); ti, Ri = from_urdf(a.urdf, "imu_joint")
    else:
        tl, Rl = parse6(a.rfans); ti, Ri = parse6(a.imu)
    R = Rl.inv() * Ri; t = Rl.inv().apply(ti - tl)
    q = R.as_quat()  # x y z w
    tum = [*t.round(6).tolist(), *q.round(7).tolist()]
    print("T_lidar_imu (TUM x y z qx qy qz qw):")
    print("  " + ", ".join(f"{v:.7g}" for v in tum))
    print("  JSON 形式:\n    \"T_lidar_imu\": [\n      " + ",\n      ".join(f"{v:.7g}" for v in tum) + "\n    ],")
    if a.json:
        txt = open(a.json).read()
        m = re.search(r'"T_lidar_imu"\s*:\s*\[([^\]]+)\]', txt)
        old = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", m.group(1))]
        Ro = Rot.from_quat(old[3:7]); dang = np.degrees((Ro.inv() * R).magnitude()); dt = np.linalg.norm(np.array(old[:3]) - t)
        print(f"  既存 {a.json}: 回転差 {dang:.3f}°, 並進差 {dt:.4f} m")


if __name__ == "__main__":
    main()
