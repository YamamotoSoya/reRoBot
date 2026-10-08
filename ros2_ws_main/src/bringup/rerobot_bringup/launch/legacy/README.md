<!-- claude: 2026-10-08 作成 (ユーザ依頼「使わない launch を 1 か所にまとめて」) -->
# launch/legacy/ — 役目を終えた launch

新しく使う理由はない。古い手順書・docs/issue・scripts から名前で呼ばれているので消さずに置いている。
`ros2 launch` はファイル名だけで探すので、ここに移しても `ros2 launch rerobot_bringup <名前>` はそのまま動く。

| launch | 代わりに使うもの | 備考 |
|---|---|---|
| `rerobot_bringup_2d` / `_3d` | `rerobot_bringup.launch.py` (全部入り) | IMU・EKF なし。`ekf:=false` を明示して旧挙動を維持 |
| `rerobot_bringup_2d_imu` / `_3d_imu` / `_2d3d_imu` | `rerobot_bringup.launch.py` | EKF は引数 `ekf` (既定 false) |
| `nav2` | `nav_amcl` / `nav_amcl_scanall` (`launch/nav/`) | 中身は部品 (`launch/parts/`) の組み合わせ。入力の既定は `/urg_front/scan`、`amcl_scan:=all/far` は `/rfans/scan_all` / `_far`。`scripts/nav2d.sh` がまだ使っている |

使い分けの全体は `docs/manual/15_launch_guide.md`。
