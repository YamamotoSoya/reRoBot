<!-- claude: 2026-10-01 新設 -->
# amcl_compare — AMCL 並走比較ツール

3D LiDAR → LaserScan 変換の「方位ビン内の選び方」や AMCL パラメータの違いを、**同じ bag 入力で
AMCL を名前空間付きで並走**させて比べる。本番 AMCL (TF あり) には触らない。

```
bag 再生 (/clock) ─┬─ /scan (記録済み) ──────────────────────→ /loc_base/amcl  ┐
                   ├─ rfans_scan_modes mode=farthest → /scan_far → /loc_far/amcl  │ tf_broadcast false
                   ├─ rfans_scan_modes mode=nth      → /scan_nth1 → /loc_nth1/amcl│ /initialpose 共通
                   └─ pointcloud_to_laserscan 細ビン  → /scan_all → /loc_all/amcl ┘
/initialpose (bag) → initialpose_relay.py が 4 回再送 → 全インスタンスが同時に同じ姿勢で初期化
```

| ファイル | 役割 |
|---|---|
| `replay.sh` | 再生ハーネス (main コンテナ内)。bag 再生 + `amcl_variants.launch.py` (map_server / robot_state_publisher 込み) + 記録 |
| `initialpose_relay.py` | bag の `/initialpose` を再送 (1 回だけだと発見の遅いインスタンスが取りこぼす) |
| `compare.py` | 評価。base との差・飛び・スキャン整合スコア + 反復 run の集計 (`<group>_rN`) → `.md` と `.png` |
| `compare_scans.py` | 変換ノードの実装検証 (`/scan_near` と記録済み `/scan` のビーム一致率) |
| `variants_repeat.yaml` | 各変種 4 コピーの変種定義 (反復 run 用) |

変種の定義は `rerobot_bringup/config/amcl_variants.yaml` (既定) か `VARIANTS=` で渡す YAML。
変換ノードは `ros2_ws_main/src/app/rfans_scan_modes` (mode = nearest / farthest / nth)。
「全点」は自作不要で、pointcloud_to_laserscan の `angle_increment` を R-Fans の方位刻み 0.00016 rad にする。

## 使い方

```bash
# 単発 (既定の変種 6 系統)
docker exec rerobot_env bash /workspace/tools/amcl_compare/replay.sh \
  /workspace/bags/raw/2026-09-27_1948_5goukan_auto 190 810 /workspace/bags/exp/amcl_variants/1948_A
# 反復 (各変種 4 コピー)
docker exec -e VARIANTS=/workspace/tools/amcl_compare/variants_repeat.yaml rerobot_env bash \
  /workspace/tools/amcl_compare/replay.sh /workspace/bags/raw/2026-09-27_1948_5goukan_auto 190 120 /workspace/bags/exp/amcl_variants/1948_C2
# 評価
docker exec rerobot_env bash -c 'source /opt/ros/jazzy/setup.bash; python3 /workspace/tools/amcl_compare/compare.py \
  /workspace/bags/raw/2026-09-27_1948_5goukan_auto /workspace/bags/exp/amcl_variants/1948_C2 \
  /workspace/bags/exp/amcl_variants/1948_C2_eval --segments "見失い1:200-300"'
# 実走で並走させる (nav2d.sh の後)
ros2 launch rerobot_bringup amcl_variants.launch.py
```

## ⚠ 評価上の注意

- **AMCL は乱数を使う**。同じ入力でも run ごとに見失う/見失わないが分かれる (1948 植え込み通路で base 5/16)。
  **単発の再生 1 回では変種の効果は判定できない** — `variants_repeat.yaml` で反復し、失敗頻度で比べる。
- 真値が無いので、ロボット静止中に operator が与えた `/initialpose` を参照点に使える (例: 1948 の 291 s)。
- 再生の自作スキャンは**現在の URDF** で base_link 変換する。記録済み `/scan` は収録時の URDF。
  10-01 に rfans z が 8.5 mm 変わったため nearest vs /scan の一致は 98.2% (残差は帯の縁)。
- `ros2 bag play/record` は TTY なしだと SIGINT を無視する → スクリプトは SIGTERM で止める。
- URDF を `-p robot_description:="$(cat …)"` で渡すとコメント中の記号で rcl が Abort する → launch 内で読む。
- RewrittenYaml の `param_rewrites` は**既存キーしか置換しない** (`map_topic`/`tf_broadcast` が効かなかった) → 型付き dict で上書き。
