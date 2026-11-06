<!-- claude: 2026-11-06 Smac State Lattice 用の動き集合 (control set)。nav2_params.yaml の planner_server.GridBased.lattice_filepath が参照する -->

# Smac Lattice の動き集合

| ファイル | 内容 |
|---|---|
| `diff_10cm_r0.5_h16.json` | **使用中** (11-07〜)。差動二輪用、格子 0.1 m、最小回転半径 0.5 m、16 方位。直進 16・弧 56・その場旋回 32 の計 104 動き |
| `diff_10cm_r0.3_h16.json` | 11-06 に使用。半径 0.3 m、16 方位 (96 動き)。新地図のジグザグで経路が出ないため差し替え |
| `generator_config_diff_10cm_*.json` | 生成に使った設定 |

## 前提

- **格子は global costmap の `resolution` と一致させる** (0.1 m)。Smac Lattice は動き集合の座標を `grid_resolution` で割ってセル単位に直すので、食い違うと経路が壊れる。
- global を 0.05 m にする場合は、Nav2 同梱の見本 `/opt/ros/jazzy/share/nav2_smac_planner/sample_primitives/5cm_resolution/` を使うか、0.05 m で生成し直す。

## 再生成の手順

生成ツールは Nav2 公式リポジトリの `nav2_smac_planner/lattice_primitives/` (タグ 1.3.13)。apt 版には含まれない。

1. ツールを取得する (`generate_motion_primitives.py`, `lattice_generator.py`, `trajectory_generator.py`, `trajectory.py`, `helper.py`, `constants.py`)。
2. 依存は numpy・matplotlib・rtree。rerobot_env コンテナには rtree が無いので、2026-11-06 は insert / intersection だけを総当たりで実装した最小の代替モジュールを `PYTHONPATH` に置いて実行した (`bags/tools/2026-11-06_lattice_gen/stub/`、git 管理外)。
3. コンテナ内で実行する。

```bash
cd <ツールのディレクトリ> && PYTHONPATH=<stub のディレクトリ> MPLBACKEND=Agg python3 generate_motion_primitives.py --config generator_config_diff_10cm_r0.3_h16.json --output diff_10cm_r0.3_h16.json --visualizations vis
```

4. config に新しいファイルを足したら rerobot_bringup を再ビルドする (install は symlink だが、新しいファイルは再ビルドしないと install 先に載らない)。

## 動き集合の比較 (2026-11-07、地図だけでコース全体を計画 + 11-04 bag の 696/700 s 再生)

| planner | 新地図 fixed2_filtered | 旧地図 | 11-04 696/700 s (旧地図) | その場旋回 |
|---|---|---|---|---|
| Smac Hybrid R0.5/32 (11-04 実走) | 9/9 | 11/12 | 2/2 | 0〜1 |
| Lattice R0.3/16 | 8/9 (3→4 ×) | 11/12 | 0/2 | 34〜36 |
| **Lattice R0.5/16 (採用)** | **9/9** | 11/12 (3→4 ×) | 0/2 | 30〜67 |
| Lattice R0.5/32 | 9/9 | 12/12 | 1/2 | 0〜1 |

新地図の全区間を通り、その場旋回が最も多い R0.5/16 を採用した。旧地図に戻す場合は R0.5/32 か Hybrid が安全。

## 当初の選定 (2026-11-06 bag 再生、⚠ Hybrid 側の比較設定に誤りあり)

0.1 m 格子で 3 種類 (R0.3/16 方位、R0.5/16、R0.5/32) を、11-01・11-03・11-04 の停止場面で Smac Hybrid と比べた。⚠ 11-01・11-03 の Hybrid は誤って R0.3/16 の設定で計算していた (11-07 に判明)。
成功・失敗は 3 種類とも Hybrid と同じで、R0.3/16 は経路にその場旋回が入る (2〜3 回)、計画時間 0.25〜0.96 s。
狭い場面 (幅 0.8〜1.2 m) では、その場旋回に要る直径 1.2 m (後角 0.6 m) が無いので、Lattice でも通れない。
旋回の利点が出るのは 1.2 m 以上ある角で、そこは未検証 (次の実走で確認)。
