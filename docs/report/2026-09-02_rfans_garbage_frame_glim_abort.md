<!-- claude: docs/report テンプレート。debug-report スキルから生成。-->
# surestar_rfans_ros2 のゴミフレーム publish による GLIM オフライン SLAM の abort

- 日付: 2026-09-02
- 環境: glim_env コンテナ (koide3/glim_ros2:jazzy ベース, GTSAM 4.3a0) / rerobot_env (ROS 2 Jazzy) / R-Fans V6K-16G
- 対象ブランチ: `main` (HEAD: `1056b0c renew params for new body`)
- 関連ファイル:
  - `ros2_ws_main/src/drivers/surestar_rfans_ros2/src/bufferDecode.cpp` (修正箇所: `publishCloud`)
  - `bags/5goukan/2d3d_imu/online/rosbag/2026-08-31_0044` (問題の bag)
  - `bags/5goukan/2d3d_imu/online/rosbag/2026-08-31_0044_filtered` (救済フィルタ済み bag)

## TL;DR

LiDAR (R-Fans V6K-16G) が録画中に 4 回、約 3 秒間データ異常 (全点 range=0) を起こし、その間ドライバのフレーム分割条件 (角度積算 ≥ 359°) が発火せず、内部バッファ上限 262143 点 (= `LINE_POINT_COUNT` 256×1024 − 1) まで **全点 xyz=(0,0,0) のゴミフレーム**が溜まって publish されていた。GLIM の前処理 (距離フィルタ 0.5–100 m) を通すと 0 点になり、`gtsam_points::IntegratedGICPFactor` のコンストラクタが「source frame doesn't have required attributes for gicp」を出して **`abort()` 直呼び** → `[ros2run]: Aborted`。対処は (1) ドライバ `publishCloud` に使用可能点 <100 のフレームを捨てるガードを追加、(2) 既存 bag は壊れ 12 メッセージを除去したフィルタ済みコピーで救済。

## 症状

- **いつ**: `bags/5goukan/2d3d_imu/online/rosbag/2026-08-31_0044` (2026-08-31 車体改修後の flat 取付・22 分走行) に対して glim_rosbag でオフライン SLAM を実行中、bag 開始から約 558 秒地点。
- **どこで**: glim_env コンテナ内の `ros2 run glim_ros glim_rosbag`。
- **何が起きるか**: 下記ログを出してプロセスが Aborted で停止。
- **正常時**: bag を最後まで読み切り dump を保存して自動終了する (`auto_quit:=true`)。

```
[odom] [warning] insert_frame points=0
error: source frame doesn't have required attributes for gicp
[ros2run]: Aborted
```

ユーザ環境では直前に `"imu_link" passed to lookupTransform argument target_frame does not exist` も出るが、これは **無関係の警告** (rviz_viewer 拡張が TF 公開のため `imu_link`→`glim_base` を lookup するが、オフラインでは robot_state_publisher が居ないので毎フレーム失敗する。catch 済みで致命傷ではない)。

## 切り分けの記録

### 1. エラーメッセージの出所特定 — `imu_link` 警告は無関係、`abort()` は gtsam_points 内

`imu_link` の TF lookup 失敗が死因という仮説をまず検証。glim_ros2 ソース (`rviz_viewer.cpp:245`) を確認すると `tf2::TransformException` を catch して warn を出すだけで続行する — **棄却**。一方 `error: source frame doesn't have required attributes for gicp` は `gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp` の `IntegratedGICPFactor` コンストラクタにあり、source 点群が points/covs を持たないと **`std::abort()` を直呼び**する。`[ros2run]: Aborted` と整合。

### 2. bag の健全性チェック — トピック欠落なし、生フレームの点数も正常

| 項目 | 観測値 | 備考 |
|------|--------|------|
| bag 長 | 1335.3 s | 22 分 |
| /rfans_driver/rfans_points | 26165 msgs = 19.6 Hz | 欠落なし |
| /imu/data | 133486 msgs = 100.0 Hz | 欠落なし |
| 生点数 <1000 のフレーム | 0 件 | 「空フレームが録れていない」仮説は棄却 |

### 3. 距離フィルタ後の有効点数を全フレーム走査 — 「有効 0 点」の壊れフレームを発見 (的中)

GLIM の前処理条件 (0.5 m < r < 100 m, 有限値) を bag の全 26165 フレームに適用して走査した結果、**有効点 0 のフレームが 12 件**、4 クラスタ (各 3 連続フレーム) で存在。うち 1 件の stamp **1788137610.806 がユーザのクラッシュログの stamp と完全一致**。

| クラスタ | bag 時刻 | width | 特徴 |
|------|--------|------|------|
| 1 | +5.3〜6.7 s | 262143 / 262143 / 135456 | 全点 xyz=(0,0,0) |
| 2 | +558.4〜559.7 s | 262143 / 253440 / 141888 | ← ここでクラッシュ |
| 3 | +1058.6〜1060.2 s | 262143 / 262143 / 141120 | 〃 |
| 4 | +1282.3〜1283.6 s | 262143 / 255456 / 136544 | 〃 |

壊れフレームの中身: 全点 xyz=(0,0,0)・intensity 一律 28・per-point time が最大 0.87 s 分 (正常は 1 回転 0.05 s)・**直前の publish 間隔が 2.8〜3.2 s** (正常 0.05 s)。ヘッドレス再現ラン (viewer 拡張なし) でも同一 stamp・同一エラーで abort を再現。クラスタ 1 (+5.3 s) で落ちないのは GLIM の初期 IMU 状態推定が完了する前でフレームが odometry に入らないため。

## 根本原因

因果連鎖は 3 段:

1. **デバイス側**: R-Fans が約 3 秒間の測距異常 (全点 range=0、intensity=28 固定) を 22 分中 4 回起こした。原因は未特定 (デバイス内部 or UDP 経路)。
2. **ドライバ側 (直接原因)**: `surestar_rfans_ros2` の V6K-16G 経路 (`processPacketUser` → `checkOneRound`) はフレーム分割を「方位角の増分積算 ≥ 359°」で判定する。異常中は方位角が正常に進まず分割が発火せず、点が `s_lineData` (上限 `LINE_POINT_COUNT` = 256×1024) に溜まり続け、`s_lineCount` は 262143 で飽和 (`bufferDecode.cpp` の `if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1`)。復帰後に width=262143 の全ゼロ巨大フレームとして publish される。壊れフレームの width 262143 = この飽和値そのもの。また range=0 は `min_range` (既定 0) を下回らないため NaN マスクも効かず xyz=(0,0,0) のまま通過する。
3. **GLIM 側**: 前処理の距離フィルタで 0 点になった frame がそのまま odometry に入り (`[odom] insert_frame points=0`)、`odometry_estimation_cpu.cpp:96` の `IntegratedGICPFactor` 生成で `abort()`。sub_mapping には「500 点未満なら単位共分散で逃げる」ガードがあるが、odometry 側にはガードが無い。

なぜ今まで顕在化しなかったか: 08-12〜08-16 の 9 号館 bag ではデバイス異常が起きていなかった (このデータ異常は確率的事象)。録画自体は正常なので、オンライン走行中も同じゴミフレームが Nav2/GLIM に流れ得る状態だった。

## 修正

### (1) ドライバ恒久修正 — `publishCloud` にゴミフレームガード (要ビルド)

`ros2_ws_main/src/drivers/surestar_rfans_ros2/src/bufferDecode.cpp` の `publishCloud` (全 publish 経路が通る単一チョークポイント) 冒頭に追加。main コンテナで `colcon build --packages-select surestar_rfans_ros2 --executor sequential` が必要 (ビルド済み・8 s)。

```cpp
// claude: garbage-frame guard (2026-09-02). ...
static const size_t MIN_USABLE_POINTS = 100;
size_t usable = 0;
for (size_t i = 0; i < n && usable < MIN_USABLE_POINTS; ++i) {
    float xyz[3];
    memcpy(xyz, &cloud.data[i*cloud.point_step], 12);
    if (std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]) &&
        (xyz[0] != 0.0f || xyz[1] != 0.0f || xyz[2] != 0.0f)) {
        ++usable;
    }
}
if (usable < MIN_USABLE_POINTS) {
    RCLCPP_WARN(rclcpp::get_logger("rfans_driver"),
                "dropping garbage frame: width=%u but only %zu usable points "
                "(device data stall?)", cloud.width, usable);
    return;
}
```

### (2) 既存 bag の救済 — 壊れフレーム除去コピー

glim コンテナ内で rosbag2_py により「有限かつ非ゼロ xyz の点が 100 未満」の点群メッセージ 12 件を除去したコピーを作成: `bags/5goukan/2d3d_imu/online/rosbag/2026-08-31_0044_filtered` (元 bag は無変更)。

### なぜこの値 / この方法か

- **publishCloud で捨てる**: `Depacket` 内の 6 箇所の publish 呼び出しがすべてここを通るため 1 箇所で全経路を守れる。checkOneRound (分割ロジック) 側の修正は vendor コードの角度状態機械に手を入れることになり回帰リスクが大きい。
- **閾値 100 点**: 正常フレームは ~13,000 点、壊れフレームの有効点は 0。2 桁 vs 4 桁の間で余裕を持って分離でき、誤爆しない。早期 break 付き線形走査なので正常時のコストは先頭 ~100 点分の memcpy のみ。
- **GLIM 側を直さない理由**: GLIM は公式ビルド済みイメージ (`koide3/glim_ros2:jazzy`) で運用しており、fork 維持のコストが大きい。上流 (ドライバ) で止めるのが層として正しい。
- **破綻条件**: `min_angle`/`max_angle` を狭めて視野を絞る運用 (有効点が構造的に 100 未満になる) をする場合は閾値の見直しが必要。

## 検証

| 項目 | 修正前 (元 bag) | 修正後 (フィルタ済み bag) |
|------|--------|--------|
| glim_rosbag の結果 | stamp 1788137610.806 (+558 s) で Aborted | **完走 (EXIT=0, 3.2 倍速・約 8 分)** |
| dump | 保存されない | `bags/5goukan/2d3d_imu/offline/glim/2026-08-31_0044_dump` (submap 123 個 + values.bin) |
| 点群メッセージ数 | 26165 | 26153 (壊れ 12 件除去) |
| ドライバのビルド | — | `colcon build --packages-select surestar_rfans_ros2` 成功 (警告なし) |

ドライバガードの実機動作 (異常再発時にドロップ warn が出ること) は次回走行での確認待ち。**未計測**。

地図品質は別問題として残る: GLIM 経路長 495.4 m vs 車輪 odom 331.8 m (+49%)、**z が −80.5〜+19.7 m にドリフト** — 08-16 に確定した縦角系統誤差 (flat で −1.3% スケール) 系列の既知課題であり、本件 (クラッシュ) とは独立。縦角再較正が前提。

検証手順:

```bash
# 壊れフレーム走査 (glim コンテナ内, rosbag2_py): 有効点 = 有限・非ゼロ xyz・0.5<r<100 m
# の点数を全フレームで数える → 12 件検出、stamp がクラッシュログと一致

# 再現 (元 bag → Aborted)
docker exec -it glim_env bash
ros2 run glim_ros glim_rosbag /workspace/bags/5goukan/2d3d_imu/online/rosbag/2026-08-31_0044 \
  --ros-args -p config_path:=/glim_config -p auto_quit:=true

# 修正後 (フィルタ済み bag → 完走)
ros2 run glim_ros glim_rosbag /workspace/bags/5goukan/2d3d_imu/online/rosbag/2026-08-31_0044_filtered \
  --ros-args -p config_path:=/glim_config -p auto_quit:=true \
  -p dump_path:=/workspace/bags/5goukan/2d3d_imu/offline/glim/2026-08-31_0044_dump
```

## 教訓 / 今後の予防

1. **glim_rosbag が Aborted で落ちたら、最後の警告ではなく `error:` 行を探す** — `imu_link` の TF 警告のような「毎フレーム出る無害な warn」がログの大半を占め、真の死因 (gtsam_points の `abort()`) を隠す。オフライン実行での `lookupTransform` 失敗は仕様 (robot_state_publisher 不在)。
2. **bag 取得直後に壊れフレーム検査を回す** — 「有効点数 (0.5–100 m, 有限, 非ゼロ) が異常に少ないフレーム」「width が 262143 (バッファ飽和値) のフレーム」「publish 間隔 > 0.5 s」の 3 点をチェックすれば今回の異常は録画当日に検出できた。長時間 bag を撮ったらまずこの走査。
3. **vendor ドライバは「異常入力時に何を publish するか」を疑う** — 正常系のフレーム分割ロジック (角度積算) はデータ異常で簡単に破綻し、飽和バッファをそのまま吐く実装だった。下流 (GLIM) は空フレームで即 abort する設計 (odometry 側にはガードなし) なので、境界のドライバで守るしかない。
4. **フォロー TODO**: (a) デバイス側一次原因の追跡は `docs/issue/2026-09-02_rfans_scan_motor_dropout.md` へ移管 (同日 0106 bag の再発を受けて深掘り済み — 正体は**スキャンモータの定型的な脱落→惰性減速→再加速 (~10 s)**。主基板は無停電・走行状態/場所/衝撃と無相関。内部保護 vs 電源経路の 2 択まで絞り込み、切り分け実験は issue 参照)。(b) ガードは「捨てて警告」なので、異常発生を bag から気づけるようドロップ warn の有無をログ確認する運用にする。
