<!-- claude: docs/report テンプレート。debug-report スキルから生成。-->
# rfans_driver の点群更新頻度が遅い(RViz で約 1〜2 Hz)

- 日付: 2026-06-13
- 環境: reRoBot コンテナ `rerobot_env` / ROS 2 Jazzy / `rfans_driver`(submodule `src/external/StarROS2`)。R-Fans-16 実機(UDP, rps=10)。描画 RViz2(`/sdk_could`)。
- 対象ブランチ: `main`(HEAD: `9a158c0 ...`)。submodule `StarROS2` は `master`(HEAD: `a0320ba modify for ros2 (issue : z=0 and low fps)`)
- 関連ファイル:
  - `src/external/StarROS2/src/rfans_driver.cpp`(`calout_fansxyz` / `syn_date` / `Asyn_date`)
- 関連レポート: `docs/report/2026-06-13_rfans_driver_dataid_0x37_z0_2d.md`(同コミットの "z=0" 側。本件は "low fps" 側)

## TL;DR

点群充填関数 `calout_fansxyz()` の **1点ごとのループ内で `rclcpp::spin_some()` を呼んでいた**のが原因。1 スキャン約 15000 点 → 1 フレームあたり約 15000 回 spin_some が走り、エグゼキュータの wait set 走査がパイプラインを律速して publish が約 1〜2 Hz に低下していた。対処は点ごとの spin を撤去し、**1 スキャン 1 回**の spin に集約(`syn_date` は既存のループ末尾 spin を流用、`Asyn_date` には新規追加)。実測 `ros2 topic hz /sdk_could` の average rate が **約 1〜2 → 約 20** に改善。

## 症状

- **いつ**: R-Fans-16 実機を `ros2 launch StarROS2 rfans_driver.launch.py` で起動中、常時。
- **どこで**: `/sdk_could`(PointCloud2)の publish 頻度。RViz の点群更新。
- **何が起きるか**: RViz の点群が 1 秒間に 2 回程度しか更新されない。
- **正常時**: rps=10(10 Hz 回転)なら 1 回転 ≈ 15000 点 ≈ 0.1 s。publish は概ね回転数なりの ~10 Hz 以上で出るべき。

## 切り分けの記録

### 1. 実機モードで実際に回る関数の特定(観測)

コンストラクタで `simu_filepath==""`(実機)→ `flag_simu=1`。`spinOnce()` の分岐で `flag_simu` 真 → **`syn_date()`** が実機 UDP の処理ループ(`Asyn_date()` は再生ファイル用)。`syn_date` は `getPackets()` 毎に 1 回 publish しており、再生用の `packet_rate.sleep()`(`Asyn_date` 側)のような明示スロットルは無い。

| 項目 | 観測値 | 備考 |
|------|--------|------|
| 実機の処理関数 | `syn_date()` | `flag_simu=1` |
| publish 箇所 | `getPackets()` 毎に 1 回 | ループ末尾 |
| 明示 sleep | 無し(`repeat_delay`=0 既定) | スロットルが原因ではない |
| 1 回の点数 | ~15008 点/回 | ≈ 1 回転ぶん。本来 ~10 Hz 出せる量 |

→ 「sleep でわざと落としている」仕様ではない。処理が回転に追いつかず、実効頻度が落ちていると推測。

### 2. ホットループ内の spin_some(最終的に当たった原因)

`calout_fansxyz()`(全点について呼ばれる)の点ループ内に `rclcpp::spin_some(get_node_base_interface())` があった(`// claude: was ros::spinOnce()`)。1 スキャン ~15000 点 × 各エコーぶん呼ばれ、毎回エグゼキュータの wait set を走査するため CPU を浪費し、処理が回転速度に追いつけず UDP 受信が滞留 → 実効 publish ~1〜2 Hz。元 ROS1 も `ros::spinOnce()` を同じ位置で呼んでおり、rclcpp 移植でコストが顕在化した。

## 根本原因

`rclcpp::spin_some()` を**点群の各点で呼んでいた**こと。spin_some は 1 回ごとにエグゼキュータの wait set を走査するため本来「イベントループ 1 階層で 1 回」呼ぶもの。これを ~15000 回/フレーム実行すると、点群 1 枚を組み立てるたびに膨大な固定コストが乗り、`getPackets()→処理→publish` のサイクル時間が回転周期(0.1 s)を大きく超え、結果として publish 頻度が回転速度の下に張り付く。

**なぜ今まで顕在化しなかったか**: 元 ROS1 実装(`640892e`)から `ros::spinOnce()` を同じ位置で呼んでおり、ROS1 の spinOnce は比較的軽量だったため見過ごされていた。ROS2 移植(`a0320ba`)で機械的に `rclcpp::spin_some()` へ置換した結果コストが顕在化し、同コミットメッセージに `low fps` として残っていた。

## 修正

submodule `src/external/StarROS2/src/rfans_driver.cpp`。**`colcon build --packages-select rfans_driver` での再ビルドが必要**。

`calout_fansxyz()` の点ループから spin_some を撤去:

```cpp
    if (INPUT.lasShot.m_pulse[idx].flag) {
      // claude: ここで点ごとに spin_some していたのが low-fps の主因(1スキャン
      //         ~15000 点 = ~15000 回/フレーム)。1スキャン1回の spin に集約した。
      if(INPUT.lasShot.m_pulse[idx].m_fRange > max_range_ || ... ) { continue; }
```

実機ループ `syn_date()` は元々ループ末尾に 1 回/スキャンの spin があるためそれを利用。再生ループ `Asyn_date()` にはループ内 spin が無かったので publish 直後に新規追加:

```cpp
    sdk_output_->publish(outCloud_sdk_);
    out_count_=0;
    rclcpp::spin_some(get_node_base_interface());   // claude: 1スキャン1回の spin(点ごと spin を廃止した代替)
    if(reader.eof())
```

### なぜこの値 / この方法か

- **なぜ「1 スキャン 1 回」か**: spin_some の目的は `rfans_control` サービスとパラメータ更新コールバックの処理。これらは ~10 Hz の頻度で十分応答でき、点ごとに呼ぶ必要は皆無。
- **なぜ完全削除でなく Asyn_date に追加したか**: `Asyn_date`(再生モード)は他に spin 箇所が無く、点ごと spin を消すだけだとサービス/パラメータが一切処理されなくなるため、ループ 1 周 1 回を補った。
- **破綻条件**: もし将来サービス応答にミリ秒単位の即応性が要るなら、処理を別スレッド/コールバックグループに分離する設計に見直す(現状は単一ループで十分)。

## 検証

| 項目 | 修正前 | 修正後 |
|------|--------|--------|
| `/sdk_could` average rate(`ros2 topic hz`) | 約 1〜2 Hz | 約 20 Hz |
| RViz 体感更新 | カクつく(2回/秒程度) | 滑らか(大幅向上) |

検証手順(Jazzy コンテナ `rerobot_env` 内):

```bash
colcon build --symlink-install --executor sequential --packages-select rfans_driver
source install/setup.bash
ros2 launch StarROS2 rfans_driver.launch.py
# 別ターミナル
ros2 topic hz /sdk_could    # average rate を確認(修正後 ~20)
```

注: average rate ~20 は実測値(ユーザ確認)。rps=10 設定に対し 20 が出るのは、1 回転を複数の `getPackets()` バッチに分けて publish しているため(1 publish = 1 全回転とは限らない)。絶対値より「~1〜2 → ~20 への一桁以上の改善」が要点。

## 教訓 / 今後の予防

1. **ROS1→ROS2 移植で `ros::spinOnce()` を `rclcpp::spin_some()` に機械置換するときは、呼び出し位置がホットループ内でないか必ず確認する**。spin は「イベントループ 1 階層で 1 回」が原則。点/メッセージ単位で呼ぶと rclcpp では重い。
2. **更新頻度が遅い系の調査は publish 頻度を握る箇所(ループ構造・sleep・spin・QoS)を順に見る**。本件は spin の置き場所だった。
3. フォローアップ TODO(本件では未対応、さらに上げたい場合の候補):
   - `calout_fansxyz` の `temple_1_.resize(out_count_+1)` 毎点呼び出しを `reserve`+`push_back` 化。
   - Publisher QoS が `depth=10000` reliable(`rfans_driver.cpp` のパブリッシャ生成箇所)。大点群の滞留要因になり得るので depth を下げる / Best Effort 化を検討。
   - コミット `a0320ba` の "low fps" はこれで解消。"z=0" 側は別レポート参照。
