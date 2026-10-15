<!-- claude: 2026-10-15 作成 (同日の自律走行 bag 6 本の解析。critic 査読 1 回) -->
# 10-15 自律走行: keepout で狭めた通路で停止 / AMCL 廊下跳び / IMU yaw 跳び / R-Fans ディップ

**ステータス: 原因特定 (1, 2, 3) + 対策未実施**。狭窄通路の停止は「keepout が膨張されず経路が縁を這う」が本命 (critic 維持、交絡 = AMCL の横分散)。
1802 の区間失敗は AMCL の廊下方向跳び (確定) で、`transform_tolerance: 8.0` が補正を 8 s 遅らせる副作用も判明。
IMU yaw 跳びは BNO086 内部融合の再収束 (本命、磁気起因かは未確認)。R-Fans の走行中ディップは 0 回。

対象 bag (`bags/raw/`、T は bag 先頭からの秒): 1643 (212 s, tol 10 の TF 失敗)、1658 (92 s)、1709 (838 s, 東行き + ベンチ通路)、1728 (55 s)、1752 (397 s, 狭窄通路で停止)、1802 (518 s, 狭窄の先から帰還)。
nav ログは volume `rerobot_main_log` の `nav_2026-10-15_{1655,1706,1725,1735,1751,1800}.log`。解析スクリプト・出力は scratchpad `ana/ana1015/` (pass1/21/22/23, keepout_view, cost_along_plan, critic/crit_ko, crit_gap)。
前段: `docs/issue/2026-10-14_...` は未作成 (10-14 の結論はメモリ `auto-run-1014-collision-stops` と本文 §1 の前提)。

## 0. 全体像

```
10-15 の走行 (10-14 の対策: failure_tolerance -1 / urg ±70° / footprint 四角 / amcl tol 8 / 帯分離 を入れて)
├─ 直った: waypoint 以外の区間失敗 … 1709 の東行き 629 s で Goal failed 0 回
│     ├─ urg 自己検出 … 停止時刻 9 点で footprint 内 lethal 0 (pass15 touch=0) → ±70° が効いた
│     └─ collision ahead は人・物が前方 0.46〜0.6 m にある間だけ (182〜230 s は 48 s 連続 = 何かが前に居た)
├─ 直っていない: 狭い通路 (§1)
│     ├─ 1752 T=327〜383 (66.2,-73.5) で静止 → ユーザ q。nav 1735/1751 でも同じ場所で collision ahead 連続
│     └─ 原因 = keepout filter は inflation の後に lethal を直書き (膨張なし) → 経路が keepout 縁から 0.35〜0.45 m
│           → footprint 半幅 0.325 + 1 セルで keepout に乗る → RPP 衝突 → failure_tolerance -1 で永久待機
├─ 新しい失敗: 1802 T=426.8 Goal failed (§2) … AMCL が廊下方向に 2.97 m 跳び → 8 s 後に "Resulting plan has 0 poses"
├─ ベンチ通路 (§3) … 1709 T=650〜829 入口で 180 s 旋回、直行 130 m ↔ 迂回 405 m が 3 s ごとに交代 (10-14 と同じ)
├─ IMU yaw (§4) … 単発 identity は無害、本物の跳び = BNO 再収束 (+34° / +9° / +8°)。EKF が追従
├─ R-Fans (§5) … 走行中ディップ 0。1802 末尾 511 s にスピンダウン署名 (静止中、電源断か要確認)
└─ 副産物 … 1709 T=402.4 ホスト時計 +0.65 s ステップ (以後 IMU stamp 0.6 s 遅れ、1728 まで持続)、車輪 odom yaw +3% (トレッド)
```

## 1. 狭窄通路 (wp7→wp8、x 70→28, y −74→−67) で永久停止

### 1.1 事実
- 1752: T=296 に wp8 へ出発 → T=327 に (66.2,−73.5) で速度 0 → 383 s まで `collision ahead` 20 Hz 連続 → 390 s ユーザが cancel (`cmd.txt`、`rosout.txt`)。
- nav 1735 (bag なし) 同区間で collision ahead ×4320 (= 216 s 連続)、nav 1751 (= bag 1752) ×1540。
- 1802 は (22.3,−67.4) = 狭窄の西側から開始したので通っていない (keepout マスクは 3 bag とも同一: occ 7794756)。

### 1.2 機構 (critic 維持、数値訂正)
```
global costmap の横断プロファイル (経路に直交、cost_along_plan.py、1752 T=291 / 330)
  keepout 側: 100 100 100 100 100 100 100 100 | 0 0 0 0 0 … ← 段差、勾配なし
  生垣側    : … 0 0 0 | 44 59 80 99 99 99 100             ← inflation の勾配あり
```
- Nav2 の costmap は plugins (static/obstacle/inflation) を適用した後に filters (keepout) を master に書く → keepout の lethal は膨張されない。
- NavFn はコスト 0 の帯で最短 (弦) を取り、生垣側だけ勾配で押されるので経路は keepout 寄り: 経路〜keepout lethal セル中心 0.4〜0.5 m、セル縁まで 0.35〜0.45 m (critic 計測。「0.3 m」は過小)。
- local costmap (0.05 m) では footprint 右縁 (半幅 0.325) の外 1 セル以内に keepout セル (by −0.33〜−0.39、`crit_ko.py`: 1.2 m 以内の lethal 544/544 が mask==100、掃引箱内 9/9)。RPP の衝突判定は footprint を掃引して 254 を探すので当たる。
- 停止後は `failure_tolerance: -1.0` で FollowPath が失敗せず、速度 0 のまま待つ (10-15 の意図どおりの挙動、ただしこの場合は永久)。
- costmap 上の通路幅 = keepout 縁〜生垣 lethal 縁で約 1.3 m (critic)。ロボット幅 0.65 → 両側合計 0.65 m の余裕があるのに縁を這って止まった。ko_zoom.png で keepout 縁〜生垣の黒セルは 1.5〜2 m に見えるが、生垣は散点なので costmap の 1.3 m を採る。

### 1.3 交絡 (棄却できず)
- AMCL の横分散: 停止直前 T=326.6 の syy=0.313 (σ_y≈0.56 m)。keepout は観測されない仮想壁なので、自己位置の y 誤差がそのまま keepout との距離誤差になる。膨張しても σ_y が 0.3 m を超える区間では間欠停止が残りうる。検証: 1752 を AMCL 再生 5 run で T=320〜327 の y 分散。

### 1.4 対策案 (未実施)
1. **keepout を StaticLayer として積む** (両 costmap)。膨張されるので経路が中央に寄り、RPP の footprint も当たらない。
   ```yaml
   plugins: ["keepout_static", "static_layer", "obstacle_layer", "inflation_layer"]   # global
   plugins: ["keepout_static", "obstacle_layer", "inflation_layer"]                   # local
   # filters: ["keepout_filter"] は外す (残すと非膨張 lethal を再度上書きして同じ現象)
   keepout_static:
     plugin: nav2_costmap_2d::StaticLayer
     map_topic: /keepout_filter_mask
     map_subscribe_transient_local: true
     use_maximum: true            # mask の 0 で本地図の lethal を消さない
     subscribe_to_updates: false
   ```
   パラメータ名は 1.3.13 の liblayers.so で確認済み (map_topic / use_maximum / map_subscribe_transient_local)。
   注意: 1 m 通路なら inscribed (0.325) 帯を両側引いて 0.35 m の通行帯が残る。0.6 m 通路は物理的に通れない。
2. 代替: mask 画像に 0.55 m 幅の勾配 (99→1) を描く (KeepoutFilter は 1..99 を比例コストにする)。Nav2 無変更だが 253 リングの意味は持たない。
3. mask の描き方: costmap 上 1.3 m と目視 1.5〜2 m の差があるなら keepout 帯が物理境界より内側に 0.2〜0.7 m 描かれている (mask 側で広げられる)。
4. 1 m 通路の収支は `docs/trial` の議論どおり: 片側 0.175 m を AMCL 横誤差・urg ヨー・セル 0.05 で食う。対策 1 の後に段ボール模擬試験 (1 / 0.9 / 0.8 m) で実測。

## 2. 1802 T=426.8 Goal failed = AMCL 廊下跳び + transform_tolerance 8 s の遅延

- 北側廊下 (y≈−2〜−3) を x 20→13 へ西進中、AMCL の sxx が 0.8→2.4 m に膨張 (廊下方向の退化)。T=418.94 に amcl_pose が (15.73,−2.84)→(18.45,−1.63) と 2.97 m 跳ぶ (6 bag で >1 m の跳びはこの 1 回)。EKF/odom は連続 (critic)。
- その後 2 周期 (421.6, 424.6) の plan start は跳ぶ前の軌道の延長 (15.3→14.2→13.1)。abort 426.76 = 跳び + 7.8 s。
- 機構: `amcl.yaml transform_tolerance: 8.0` は map→odom を 8 s 未来日付で出す。Nav2 は Time(0) で map→base_link を引くので chain の latest common time (= odom→base_link の今) で map→odom を補間 → **8 s 前の AMCL 補正**が使われる (tfmap.txt: stamp − 受信 = 7.78 s)。跳びが 8 s 遅れて Nav2 に届いた瞬間、RPP `transformGlobalPlan` で最寄り経路点が local costmap 半幅 2 m を超え "Resulting plan has 0 poses in it" → InvalidPath → 即 abort (failure_tolerance では吸収されない)。
- 遅延ゼロでも跳び 3 m なら同じ abort は起きる。遅延の害は別: 補正が常に 8 s 古い (旋回直後の yaw 補正が遅れる)、global_costmap の "earlier than all the data in the transform cache" (1643 で 3 s ごと)。
- 対策案: `transform_tolerance` を 2.0〜3.0 に下げる (scan_all の穴は最大 0.53 s なので足りる。R-Fans ディップ 6.7 s は守れないが 8.0 でも守れない)。廊下退化は `docs/issue/2026-10-01_amcl_hedge_corridor_scan_reduction.md` の続き (alpha 0.02 は適用済みか要確認)。
- 補助: waypoint_runner 側で「Goal failed 直後に同じ区間を自動再送 (1 回)」があれば 1802 のような単発は人手なしで復帰する。

## 3. ベンチ通路 (1709 T=650〜829)

- 直行 130 m ↔ 迂回 405 m が 3 s ごとに交代、NavFn "Failed to create a plan from potential when a legal potential was found" ×2 (709.6, 753.0)、入口で 180 s 旋回 → ユーザが飛ばした。AMCL は健全 (sxx ≤0.1)。
- 直行経路の最小 clearance は静的地図+keepout に対し 0.30 m の区間が複数 (ko_bench.png: ベンチと keepout 棒の間のスラローム) → footprint 半幅 0.325 より狭く **RPP では最初から通れない**。交代は、絞り部に obstacle layer (rfans 5 m) の 254 が出入りするため (critic: AMCL のずれは static/keepout を動かさず、観測ベンチの位置をずらす)。
- 判断はユーザ: 通す意図なら keepout 棒とベンチ周りを ≥0.65 + 2×マージンに直す (対策 1 の膨張化で経路は中央に寄る)、通さない意図なら絞りを 254 で閉じて 405 m 迂回で計画を安定させる。
- 未決: NavFn が 253 を通すか。navfn.cpp の setCostmap (v < 253 のみ通行可、10-14 に確認) では障害物扱いだが、直行経路に 253 セルが 33〜45 個乗っていた (plan 時刻と costmap 時刻のずれで説明できる範囲)。critic は「253 は cost 252 で通行可」と主張、未決着。
- 「plan 長が直線距離の 3 倍超なら進まない」保険 (waypoint_runner) は 1752 T=178〜205 でも効いた場面 (84,−55.6 で 7.7 m ↔ 381 m の交代、旋回 30 s)。

## 4. IMU yaw 跳び (BNO086)

```
/imu/data の異常
├─ (a) 単発 identity quaternion (q=(0,0,0,1), orientation_covariance[0]=−1) … 1709 ×6, 1752 ×9, 1802 ×4
│     └─ imu_node.py が quat 無効時に出す REP-145 形式。robot_localization は cov −1 の orientation を無視 → EKF 跳び 0 件。無害
└─ (b) 本物の跳び = BNO 内部融合の再収束 (本命)
      ├─ 1643 T=153.18: 20 ms (4 サンプル) で +34°、同時刻 gz 0.05 rad/s、cov 2.6→0.9 に単調低下 (精度推定が改善 = 再収束の署名)
      │     2 s 窓: ジャイロ −26.4 / 車輪 odom −25.4 / BNO +7.6 / EKF +6.4 → EKF が BNO に追従して +33° ずれた
      │     3.6 s 後に AMCL yaw が 62→73→58→43→34 と揺れ、156.8 に "Unable to transform" で区間失敗 (tol 10 の bag、帰結かは未解析)
      ├─ 1709 T=274.9: +9° (15 ms)      1802 T=6: −4°、T=510: +8° (静止中、ジャイロ 0)      1752 T=78: +2.7° (静止)
      └─ 磁気起因かは未確認 (/imu/mag を記録していない)。use_game_rotation_vector: false = 磁気補正あり
```
- EKF (`ekf.yaml`) は imu0 の yaw (絶対、relative true) と yaw rate を融合、odom0 は vx/vy/vyaw のみ。orientation_covariance 1〜2.6 rad² でも 200 Hz で 2 s あれば EKF yaw は BNO に収束するので、BNO の跳びはそのまま odom→base_link の yaw 跳びになる。
- 対策案 (どちらか): `bno086.yaml use_game_rotation_vector: true` (磁気なし。跳びが消えれば RV 由来と確定)、または `ekf.yaml imu0_config` の yaw を false にして yaw rate だけ使う (ジャイロ積分、車輪 vyaw の 3% 過大が効き始める点に注意)。次回から `/imu/mag` も記録。

## 5. R-Fans 回転ディップ

- 走行中: 6 bag でフレーム欠落・肥大なし (1709 の 402.4 s の 0.65〜0.74 s は全 topic 同時 → §6 の時計ステップ)。
- 1802 末尾 T=510.9〜511.9: 点数 51k→65k→90k→155k、間隔 0.14→0.30 s と肥大したあと points 停止、urg/imu/odom は 516.9 s (bag 終了) まで継続。= モータ減速中もパケットが復号されていた (スピンダウン署名)。ロボットは 490 s から静止。同時刻に BNO 方位が静止中に +7.9°。
- 「停止中ディップ (09-27 の静止中 2 回と同型)」寄りだが、「走行終了後に LiDAR 電源を切った」でも同じ署名になりうる → **18:11:06 (bag 終了 6 s 前) に何をしたかユーザに確認**。driver の poll timeout WARN が /rosout に乗るかも未確認 (6 bag とも rfans_driver のログ行が 0)。
- 今日の手動 bag には `/rfans_driver/rfans_packets` が無い (ws を source せずに record した。メモリ `bag-record-needs-ws-sourced`)。1643 (auto_run) だけ入っている。

## 6. 副産物

- **ホスト時計 +0.65 s ステップ** (1709 T=402.4): IMU の device stamp は連続、全 topic が 9.35 s 分しか無く遅延バーストもない (`crit_gap.py`)。bno086 driver の device_clock min フィルタ (MAX_DRIFT_PPM 100) は前へ飛んだ時計に追従できず、以後 /imu/data の stamp が 0.62→0.58 s 遅れたまま (1728 bag 全体 0.55 s、1752 以降は再起動で 0)。1709 後半の EKF 1 サンプル 3° 跳び 17 件と整合。確認: ロボット PC の journalctl で Clock change。対策: 走行中 `timedatectl set-ntp false` (メモリ `ntp-clock-step-during-bag`)、driver に正方向オフセットの再ラッチ。
- **車輪 odom yaw が +2.5〜3.4%** (ジャイロ積分比、3 bag)。実効トレッド > 0.529 (タイヤ幅 12 cm で接地中心 0.8 cm 外寄り) が本命、スリップ/ジャイロスケールと未分離 (+110° 窓 +2%、−110° 窓 −4% の非対称)。EKF yaw は BNO 支配なので現状の影響は小。検証: 床マークでその場 5 回転 × 2 速度。
- **collision ahead の中身 (1709)**: 停止 9 時刻で footprint 内 lethal 0、前方 0.46〜0.60 m に lethal (urg が直接見る、az +14〜+31°) が 5、横 0.5 m が 3。182〜230 s の 48 s 連続停止は正面左 0.46 m に何かが居続けた (人?)。failure_tolerance −1 なので前に人が立ち続けると永久停止 — 本番は随伴者の立ち位置ルールで回避。

## 7. 次の作業 (優先順)

1. keepout を StaticLayer 化 (§1.4-1) + `transform_tolerance` 8.0 → 2.0 (§2) → 1752 の区間だけ再走、停止の有無と AMCL syy を見る。失敗なら σ_y が主犯 → mask を後退。
2. bno086 `use_game_rotation_vector: true` で 1 走行、/imu/mag も記録 (§4)。
3. ベンチ通路の意図をユーザが決める (§3)。
4. `bags/` の既存データで決着できる検証: ベンチ絞り部の 254 セル数 vs plan 長 (トグル機構)、journalctl の Clock change (§6)。
5. waypoint_runner: Goal failed の自動再送 1 回、plan 長 3 倍保険。
