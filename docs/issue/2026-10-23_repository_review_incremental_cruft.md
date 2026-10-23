<!-- claude: 2026-10-23 リポジトリ全体の見直し (ユーザ依頼「新しいエージェントが部分的に読んで継ぎ足してきた非効率を全ファイル見返して調べて」)。
     2026-07-07 の repository_audit.md の続編。§1 は本セッションの要約、§2 以降は調査 subagent 2 体の全文 (file:line 付き)。
     指摘の上位 10 件は本セッションで現物を再確認済み (§1.2 に印)。 -->

# リポジトリ全体の見直し (2026-10-23) — 継ぎ足し開発による重複・残骸・不整合

**ステータス: 一部実施 — docs 構成 (§1.4) と Stop hook (#4) は 2026-10-23 に実施済み。コード・設定側 (#1 #2 #3 #8 #9 #10 #11 #14) と規約浄化 (#6 #7)、issue ステータス行の正規化 (#12) は未着手**
(hook は監視パス不在に加えて**ホストに `jq` が無い**ため、旧版は一度も動いていなかった → jq 非依存に書き直し、動作確認済み)
**前回の全体監査**: [2026-07-07_repository_audit.md](2026-07-07_repository_audit.md) (25 件。うち Issue 1 は実装済み、16/17 は解消済みだがステータス未更新)

## 1. 要約

### 1.1 全体像

| 観点 | 件数 | 傾向 |
|---|---|---|
| コード・launch・設定・スクリプト・Docker | 重複 12 / 残骸 22 / 不整合 20 / 非効率 9 / 命名 10 | 「古い構成の既定値・注釈・依存」が新構成の横に残る。実行時に効く不整合 (teleop の符号・EKF 既定・不在地図) が 4 件 |
| ドキュメント・Claude 設定 | CLAUDE.md 自己矛盾 5 / PROJECT_STATE 肥大 1 / issue ステータス未更新 8 / skills 陳腐化 12 / 索引欠落 4 | 「規約ファイルに履歴が混ざる」「状態メモが issue に追従しない」「skill の期待値が 2 世代前」。構造的原因は Stop hook の監視パスが再編前のまま (= 一度も発火していない) |

### 1.2 優先度つき上位 15 件 (✔ = 本セッションで現物確認済み)

| # | 区分 | 指摘 | 場所 | 直し方 (案) | 工数 |
|---|---|---|---|---|---|
| 1 | 実バグ ✔ | キーボード teleop の `invert_left/right: false` が bringup 側 (true) と逆 → 走行距離表示の符号が反転。`/params-sync` 規約違反 | `ros2_ws_main/src/app/epos4_teleop/config/params.yaml:5-6` vs `rerobot_bringup/config/params.yaml:13-14` | true に揃える。params-sync skill の期待値も更新 | S |
| 2 | 不整合 ✔ | `bringup2d.sh` の `EKF` 既定 false が launch 既定 true を打ち消す → `slam2d.sh` 経由は EKF 無し。CLAUDE.md の説明と食い違う | `scripts/bringup2d.sh:13` vs `launch/rerobot_bringup.launch.py` (ekf 既定 true) | スクリプト既定を true に (または env を渡さない) | S |
| 3 | 不整合 ✔ | nav 構成 3 本 + legacy + nav2_params の既定 `map_dir` が存在しない `maps/2d/slam_toolbox/2026_6_9__22:00` → 引数なし起動が落ちる | `launch/nav/nav_{amcl,amcl_scanall,emcl2}.launch.py`, `config/nav2_params.yaml:21,33` | 既定を空にして必須化 (auto_run は必須にしている) | S |
| 4 | 構造 ✔ | Stop hook の監視パス `src` / `Dockerfile` が 07-26 再編後に存在しない → コード変更で一度も発火していない。PROJECT_STATE が追従しない原因 | `.claude/hooks/check-project-state.sh:63` | `ros2_ws_*/src docker/ scripts/` に変更 | S |
| 5 | 肥大 ✔ | PROJECT_STATE.md が 210 KB。6〜7 行目だけで 39 KB (更新履歴を「最終更新」行に連結し続けた)。§6 タイムラインとも二重 | `docs/claude/PROJECT_STATE.md:6-7` | 冒頭履歴を削除、§6 を `docs/monthly/2026_MM.md` (本日新設) に委譲して本体 ≤150 行へ | M |
| 6 | 自己矛盾 ✔ | CLAUDE.md: `gear_ratio: 5.0` (L170) vs 現物 92.25 / IMU 向き `rpy=(0,0,π/2)` (L148) vs L141 `(π,0,−1.6257)` / urg topic `/scan` (L141) vs `/urg_front/scan` (L95) / `nav2.launch.py` を現役扱い (L100,150) / params-sync「3 ファイル」(L188) vs 2 | `CLAUDE.md` | 値を現物に合わせ、日付入り履歴 38 行を features/issue/monthly へ移す (規約 ≈45 行に圧縮) | M |
| 7 | 陳腐化 | skills の期待値が 2 世代前: `params-sync`「gear_ratio 正値 5.0」、`verify`「gear 1.25」「`rerobot_bringup_2d`」「`--metas` なし」、`stack-health`「`/sdk_could`」「`/scan` 40 Hz」 | `.claude/skills/{params-sync,verify,stack-health}/SKILL.md` | 現物に合わせて更新。bus.yml の sync 10 ms / PDO 50 ms の区別も明記 | M |
| 8 | 残骸 ✔ | `package.xml` が COLCON_IGNORE 済み旧ドライバ `rfans_driver` を exec_depend、現行 `surestar_rfans_ros2` / pointcloud_to_laserscan / robot_localization / twist_mux など 7 件が未宣言 | `rerobot_bringup/package.xml:20` | 依存を現行に書き換え | S |
| 9 | 残骸 ✔ | `__pycache__` 20 ファイルが git 管理下 (cpython-310 = ホスト実行の痕跡)。`.gitignore` に `__pycache__/` なし | `git ls-files \| grep __pycache__` | `.gitignore` 追加 + `git rm --cached` | S |
| 10 | 危険な残骸 | `imu_check.launch.py` が bno086 を params_file なしで include → ドライバ既定 `auto_tare: all` のまま IMU 向きを検証する (09-24 の誤認を再現する装置) | `launch/tools/imu_check.launch.py:48-57` | `config/bno086.yaml` を渡す | S |
| 11 | 注釈の矛盾 ✔ | nav2_params の「10-17 試験的に urg を外す」注釈直下で `observation_sources: urg_front rfans` (urg 入り)。10-21 コミット `remove URG and retauch URG` で戻した際に注釈だけ残った模様 | `config/nav2_params.yaml:159-162` | 注釈を「10-21 に戻した。外した場合の影響は…」に書き換え | S |
| 12 | 衛生 | docs/issue 30 本中、解決・前提消失・対象廃止なのにステータス未更新が 8 本 (utm30lx / rfans_calc_review / 08-15・08-16 の棄却済み仮説 / 09-13 / repository_audit 全 25 件…)。PROJECT_STATE §5 は 10 本未掲載・5 本が古い、§10 に完了済み「最優先」が 3 段積層 | `docs/issue/*`, `PROJECT_STATE.md §5, §10` | ステータス行を正規化 → `docs/issue/INDEX.md` を生成、解決済みは `closed/` へ | M |
| 13 | 索引欠落 | issue/report/features/text に一覧 index がなく、docs/ の入口が README・CLAUDE.md・PROJECT_STATE §8・manual/00_index の 4 か所に分散 (PS §8 は text を「3 冊」と記載、実際 7 冊) | `docs/` | `docs/README.md` 新設 + 各 dir に INDEX.md。CLAUDE.md / PS §8 の表はリンク 1 行に | M |
| 14 | 死に経路 | RealSense 連鎖 (submodule フルビルド + Dockerfile 層 + `realsense2_camera_launch/` + `realsense_imu.launch.py`) は凍結中 LIO-SAM 専用。legacy `rerobot_bringup_*` 5 本は実行時参照ゼロ (「scripts が使う」注釈は偽) | `docker/Dockerfile_main:56-65,80-83`, `launch/legacy/` | realsense-ros に COLCON_IGNORE、legacy ラッパは削除 (v1-monolithic に残る) | S〜M |
| 15 | README 旧記述 | 旧 symlink 復旧手順 (07-26 撤廃)、`docker compose up --build` (鉄則に抵触)、`copose` typo、legacy launch 名、`/dev/bno086` を「指定せよ」と言うが全 launch 既定は `/dev/ttyACM0` | `README.md:16-26,168` | 現行手順に書き換え、`scripts/` と `docs/README.md` への導線を冒頭に | S |

### 1.3 推奨する直し順

1. **実行時に効くもの** (#1 #2 #3 #10 #11) — 30 分。走行結果に影響する。
2. **構造的原因** (#4 hook, #9 pycache, #8 package.xml) — 30 分。これを直さないと以後も劣化が続く。
3. **規約ファイルの浄化** (#6 CLAUDE.md, #7 skills) — 半日。全セッションに伝播する誤りを止める。
4. **状態メモと索引** (#5 PROJECT_STATE 分割, #12 issue ステータス, #13 INDEX) — 半日〜1 日。`docs/monthly/2026_MM.md` (本日新設) を §6 の後継にして PROJECT_STATE を軽くする。
5. **死に経路の整理** (#14, #15) — 任意。

### 1.4 ディレクトリ構成の提案 (docs/)

詳細は §3 の「4. ディレクトリ構成の提案」。要点:

```
docs/
├── README.md                 ★ 全体の入口 (各 dir の役割 1 行 + 索引リンク)。CLAUDE.md / PROJECT_STATE §8 の表はここへのリンクに
├── claude/
│   ├── PROJECT_STATE.md      ← §6 タイムラインを monthly/ に委譲し ≤150 行に
│   ├── handoff/              ← issue/*.prompt.md をここへ (指示書の置き場を 1 つに)
│   └── knowledge/
├── issue/  + INDEX.md ★ + closed/ ★ (解決・前提消失は移動)。img/ の dir 名を本体 slug に揃える
├── report/ + INDEX.md ★ (pdf 2 本は md と重複 → 削除候補、09-26 bno086 申し送りは reference/ へ)
├── features/ + INDEX.md ★
├── reference/                ← そのまま (wiring_diagram.mmd は v1 配線のまま → v2 へ更新要)
├── trial/  improvements.md (ユーザ) + analysis/ ★ (Claude 作成 2 本を分離)
├── manual/                   ← そのまま。memo_2026_10_17.md は章番号付きへ (命名規約)
├── text/   + README.md ★ (7 冊一覧)。waypoint_nav/ は仕様書なので features/ へ移すか章分割
└── monthly/ 00_index.md + 2026_MM.md (本日新設) + 2026_6 (ユーザ月報、そのまま)
```

CLAUDE.md の「`docs/monthly/` は Claude 編集禁止」は、本日ユーザ指示で Claude 作成の `2026_MM.md` を同居させたので、「拡張子なしのユーザ月報は編集禁止、`2026_MM.md` は変更履歴として Claude が更新」に書き換えが必要。

---

## 2. 詳細 (A) コード・launch・設定・スクリプト・Docker — 調査 subagent 全文



対象: `/home/nomface/reRoBot` (2026-10-23 時点、HEAD 3f0c8f9)。読み取り専用。パスはリポジトリ直下からの相対、`行` は `cat -n` の行番号。
略記: `bringup/` = `ros2_ws_main/src/bringup/rerobot_bringup/`、`app/` = `ros2_ws_main/src/app/`、`glim/` = `ros2_ws_glim/config/`。
「要確認」= 根拠は見つけたが意図 (残す/消す) をユーザに確認しないと判断できないもの。

### A. 重複

| # | 場所 (file:line) | 内容 | 影響 | 提案 | 工数 |
|---|---|---|---|---|---|
| A1 | `bringup/config/params.yaml:9-14` (controller 節), `:34-41` (odometry 節), `app/epos4_teleop/config/params.yaml:3-6`; C++ 既定値 `app/epos4_controller/src/epos4_controller.cpp:117-121`, `src/epos4_odometry.cpp:24-31`, `app/epos4_teleop/src/teleop_keyboard.cpp:49-52` | 車体パラメータ (tread_width / tire_diam / gear_ratio / invert_*) が YAML 3 節 + C++ 既定値 3 か所 = 6 か所。C++ 既定 (0.41 / 0.15 / 1.0) は 2026-07 以前の車体値で、yaml を渡し忘れると静かに旧車体で動く | **teleop 側は invert_left/right が false で bringup (true) と不一致** → C1 として別掲。既定値の 3 重保持は「yaml 忘れ」の検知を鈍らせる | yaml は 1 節に統合して `/**:` ワイルドカードで両ノードに配る、または teleop launch を作って bringup の params.yaml を渡す。C++ 既定値は意図的にありえない値 (0.0) にして起動時に FATAL | S |
| A2 | `bringup/config/params.yaml:56-62` (rfans_driver), `:80-83` (rfans_calculation), `bringup/launch/rerobot_bringup.launch.py:109-117`, `launch/legacy/rerobot_bringup_{2d3d_imu:20-25,2d_imu:19-24,3d_imu:20-25,3d:18-23}.launch.py` | R-Fans の `device_ip 192.168.0.3` / `rps 10` / `model R-Fans-16` が 7 か所 | 変更時の取りこぼし。driver/calculation の 2 節同値は ROS2 の制約で必須 (params.yaml:79 注記) だが、launch 既定 + legacy 4 本は冗長 | launch 既定を params.yaml の値に委ねる (引数未指定なら上書きしない) か、legacy を削除 (B2) | S |
| A3 | serial_port `/dev/ttyUSB-utm-30lx`: `rerobot_bringup.launch.py:105`, legacy 4 本, `scripts/bringup_check.py:70`; imu_port `/dev/ttyACM0`: `rerobot_bringup.launch.py:122`, `config/bno086.yaml:16`, `launch/tools/imu_check.launch.py:26`, legacy 3 本, `scripts/bringup_check.py:84`; imu_wit_port `ttyUSB-wt901`: `rerobot_bringup.launch.py:67`, `config/wt901.yaml:12`, `launch/tools/wt901_imu.launch.py:24`, legacy 3 本, `scripts/bringup_check.py:87` | デバイスポート名が種類ごとに 5〜7 か所 | ポート変更時の整合漏れ (C3 の `/dev/bno086` 問題の温床) | ポート既定は各ドライバ yaml に 1 か所だけ持ち、launch は引数指定時のみ上書き。bringup_check のヒント文字列は launch から param で渡す | S |
| A4 | `launch/nav/nav_amcl.launch.py:34-63`, `nav_amcl_scanall.launch.py:32-57`, `nav_emcl2.launch.py:31-61`, `nav_lidar3d.launch.py:35-71`, `launch/legacy/nav2.launch.py:58-99`, `launch/auto_run.launch.py:81-86` | `use_sim_time / use_rviz / map_dir / map_yaml / keepout_yaml / use_keepout` の DeclareLaunchArgument 6 本 + `include()` ヘルパ + rviz include が 5 ファイルでほぼ同文 (各 ~40 行)。「include は GroupAction(scoped=True) で包む — … controller_server が落ちた」の 3 行注釈も 7 か所にコピー (上記 4 + legacy nav2:92-94 + `parts/localization/loc_amcl.launch.py:70-72`, `loc_emcl2.launch.py:56-58`) | 1 引数を足すと 5 ファイル編集。既に `nav_lidar3d` だけ `scan_topic` が無い等、揃いが崩れ始めている | `launch/nav/_common.py` に `declare_map_args()` と `include_part()` を置き、各 nav_* は差分 (localization 部品と scan 設定) だけ書く | M |
| A5 | URDF: `bringup/urdf/rerobot.urdf` ≡ `rerobot.flat.urdf` (diff 空)、tilted15/45 との差は rfans_joint と注釈のみ (diff で確認、**整合 OK**)。GLIM: `glim/config_sensors.json` ≡ `config_sensors.flat.json` (diff 空)、tilted15/45 は T_lidar_imu と注釈のみ差 (**OK**)。ただし `config_sensors.flat_oldchassis_cal.json` は機械出力 (注釈ゼロ・キー順が別・T_lidar_camera が先頭) で形式が他 5 本と揃っていない | プリセット丸ごとコピー方式自体は規約どおり機能している。oldchassis_cal だけ diff が全行差分になり、プリセット間比較ができない | oldchassis_cal を他と同じキー順・注釈付きに整形 (値は不変)。または bag 専用プリセットは `glim/legacy/` に分ける | S |
| A6 | `tools/loc_eval/variants/amcl_alpha002.yaml`, `amcl_range100.yaml`, `amcl_range100_alpha002.yaml` ↔ `bringup/config/localization/amcl.yaml`; `variants/emcl2_range100.yaml` ↔ `config/localization/emcl2.yaml`; `variants/lidar3d_fast_res{2,5}.yaml` ↔ `config/localization/lidar_localization.yaml` | 評価用変種が本番 yaml の**全文コピー**で 1〜4 値だけ違う。10-11/10-12 の結論 (alpha 0.02, laser_max_range 100, voxel 0.3 + crop off) は本番に取り込まれたので、`amcl_alpha002.yaml` は本番と alpha が同じになり、逆に `transform_tolerance 1.0` (本番 2.0) と `laser_max_range 30` (本番 100) で**本番より古い**値を持つ | 変種を再利用すると「本番との差分」が意図しない 2 項目になり、評価が本番を代表しない | 変種は差分だけの小 yaml にし、launch 側で `[本番 yaml, 変種 yaml]` の順に重ねる (loc_lidar3d.launch.py:72-76 は既にこの方式)。役目を終えた変種は `bags/exp/` 側へ退避 | S |
| A7 | `bringup/config/bno086.yaml:27` `imu_rate_hz: 100.0` ↔ `rerobot_bringup.launch.py:130` 既定 `200.0` ↔ `scripts/bringup2d.sh:12`, `bringup3d.sh:12` `IMU_RATE=200` ↔ `scripts/glim3d.sh:11` が明示的に `IMU_RATE=200` | IMU レートが 4 層で保持。yaml の 100 は launch に常に上書きされ、読んだ人を誤解させる (CLAUDE.md は 200 を正としている) | yaml を 200 に揃える (launch の上書きは残してよい)。glim3d.sh:11 の `IMU=true IMU_RATE=200` は既定と同値で冗長 | S |
| A8 | `docker-compose.yml:21-24, 47-50, 76-80, 113-118, 141-144` (environment ×5), volumes の maps/bags/tools/X11 ×5、`glim` (60-87) と `glim_gpu` (98-129) はほぼ同文 | compose の 60% が繰り返し | YAML anchor (`x-common-env: &env`, `<<: *env`) で 1 か所に。glim_gpu は `extends: glim` + 差分 (gpus / build.args / 入れ子 bind) | S |
| A9 | `scripts/bringup2d.sh` と `scripts/bringup3d.sh` (29 行ずつ、差分は lidar_2d/3d フラグと ekf とログ名のみ); `scripts/build.sh:20-36` の colcon 3 分岐 | シェルの定型 (compose up → install 確認 → docker exec source …) の重複 | D2 参照 (共通 lib 化)。bringup2d/3d は `scripts/bringup.sh [2d|3d|all]` 1 本に | S |
| A10 | `bringup/config/nav2_params.yaml:131-136` と `:203-208` (footprint + 5 行の実測注釈が同文)、`:156-158` と `:232-234` (observation_sources 注釈同文) | Nav2 の仕様上 footprint は両 costmap に必要だが、長い導出注釈まで二重 | 注釈は片方に置き、他方は「local_costmap の注記参照」 (keepout_static では既にそうしている :214) | S |
| A11 | `scripts/bringup_check.py:23` と `scripts/waypoint_runner.py:60` (ANSI 色定数)、`bringup_check.py:109-112` と `auto_run_check.py:80-83` (`_spin_for` 同文)、`bringup_check.py:36-39` と `auto_run_check.py:38-40` (`_pad` / `_width` 全角幅計算) | 小さいが同じ関数が 2〜3 本 | `scripts/_tui.py` に色・幅・spin_for を置き import (auto_run_check は既に bringup_check を import しているので延長で済む) | S |
| A12 | 「2026-10-12 既定 false。機上の RViz (+Xorg) は約 1.4 コア食い…」注釈: `nav_amcl:36-37`, `nav_amcl_scanall:34-35`, `nav_emcl2:33-34`, `nav_lidar3d:37-38` | 同一の 2 行注釈 ×4 | A4 の共通化で消える | S |

### B. 死にコード・残骸

| # | 場所 (file:line) | 内容 | 影響 | 提案 | 工数 |
|---|---|---|---|---|---|
| B1 | `git ls-files | grep pycache` = **20 ファイル**: `bringup/launch/__pycache__/*.cpython-{310,312}.pyc`, `launch/nav/__pycache__/*`, `scripts/__pycache__/*`, `app/rfans_scan_modes/rfans_scan_modes/__pycache__/*`, `tools/{glim_dump_to_2dmap,glim_traj_to_2dmap,loc_eval,waypoint_editor}/__pycache__/*`; `.gitignore` に `__pycache__/` が無い | バイトコードが git 管理下。`cpython-310` はホスト (Humble/Python 3.10) で実行された痕跡 = 「ホストで実行しない」規約の反証でもある | `git rm -r --cached '**/__pycache__'` + `.gitignore` に `__pycache__/` `*.pyc` | S |
| B2 | `bringup/launch/legacy/rerobot_bringup_{2d,3d,2d_imu,3d_imu,2d3d_imu}.launch.py` (5 本、計 241 行) | 実行時参照ゼロ: `scripts/bringup2d.sh:25` / `bringup3d.sh:25` は実体 `rerobot_bringup.launch.py` を直接呼ぶ。残る言及は注釈のみ (`bringup2d.sh:22`, `bringup3d.sh:22`, `ros2_ws_slamtoolbox/.../slam.launch.py:2`, `rerobot_slamtoolbox/package.xml:7`, `README.md:168`)。各ファイル冒頭 (`_2d:2-3`, `_3d:2-3`) の「scripts/bringup2d.sh 等の既存呼び出しを壊さないために名前を維持」は既に偽。CLAUDE.md・`rerobot_bringup.launch.py:23` も「削除候補」と明記 | 引数既定 (A2/A3) の重複源。`_2d_imu` 等は `ekf` 既定 false で実体 (true) と逆 → 名前で起動すると意図せず EKF 無し | 削除。注釈側 5 か所を `rerobot_bringup.launch.py lidar_3d:=false` 等に書き換え | S |
| B3 | `bringup/launch/legacy/nav2.launch.py` (114 行) | 唯一の実行時呼び出しは `scripts/nav2d.sh:17`。`launch/legacy/README.md:11` も「nav2d.sh がまだ使っている」と記録 | `amcl_scan` 3 択の読み替え (:54-57) など旧 API を保守し続ける | nav2d.sh を `nav_amcl.launch.py scan_topic:=/urg_front/scan` に切替えて nav2 を削除 (または nav2d.sh ごと削除し auto_run に一本化。CLAUDE.md の本番起動は auto_run) | S |
| B4 | `bringup/package.xml:19-20` `<exec_depend>rfans_driver</exec_depend>` (注釈に `src/drivers/StarROS2`) | `rfans_driver` は COLCON_IGNORE 済み旧ドライバ (`drivers/StarROS2/package.xml:4`) のパッケージ名。現行 `surestar_rfans_ros2` (launch.py:258) は未宣言 | `rosdep install` が未知キーで止まる / 依存グラフが嘘 | `surestar_rfans_ros2` に差し替え (C17 の欠落分も同時に) | S |
| B5 | `docker/Dockerfile_main:18` `ros-jazzy-ros2-control`、`:46` `ros-jazzy-nav2-minimal-tb*`、`:13-14` `libudev-dev udev` | ワークスペース内に利用箇所なし (grep: ros2_control / controller_manager / nav2_minimal / turtlebot / libudev いずれも 0 件)。`nav2-minimal-tb*` は TurtleBot シミュレーション用 | イメージ肥大・ビルド時間。`ros2-control` は canopen の依存だった可能性 → **要確認** (`apt-cache depends ros-jazzy-canopen-402-driver`) | 利用確認のうえ削除。`rqt*` (23-25) は開発用として残すなら「開発用」と節を分ける | S |
| B6 | RealSense 系一式: submodule `drivers/realsense-ros` (C++ 大物ビルド)、`bringup/realsense2_camera_launch/` (171 行、上流 rs_launch.py のコピーで既定 7 項目だけ変更)、`bringup/launch/tools/realsense_imu.launch.py`、`Dockerfile_main:56-65` (cv_bridge / image_transport / diagnostic_updater / eigen / madgwick)、`:80-83` (librealsense2) | 用途は LIO-SAM 入力 (`realsense_imu.launch.py:5,9`) のみで、LIO-SAM は凍結、IMU は 08-11 から BNO086。`rerobot_bringup.launch.py:292` も「同時起動すると /imu/data が衝突」と警告 | `build.sh main` のたびに realsense-ros を sequential ビルド。Docker 層も重い | realsense-ros を `COLCON_IGNORE` (StarROS2 と同じ扱い) し、realsense2_camera_launch / realsense_imu.launch.py / Dockerfile の該当行を liosam 側に移すか削除。復活時は `v1-monolithic` から | M |
| B7 | `app/epos4_controller/src/epos4_controller.cpp:58-61, 86-89, 100, 194-195, 209-212, 215, 230, 234-236, 396-412, 446-456` (コメントアウト済み旧 joint_states 購読 / encoder_publisher)、`:207, 228` `m1_publisher_` / `m2_publisher_` (宣言のみ、生成も使用もなし)、`:37-53, 65-81` の `halt_ / shutdown_ / vel_mode_ / target_` クライアント 8 本 (生成のみ、呼び出しゼロ)、`:173-174` 「To be used with 1 EPOS4: /cia402_device_1」バナー (2 台構成に不一致) | 889 行中 ~60 行が死にコード。未使用 client は起動時に 8 本の service 探索を増やす | 削除 (git 履歴にある)。使うかもしれない halt/recover は `#if 0` でなく削除して必要時に戻す | S |
| B8 | `glim/config_odometry_ct.json`, `config_sub_mapping_passthrough.json`, `config_global_mapping_pose_graph.json` | `config.json:15-17` / `config.gpu.json:31-33` のどちらからも参照されない (CT-ICP 復帰用に `config.json:12-14` が言及)。`config_sensors.flat_tareall_legacy.json` / `flat_oldchassis_cal.json` は過去 bag 専用 | 「どれが live か」が一覧で分からない | `glim/presets/` or `glim/legacy/` サブディレクトリに移し、`config.json` 先頭に「live = これ、他は presets/」と 1 行。bind mount は `/glim_config` ディレクトリごとなので移動しても可 | S |
| B9 | 10-01 の選点法実験の残骸: `app/rfans_scan_modes/` (パッケージ丸ごと)、`bringup/launch/tools/rfans_scan.launch.py:128-154` (`farthest` / scan_far ノード)、`bringup/launch/tools/amcl_variants.launch.py` + `bringup/config/amcl_variants.yaml`、`tools/amcl_compare/` | 結論は出ている (farthest は不採用 `rfans_scan.launch.py:130`、全点は p2l で実現 `node.py:4-5`、評価系は 10-11 に `tools/loc_eval` へ移行)。`amcl_variants.yaml:21,43` と `tools/amcl_compare/replay.sh:50` は旧トピック名 `/scan` `/scan_all` のままで現行 bringup では動かない。bringup は常に `farthest:=false` (launch.py:244) | 誰も起動しないノード定義・パッケージがビルド対象に残る (rfans_scan_modes は ament_python で軽いが) | **要確認** (ユーザが再実験予定か)。不要なら rfans_scan_modes + amcl_variants.{launch.py,yaml} + scan_far を削除し、tools/amcl_compare は README に「10-11 以降は loc_eval を使う。本ツールは /scan 時代の bag 専用」と明記して凍結 | M |
| B10 | `bringup/config/nav2_params.yaml:295-299` behavior_server の `spin` / `backup` プラグイン | 自作 BT (`behavior_trees/navigate_through_poses_follow.xml:22-24`) は Spin/BackUp を外した。残る利用者は RViz 単発ゴール (Nav2 既定 BT) のみ | 読み込みだけ。小 | 既定 BT も使わない運用なら外す。使うなら注釈で「NavigateToPose 用」と明記 | S |
| B11 | `bringup/config/ekf.yaml:49` `imu0_remove_gravitational_acceleration: true` | `imu0_config:41-45` で加速度は全 false → この設定は無効 | 誤解のみ | 削除か注釈 | S |
| B12 | `bringup/launch/joy_teleop.launch.py:13-17`, `bringup/config/joy_teleop.yaml:5-10` 「この機能をやめたいとき (完全撤去手順) … 既存の launch / config には手を入れていない」 | 10-16 以降は twist_mux (bringup 常駐)・auto_run の include・`waypoint_runner` 節 (yaml:54-63) が依存しており、手順どおり消すと auto_run が壊れる | 誤った撤去手順 | ブロック削除 (joy は本番の必須部品になった) | S |
| B13 | `README.md:16-19` (symlink 再作成 `cd ~/reRoBot/src; ln -s external/...` — 07-26 に撤廃された構造)、`:24` `docker compose up --build` (3 イメージ並列ビルド = CLAUDE.md の禁止事項)、`:26` `docker copose down` (typo)、`:28-36` ホストで `colcon build` する手順 (CLAUDE.md「必ずコンテナ内」と矛盾、`--metas` も無い)、`:168` `rerobot_bringup_2d.launch.py` | 新規参加者が最初に読む手順が規約と逆 | Setup 節を「`git clone --recursive` → `docker compose up -d main` → `./scripts/build.sh main`」に書き換え、詳細は docs/manual/02 へ委譲 | S |
| B14 | `.gitmodules:1,4,7,10` submodule 名 `src/external/...` (path は移動済み) | 動作に影響なし (git は path で解決) | 混乱のみ | `git mv` 時に名前も直す (`git config -f .gitmodules --rename-section`) | S |
| B15 | `ros2_ws_liosam/src/liosam_config/params_rfans.yaml:7-11, 54-64`, `tools/rfans_bag_to_liosam.py` | extrinsic が 08-30 改修前の車体 + 09-24 以前の IMU 向き (Rz 90°) 前提。ファイル自身が警告済み | 凍結中なので実害なし。復活時に再計算必須 | 冒頭に「09-24 の IMU 向き修正 (上下逆 + yaw −93°) も未反映」を 1 行追加 | S |
| B16 | `app/epos4_teleop/src/teleop_keyboard.cpp:76-77` 「Distance display will be 5x too large」 | gear_ratio は 5.0 → 92.25 (08-30)。警告文が旧値 | 誤誘導 | 「約 gear_ratio 倍」に書き換え | S |
| B17 | `tools/README.md:4-5` 「glim コンテナに /workspace/tools として bind mount」 | 現在は main (`docker-compose.yml:29`)・glim (:85)・glim_gpu (:127) の 3 つ | 誤誘導 | 修正 | S |
| B18 | `maps/3d/README.md:2` 「2D 占有格子は ../2d/ で版管理」 ↔ `.gitignore:1` `maps/` (全部 ignore、`git ls-files maps` = 0) | 2D 地図は版管理されていない。README が嘘 | 地図の喪失リスクの認識ずれ (bags は Drive 方針があるが maps は不明) | 方針を決めて片方を直す。2D 地図 (数 MB) を版管理するなら `.gitignore` を `maps/3d/` だけに | S |
| B19 | `docker/Dockerfile_main:4` 「GLIM → 公式イメージ (Dockerfile なし)」(Dockerfile_glim が存在)、`:51` 「Sure-Star RFans/CFans rfans_driver」(旧名)、`docker/Dockerfile_glim:5-11, 28` と `docker-compose.yml:65` 「GPU は docker-compose.override.yml で」 ↔ `:98-129` の `glim_gpu` サービス (10-07 以降の正) | GPU 有効化の手段が 2 通り書かれている | 新 PC セットアップで迷う | override.yml 方式は「描画のみ・CPU 版 glim 用」と限定して書くか削除 | S |
| B20 | 積層した旧注釈: `bringup/urdf/rerobot.urdf:129-139` (imu rpy (0,0,π/2) を「最終確定」と 3 証拠付きで断言 → 直後の `:152-163` で否定)、`glim/config_sensors.json:53-57` (R=Rz(90°), q=(0,0,√2/2,√2/2) と書くが実値 `:70-77` は別)、`config_sensors.tilted15.json` / `tilted45.json` の同箇所 (注釈の四元数が実値と不一致) | 同一ファイル内で結論が 2 回ひっくり返る注釈が残り、値だけ見ると注釈と矛盾 | 読み手が旧値を正と誤認 (特に GLIM json は注釈内の数値をコピーしがち) | 現在値の導出だけ残し、経緯は `docs/issue/2026-09-24_...` と `2026-10-16_...` への参照 1 行に圧縮。全 URDF 4 本 / GLIM json 3 本同時に | S |
| B21 | `bringup/config/nav2_params.yaml:21, 33` `yaml_filename` 既定 `/workspace/maps/2d/slam_toolbox/2026_6_9__22:00/...` | 存在しないディレクトリ (実在は `2026_8_11__18-20`)。常に launch が上書きするので死に値 | C6 と同根 | 削除 (map_server は launch から必ず受ける) | S |
| B22 | `bringup/config/localization/amcl.yaml:14` `scan_topic` / `:39` `max_beams` | `parts/localization/loc_amcl.launch.py:49-54` が常に上書き (注釈もそう書いている) | 死に値 | yaml から外すか「launch が必ず上書き」のまま残すなら値を既定 (nav_amcl と同じ) に揃える | S |

### C. 不整合

| # | 場所 (file:line) | 内容 | 影響 | 提案 | 工数 |
|---|---|---|---|---|---|
| **C1** | `app/epos4_teleop/config/params.yaml:5-6` `invert_left: false / invert_right: false` ↔ `bringup/config/params.yaml:13-14, 40-41` `true / true` | 車体パラメータの同期規約 (CLAUDE.md、`/params-sync`) に反する不一致。teleop の距離表示 (`teleop_keyboard.cpp:212, 221` で invert を適用) は**符号が逆**になる (前進で負の距離) | 台上検証で左右・前後の判断を誤る | teleop 側を true に (または A1 の統合で重複自体を消す)。`/params-sync` を CI 的に `verify` スキルへ組み込む | S |
| **C2** | `scripts/bringup2d.sh:13` `EKF="${EKF:-false}"` ↔ `bringup/launch/rerobot_bringup.launch.py:72-73` `ekf` 既定 `true` (10-08) ↔ CLAUDE.md「bringup2d.sh … IMU=true EKF=true を前置すると EKF 融合込み」 ↔ `scripts/slam2d.sh:7` は素の `./scripts/bringup2d.sh` | スクリプト経由だと EKF が**既定で切れる**のに、launch 直接だと入る。slam2d では EKF 無し (odom→base_link は epos4_odometry)、nav2_params.yaml:47,64 は `/odometry/filtered` 前提。bringup2d/3d は `lidar_3d:=false` / `lidar_2d:=false` も固定し「全部入り既定」から外れる | 起動経路で TF 配信者が変わる。bag の比較・再現性に影響 | bringup2d.sh の既定を launch と同じ true に (または env 未指定時は launch に渡さない)。CLAUDE.md の記述も現状に合わせる | S |
| **C3** | `README.md:196-201` 「udev で `/dev/bno086` が作られる。`port:=/dev/bno086` を指定する」 ↔ 全 launch 既定 `/dev/ttyACM0` (`rerobot_bringup.launch.py:122`, `tools/imu_check.launch.py:26`, legacy 3 本, `config/bno086.yaml:16`) ↔ `scripts/bringup*.sh` は imu_port を渡さない ↔ `scripts/bringup_check.py:84` ヒントも `/dev/ttyACM0` | README が「指定せよ」と言う安定名を、どの起動経路も使っていない。ACM 番号が変わると (WT901 の USB-TTL 等と競合) 起動失敗 | 稀だが当日トラブルの種 | launch 既定を `/dev/bno086` に。`rerobot_bringup.launch.py:118-120` の「udev rule (tools/99-bno086.rules) 導入後は」注釈は実在パス `drivers/BNO086_ROS2Board-main/tools/99-bno086.rules` に修正 | S |
| **C4** | `bringup/launch/tools/imu_check.launch.py:48-57` が `bno086.launch.py` を `params_file` なしで include | ドライバ既定 yaml は `auto_tare: all` (`drivers/BNO086_ROS2Board-main/.../config/bno086.yaml:55`)。本リポの `config/bno086.yaml:4-10` が「必ず off」と警告する、まさにその条件で**取付向きの検証ツール**が動く → 09-24 の誤認 (正立に見えた) を再現する | 取付向き確認の結果が信用できない | bringup と同じく `params_file: config/bno086.yaml` を渡す。imu_port 既定も C3 と揃える | S |
| **C5** | 高さ帯「costmap 用 /rfans/scan」の既定が 4 値: `launch/tools/rfans_scan.launch.py:46` `6.0`、`rerobot_bringup.launch.py:92` が渡す `2.0`、同 `rfans_scan.launch.py:10-12` 注釈「床 +0.3〜1.5 m と同じにすること」、`config/amcl_variants.yaml:15` `1.5` (「rfans_scan.launch.py と同じ」と注記)、`app/rfans_scan_modes/.../node.py:51` `1.5` | 単体起動と bringup 経由で帯が違う。注釈は 3 世代前の値 | bag 再生時 (`rfans_scan.launch.py` 単体) と実機で costmap 入力が変わる | rfans_scan.launch.py の既定を bringup と同じ 2.0 にし、注釈を 10-15 の決定 (costmap 0.3〜2.0 / AMCL 0.3〜100) に更新。amcl_variants は B9 | S |
| **C6** | `launch/nav/nav_amcl.launch.py:39`, `nav_amcl_scanall.launch.py:37`, `nav_emcl2.launch.py:36`, `launch/legacy/nav2.launch.py:70`, `config/nav2_params.yaml:21,33` の既定 map_dir `/workspace/maps/2d/slam_toolbox/2026_6_9__22:00` | 存在しない (実在は `maps/2d/slam_toolbox/2026_8_11__18-20.{pgm,yaml}` で、しかも `nav2/` サブ構造でない)。パスに `:` を含み、`ros2 launch` の `key:=value` 解析とも相性が悪い。`nav2d.sh` を引数なしで叩くと map_server が落ちる | 「とりあえず起動」が失敗する | 既定を空にして必須化 (`auto_run.launch.py:66-68` と同じ `RuntimeError`) か、実在する GLIM 地図に | S |
| **C7** | `bringup/config/nav2_params.yaml:159-162`: 注釈「2026-10-17 試験的に urg を外す (戻すときは `urg_front rfans`)」の直下で `observation_sources: urg_front rfans` | 注釈は「外した」と言い、値は入っている。10-17 の試験を戻したのか、外し忘れか不明 | 10-17 つくば bag (草で停止) の対策が入っているかどうか判断できない | **要確認**。意図に合わせて値か注釈を直す | S |
| C8 | `bringup/config/nav2_params.yaml:10` 「/urg_front/scan (… FOV ±90deg)」 ↔ `rerobot_bringup.launch.py:226-227` ±70° (10-15); 同 yaml `:16,20,26` 「nav2.launch.py の `map` / `keepout_mask` 引数」 ↔ 実引数名 `map_yaml` / `keepout_yaml` | 注釈が旧仕様 | 誤誘導 | 修正 | S |
| C9 | `bringup/config/ekf.yaml:12-14` 「BNO086 の絶対 yaw は磁北基準 (auto_tare でゼロ化はされるが…)」 | 09-24 から tare は off。前提説明が古い | 誤誘導 | 修正 | S |
| C10 | CLAUDE.md:146-147 `launch/realsense_imu.launch.py`, `launch/rfans_scan.launch.py` ↔ 実体は `launch/tools/` (10-09 移動、`rerobot_bringup.launch.py:236, 320` 注釈)。CLAUDE.md の packages 一覧に `app/rfans_scan_modes` と `drivers/BNO086_ROS2Board-main` (submodule でなく 74 ファイルの直置きコピー、`-main` は GitHub zip 名) が無い | 「新しいセッションの読み順」の起点が実構成とずれる | 迷子 | CLAUDE.md を現構成に更新。BNO086 は submodule 化するか「直置き (理由)」を明記 | S |
| C11 | `bringup/urdf/rerobot.urdf:56` 「motor1 = 右輪 (y-)」 ↔ `:61` `m1_wheel` origin `y=+0.2645` (左側) ↔ `app/epos4_controller/src/epos4_odometry.cpp:75` 「m1_wheel(+y)/m2_wheel(-y)」 | URDF 内の注釈と joint 配置が逆。可視化のみ (注釈 :57 もそう言う) だが、2 ファイルの注釈同士も矛盾 | RViz で車輪の回転が左右逆に見える | joint 名を wheel_left/right にして物理側に合わせるか、注釈を joint 配置に合わせる | S |
| C12 | `app/epos4_controller/src/epos4_controller.cpp:599-603, 869-870` `"motor1(left)"` `"motor2(right)"` ↔ `:713-714` `"motor1(right)"` `"motor2(left)"` ↔ `:440` claude_swap「motor1 = RIGHT」 | 同一プロセスのログで motor1 が left と right の両方で出る (CLAUDE.md も「旧ラベル残存」と認識) | ログ解析で左右を取り違える | 定数 `kM1Label = "motor1(right)"` を 1 か所に | S |
| C13 | `bringup/launch/rerobot_bringup.launch.py`: `rfans_scan_include` は `GroupAction(scoped=True)` (:235-248) だが `bus_config` (:142-150)、`imu_include` (:293-314)、`imu_wit_include` (:318-330) は非 scoped で、`:323-325` 注釈どおり「params_file を明示しないと漏れる」に頼っている | CLAUDE.md の「include は scoped で包む」規約が同一ファイル内で片側だけ適用 | 次に include を足した人が漏れを踏む (10-01 の再発) | 3 include も scoped に (引数は明示しているので挙動不変) | S |
| C14 | `parts/localization/loc_amcl.launch.py:29` map_topic `'map'` (相対) ↔ `loc_emcl2.launch.py:28` `'/map'` | 同じ役割の部品で表記が違う。名前空間なしなら同値 | 将来 namespace を切ると挙動差 | `/map` に統一 | S |
| C15 | 旧トピック名 `/scan` の注釈: `rerobot_bringup.launch.py:4, 51-52` (「urg_node (HOKUYO, /scan)」だが実装は `:228` で `/urg_front/scan`)、`nav_amcl.launch.py:1`, `nav_emcl2.launch.py:1`, `ros2_ws_slamtoolbox/.../slam.launch.py:2-3, 65`, `rerobot_slamtoolbox/package.xml:7`, `scripts/slam2d.sh:4`, `rviz/nav2.rviz:204`, `slam.rviz:134`, `parts/localization/loc_map.launch.py:6` (`/scan_all` `/scan_far`) | 10-08 の「裸の /scan は誰も出さない」規約が注釈に反映されていない (値は全て新名で**正しい**) | 読み手が bag の `--remap` 要否を誤る | 一括置換 | S |
| C16 | `bringup/package.xml` の exec_depend 欠落: `pointcloud_to_laserscan` (`rfans_scan.launch.py:72`)、`robot_localization` (`rerobot_bringup.launch.py:176`)、`rfans_scan_modes` (`rfans_scan.launch.py:138`)、`joint_state_publisher` (`imu_check.launch.py:42`)、`nav2_common` (`amcl_variants.launch.py:23`)、`imu_filter_madgwick` / `realsense2_camera_launch` (`realsense_imu.launch.py:32, 47`)、`surestar_rfans_ros2` (B4) | 依存宣言が実装より 7 件少なく、1 件 (rfans_driver) 余計 | `rosdep` で環境再現できない (Dockerfile が手書きで補っている状態) | package.xml を実装に合わせる。Dockerfile は `rosdep install --from-paths src` に寄せるとさらに重複が減る | S |
| C17 | `scripts/nav2d.sh:17` → legacy `nav2.launch.py` (既定 `scan_topic` `/urg_front/scan`、既定 map_dir は C6 の不在パス) ↔ CLAUDE.md「nav2d.sh = 自律走行一発」 ↔ 本番は `auto_run.launch.py` (GLIM 地図 + `/rfans/scan_all`) | CLAUDE.md が「一発」と呼ぶスクリプトは urg + slam_toolbox 地図前提の旧構成で、しかも引数なしでは起動しない | 当日に誤って叩くと別構成 | nav2d.sh を auto_run 相当に置き換えるか削除し、CLAUDE.md の表を更新 | S |
| C18 | `docs/claude/PROJECT_STATE.md:6` 「最終更新 2026-10-16」 ↔ コードは 10-17 (`behavior_trees/...xml:26-33` ReactiveSequence)、10-22 (`nav2_params.yaml:99` transform_tolerance 0.5、`config/localization/lidar_localization.yaml:17-20` future_dating、submodule ローカル変更)、10-23 コミットまで進んでいる | 「現在地」文書が 1 週間遅れ (CLAUDE.md の読み順 2 番目) | 次セッションが古い前提で始まる | `/project-state` で 10-17〜10-23 分を反映 | S |
| C19 | `glim/config_sensors.tilted15.json` / `tilted45.json` の T_lidar_imu は 10-16 の yaw 合成を反映した値 (diff で確認) だが、注釈 (diff 行 66-71 / 100-106 相当) は旧四元数のまま | 値は正しく注釈だけ古い | B20 と同根 | B20 と同時に | S |
| C20 | `bringup/launch/rerobot_bringup.launch.py` の `TimerAction` と `LaunchConfiguration` — **問題なし**: `_delayed_check` は OpaqueFunction で perform 済み (:368-395)、`delayed_nodes` は定数 5.0 (:400-405)、`auto_run.launch.py:57` は定数、`amcl_variants.launch.py:166-167` は float。CLAUDE.md の禁止パターンの違反は見つからなかった | — | — | — | — |

### D. 非効率

| # | 場所 (file:line) | 内容 | 影響 | 提案 | 工数 |
|---|---|---|---|---|---|
| D1 | `bringup/launch/rerobot_bringup.launch.py:235-248` (rfans_scan を既定 include) ↔ `launch/tools/rfans_scan.launch.py` 単体起動 (CLAUDE.md が「二重になる」と警告) ↔ `nav_amcl_scanall.launch.py:4` は bringup 側 `rfans_scan_all:=true` を**要求するが検査しない** (`auto_run.launch.py:107` だけが渡す) | 二重起動の防止も、必要ノードの不在検知も人手 (bringup_check は bringup 自身の引数しか見ない)。`nav_amcl_scanall` を手で上げると `/rfans/scan_all` が無く AMCL が黙る | 現場で「AMCL が動かない」の原因が見えない | nav_amcl_scanall 側で `/rfans/scan_all` の publisher を待って WARN を出す (auto_run_check の bringup 行はこれを兼ねているので、nav_* 単体用に小ノード化) | S |
| D2 | `scripts/bringup2d.sh:16-27`, `bringup3d.sh:15-27`, `nav2d.sh:15-17`, `slam2d.sh:10-18`, `teleop.sh:6-9`, `build.sh:19-36`: `docker compose up -d <svc>` → `test -f install/setup.bash` → `docker exec … bash -c 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && …'` の定型が 8 回 | 1 行変える (例: `set -o pipefail`、ログ先) と 6 ファイル | 保守コスト | `scripts/_lib.sh` に `ensure_container()`, `ros_exec <container> <cmd...>` を置く。CLAUDE.md/メモリの「素のコマンドを提示」方針とは別で、スクリプト内部の重複排除 | S |
| D3 | `docker/Dockerfile_main`, `_slamtoolbox`, `_liosam`, `_glim`: 共通 apt (`ros-jazzy-rviz2` ×4、`python3-colcon-common-extensions` ×3、`libpcl-dev` ×3、`libeigen3-dev` / `ros-jazzy-cv-bridge` / `tf2*` ×2) と `.bashrc` 設定 4 回 (`_main:89-92`, `_slamtoolbox:16-17`, `_liosam:38-39`, `_glim:37-39`) | 共有ベース層が無く、各イメージが同じ層を別々にダウンロード・保持。`build.sh images` の総時間 ≈ 4 本の和 | ディスク (16 GB 機) とビルド時間 | `docker/Dockerfile_base` (ros:jazzy-ros-base + colcon + rviz2 + .bashrc) を作り main/slamtoolbox/liosam は `FROM rerobot_base`。glim は公式イメージベースなので対象外 | M |
| D4 | B6 の RealSense 連鎖 (`drivers/realsense-ros` を毎回 `--executor sequential` でビルド) | main ws のフルビルド時間の相当部分を死に経路が占める (計測していないので比率は**要確認**) | 「このマシンは重いビルドで落ちる」(CLAUDE.md) 環境で負荷増 | B6 と同じ (COLCON_IGNORE) | S |
| D5 | `docker-compose.yml` の繰り返し (A8) | — | — | A8 | S |
| D6 | `app/epos4_teleop/CMakeLists.txt:27-30` `install(DIRECTORY config)` ↔ `scripts/teleop.sh:9` は `/workspace/src/app/epos4_teleop/config/params.yaml` (ソース側) を直接参照 | install される share 側コピーは誰も読まない (symlink install なので実害なし) | 2 経路 | teleop.sh を `$(ros2 pkg prefix epos4_teleop)/share/...` に、または A1 で bringup の params.yaml に一本化 | S |
| D7 | `bringup/rviz/nav2.rviz:289` `/rfans/scan_all` (39,270 本/フレーム) と `:325` `/rfans_driver/rfans_points` を表示に持つ | 10-12 に「機上 RViz が 1.4 コア食う」と判断して `use_rviz` 既定 false にしたが、`auto_run` では RViz を 35 s 後に上げる (`auto_run.launch.py:27, 123-126`)。重い表示が有効かどうか (Enabled フラグ) は未確認 → **要確認** | 当日の CPU | scan_all / points の Enabled を false 既定に、RViz 用に軽量な `nav2_onboard.rviz` を分ける | S |
| D8 | `parts/map_keepout.launch.py` (manager 2 本)、`parts/localization/loc_map.launch.py` (1 本)、`loc_amcl.launch.py` (1 本)、`parts/navigation.launch.py` (1 本) = lifecycle_manager プロセス 5 本 (+ map_odom_holder、bringup_check、auto_run_check) | 設計上の理由 (条件付き起動を manager 単位で) は妥当。ただし 800 MHz 固定問題のあった機体で常駐プロセスが増え続けている (`auto_run_check.py:43-50` が 7 ノードを個別 get_state) | 起動時 CPU・メモリ | 現状維持可。将来 `nav2_bringup` 流に 1 manager + `node_names` を OpaqueFunction で組む案を検討 | M |
| D9 | `bringup/realsense2_camera_launch/launch/rs_launch.py` (171 行) は上流 `rs_launch.py` のコピーで変更点は既定値 7 行 (`:37, 41, 43-44, 62, 64, 83` の `# claude` 行) | 上流更新のたびに追従が必要 | 保守 | 上流 launch を include して `config_file:=imu_only.yaml` を渡す形に (B6 で消す場合は不要) | S |

### E. 命名・配置

| # | 場所 (file:line) | 内容 | 影響 | 提案 | 工数 |
|---|---|---|---|---|---|
| E1 | `maps/2d/glim/`: `2026-09-180915`, `2026-09-180915_5goukan_dumpbase`, … (7 本、日付と時刻の間に `_` なし) ↔ `2026-10-09_1611_5goukan_chal`, `2026-10-03_1126_tsukuba_composite` (bags/ と同じ `YYYY-MM-DD_HHMM_slug`) ↔ `2026-08-14_N12`, `2026-09-13_5goukan` (時刻なし) ↔ `maps/2d/slam_toolbox/2026_8_11__18-20` (別書式) ↔ 既定値 `2026_6_9__22:00` (C6)。`maps/` 直下に `rfans2d.npz`, `rfans2d_slam.png`, `rfans2d_slam_imuyaw.png`, `traj_overlay.png` が裸置き | `auto_run.launch.py:77` の tag 正規表現 `^\d{4}-\d{2}-\d{2}_\d{4}_` は `_` 入りしか剥がせず、`2026-09-180915_...` 地図では bag 名が `2026-09-180915_5goukan_dumpbase_auto` になる | 命名が 4 流儀。スクリプトの前提が崩れる | `bags-layout-and-drive-policy` (メモリ) と同じ `YYYY-MM-DD_HHMM_slug` に統一し、裸ファイルは `maps/misc/` へ。maps は git 管理外なので rename は安全 | S |
| E2 | `tools/` 直下: `rfans_bag_to_liosam.py` (裸ファイル) と `99-wt901.rules` ↔ 他はすべてサブディレクトリ。udev rule は 3 か所に分散: `tools/99-wt901.rules`、`drivers/BNO086_ROS2Board-main/tools/99-bno086.rules`、hokuyo/canusb は `README.md:68, 183` の本文のみ (ファイルなし) | 新 PC セットアップで rule を 3 か所から集める | 再現性 | `setup/udev/99-rerobot.rules` 1 本にまとめ (canusb / utm-30lx / bno086 / wt901)、README は `sudo cp` 1 行に | S |
| E3 | `bringup/scripts/` (ROS ノード 7 本: bringup_check / auto_run_check / initialpose / waypoint_runner / teach_marker / map_level_bridge / map_odom_holder) ↔ リポジトリ直下 `scripts/` (ホスト用シェル 8 本) | 同名ディレクトリが 2 つで役割が逆 (ノード vs ホスト操作)。CLAUDE.md も `scripts/bringup_check.py` と `scripts/bringup2d.sh` を同じ字面で指す | 会話・文書で取り違え | パッケージ側を `nodes/` (または `rerobot_bringup/` Python モジュール化) に。CMake `install(PROGRAMS)` のパスだけ変わる | S |
| E4 | `/initialpose` 送信が 3 実装: `bringup/scripts/initialpose.py` (CLI)、`tools/amcl_compare/initialpose_relay.py` (bag 中継)、`tools/loc_eval/record_pose.py` (odom 補正つき再送)。QoS 変換も 2 方式 (`tools/loc_eval/scan_relay.py` と `qos_override.yaml`) | 似た機能が tools の実験ごとに書き起こされた | 修正の取りこぼし | initialpose は `scripts/initialpose.py` を関数化して tools から import (tools は `/workspace/tools`、scripts は install 先なので `rerobot_bringup` パッケージの Python モジュールにするのが筋) | M |
| E5 | `bringup/launch/tools/`: 本番 bringup が include する `rfans_scan.launch.py` (`rerobot_bringup.launch.py:236`) と `wt901_imu.launch.py` (`:320`) が、診断専用の `imu_check` / `amcl_variants` / `realsense_imu` と同居 | 「tools = 使い捨て」と読めるが 2 本は本番部品。`parts/` が本来の置き場 | 削除候補を選ぶとき誤って消す | rfans_scan / wt901_imu を `launch/parts/sensors/` へ移動 (`rerobot_bringup.launch.py` のパス 2 行だけ変更) | S |
| E6 | `bringup/config/amcl_variants.yaml` (config 直下) ↔ 他の自己位置推定設定は `config/localization/` | 配置が揃っていない (B9 で削除なら解消) | — | 移動 or 削除 | S |
| E7 | `drivers/BNO086_ROS2Board-main/` — GitHub zip 名の `-main` 付き、submodule でなく直置き (74 ファイル、PCB/firmware/CLAUDE.md 同梱)。CLAUDE.md は drivers を「submodule」と説明し、この dir を列挙していない | 規約と実態のずれ。上流更新を取り込む手段が無い | 保守 | 上流がリポジトリならば submodule 化して `bno086_ros2board` に rename。自作基板で上流が無いなら CLAUDE.md に「直置き・理由」を明記 | S |
| E8 | `.gitmodules` のセクション名 `src/external/*` (B14) | — | — | B14 | S |
| E9 | GLIM: `glim/config.gpu.json` (ドット区切り) ↔ `config_odometry_gpu.json` (アンダースコア); `config_sensors.flat_oldchassis_cal.json` のような多段サフィックス | 「入口ファイル」であることをドットで示す意図は `config.gpu.json:3-12` 注釈で分かるが、ファイル一覧では不統一 | 軽微 | B8 のディレクトリ分けで `presets/sensors/{flat,tilted15,...}.json` にすれば接尾辞自体が不要 | S |
| E10 | `app/epos4_controller/` パッケージに `epos4_odometry` 実行ファイル同居 (`CMakeLists.txt:33`); `bringup/realsense2_camera_launch/` はドライバ系ラッパだが `bringup/` 配下; `app/rfans_scan_modes/` は実験専用ノードだが `app/` | CLAUDE.md の「app = 自作 C++ / bringup = launch 資産 / drivers = センサ」の区分けに対して境界例が 3 つ | 軽微 | realsense2_camera_launch は drivers/ か削除 (B6)、rfans_scan_modes は B9 | S |

### 優先度つき要約 (上位 10 件)

| 順 | 項目 | 根拠 | なぜ上位か | 工数 |
|---|---|---|---|---|
| 1 | **C1** teleop の `invert_left/right` が bringup と逆 | `app/epos4_teleop/config/params.yaml:5-6` vs `bringup/config/params.yaml:13-14` | 規約 (params-sync) 違反の実バグ。台上検証の距離符号が逆 | S |
| 2 | **C2** `bringup2d.sh` が EKF を既定 off にし、launch 既定 (on) と逆。slam2d はこの経路 | `scripts/bringup2d.sh:13`, `launch/rerobot_bringup.launch.py:72`, `scripts/slam2d.sh:7` | 起動経路で TF 配信者が変わる (再現性・bag 比較に直結) | S |
| 3 | **C7** local costmap の urg: 注釈「外した」/ 値「入っている」 | `config/nav2_params.yaml:159-162` | 10-17 つくば停止対策の有無が不明 (要確認で即決できる) | S |
| 4 | **C6/B21** 既定 map_dir が不在パス (`:` 入り) ×5 + nav2_params の死に値 | `launch/nav/nav_amcl.launch.py:39` 他 | 引数なし起動が落ちる。空既定 + 必須化で解決 | S |
| 5 | **B4/C16** package.xml の依存が実装とずれ (旧 `rfans_driver` 残り、新 7 件欠落) | `bringup/package.xml:20` | rosdep で環境再現不能。Dockerfile 手書きとの二重管理の根 | S |
| 6 | **C4** imu_check が `auto_tare: all` のまま IMU 向きを検証 | `launch/tools/imu_check.launch.py:48-57` | 09-24 に 1 日潰した誤認を再現する装置になっている | S |
| 7 | **B1** `__pycache__` 20 ファイルが git 管理下 | `git ls-files` | 無害だが規約 (ホスト非実行) の反証が混入。即消せる | S |
| 8 | **B6/D4** RealSense 連鎖 (submodule ビルド + Dockerfile 層 + launch 2 本) が死に経路 | `Dockerfile_main:56-65, 80-83`, `drivers/realsense-ros` | 落ちやすいマシンでのビルド時間・イメージ容量。COLCON_IGNORE 1 ファイルで止まる | M |
| 9 | **A4/A12** nav 構成 launch 5 本の 40 行重複 + 7 か所の同文注釈 | `launch/nav/*.launch.py`, `legacy/nav2.launch.py` | 引数追加のたびに 5 ファイル。`nav_lidar3d` だけ既に揃いが崩れている | M |
| 10 | **B2/B3/C17** legacy 5 ラッパ + legacy nav2 + nav2d.sh の三つ巴 | `launch/legacy/*`, `scripts/nav2d.sh:17` | 既定値重複 (A2/A3) の源。参照ゼロの 5 本は即削除可、nav2d は auto_run に統合 | S |

次点: B13 (README の旧手順・typo)、C3 (`/dev/bno086` を誰も使わない)、B20/C19 (URDF・GLIM json の積層注釈)、D3 (Dockerfile 共通ベース層)、B9 (10-01 選点法実験の残骸、要確認)。

---

### 調査範囲と行数
- 読んだファイル: 158 本 (git 管理下、`.pyc` 除く)。本文 15,614 行 + rviz 3 本 888 行。URDF 4 本・GLIM JSON 6 本は `diff` で突き合わせ。submodule は依存名の確認 (`surestar_rfans_ros2/package.xml`, `StarROS2/package.xml`, BNO086 driver の既定 yaml) のみ参照。
- **問題なしだった領域**: URDF 3 プリセットの rfans 以外の差分 (完全一致)、`config_sensors.json` ≡ `flat.json`、`TimerAction` への `LaunchConfiguration` 直書き (違反なし、C20)、nav/parts の `GroupAction(scoped=True)` 適用 (nav 側は全 include で適用済み)、`twist_mux.yaml` と joy/Nav2 の優先度配線 (publisher 1 本の設計どおり)、`behavior_trees/navigate_through_poses_follow.xml` (ReactiveSequence 修正済み・参照パス一致)、`CMakeLists.txt` の `install(PROGRAMS)` と `scripts/*.py` の対応 (7/7 一致)、`colcon.meta` (emcl2 のみ・build.sh が `--metas` で読む)、`rfans_scan.launch.py` と bringup 間の引数受け渡し (全引数明示)、`epos4_odometry.cpp` (死にコードなし、twist を位置差分で計算する実装は CLAUDE.md と一致)、`map_odom_holder.py` / `map_level_bridge.py` / `waypoint_runner.py` (重複・残骸なし)、`stop.sh` (glim_gpu_env がコンテナ一覧に無い点だけ軽微な漏れ: `scripts/stop.sh:8`)。

---

## 3. 詳細 (B) ドキュメント・Claude 設定 — 調査 subagent 全文



調査日: 2026-10-23。対象リポジトリ `/home/nomface/reRoBot` (読み取り専用、変更なし)。
根拠は `file:line` で示す。確認できず推測にとどまるものは **要確認** と明記した。

---

### 1. CLAUDE.md

#### 1.1 規模

| 指標 | 値 |
|---|---|
| 総行数 | 199 行 (空行除き 161 行) |
| 日付 (`2026-MM-DD` / `MM-DD`) を含む行 | **38 行** (全 161 非空行の 24%) |
| 1 行 2,000 文字超の「段落行」 | 3 行 (L141 ≈ 1,900 字、L143、L151) — 1 行に 10 件以上の事実が詰まっている |

#### 1.2 規約行 / 履歴行の内訳 (非空行 161 を分類)

| 分類 | 行数 (概算) | 例 |
|---|---|---|
| **A. 規約として残すべき** (ビルド規則・命名規約・禁止事項・作業慣例) | 約 70 | L37-39 (ビルドの鉄則)、L95 (裸の `/scan` 禁止)、L165-168 (namespace・スケーリング・`ros__parameters`)、L192-199 (作業慣例) |
| **B. 現状説明として残すが「いつ変えた」は不要** (日付を削れば規約) | 約 45 | L11 (`2026-07-26 再編`)、L41 (`旧 symlink 方式は 2026-07-26 に撤廃`)、L62 (`2026-10-15 新設、10-16 にパッド運用化`)、L69/L72 (`2026-10-08 から…`) |
| **C. 履歴・調査結果 — PROJECT_STATE / docs/issue / features に移すべき** | 約 35 | 下表 |
| D. コマンド例・図 | 約 11 | L23-35, L50-60, L112-127 |

#### 1.3 移すべき行の例 (行番号付き、7 件)

| 行 | 内容 | 移動先 |
|---|---|---|
| **L139** | `joint_states.velocity` が常に 0 であることを「2026-08-11 の実走 bag で実証: pose が 20 m 動く間 twist は常に 0」と bag の証拠まで書く | 規約部分 (「velocity は常に 0、位置差分で計算」) は残し、実証経緯は PROJECT_STATE §7-2 (既に同内容あり) に任せる |
| **L141** (約 1,900 字) | bringup launch の説明に `imu_rate` の変更履歴 (`2026-09-24 に 100 から変更`)・`実機 199.9 Hz 確認 2026-09-13`・`WT901C が bno086.yaml を拾った実例あり`・`2026-10-16` の TimerAction 罠 まで同居 | 現在値だけ残す。罠 2 件は PROJECT_STATE §7 に既にある (L259 TimerAction) ので重複削除 |
| **L147** | `vangle_override は 2026-09-13 に有効化 (リング 0〜5 = 09-02 実測、6〜15 = 公称。odometry 段の傾き −20%、global 地図には中立。z ドリフトの主因ではない)` | 調査結論は `docs/issue/2026-09-10_glim_z_drift_not_vangle.md` の担当。CLAUDE.md には「`vangle_override` がある」だけでよい |
| **L148** | IMU 向きの確定経緯「`2026-08-11, 実走 bag の 3 証拠: 静止 accel z=+9.815 / gyro_z⇔車輪 yaw rate 同符号 +1.015 / 裏返し URDF だと EKF yaw が鏡像`」 | PROJECT_STATE §4 (L104-110 に同内容) — しかも **L141 と矛盾** (後述 1.4-①) |
| **L151** (約 1,500 字) | 追従 BT の設計詳細 (`{new_path}`・`have_path`・車間 0.6/0.3 m・連続失敗 20 回・`2026-10-17 修正` の経緯・`bags/exp/2026-10-17_tsukuba_wander/`) | `docs/features/2026-10-16_follow_bt_narrow_corridor.md` が設計文書。CLAUDE.md には「BT は follow 版、設計は features」の 1 行で足りる |
| **L160** | GLIM `acc_scale` の値の変遷 (`0.9705 (09-13) → 0.0 (09-27)`、`較正前 (〜09-25) の新車体 bag は 0.9705`、`旧車体 bag 用 preset`) | 再処理時の値の選び方は `docs/issue/2026-09-10_…md §9.4/§10` に既にある (本文も参照している)。CLAUDE.md は「現在値 0.0、過去 bag の値は issue §10」で足りる |
| **L170** | `gear_ratio: 5.0 は物理ギヤ比 … 2026-07-29 に根本修正済み — それ以前は … 1.25 で相殺` | **現状と食い違い** (1.4-④)。経緯は `docs/issue/2026-07-07_wheel_odometry_encoder_scaling_4x.md` にある |
| L199 | critic 導入の背景「08-14〜16 の『縦角表が本命』が 3 文書で相互に根拠化され、09-10 の因果実験で棄却」 | 同じ話が `.claude/agents/critic.md:97-102` と `.claude/skills/critic/SKILL.md:11,53-59` にもある (3 重)。CLAUDE.md は 1 文で足りる |

#### 1.4 現状と食い違う記述 (確認済み)

| # | 行 | 記述 | 実態 | 根拠 |
|---|---|---|---|---|
| ① | **L148** vs **L141** | L148「IMU の向きは **rpy=(0, 0, π/2) (正立+90°) で最終確定**」、L141「URDF の imu_joint は **上下逆 + yaw −93° = rpy (π,0,−1.6257)**」 | **同一ファイル内で矛盾**。URDF の実値は後者 | `urdf/rerobot.urdf:167` `rpy="3.14159 0 -1.625736"` |
| ② | **L141** | `lidar_2d → urg_node (frame_id laser, /scan)`、末尾「`/scan` は 2D LiDAR のみ (3D 点群の LaserScan 変換は含まない)」 | 2026-10-08 以降 urg は `/urg_front/scan`、bringup は `rfans_scan` 既定 true で `/rfans/scan` も出す。**L95・L143 の記述と矛盾** | `launch/rerobot_bringup.launch.py:228` remap、`:77` `rfans_scan default true` |
| ③ | **L141** 末尾 / L150 | 「visualization is owned by `nav2.launch.py` / `slam.launch.py`」「`rviz/nav2.rviz` … Launched by `nav2.launch.py`」 | `nav2.launch.py` は legacy/ へ移動済み (L71-72, L144 自身が書いている)。RViz は `parts/rviz_nav.launch.py` が担当 | `launch/legacy/nav2.launch.py` 存在、`launch/parts/rviz_nav.launch.py` 存在 |
| ④ | **L170** | 「`gear_ratio: 5.0` は物理ギヤ比」 | 現在値は **92.25** (2026-08-30 減速機交換) | `config/params.yaml:11,36`、`epos4_teleop/config/params.yaml:4` |
| ⑤ | **L100** | 個別 launch 一覧に `nav2.launch.py` を「main コンテナ: map_server + amcl + Nav2 …」と現役扱いで列挙 | legacy。`legacy/README.md` が「新しく使う理由はない」 | `launch/legacy/README.md:3` |
| ⑥ | **L188** | params-sync を「車体パラメータ **3 ファイル**の整合検査」 | skill 本体は **2 ファイル** (L169 自身も 2 ファイルと書く) | `.claude/skills/params-sync/SKILL.md:3,7-8,25-28` |
| ⑦ | **L190** | MCP `memory` (`~/.claude/rerobot-mcp-memory.json`) | 当該ファイルは存在せず、`npx` も PATH に無いため memory/playwright は接続失敗 (このセッションで観測)。**要確認** (別マシンでは動くかもしれない) | `ls ~/.claude/rerobot-mcp-memory.json` → No such file |
| ⑧ | **L174** | 読み順に `docs/claude/USER_LEVEL.md` を必須のように列挙 | git 管理外で、このチェックアウトには **存在しない**。skills 側は「無ければ較正なしで進めてよい」と書いており、CLAUDE.md だけ任意性が書かれていない | `.gitignore:12`、`knowledge-check/SKILL.md:28` |
| ⑨ | L52-54 | `bringup2d.sh` 「IMU=true EKF=true を前置すると…」、`nav2d.sh` 「bringup2d (IMU+EKF 標準) + Nav2」 | L62 で本番起動は `auto_run.launch.py` 1 本。`nav2d.sh` は legacy の `nav2.launch.py` をまだ使っている (legacy/README) — scripts 行と auto_run 行のどちらが推奨か読者に伝わらない | `launch/legacy/README.md:12` |

#### 1.5 同じことを 2 回言っている箇所

- **ekf の既定値**: L141 「`epos4_odometry` の publish_tf は launch 側で自動 false (TF 二重配信防止。**既定 false**)」と L143 「**ekf / rfans_scan 既定 true**」。ekf の既定が false か true か、2 箇所で逆 (要確認: launch 実装では `ekf` 引数の default を見る必要あり。少なくとも記述は不一致)。
- **legacy ラッパ**: L71-72、L142、L143 末尾「旧 `rerobot_bringup_*.launch.py` ラッパは削除候補として残置」の 3 回。
- **`/odometry/filtered` が Nav2 の odom 入力**: L144 と L54。
- **`joint_states.velocity` = 0**: L139 と L167 (後者は「`joint_states.velocity` is rad/s」とだけ書き、0 である注意が無い — 読者が混乱する)。
- **submodule bus.yml の変更手順**: L156 と PROJECT_STATE §7-7 (`src/external/epos4compact50-5can` と旧パスのまま)。
- **critic 背景**: L199 / agents/critic.md / skills/critic/SKILL.md (1.3 最終行)。

---

### 2. PROJECT_STATE.md

#### 2.1 規模と構造

| 指標 | 値 |
|---|---|
| 行数 / バイト数 | **408 行 / 210 KB** (CLAUDE.md の約 6 倍の容量) |
| 1 行 2,000 字超 | 13 行。**L6 = 20,678 字、L7 = 18,509 字** (冒頭「最終更新」行に 10-16 → 09-14 までの更新履歴が 1 行で連結されている) |
| 日付タグを含む行 | 216 / 408 行 |
| 節構成 | §1 正体 / §2 HW (「2026-07 時点」のまま) / §3 SW 全体像 / §4 到達点 (L57-127) / §5 既知の問題 (L128-160) / §6 タイムライン (L161-257、約 95 行) / §7 罠 (L258-281) / §8 docs 地図 / §9 作業慣例 / §10 次の作業 (L307-408、約 100 行) |

#### 2.2 問題点

**(a) 冒頭 L6-L7 が「更新履歴の全文連結」になっている** — `最終更新: 2026-10-16 (10-16 (3): … 以下 10-16 (2)。10-16 (2): … 以下 10-12 (2)。10-12 (2): …` と、本来 §6 タイムラインに置くべき要約が冒頭行に **40 KB** 積まれている。読み出し時に最初の 2 行で 4 万字を消費する。`project-state` skill の手順 3「冒頭の `最終更新` 日付と『時点』の説明も更新する」(`.claude/skills/project-state/SKILL.md:33`) を各セッションが「追記」と解釈した結果と見られる (**要確認**: 意図的に残しているのかもしれない)。

**(b) §6 タイムラインが docs/issue と重複し、かつ順序が崩れている** — §6 の各行は 1,000〜4,000 字で、該当 issue の冒頭ステータス行とほぼ同じ内容 (例: L226 `08-14 (4)` 4,487 字 ↔ `docs/issue/2026-08-14_rfans_driver_renewal.md`)。日付順も `08-12 (4) → 08-12 (5) → 08-12 (3) → 08-12 (4) → 08-12 (3) → 08-11 (5) → 08-12 (2) …`、`09-04 → 08-31 (2) → 07-31 → 09-10` と前後している (L186-196, L213-215)。タイムラインとして追えない。

**(c) §5 既知の問題の表が docs/issue と不整合** — 30 本中 **10 本が未掲載**、掲載分も 5 本が古い:

| issue | §5 の記述 | issue 側の実態 | 備考 |
|---|---|---|---|
| `2026-09-24_bno086_auto_tare…` | 「対策**未適用**」(L131) | 「対策**適用済み**・物理向き実測済み・accel 再較正済み (09-26)」(issue L4) | §4 L116 では「`auto_tare: off` 適用」と書いており **§4 と §5 で矛盾** |
| `2026-09-26_rfans_frame_stamp…` | 「live **未確認**」(L130) | 「解決 (09-27 **live 確認済み**)」(issue L4) | |
| `2026-08-14_rfans_driver_renewal` | 「新ドライバ完成 / **統合待ち**」(L148) | 「統合完了 — 全面切替済み」(issue L3) | §3 L52 も「surestar_rfans_ros2 は … **COLCON_IGNORE 中**」と逆 (実際は StarROS2 が IGNORE) |
| `2026-08-11_utm30lx_usb_instability` | 「調査中: 現在 USB バス上に不在」(L145) | issue も「調査中」のまま。だが UTM-30LX は現在 `/urg_front/scan` として常用 (bringup 既定 true) | **issue も §5 も未クローズ** (3.1 参照) |
| `2026-08-16_rfans_mount_angle…` | 「原因特定済み: 15° = 縦角系統誤差の z 投影増幅」(L147) | issue 冒頭に 09-10 追記「縦角表が本命」は棄却 | §5 が棄却前の結論のまま |
| 未掲載 10 本 | — | `2026-07-07_rfans_driver_calc_review`, `2026-08-01_odometry_stop_wobble`, `2026-08-15_9goukan_building_wall_doubling`, `*.summary.md` ×2, `2026-10-01_amcl_hedge…`, `2026-10-02_scan_ground_ghost…`, `2026-10-08_glim_gpu_cuda…`, `2026-10-11_loc_eval…`, `2026-10-16_amcl_lateral_bias…`, `2026-10-22_epos_fault_8250…` | 10 月の issue 6 本は §5 に 1 本も無い (10-15 のみ) |

**(d) §5 末尾「優先度の要約」(L155-159) が 07 月時点のまま** — 「🔴 安全系 (監査 Issue 1〜3): `/robot_speed_cmd` ウォッチドッグなし」とあるが、ウォッチドッグは **09-19 に実装・実機検証済み** (`docs/features/2026-09-19_epos4_controller_watchdog_fault_monitor.md`、§10 L314 自身が ✅ と書く)。「🟡 LiDAR FOV ±90°→±135° (A1)」「ROS_DOMAIN_ID=150 → 101 以下へ」も進捗不明のまま。

**(e) §10 次の作業 (約 100 行) に完了済み・前提消失の項目が積層** —

| 行 | 項目 | 状態 |
|---|---|---|
| L321-330 | 「**最優先: リポジトリ再編の仕上げ** … `feat/workspace-split` を main にマージして再編クローズ。空 `ros2_ws_nav2/` の削除」 | 07-26 の話。main 直コミット運用 (§9) になっており、ブランチは存在しない (git status は main のみ)。**3 か月前の「最優先」が残存** |
| L316-320 | 「⚠️ 割込み最優先: 車体改修 (08-30) の追従残タスク — これを済ませるまで実走系の検証は全て無効」 | 08-31 に (a)(b) 完了、以後 10 月まで実走多数。(c)(d)(e) の消し込み記録なし |
| L314 | 「🔴 割込み最優先 (09-19): … 走行は 0.25 m/s turbo なし」 | 10 月は 0.4 m/s で自律走行している (10-22 issue L5)。制限解除の記録なし |
| L331-345 (項目 3,4) | GLIM 水平ドリフト E11 反映判断、「残: この構成での Nav2 実走行確認」 | 以降の実走で事実上完了。各項目に「→ 09-xx」「→ 10-xx」の追記が 10 段以上連なり (L350-362)、1 項目が 2,000 字超 |
| L404 | 「比較用 IMU (09-23): 次回実機で (1) udev … (4) GLIM run」 | 09-24〜27 に完了 (§10 L396 自身「✅ IMU 前提整備 (09-24→27 完了)」) |
| L406-408 | 「従来のトリアージ指示書 … は再編完了後に再開」A1〜B5 | 07 月の計画。多くは別経路で解決 (B1 済、B3 は EKF、B5 LIO-SAM は凍結) |

**(f) §1〜§3 の「安定セクション」にも古い記述** — §1 L17 「この Windows チェックアウトは閲覧/文書作業用。symlink が壊れておりビルド不可」(現在は Ubuntu ホスト)、§2 見出し「(2026-07 時点)」、§2 L27 「IMU … **採用予定** (08-02 受領)」(採用済み)、§2 L30 「比較用 IMU … URDF 未登録」(CLAUDE.md L148 は `imu_wit` 位置を −8.5 mm 調整済みと記述 → 登録済み)、§3 L52 (上記 COLCON_IGNORE)、§4 L63-65 「`/sdk_could` 開通を再確認」(廃止 topic)、§4 L83 「3D 構成 (`rerobot_bringup_3d.launch.py`): /scan が無いので SLAM/Nav2 に繋がらない」(rfans_scan 同梱済み)、§7-7 L274 「`src/external/epos4compact50-5can`」(旧パス)。

**(g) §8 docs 地図が CLAUDE.md の表 (L176-186) と重複し、しかも両方とも不完全** — §8 は `docs/text/` を「現在: timestamp・ekf_fusion・ros2_cpp (各 8 ファイル)」と 3 冊で止まっている (実際は 7 冊)。`docs/claude/knowledge/`・`docs/claude/handoff/`・`docs/reference/`・`docs/trial/`・`docs/manual/` の行が無い。CLAUDE.md 側は `docs/reference/`・`docs/claude/handoff/`・`docs/claude/knowledge/`・`*.summary.md` が無い。`knowledge-check/SKILL.md:56-57` の「初回実行時は §8 に knowledge/ の行を追記」は 08-11 の初回実行後も未実施。

---

### 3. docs/issue ステータス衛生

凡例: 冒頭ステータス = ファイル先頭 10 行以内に「ステータス:」行があるか。

| ファイル | 冒頭ステータス | 本文の実態 | PROJECT_STATE §5 の記述 | 要対応 |
|---|---|---|---|---|
| `2026-07-07_monthly_2026_6_todo_triage.md` | 未解決 (L4) | T1/T3/T9 解決、残りは別経路で陳腐化 (B3→EKF、B5 凍結)。07-29 以降更新なし | 「TODO 14 件の原因分析」 | 「アーカイブ (07 月時点の分析)」に変更、または項目別に消し込み |
| `2026-07-07_monthly_2026_6_todo_triage.prompt.md` | **なし** (作業指示書) | 「次エージェント用」の残骸。B1 のみ ✅ (L73)。再開予定 (PS §10 L406) は実態なし | 「次エージェント用の作業指示書」 | **削除候補** (または handoff/ へ移して「完了/放棄」を明記)。`.prompt.md` は docs/issue の「1 issue = 1 ファイル」規約外 |
| `2026-07-07_repository_audit.md` | **なし** (一覧形式、各 Issue に「状態」) | 25 件 **全部「未解決」のまま** (L48-296)。Issue 1 (ウォッチドッグ) は 09-19 実装済み、Issue 16/17 は PS L55 が「解消済み」と書く | 「全体監査の 25 issue 一覧」、優先度要約で Issue 1〜3 を 🔴 のまま | 少なくとも Issue 1・16・17 を解決済みに。他も棚卸し |
| `2026-07-07_rfans_driver_calc_review.md` | 未解決 (L4) | 対象は旧ドライバ StarROS2 (`rerobot_bringup_3d.launch.py` + `params_3d.yaml`)。08-14 のドライバ刷新で**対象コードが廃止** | §5 に**未掲載** | 「対象廃止 (08-14 刷新)」でクローズ |
| `2026-07-07_wheel_odometry_encoder_scaling_4x.md` | 解決 (L4) | 「残: 接地での 10 m 直進 / 360° 旋回」→ 08-11 EKF テストで 10.08 m / 360° 実施済み (PS §10 L339-340) | 解決済み (07-29) 残: 接地確認 | 残タスクを消し込み完了に |
| `2026-08-01_odometry_stop_wobble…` | 解決 (L4) | 整合 | **未掲載** | — |
| `2026-08-11_joy_spin_epos_shutdown_usb_stall.md` | 未解決 (L4) + 09-19 追記 | 機構は 09-19 issue に統合済み | 「対策適用済み・再発観察中」 | 09-19 issue へ統合して「重複・参照のみ」に |
| `2026-08-11_utm30lx_usb_instability.md` | 調査中 (L4) | UTM-30LX は現在常用 (`/urg_front/scan`、bringup 既定 true)。08-11 以降更新なし | 「調査中: USB バス上に不在」 | **クローズ** (復旧経緯は要確認) |
| `2026-08-12_glim_horizontal_drift.md` | 調査完了・根因特定 (L4) | 整合。根治未達のまま GLIM は実用化 | 「原因特定済み・根治未達」 | 「現状運用で許容」等の終端を書く |
| `2026-08-13_glim_param_tuning.md` | 実験完了 (L4) | E11 反映は「ユーザ判断待ち」のまま 2 か月 | 「repo config 未反映 (ユーザ判断待ち)」 | 判断結果 (採用/不採用) を書いてクローズ |
| `2026-08-14_rfans_driver_renewal.md` | 統合完了 (L3) | 整合 (+09-10 追記あり) | 「**統合待ち**」← **古い** | PS §5 を更新 |
| `2026-08-15_9goukan_building_wall_doubling.md` | 原因特定済み (L4) + 09-10 追記で先頭仮説棄却 | ステータス行が棄却前の文のまま | **未掲載** | ステータス行を書き換え |
| `2026-08-15_offline_viewer_zenity…` | 解決済み (L4) | 整合 | 解決済み | — (report/ 相当の内容。issue に残す理由は薄い) |
| `2026-08-16_rfans_mount_angle_glim_z_collapse.md` | 原因特定済み「縦角表が本命」(L4) + 09-10 追記「主犯ではない」 | **ステータス行と追記が矛盾** | 棄却前の結論のまま | ステータス行を書き換え |
| `2026-08-30_chassis_rework_followup.md` | ほぼ完了 (L4) | 残 ④⑤ は以後の実走で実質完了 | 「未完 (一部消し込み済み)」 | クローズ |
| `2026-09-02_rfans_scan_motor_dropout.md` | 調査中 (L6) | 整合。10-22 issue で再発 (10-22 L7) — 本体への追記なし (09-23 以降更新なし) | 調査中 | 10-03 / 10-22 の再発を追記 |
| `2026-09-02_…summary.md` | (要点版、L24 に状態表) | 「最終更新 2026-09-22」。本体 09-23 更新に追従なし | **未掲載** (§8 で一般論のみ) | 本体と同期 |
| `2026-09-10_glim_z_drift_not_vangle.md` | 調査完了 (L6) + 追記 3 段 | 956 行。ステータス行が 09-13 のまま、以後は追記で延命 | L148 が 09-23 までの要約 | ステータス行を最新結論に書き直し |
| `2026-09-10_…summary.md` | (要点版) | 「最終更新 2026-09-24」、本体は 10-01 更新 | **未掲載** | 本体と同期 |
| `2026-09-13_decel_epos_stop_12v_loss.md` | 調査中 (L6、ただし L4-5 に 📌 追記が先行) | 09-19 配線 v2 化で「再現・検証不能」 | 「前提消失・凍結」 | **クローズ (再現不能)** に変更 |
| `2026-09-19_epos_stop_link_loss…` | 調査中 (L4) | 対策 3 点は実装済み (features 09-19)。10-22 で同コード再発 | 調査中 | 10-22 への参照を追記 |
| `2026-09-24_bno086_auto_tare…` | 原因特定・対策適用・再較正済み (L4) | 整合 | 「**対策未適用**」← **古い** | PS §5 を更新 |
| `2026-09-26_rfans_frame_stamp…` | 解決 (L4)。「submodule 未コミット」 | PS §10 L396 も「残作業: submodule コミット」。**要確認** (コミットされたか) | 「live **未確認**」← **古い** | PS §5 を更新 |
| `2026-10-01_amcl_hedge_corridor…` | 調査中 (L4) | 10-11/10-16 issue で後継の結論 (yaw オフセット・alpha) が出ている | **未掲載** | 後継 issue への参照を追記 |
| `2026-10-02_scan_ground_ghost_rouka_vangle.md` | 調査中 (L4) | 整合 | **未掲載** | — |
| `2026-10-08_glim_gpu_cuda…` | 回避策で解決 (L4) | 整合 | **未掲載** | — |
| `2026-10-11_loc_eval_2d_corner_loss…` | 調査中 (L4) | 整合 | **未掲載** | — |
| `2026-10-15_keepout_corridor_stall…` | 原因特定+対策実施 (L4) | 整合 | 整合 (唯一の 10 月掲載) | — |
| `2026-10-16_amcl_lateral_bias…` | 解決 (L4) | 整合 | **未掲載** | — |
| `2026-10-22_epos_fault_8250…` | 調査中 (L4) | 整合。handoff の作業 (監視修正) は未着手 (4.3 参照) | **未掲載** | — |

**ステータス行の書式揺れ**: `- **ステータス: …**` (箇条書き、07〜09 月) と `**ステータス: …**` (段落、10 月) の 2 系統。`2026-09-13` は 📌 追記 2 行がステータス行より上にあり、先頭 10 行の規約から外れる。一覧ファイル 2 本 (`repository_audit`, `todo_triage.prompt`) は冒頭ステータス自体が無い。

---

### 4. ディレクトリ構成の提案

#### 4.1 現状ツリー (役割の重なりを注記)

```
docs/
├── claude/
│   ├── PROJECT_STATE.md          ← 状態メモ。§6 タイムライン・§10 次の作業が issue と二重管理
│   ├── handoff/  (1 本)          ← 別エージェント向け指示書。docs/issue/*.prompt.md と同じ役割 (置き場が 2 つ)
│   └── knowledge/ (1 本)         ← 理解確認の記録。08-11 以降増えていない
├── issue/   (30 本 + img/ 7 dir) ← 未解決調査記録。実際は「解決済み」「一覧」「指示書」「要点版」が混在
├── report/  (11 md + 2 pdf)      ← 解決済み事後報告。pdf 2 本は同名 md の重複 (05-26, 06-04)
│                                    `2026-09-26_bno086_feedback_to_developer.md` は外部向け申し送りで report でない
├── features/ (11 本 + img/ 1 dir) ← 設計文書。`2026-09-23_wt901c…` は手順書、`2026-10-02_2dmap_threshold…` は実験記録
├── reference/ (md 1 + mmd 1)     ← 機体外設定の記録。manual 第 3 章 (パラメータ) と役割が近い
├── trial/   (3 本)               ← improvements.md はユーザ執筆、他 2 本は Claude の議論メモ (同居)
├── manual/  (00_index + 15 章 + memo + img/) ← ユーザ執筆の運用手引き
├── text/    (7 冊)                ← 教材。waypoint_nav/ だけ 1 ファイル・00_index 無し・内容は「仕様書」
└── monthly/ 2026_6 (拡張子なし)   ← ユーザ月報。`/attachment/...` 画像リンク 9 本は外部 (Notion 等) 由来で切れている
```

**役割の重なり**

| 組 | 重なり | 根拠 |
|---|---|---|
| `claude/handoff/` ↔ `issue/*.prompt.md` | どちらも「別エージェントへの作業指示書」 | `handoff/2026-10-22…md:1` (「完了したら消してよい」)、`issue/2026-07-07_…prompt.md:1-3` |
| `claude/PROJECT_STATE.md §6/§10` ↔ `issue/` 冒頭ステータス | 同じ結論を 2 か所で更新する運用 → §5/§10 が issue に追従できていない (2.2-c, e) | — |
| `reference/` ↔ `manual/03_parameters.md` | 「現在の設定値の記録」。reference の EPOS4 NVM 設定は manual 第 1 章の速度表からもリンクされている | `manual/01_overview.md:12` |
| `trial/` ↔ `monthly/` | どちらもユーザ執筆の「所感・振り返り」。ただし trial に Claude 作成の分析 2 本が同居 | `trial/2026-10-06…md:1-2` 「議論用のまとめ」 |
| `issue/` ↔ `report/` | 解決済みの issue (`08-15_offline_viewer`, `09-26_rfans_frame_stamp`, `10-16_amcl_lateral_bias`) は内容的に report。report 側は 09-02 以降 1 本 (申し送り) のみで、解決記録が issue に留まる傾向 | 各 issue 冒頭 |
| `features/2026-09-23_wt901c…` ↔ `manual/` | 「手順書」(自称、L3) | `features/2026-09-23…md:3` |

#### 4.2 日付接頭辞の形式揺れ

| 形式 | 使用箇所 |
|---|---|
| `YYYY-MM-DD_slug` | issue / report / features / reference / trial / handoff / knowledge (主流) |
| `YYYY_M` (拡張子なし) | `docs/monthly/2026_6` |
| `memo_YYYY_MM_DD` | `docs/manual/memo_2026_10_17.md` (manual は章番号 `NN_` が規約なのに例外) |
| img サブディレクトリ名が本体 md と不一致 | `issue/img/2026-08-12_glim_drift` ↔ `2026-08-12_glim_horizontal_drift.md` / `img/2026-08-14_rfans_glim_eval` ↔ `2026-08-14_rfans_driver_renewal.md` / `img/2026-08-15_9goukan_double_walls` ↔ `…_building_wall_doubling.md` / `img/2026-08-16_mount_angle` ↔ `…_rfans_mount_angle_glim_z_collapse.md` |

#### 4.3 画像ディレクトリの分散

`docs/issue/img/` (7 dir, 111 ファイル)、`docs/features/img/` (1 dir)、`docs/manual/img/` (2 ファイル)、`docs/text/map3d_to_nav2/img/` (6 ファイル) の 4 か所。参照はすべて本体と同じディレクトリ配下の相対パスで整合している (切れリンク無し)。分散自体は GitHub 表示上は問題ないが、`issue/img/2026-08-13_glim_param_tuning/` だけ 33+7 ファイルあり、issue 全体の画像の 4 割を占める。

#### 4.4 提案ツリー

```
docs/
├── README.md                     ★新設: docs/ 全体の入口 (各 dir の役割 1 行 + 索引へのリンク)
├── claude/
│   ├── PROJECT_STATE.md          ← §1-5, §7-10 のみ (≤ 150 行目安)。§6 は timeline.md へ分離
│   ├── TIMELINE.md               ★ §6 を分離、日付昇順に並べ直す。1 行 ≤ 200 字、詳細は issue へリンク
│   ├── handoff/                  ← issue/*.prompt.md をここへ (完了したら削除 or `done_` 接頭辞)
│   └── knowledge/
├── issue/
│   ├── INDEX.md                  ★ 自動生成可: ファイル | ステータス | 最終更新 (3 章の表がそのまま種)
│   ├── (未解決・調査中のみ)
│   ├── closed/                   ★ 解決・前提消失・対象廃止の issue を移動 (リンクは相対パス修正要)
│   └── img/<同名 slug>/          ← dir 名を本体 md の slug に揃える
├── report/
│   ├── INDEX.md                  ★
│   └── (pdf 2 本は削除 — md が正。外部申し送り `09-26_bno086_feedback` は reference/ か features/ へ)
├── features/  (+ INDEX.md ★)      ← `09-23_wt901c` の手順部分は manual へ、`10-02_2dmap_threshold` は issue/closed か trial へ
├── reference/                    ← そのまま (EPOS NVM・配線図)。`wiring_diagram.mmd` は v1 のまま (09-19 issue L8 「v2 は未転記」) → 要更新
├── trial/
│   ├── improvements.md           ← ユーザ執筆 (そのまま)
│   └── analysis/                 ★ Claude 作成の 2 本をサブディレクトリへ (ユーザ領域と分離)
├── manual/                       ← そのまま。memo_2026_10_17.md は `16_waypoint_editor.md` 等の章番号へ、または tools/README へ統合
├── text/
│   ├── README.md                 ★ 7 冊の一覧 (PROJECT_STATE §8 の「3 冊」記述を置き換える)
│   └── waypoint_nav/             ← 00_index + 章分割に揃えるか、「仕様書」として features/ へ移す (内容は 1 機能の仕様)
└── monthly/
    └── 2026-06.md                ← 拡張子付与・形式統一 (ユーザ領域なので提案のみ)
```

**移動するファイルのリスト (提案)**

| 現在地 | 移動先 | 理由 |
|---|---|---|
| `docs/issue/2026-07-07_monthly_2026_6_todo_triage.prompt.md` | `docs/claude/handoff/` (または削除) | 指示書。issue 規約外 |
| `docs/issue/2026-07-07_rfans_driver_calc_review.md` | `docs/issue/closed/` | 対象コード廃止 |
| `docs/issue/2026-08-01_odometry_stop_wobble…`, `2026-08-15_offline_viewer…`, `2026-09-26_rfans_frame_stamp…`, `2026-10-16_amcl_lateral_bias…`, `2026-07-07_wheel_odometry…`, `2026-08-30_chassis_rework…`, `2026-09-13_decel_epos_stop…` | `docs/issue/closed/` | 解決 / 前提消失 |
| `docs/report/*.pdf` (2 本、1.3 MB) | 削除 | 同名 md と重複 |
| `docs/report/2026-09-26_bno086_feedback_to_developer.md` | `docs/reference/` | 外部向け申し送り、デバッグ報告ではない |
| `docs/trial/2026-10-03_…`, `2026-10-06_…` | `docs/trial/analysis/` | ユーザ執筆領域との分離 |
| `docs/manual/memo_2026_10_17.md` | `docs/manual/16_waypoint_editor.md` | 命名規約 |
| `docs/text/waypoint_nav/waypoint_nav.md` | `docs/features/2026-10-04_waypoint_runner_spec.md` または章分割 | 教材でなく仕様書 |
| `docs/issue/img/{2026-08-12_glim_drift, 2026-08-14_rfans_glim_eval, 2026-08-15_9goukan_double_walls, 2026-08-16_mount_angle}` | 本体 md と同じ slug に rename | 対応関係を名前で追えるように |
| `maps/rfans2d.npz`, `maps/*.png` (08-12、4 ファイル) | `bags/exp/` か削除 | maps/ 直下の作業残骸 (**要確認**: 参照元不明) |

---

### 5. skills / hooks

#### 5.1 hooks (`.claude/settings.json` → `.claude/hooks/check-project-state.sh`)

- スクリプトは実在し実行権あり。しかし監視対象 `code_paths=(src Dockerfile docker-compose.yml .gitmodules .mcp.json)` (`check-project-state.sh:63`) のうち **`src` と `Dockerfile` はリポジトリ直下に存在しない** (07-26 再編で `ros2_ws_*/src/`・`docker/Dockerfile_*` に移動)。`git status --porcelain -- src Dockerfile` と `git log -- src Dockerfile` は空を返すので、hook は事実上 `docker-compose.yml`・`.gitmodules`・`.mcp.json` の 3 ファイルしか見ていない。**コード変更 (ros2_ws_main/src/…) では一切発火しない**。これが PROJECT_STATE の §5/§10 が追従しない一因と推定 (**要確認**: Stop hook のログで発火履歴を見る)。
- `settings.local.json` は `enabledMcpjsonServers` のみ。問題なし。
- `.mcp.json`: `playwright` と `memory` は `npx` 依存で、この環境では接続失敗 (1.4-⑦)。

#### 5.2 SKILL.md の陳腐化 (参照先・値が現状と違う)

| skill | 行 | 記述 | 実態 |
|---|---|---|---|
| `verify` | L29 | ビルドコマンドに `--metas ./src/colcon.meta` が無い | CLAUDE.md L43 は emcl2 のため必須と明記。素のコマンドでは emcl2 が GCC 13 で落ちる |
| `verify` | L29 | `MAKEFLAGS`/`BUILD_JOBS` の並列制限なし | CLAUDE.md L39「並列度は控えめに」。handoff L196 の例は `MAKEFLAGS=-j2` を付けている |
| `verify` | L44 | 実機モードで `rerobot_bringup_2d.launch.py` を起動 | legacy。現行は `rerobot_bringup.launch.py` |
| `verify` | L107-108 | 合格例「`-15` (≒ 車輪 12.7 rpm × **gear 1.25**)」 | gear_ratio は 92.25 (07-29 に 5.0、08-30 に 92.25)。期待値が 2 世代古い |
| `verify` | L54, L138 | 「PROJECT_STATE §9」「§7-1」を規約の出典にしている | 規約の正は CLAUDE.md (L195) のはず。PROJECT_STATE §9 は CLAUDE.md L192-199 の重複 |
| `stack-health` | L59 | 3D topic を `/sdk_could` 約 20 Hz と記載 | 08-14 に廃止。現行 `/rfans_driver/rfans_points` 10 Hz |
| `stack-health` | L58 | `/scan` (2D) 約 40 Hz | 10-08 から `/urg_front/scan`。`/rfans/scan`・`/imu/data`・`/odometry/filtered` の確認項目が無い |
| `stack-health` | L55-56 | joint_states 約 20 Hz (PDO sync 50 ms) | handoff L218 は bus.yml「SYNC 10 ms」と記載。CLAUDE.md L166 は 50 ms。**どちらが現行か要確認** (bus.yml は submodule) |
| `stack-health` | L28, L49 | 「`rfans_driver` 系なら 3D」「2D / 3D どちらの構成か」 | 現行 bringup は全部入り 1 本 (2D+3D 同時)。二者択一の前提が崩れている |
| `params-sync` | L55-56 | 「**`gear_ratio` の正値は 5.0**」 | 92.25 (`params.yaml:11`)。この skill を今走らせると「5.0 でない」を異常と報告しかねない |
| `textbook` | L84 | 完了後「CLAUDE.md の docs 表・PROJECT_STATE のタイムラインに反映」 | PS §8 は 3 冊止まり (7 冊ある) → 手順が守られていない |
| `knowledge-check` | L56-57 | 「初回実行時は PROJECT_STATE §8 に knowledge/ の行を追記」 | 08-11 の初回以後も §8 に行が無い |
| `knowledge-check` | L31 | 「ユーザは『なぜそうなるか』まで求める (**PROJECT_STATE §9**)」 | 規約の出典は CLAUDE.md L196 であるべき |
| `critic` | L27-29 | 「同じセッション内で新設した直後は `Agent type 'critic' not found`」の回避手順 | 09-10 の新設時だけの話。現在は定義済みなので削除可 |
| `project-state` | L24 | 「PROJECT_STATE.md を**全文 Read**」 | 210 KB を毎回読む運用がファイル肥大の負のループ (読む→追記→さらに重く)。分割 (4.4) と合わせて見直し |

#### 5.3 skill 間の重複

| 重複内容 | 出現箇所 |
|---|---|
| 「template.md を `docs/<dir>/<YYYY-MM-DD>_<slug>.md` にコピー → Edit で埋める (Write 禁止) → `<...>` を全置換 → 該当なしは『該当なし』と明記 → 命名規則 → 機微情報マスク」 | `debug-report/SKILL.md:33-54`, `feature-doc/SKILL.md:33-55`, `knowledge-check/SKILL.md:45-69` (3 本がほぼ同文) |
| 「USER_LEVEL.md は git 管理外・コミット禁止・無ければ較正なし」 | `annotate:38-41`, `explain-edits:48`, `knowledge-check:26-28`, `user-level:16-21`, CLAUDE.md L179, PROJECT_STATE §8 L286 (6 か所) |
| critic 導入の背景 (08-14〜16 縦角説 → 09-10 棄却) | `agents/critic.md:97-102`, `skills/critic/SKILL.md:11,53-59`, CLAUDE.md L199 |
| 「Write 禁止・Edit のみ」 | `project-state:39`, `debug-report:42`, `feature-doc:42`, `user-level:43` |
| 「joint_states.velocity は常に 0」 | `verify:139`, `stack-health:85,94`, CLAUDE.md L139, PROJECT_STATE §7-2 |
| 作業慣例 (// claude タグ・clang-format・main 直コミット・浮かせ→接地・日本語) | CLAUDE.md L192-199 と PROJECT_STATE §9 L296-306 が同内容 |

#### 5.4 skill 名と CLAUDE.md の説明の食い違い

- CLAUDE.md L188 `params-sync` 「3 ファイル」 ↔ skill「2 ファイル」(1.4-⑥)。
- CLAUDE.md L188 は 12 skills を列挙しているが `.claude/skills/` も 12 本で一致。agents は `critic` 1 本で一致。

---

### 6. 索引の欠落と提案

#### 6.1 現状の入口

| 入口 | 何が辿れるか | 欠け |
|---|---|---|
| `README.md` L4-7 | manual/00_index, CLAUDE.md, docs/text/ | docs/issue・report・features・PROJECT_STATE への導線なし。本文は udev/network セットアップが 9 割 (L37-247)。L16-20 の「symlink 復旧 (`ln -s external/…`)」は 07-26 撤廃済みの旧方式、L24 `docker compose up --build` は CLAUDE.md L39 の「`docker compose build` を引数なしで叩かない」に抵触、L26 `docker copose down` は typo、L168 `rerobot_bringup_2d.launch.py` は legacy、L30-35 の colcon 例に `--metas` なし |
| `CLAUDE.md` L176-186 の表 | docs/ 各 dir の役割 | reference / handoff / knowledge / `*.summary.md` が無い。索引へのリンクも無い |
| `docs/manual/00_index.md` | manual 15 章 + 関連資料 5 行 (L43-49) | manual 専用。TODO 2 か所 (L6, L32) が 1 か月残置 |
| `docs/claude/PROJECT_STATE.md §8` | docs 地図 | CLAUDE.md の表とほぼ重複、text は 3 冊止まり |
| `docs/text/*/00_index.md` | 各冊の章 | **7 冊を束ねる上位索引が無い** (`waypoint_nav/` は 00_index 自体が無い) |
| `docs/issue/` `docs/report/` `docs/features/` | — | **一覧 index が無い**。30 本の issue のステータスは各ファイルを開かないと分からない (3 章の表が初めての一覧) |
| `tools/README.md` (302 行) | tools 配下 10 ツール | 実質 index 付き。ただし `waypoint_runner.py` は `rerobot_bringup/scripts/` にあるのに tools/README に手順がある (L225-236、しかも `nav2.launch.py` 起動 + `g` キー前提で、10-16 のパッド Start 運用に未追従) |
| `maps/3d/README.md` | 2 行 (置き場の説明) | `maps/2d/` は別 git リポジトリ (`.git` あり) だが説明なし。maps/ 直下の png/npz 4 本は由来不明 |

#### 6.2 提案

1. **`docs/README.md` を新設** — dir ごとに 1 行 (役割・編集ルール・索引へのリンク)。CLAUDE.md L176-186 と PROJECT_STATE §8 の表はこれへのリンク 1 行に置き換えて重複を解消。
2. **`docs/issue/INDEX.md` / `docs/report/INDEX.md` / `docs/features/INDEX.md`** — 列 = ファイル | 日付 | ステータス (issue のみ) | 1 行要約 | 関連。issue のステータス行を正規化すれば `grep -m1 'ステータス'` で自動生成できる。
3. **`docs/text/README.md`** — 7 冊 + 各冊の 1 行紹介 + 読む順。
4. README.md は「ハード側セットアップ手順書」として独立させ、冒頭に `scripts/` と `docs/README.md` への導線を置く。L16-20 (旧 symlink) は削除、L24/L26/L168 を修正。
5. manual/00_index の TODO 2 件はユーザ執筆領域なので提案のみ。

---

### 優先度つき要約 (上位 10 件)

| # | 指摘 | 影響 | 根拠 |
|---|---|---|---|
| 1 | **PROJECT_STATE.md 冒頭 L6-L7 (計 39 KB) が更新履歴の連結**になっており、§6 タイムラインとも二重。ファイル全体 210 KB は「毎セッション全文 Read」(project-state skill L24) の前提を壊している | 新セッションの読み込みコストと劣化の負のループ | `PROJECT_STATE.md:6-7`, 行長 20,678 / 18,509 字 |
| 2 | **Stop hook が実質無効** — 監視パス `src`・`Dockerfile` が再編後に存在しない | コード変更しても PROJECT_STATE 更新を促されない → §5/§10 の陳腐化の構造的原因 | `.claude/hooks/check-project-state.sh:63` |
| 3 | **CLAUDE.md 内の自己矛盾 3 件** — IMU 向き (L148 vs L141)、urg の topic 名 (L141 vs L95)、`gear_ratio: 5.0` (L170 vs params.yaml 92.25) | 規約ファイルの誤りは全セッションに伝播する | 1.4 ①②④ |
| 4 | **PROJECT_STATE §5 が docs/issue と不整合** — 30 本中 10 本未掲載 (10 月分は 1 本のみ)、5 本が古いステータス (bno086「対策未適用」等)、§5 優先度要約が 07 月時点 (ウォッチドッグなし) | 「どこを見ればよいか」の表が信用できない | 2.2 (c)(d) |
| 5 | **PROJECT_STATE §10 に完了済み「最優先」が 3 段積層** — 07-26 再編仕上げ・08-30 車体改修・09-19 速度制限 | 次の作業が読み取れない | `PROJECT_STATE.md:314-330` |
| 6 | **skills の値が 2 世代古い** — `params-sync` 「gear_ratio 正値 5.0」、`verify` 「gear 1.25」「`rerobot_bringup_2d`」「`--metas` なし」、`stack-health` 「`/sdk_could`」 | skill を実行すると誤報告・ビルド失敗 | 5.2 |
| 7 | **docs/issue のステータス未更新 8 本** — utm30lx (常用中なのに調査中)、rfans_calc_review (対象廃止)、08-16/08-15 (棄却済み仮説がステータス行)、09-13 (再現不能)、repository_audit 25 件全部未解決 (Issue 1 は実装済み) | 「未解決問題の調査記録」dir に解決済みが半数以上 | 3 章 |
| 8 | **CLAUDE.md の 38/161 行が日付入り履歴** — 特に L141/L143/L151/L160 の 4 段落 (各 1,000〜1,900 字) は features/issue に移すべき調査結論 | 規約と履歴が混ざり、規約が探せない | 1.2-1.3 |
| 9 | **一覧索引の不在** — issue/report/features/text に INDEX が無く、docs/ の入口が README・CLAUDE.md・PROJECT_STATE §8・manual/00_index の 4 か所に分散して各々不完全 | 新規参加者 (人・エージェント) が全ファイルを開かないと現状が分からない | 6.1 |
| 10 | **README.md の旧記述** — 旧 symlink 復旧手順 (L16-20)、`docker compose up --build` (L24)、`copose` typo (L26)、legacy launch 名 (L168) | 公開リポジトリの最初のページが現行手順と食い違う | README.md 該当行 |

---

### 付記: 調べた範囲と行数、問題なしだった領域

- 行数: CLAUDE.md 199 / README.md 246 / PROJECT_STATE.md 408 (210 KB) / docs/issue 30 本 計 5,968 行 / docs/report 11 md 計 1,446 行 (+pdf 2) / docs/features 11 本 計 2,167 行 / docs/text 7 冊 61 ファイル 計 約 10,900 行 / docs/manual 17 ファイル 計 1,655 行 / docs/trial 3 本 813 行 / skills 12 本 計 936 行 + template 3 本 / agents 1 本 59 行 / hooks 1 本 41 行 / tools/README 302 行。
- 問題なし: `docs/text/` 6 冊 (timestamp・ekf_fusion・glim・slam_toolbox・ros2_cpp・map3d_to_nav2) は 00_index + 章分割 + 章間リンクが揃い、画像参照も切れていない (`/sdk_could` 等の旧名は事例解説として歴史的文脈で使われており誤りではない)。
- 問題なし: `docs/issue/img/*` / `docs/features/img/*` の画像はすべて本体 md から参照されている (孤立ディレクトリ無し)。
- 問題なし: skills のテンプレート 3 本 (debug-report / feature-doc / knowledge-check の template.md) は構造が一貫。`.claude/agents/critic.md` のフロントマター (tools・model) は有効。
- 問題なし: `.gitignore` の USER_LEVEL.md・build/install/log・trial の研究データ除外は一貫しており、CLAUDE.md の記述と一致。
- 問題なし: `docs/manual/15_launch_guide.md`・`launch/legacy/README.md` は 10-08 構成を正しく反映し、相互リンクも生きている。
- 問題なし: `docs/reference/2026-07-27_epos4_studio_startup_settings.md` は 08-31 更新で現行 NVM 値を保持 (ただし `wiring_diagram.mmd` は v1 配線のまま、09-19 issue L8 が自認)。
