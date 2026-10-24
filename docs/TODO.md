# TODO — 今取り組むこと

<!-- 基本はユーザが更新する (Claude は依頼時のみ編集)。
     元ネタ: docs/trial/improvements.md (走行ごとの改善点メモ、こちらが一次資料)。
     ここは「ジャンル別に短く」。経緯・詳細は各リンク先。済んだものは [x] にして「→ どう直したか」を 1 句だけ残す。
     初版 2026-10-23 (Claude が improvements.md 10-03 / 10-04 / 10-21 / LT 大会の節から抽出) -->

凡例: `[ ]` 未着手・継続中 / `[x]` 済 (→ 対処)。**太字** = 次の実走までに触りたいもの。

## 自己位置推定 (AMCL / 3D)

- [x] 低い帯だと市役所の壁や遠方の建物が映らず誤マッチ → AMCL 専用スキャン `/rfans/scan_all` (0.3〜100 m) + 専用地図 `/map_loc` に分離 (10-04)
- [x] emcl2 の導入 → `nav_emcl2.launch.py` (10-08、未チューニング)
- [ ] **3D 自己位置推定 (`nav_lidar3d`) の実走安定化** — 10-22 の TF 遅れは `map_odom_tf_future_dating_sec 0.5` で対処済み、再実走で確認 ([features](features/2026-10-10_lidar_localization_3d.md))
- [ ] 2D の東端の角での見失い・共通の横バイアス 0.5〜1 m (未解明) ([issue 10-11](issue/2026-10-11_loc_eval_2d_corner_loss_vs_3d.md))
- [ ] 建物の近くではマッチするが keepout のズレで草地を走る
- [ ] 地形によって AMCL のマッチング強さを変える (AMCL 用の別途「価値 map」)
- [ ] AMCL は停止時に推定を走らせない
- [ ] R-Fans の上限 180 m を自己位置推定・地図生成に含める
- [ ] 自前 map と他チーム製 map、どちらがマッチしやすいか (LT)
- [ ] 元mapの高密度化による先鋭化、2dmapの改造度upによる自己位置推定制度の変化
- [ ] 植木や木、道路のごま塩をmapから取り除くと自己位置推定は安定するか不安定になるか
- [ ] 車などの障害物のmapからの除去
- [ ] スタート地点の人混み対策

## 経路計画・Nav2

- [x] 過去の waypoint へ戻る → RemovePassedGoals を ReactiveSequence に (ff8b606)
- [x] 坂道で scan_all を costmap に入れると見えない壁 → costmap は `/rfans/scan` (0.3〜1.5 m) のみ (10-04/08)
- [x] 狭い空間でロボットが待つ動作 → 追従型 BT (10-16、実走検証はまだ)
- [x] 横断歩道前の一時停止・再開 → waypoint_runner の停止点 + パッド Start/Back (10-04/16)
- [x] 途中復帰も含めた waypoint 計画 → `--start N`、`--auto-retry`
- [ ] **global planner が遅く経路間で右往左往 — 重さは map の広さ依存** → 区画ごとに地図を切り替える / 範囲内だけ計算。暫定: `tools/map_crop`
- [ ] **かくかく進む・常に真ん中を取ろうとして凸凹に進む** → smoothing (velocity_smoother / smoother_server は未導入)
- [ ] 危険域が衝突すると前後左右が障害物認定されスタック → 衝突時のバック回避
- [ ] 狭い場所で膨張の板挟み
- [ ] waypoint に障害物が重なっている可能性を考慮 (LT)
- [ ] どこまで塞がっていて・どこまで通れるかの判断を明快に (LT)
- [ ] 白線・横断歩道前の停止位置がシビア → 側面にメジャー + カメラで waypoint 微調整
- [ ] 相対高さで区切っているが、機体が坂で斜めのときはどうなるか

## 障害物検知・センサ

- [x] UTM が足元の草を見て経路を塞ぐ → costmap から UTM を外して踏破 (10-17/21、暫定)
- [ ] **足元の低い障害物 (縁石・箱・20 cm の草) の扱い** — R-Fans は車体 1 m 以内で 0.5 m より低い物を見られない。VoxelLayer / 地面除去 ([計画 P2/P3](trial/improvements.md))
- [ ] 飛び出た草木による通路の遮断
- [ ] 安全管理責任者の左右に謎の物体 (安全ベストの反射板?)
- [ ] 人は人として推定し進行方向にコストを伸ばす / ロボットは認識できない / すれ違い・追いつき・追いつかれの回避 (LT、最難関)
- [ ] R-Fans 回転ディップによるデータ欠損 (一次原因未特定) ([要点版](issue/2026-09-02_rfans_scan_motor_dropout.summary.md))

## 運用・起動

- [x] コントローラ → 自律移動の切り替え (cmd_vel の干渉) → twist_mux (10-16)
- [x] `/scan_all` の常時表示 → nav2.rviz
- [x] bag 記録中に停止位置を記録 (ティーチング) → teach_marker + teach_from_traj
- [x] waypoint editor の位置ずれ
- [ ] **起動の簡素化** — `auto_run.launch.py` はあるが確認表が見にくく実戦では未使用 → 表示を直す ([features](features/2026-10-16_auto_run_joy_mux.md))
- [ ] **launch・設定の一本化 (run ファイル方式)** — 当日の起動を `run.launch.py run:=config/runs/<日付>.yaml` の 1 引数にし、差分を git で残して過去設定を再現できるようにする (10-23 検討、未着手)
  - 方式: (1) run yaml に `stack` (起動部品) / `nav` (amcl・emcl2・lidar3d) / `args` (launch 引数) / `params` (ノード差分 = overlay、ROS の params 複数渡し後勝ちで上書き) の 4 節。auto_run.launch.py の一般化
  - 代替: (2) 引数だけの env ファイル (launch 引数しか変えられない) / (3) git hash + diff を bag dir に書くだけ (仕組み最小、変更点が見えない) / (4) 全 ROS yaml を 1 ファイルに物理統合 (`/**:` 漏れの事故面が広がる)
  - 再現性を閉じる記録: bag dir に run yaml のコピー + git hash + 全ノード `ros2 param dump` (起動後の実効値)
  - 統合できないもの: GLIM JSON (base + 差分をマージ生成)、bus.yml (ビルド時設定)、URDF (xacro 化が要る)、別コンテナの slam / GLIM は起動コマンドが別に残る
  - ログ混在の対処: Node `output='log'` でノード別ファイル (`ROS_LOG_DIR` を bag と並べる) + /rosout 選別の console ノード (WARN 以上 + 節目) + 常駐の状態表。GUI は独立アプリでなく RViz のオーバーレイ (rviz_2d_overlay_plugins) → 必要なら RViz パネル plugin。launch の制御は GUI に持たせない
- [x] パーティクルが見られるように → nav2.rviz を ParticleCloud 型に修正済み (10-21)、実機で確認したら [x]
- [ ] bag の起動コマンドが長い (auto_run には統合済み、単体記録は未)
- [ ] 非常停止後の再起動不要へ / USB を抜いても止まるように ([issue 10-22](issue/2026-10-22_epos_fault_8250_pdo_freeze_lidar3d_run.md)、[引き継ぎ](claude/handoff/2026-10-22_epos_fault_monitor_fix.md))
- [ ] ThinkPad の Chrome Remote Desktop + GNOME 設定が奪い合う問題 / ~~Parsec 導入~~

## 電源・PC・機体

- [x] ギヤの遊びによる騒音 / キャスタ固定具の再設計 / タイヤカバー・外装
- [ ] **PC バッテリ低下で powersaver → EPOS エラー** (performance 固定は済、バッテリ時の降格経路が残る)
- [ ] 日光で PC が熱い / ロボットの角の突起 / ホイールの挟み込み防止カバー
- [ ] joy_teleop の加減速をもう少しゆっくり
- [ ] R-Fans16の電源隔離

## 参考: Claude 側の整備 (コードではなく repo の衛生)

- [ ] リポジトリ見直しの修正 15 件 (teleop `invert_*` 逆転、`bringup2d.sh` の EKF 既定、不在 `map_dir` 既定、CLAUDE.md の自己矛盾、skills の古い期待値 …) → [docs/issue/2026-10-23](issue/2026-10-23_repository_review_incremental_cruft.md) §1.2
- [x] docs 構成の整理 (issue/closed・INDEX・docs/README・monthly 変更履歴・構成図) (10-23)
