<!-- claude: 経路計画・経路追従読本 (2026-11-06)。11-01〜11-06 の「障害物の横で止まり続ける」調査と
     NavFn → Smac Hybrid-A* の切替・実走・bag 再生実験をユーザ依頼で書籍化。
     軸足は一般論 (Nav2 の planner / controller / costmap の役割分担と各アルゴリズムの原理)、
     事例は reRoBot の実走 bag と再生試験。 -->

# 経路計画・経路追従読本 — Nav2 の planner と controller を車体の形から理解する

自律走行のロボットが「明らかに通れるのに止まる」「人の横で固まる」「角を大回りして曲がれない」とき、
原因は 1 つの部品ではなく、**経路を引く部品 (planner)** と **経路を追う部品 (controller)** が
それぞれ違うロボット像・違う地図・違う周期で判断していることの食い違いにあることが多い。
reRoBot では 2026-11-01〜11-05 にこの食い違いが立て続けに表に出た。
後ろに 0.51 m 張り出した四角い車体 (幅 0.65 m) は、点として計画する NavFn とも、
車のための Smac Hybrid-A\* とも相性が悪く、停止・大回り・角での接触が起きた。

本書は、Nav2 の planner と controller が「何を見て」「何を保証して」「何を保証していないか」を、
車体の形 (内接円と外接円) から積み上げて説明する。
有名な planner と controller を一覧し、本機への適合度を付け、実走と bag 再生で何が効いて何が効かなかったかを解剖する。

## 本書の構成

```
経路計画・経路追従読本
├── 第1章 Nav2 の役割分担 ................... 01_nav2_architecture.md
│   ├── planner / controller / 行動木 (BT) / costmap
│   ├── global costmap と local costmap — 誰が何を読むか
│   └── 「local は迂回しない、止めるだけ」
├── 第2章 costmap と車体の形 ............... 02_costmap_and_footprint.md
│   ├── コスト値 0〜254 と膨張 (inflation)
│   ├── 内接円と外接円 — 水色の帯は「必ず当たる」集合
│   └── 道幅の収支 — 格子・向きの刻み・自己位置ずれ
├── 第3章 planner の地図 ................... 03_planners.md
│   ├── 系統別の一覧と適合度 (格子・向き付き・サンプリング・最適化・構造化)
│   ├── NavFn / Smac 2D / Theta* — 点で引く
│   ├── Smac Hybrid-A* — 車のための探索
│   └── Smac State Lattice — 差動二輪のための探索
├── 第4章 controller の地図 ................ 04_controllers.md
│   ├── RPP の解剖 — 先読み点・曲率・衝突掃引・その場旋回
│   ├── DWB / MPPI / TEB / Rotation Shim
│   └── planner の保証と controller の動きの隙間
├── 第5章 reRoBot での構成と現在地 ......... 05_rerobot_config.md
│   ├── 現行設定の解剖 (行番号つき)
│   ├── NavFn → Smac Hybrid → 楽観 footprint → 実寸、の経緯
│   └── 残っている隙間と次の候補
├── 第6章 事例集 ........................... 06_case_studies.md
│   └── 事例 A〜G (後端振り出し・隙間進入・ジグザグ・角の接触・効かなかった対策 ほか)
└── 第7章 実務 — 逆引きと再生試験 .......... 07_checklist.md
    ├── 症状からの逆引き表
    ├── 設定変更時のチェックリスト
    └── bag 再生試験の手順 (planner 単体 / controller 単体)
```

## 読み方

- **初読は 1 → 2 → 3 → 4 章を順に**。第3章と第4章は第2章の「内接円と外接円」が分からないと読めない。
- **止まる原因を急いで知りたければ第6章から**。各事例の「機構」節が第2〜4章の該当節を指している。
- **設定を触る前には第7章のチェックリスト**。Smac には「黙って失敗する」設定の組み合わせが 2 つある。
- 一覧表だけ欲しければ第3章 3.1 (planner) と第4章 4.1 (controller)。

## 凡例

| 記法 | 意味 |
|---|---|
| `ros2_ws_main/.../nav2_params.yaml:354` | リポジトリ内の実コード位置 (行番号は 2026-11-06 時点、楽観 footprint を実寸に戻した未コミット状態) |
| `nav2_smac_planner/src/node_hybrid.cpp:94` | Nav2 上流のソース位置 (タグ 1.3.13 = Jazzy の apt 版) |
| ✅ / ⚠️ / ❌ | 適合 / 条件つき / 不適合 |
| ◎ ○ △ × | 本機への適合度 (本命 / 使える / 弱点あり / 不向き) |
| 「11-04 bag」 | `bags/raw/` の実走記録。時刻は bag 先頭からの秒 |

本機の寸法 (第2章で何度も使う):

```
車体 (base_link = 車軸中心)
├── 幅 0.65 m (車輪外面間) → 半幅 0.325 m = 内接半径
├── 前 0.14 m (実体)、footprint では 0.325 m に揃えてある (第5章 5.2)
├── 後 0.51 m
└── 後角までの距離 √(0.51² + 0.325²) = 0.60 m = 外接半径 (footprint_padding 0.01 込みで 0.62)
```

## 関連資料 (一次資料)

- `docs/claude/PROJECT_STATE.md` — 11-01〜11-06 の到達点と数値 (冒頭の「最終更新」行と §10)
- `docs/issue/2026-10-15_keepout_corridor_stall_imu_yaw_jump.md` — keepout が膨張されず縁を這う経路で停止 (事例 F)
- `docs/features/2026-10-16_follow_bt_narrow_corridor.md` — 追従型 BT (計画失敗で旧経路を保持)
- `docs/features/2026-10-30_global_costmap_rolling_window.md` — global costmap の追従窓 (200 m) と waypoint の窓送り
- `docs/features/2026-10-30_nav2_smoothing.md` — 経路平滑化と速度平滑化
- `bags/tools/2026-11-02_person_stop/` — 11-01〜11-04 の解析と再生試験の道具 (⚠️ `bags/` は git 管理外。ローカルにのみ存在)
- `bags/tools/2026-11-04_smac_run/` — Smac 初実走の解析
- 本書の前提: [3D 地図→Nav2 接続読本](../map3d_to_nav2/00_index.md) (地図と自己位置の側。本書は経路の側)

→ [第1章 Nav2 の役割分担](01_nav2_architecture.md)
