<!-- claude: slam_toolbox 読本 (2026-08-12)。ユーザ依頼「手法・概念とパラメータをできるだけ詳細に図で解説」による書籍化。
     軸足は 2D グラフ SLAM の一般論、事例は reRoBot 実コード・実設定。 -->

# slam_toolbox 読本 — 2D グラフ SLAM の手法と設定

slam_toolbox は「/scan を入れると /map が出てくる箱」ではない。中身は 20 年前の
Karto から続く**グラフベース SLAM** の実装であり、スキャンをいつ採用するか・どう照合するか・
いつループを閉じるかのすべてが 60 個超のパラメータで制御されている。reRoBot では
「地図が絶妙にずれる」「scan queue full で地図が出ない」という形でこの中身と既に 2 度衝突した。
本書は 2D グラフ SLAM の理論を土台に、slam_toolbox の全パラメータを「何を変えると何が起きるか」
まで下ろして解説する。

## 本書の構成

```
slam_toolbox 読本
├── 第1章 2D SLAM の基礎 ................. 01_2d_slam_basics.md
│   ├── SLAM が解いている問題 (鶏と卵)
│   ├── 二層構造: ポーズグラフと占有格子
│   └── odom prior — 車輪オドメトリが効く理由
├── 第2章 slam_toolbox の構造 ............ 02_architecture.md
│   ├── Karto の系譜と 5 つの実行ファイル
│   ├── ノード I/O (topic / service / TF)
│   ├── MessageFilter — scan と TF の待ち合わせ
│   └── lifecycle node の状態遷移
├── 第3章 スキャンマッチングの仕組み ..... 03_scan_matching.md
│   ├── 相関スキャンマッチャ (総当たり探索)
│   ├── スキャン採用ゲート (距離 OR 旋回 + 時間)
│   ├── ループ閉じ込みの 3 段検査
│   └── Ceres によるグラフ最適化
├── 第4章 パラメータ大全 ................. 04_parameters.md
│   ├── solver / ROS / 地図生成 / 採用ゲート
│   ├── 相関探索・ペナルティ・ループ閉じ込み
│   ├── yaml に載らない隠しパラメータ
│   └── 上流デフォルト vs reRoBot 差分表
├── 第5章 reRoBot での適用 ............... 05_rerobot_setup.md
│   ├── コンテナ分割と DDS 疎通
│   ├── slam.launch.py の lifecycle 発火チェーン解剖
│   ├── SLAM と Nav2 で odom の出し手が違う件
│   └── 地図の保存先と消費側 (nav2)
├── 第6章 事例集 ......................... 06_case_studies.md
│   ├── 事例A: scan queue full (解決済)
│   ├── 事例B: 地図が「絶妙にずれる」(未解決)
│   ├── 事例C: GLIM 診断の測定器になった 2D SLAM
│   └── 事例D: autostart 不発と手動 lifecycle 発火
└── 第7章 実務チェックリスト ............. 07_checklist.md
    ├── 起動手順 (素のコマンド)
    ├── 地図の保存手順 (map_saver_cli)
    ├── 症状からの逆引き表
    └── チューニングの順序
```

## 読み方

- **初読は 1 → 2 → 3 章を順に**。第 3 章のマッチング機構を知らないと第 4 章のパラメータ表は
  「値の羅列」にしか見えない (パラメータの 7 割はマッチング機構の制御である)
- **事故から入りたければ第 6 章から**。各事例が破った原則の章へリンクしている
- **実務で繰り返し参照するのは第 4 章 (パラメータ) と第 7 章 (逆引き表)**
- 3D SLAM (GLIM) との対比は姉妹書 [GLIM 読本](../glim/00_index.md) が扱う。
  時刻・stamp の話は [タイムスタンプ読本](../timestamp/00_index.md) が正

## 凡例

| 記法 | 意味 |
|---|---|
| `path/to/file.yaml:12` | リポジトリ内の実ファイル位置 (行番号は 2026-08-12 時点) |
| 上流デフォルト | apt 配布の slam_toolbox **2.8.5** (ros-jazzy) の既定値 |
| ✅ / ⚠️ / ❌ | 正しい設計 / 注意つきで可 / アンチパターン |
| mermaid 図 | GitHub / VS Code の Markdown プレビューでそのまま描画される |

## 関連資料 (一次資料)

- `docs/report/2026-05-26_slam_toolbox_scan_queue_full.md` — 事例A の完全な調査記録
- `docs/issue/2026-07-07_monthly_2026_6_todo_triage.md` — T3/T4/T7/T11 (map 更新周期・lifecycle・autostart・ずれ)
- `docs/issue/2026-07-07_repository_audit.md` — Issue 10/11 (3D 構成の /scan 不在・FOV ±90°)
- `docs/issue/2026-08-12_glim_horizontal_drift.md` — 事例C (2D SLAM を測定器に使った記録)
- `docs/issue/2026-08-11_utm30lx_usb_instability.md` — /scan が来ない障害の記録
- `docs/text/timestamp/06_case_studies.md` — 事例D「scan queue full」の時刻視点の解剖 (本書事例A と相互補完)
- 上流: https://github.com/SteveMacenski/slam_toolbox (README にパラメータ公式解説)
