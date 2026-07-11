---
name: params-sync
description: params_2d.yaml / params_3d.yaml / epos4_teleop の 3 ファイルに重複する車体パラメータの整合を検査し、不一致を表で報告する。
---

<!-- claude: params-sync スキル定義。Claude 作成。-->

# Params Sync

車体パラメータ (`tread_width` / `tire_diam` / `gear_ratio` / `invert_left` /
`invert_right`) は 3 ファイル × 複数セクションに**手動コピーで**重複しており、
片方だけ直すと odometry と teleop の距離表示が食い違う等の症状が静かに出る。
このスキルは全コピーを突合し、不一致を検出する。

## いつ使うか

- 車体パラメータ (トレッド幅・タイヤ径・減速比・回転方向) を変更した直後
- odometry の距離・旋回角と実測が合わない、teleop の距離表示だけずれる等の症状時
- ユーザが「パラメータの整合を確認して」等と要求したとき

## 検査対象

| ファイル | セクション |
|----------|-----------|
| `src/rerobot_bringup/config/params_2d.yaml` | `epos4_controller_node`, `epos4_odometry_node` |
| `src/rerobot_bringup/config/params_3d.yaml` | `epos4_controller_node`, `epos4_odometry_node` |
| `src/epos4_teleop/config/params.yaml` | teleop ノードのセクション (ファイル内で確認) |

比較キー: `tread_width`, `tire_diam`, `gear_ratio`, `invert_left`, `invert_right`
(セクションにより一部キーが無いのは正常。**存在するキー同士**を比較する)

## 手順

1. **3 ファイルすべてを Read** し、各セクションの対象キーを抽出する。

2. **`ros__parameters` キーの綴りを検査**する。アンダースコア 1 つ (`ros_parameters`)
   だとノードが起動即死し、症状から原因が分からない (CLAUDE.md 記載の罠)。
   全セクションで 2 つであることを確認する。

3. **突合表を作成**して報告する:

   | キー | 2d/controller | 2d/odometry | 3d/controller | 3d/odometry | teleop | 判定 |
   |------|--------------|-------------|---------------|-------------|--------|------|

   不一致セルを明示し、一致していれば「全一致」と一言で報告する。

4. **不一致があっても勝手に直さない**。どの値が正かは Claude には決められない
   (実測に基づく値のことがある)。不一致の内容と「どちらに揃えるか」の質問を
   ユーザに提示し、回答を得てから Edit で修正する。修正時は既存のコメント
   (実測値の由来など) を消さない。

## 守るべきこと

- **`gear_ratio: 1.25` は対症療法値** (物理減速比は 5:1。エンコーダ分解能 4 倍ズレの
  補正 — `docs/issue/2026-07-07_wheel_odometry_encoder_scaling_4x.md` 参照)。
  「5.0 が正しいのでは」と勝手に直すのは厳禁。根本修正は EPOS Studio / bus.yml /
  gear_ratio の **3 点同時変更**が必須で、このスキルの範囲外。
- 値の横のコメント (`#measured: ...` 等) は値の由来の記録。修正時も必ず残す。
- 報告には実際に読んだ値だけを載せる。**読まずに「たぶん一致」と書かない**。
