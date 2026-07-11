<!-- claude: docs/report テンプレート。debug-report スキルから生成。-->
# lio_sam が colcon にパッケージとして認識されない (src/external/COLCON_IGNORE 規約の symlink 漏れ)

- 日付: 2026-07-11
- 環境: コンテナ rerobot_env (Ubuntu 24.04 noble / ROS 2 Jazzy)、WORKDIR=/workspace
- 対象ブランチ: `main` (HEAD: `2bf1642 chore: switch LIO-SAM submodule to ros2 branch`)
- 関連ファイル:
  - `src/external/COLCON_IGNORE` (external 全体を colcon から除外)
  - `src/LIO-SAM` (今回作成した symlink → `external/LIO-SAM`)
  - `src/external/LIO-SAM/package.xml` (name: `lio_sam`)

## TL;DR

このリポジトリは `src/external/COLCON_IGNORE` で submodule 群をまとめて colcon から隠し、ビルドに参加させたいパッケージだけ src 直下に symlink を張る規約 (前例: `maxon_epos4_ros2`, `realsense-ros`, `StarROS2`)。LIO-SAM だけ symlink が未作成だったため、フルビルドが 10 パッケージで「成功」しつつ lio_sam が対象に入っていなかった。`ln -s external/LIO-SAM src/LIO-SAM` で解決。

## 症状

- いつ: LIO-SAM submodule を ros2 ブランチへ切り替えた後の最初のフルビルド。
- どこで: `colcon build` のパッケージ検出段階 (エラーは出ない)。
- 何が起きるか: `Summary: 10 packages finished` で正常終了するが、その中に lio_sam が無い。`colcon list | grep -i lio` も空。
- 正常時: lio_sam を含む 11 パッケージが検出・ビルドされる。

```
Summary: 10 packages finished [1min 28s]
# epos4_* / maxon_epos4_ros2 / realsense2_* / rfans_driver / rerobot_bringup のみ。lio_sam なし
$ colcon list | grep -i lio
(出力なし)
```

## 切り分けの記録

### 1. 仮説: LIO-SAM ディレクトリ内に COLCON_IGNORE / AMENT_IGNORE がある → 外れ

| 項目 | 観測値 | 備考 |
|------|--------|------|
| src/external/LIO-SAM/COLCON_IGNORE | 存在しない | find でも 0 件 |
| package.xml の `<name>` | `lio_sam` | 正常 |
| コンテナからの見え方 | package.xml / CMakeLists.txt とも見えている | bind mount 正常 |

### 2. 観測: colcon list の external 系パッケージのパス表示がヒント

`colcon list` の出力で、external の submodule 由来パッケージがすべて `src/realsense-ros/...`、`src/StarROS2`、`src/maxon_epos4_ros2` と **src 直下のパス**で表示されていた → src 直下の symlink 経由でのみ検出されている。

### 3. 仮説: src/external 自体が colcon から除外されている (当たり)

| 項目 | 観測値 | 備考 |
|------|--------|------|
| src/external/COLCON_IGNORE | 存在する | external 全体が検出対象外 |
| src/ 直下の symlink | maxon_epos4_ros2, realsense-ros, StarROS2 の 3 本 | LIO-SAM だけ無い |

## 根本原因

リポジトリの規約: submodule 直下にある Dockerfile / docker-compose.yaml 等の非 ROS ファイルでワークスペースを汚さないよう、`src/external/COLCON_IGNORE` で external ツリー全体を colcon の検出から除外し、ビルド参加は src 直下 symlink の明示オプトイン方式にしている。**submodule を追加しただけでは絶対にビルドされない**。realsense-ros・StarROS2 追加時は symlink を張っていたが、LIO-SAM 追加時 (fafaa66) は当時 ROS1 版でビルド対象外だったため張られておらず、ros2 ブランチ切替後もそのまま漏れていた。CLAUDE.md には maxon の symlink しか記載がなく、規約として明文化されていなかったことも一因。

## 修正

コード変更なし。symlink を 1 本作成 (git 管理対象なので要コミット)。ビルドし直しが必要。

```bash
cd src && ln -s external/LIO-SAM LIO-SAM
```

### なぜこの値 / この方法か

- 既存 3 パッケージ (maxon_epos4_ros2 / realsense-ros / StarROS2) と同一の規約に従うため。リポジトリ内で一貫する。
- `COLCON_IGNORE` を external から外す案は、submodule 内の余計なパッケージ (例: LIO-SAM 同梱の docker 用ファイルや、epos4compact50-5can 内の非対象ディレクトリ) まで検出されるため採らず。
- 破綻条件: 規約自体を変える (external を直接検出させる) 場合はこの symlink 群を整理する。

## 検証

| 項目 | 修正前 | 修正後 |
|------|--------|--------|
| `colcon list \| grep -i lio` | (出力なし) | `lio_sam  src/LIO-SAM  (ros.ament_cmake)` |
| lio_sam のビルド | 対象外 | Finished 50.4s (Eigen3 修正後、別レポート参照) |

検証手順:

```bash
docker exec rerobot_env bash -c "cd /workspace && colcon list | grep lio"
# lio_sam  src/LIO-SAM  (ros.ament_cmake) が出ること
```

## 教訓 / 今後の予防

1. 今後このリポジトリに submodule を追加してビルドに参加させるときは、`src/` 直下に symlink を張る (`src/external/COLCON_IGNORE` 規約)。
2. 「colcon build は成功したのに目的のパッケージが無い」ときは、エラーを探す前に `colcon list` で検出状況を確認する。
3. フォローアップ TODO: この symlink 規約を CLAUDE.md に明文化する (現状 maxon の記載のみ)。`src/LIO-SAM` symlink のコミットも必要。
