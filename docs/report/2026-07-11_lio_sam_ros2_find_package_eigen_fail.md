<!-- claude: docs/report テンプレート。debug-report スキルから生成。-->
# lio_sam (ros2 ブランチ) の find_package(Eigen) が EigenConfig.cmake 不在で失敗する

- 日付: 2026-07-11
- 環境: コンテナ rerobot_env (Ubuntu 24.04 noble / ROS 2 Jazzy)、LIO-SAM = fork YamamotoSoya/LIO-SAM の `ros2` ブランチ (08af3f3)
- 対象ブランチ: `main` (HEAD: `2bf1642 chore: switch LIO-SAM submodule to ros2 branch`)
- 関連ファイル:
  - `src/external/LIO-SAM/CMakeLists.txt` (31 行目・52 行目、修正箇所)
  - `/usr/share/eigen3/cmake/Eigen3Config.cmake` (コンテナ内、参照)

## TL;DR

LIO-SAM ros2 ブランチの `find_package(Eigen REQUIRED)` (CMakeLists.txt:31) は、現代の Eigen (libeigen3-dev) がパッケージ名 **Eigen3** / `Eigen3Config.cmake` しか提供しないため常に失敗する (upstream 由来のバグ)。`find_package(Eigen3 REQUIRED)` + `include_directories(${EIGEN3_INCLUDE_DIR})` に変更し、`ament_target_dependencies` から `Eigen` を除去して解決 (Finished 50.4s)。**修正は submodule 内で未コミット — fork の ros2 ブランチへの push と親リポの gitlink 更新が必要。**

## 症状

- いつ: `colcon build --packages-select lio_sam` の CMake configure 段階。
- どこで: `CMakeLists.txt:31 find_package(Eigen REQUIRED)`。
- 何が起きるか: `EigenConfig.cmake` / `eigen-config.cmake` が見つからず 3.95s で Failed。
- 正常時: configure を通過し 4 実行ファイルがビルドされる。
- 付随事象: 最初のビルドは `colcon build ... 2>&1 | tail -40` の形で実行したため、**colcon の exit 1 がパイプに食われてタスクは exit 0 と報告され、一時「成功」と誤認した**。

```
CMake Error at CMakeLists.txt:31 (find_package):
  Could not find a package configuration file provided by "Eigen" with any of
  the following names:
    EigenConfig.cmake
    eigen-config.cmake
Failed   <<< lio_sam [3.95s, exited with code 1]
```

## 切り分けの記録

### 1. 観測: コンテナに Eigen3 の CMake config は存在する

| 項目 | 観測値 | 備考 |
|------|--------|------|
| /usr/share/eigen3/cmake/Eigen3Config.cmake | 存在 | libeigen3-dev は PCL の依存で導入済み |
| EigenConfig.cmake (無印) | 存在しない | 現代の Eigen はこの名前の config を提供しない |

→ 不足しているのはパッケージではなく、CMakeLists 側の **パッケージ名が古い**。

### 2. Eigen の使用箇所の洗い出し

| 項目 | 観測値 | 備考 |
|------|--------|------|
| find_package | CMakeLists.txt:31 の 1 箇所 | |
| ament_target_dependencies | :52 (imuPreintegration) の `Eigen` のみ | featureExtraction / imageProjection / mapOptimization には無い |
| target_link_libraries | Eigen 関連なし | Eigen はヘッダオンリーなのでリンク不要 |

### 3. 該当なし

観測 1 で原因確定のため追加の仮説検証は行っていない。

## 根本原因

Eigen 3.x の CMake サポートはパッケージ名 `Eigen3`、config 名 `Eigen3Config.cmake`、変数 `EIGEN3_INCLUDE_DIR` で提供される。`find_package(Eigen)` が探す `EigenConfig.cmake` はどのパッケージも提供しないため、この行は環境によらず失敗する upstream (TixiaoShan/LIO-SAM ros2 ブランチ) 由来のバグ。今まで顕在化しなかったのは、当リポジトリで lio_sam がビルド対象になったのが今回が初めてのため (直前まで ROS1 版 master を参照しており、colcon 検出からも漏れていた — 別レポート参照)。

## 修正

`src/external/LIO-SAM/CMakeLists.txt` (fork submodule 内、`# claude` マーカー付き)。要リビルド。

```diff
-find_package(Eigen REQUIRED)
+# 現代の Eigen は Eigen3Config.cmake のみ提供 (EigenConfig.cmake は無い) # claude
+find_package(Eigen3 REQUIRED)
 find_package(OpenMP REQUIRED)

 include_directories(
   include/lio_sam
+  ${EIGEN3_INCLUDE_DIR}
 )
```

```diff
-ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ... OpenCV PCL GTSAM Eigen)
+ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ... OpenCV PCL GTSAM) # Eigen はヘッダオンリーのため include_directories で解決 # claude
```

### なぜこの値 / この方法か

- Eigen はヘッダオンリーのため、include パスさえ通れば十分。`include_directories(${EIGEN3_INCLUDE_DIR})` が最小・最も壊れにくい。
- `ament_target_dependencies(... Eigen3)` に書き換える案は採らず: ament は `Eigen3_INCLUDE_DIRS` という変数を期待するが、Eigen3Config が定義するのは `EIGEN3_INCLUDE_DIR(S)` (プレフィックスが大文字) で不一致となり解決が不安定。
- GTSAM も自身の依存として Eigen の include を伝播するため、imuPreintegration は実質二重に担保される。
- 破綻条件: Eigen 4 系などで CMake パッケージ名がまた変わった場合は再評価。

## 検証

| 項目 | 修正前 | 修正後 |
|------|--------|--------|
| colcon build --packages-select lio_sam | Failed [3.95s] (CMake Error) | Finished [50.4s] |
| install/lio_sam/lib/lio_sam/ | (なし) | lio_sam_featureExtraction / imageProjection / imuPreintegration / mapOptimization の 4 実行ファイル |

検証手順:

```bash
docker exec rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && \
  colcon build --symlink-install --executor sequential --packages-select lio_sam"
docker exec rerobot_env bash -c "ls /workspace/install/lio_sam/lib/lio_sam/"
```

## 教訓 / 今後の予防

1. 今後 `find_package(Eigen)` (無印) を見たら `Eigen3` への読み替えを疑う。無印の EigenConfig.cmake は存在しない。
2. バックグラウンドで colcon を流すときは `| tail` 等のパイプを exit code の伝播経路に挟まない (成功と誤認する)。パイプが必要なら `set -o pipefail` か、素で流してログファイル側を tail する。
3. フォローアップ TODO: この修正を fork (YamamotoSoya/LIO-SAM) の ros2 ブランチへ commit/push し、親リポの gitlink を更新する。未実施のままだと他マシンの clone でビルドが壊れる。
