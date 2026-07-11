<!-- claude: docs/report テンプレート。debug-report スキルから生成。-->
# colcon build が named volume に残った旧成果物と衝突して realsense2_camera_msgs で失敗する

- 日付: 2026-07-11
- 環境: コンテナ rerobot_env (Ubuntu 24.04 noble / ROS 2 Jazzy)、WORKDIR=/workspace、colcon `--symlink-install --executor sequential`
- 対象ブランチ: `main` (HEAD: `2bf1642 chore: switch LIO-SAM submodule to ros2 branch`)
- 関連ファイル:
  - `docker-compose.yml` (volumes: `ros_build:/workspace/build`, `ros_install:/workspace/install`, `ros_log:/workspace/log`)

## TL;DR

`build`/`install`/`log` は docker-compose の **named volume** のためコンテナ再作成 (`docker compose up --build`) 後も永続する。過去ビルドの成果物 (実ディレクトリ) が残った場所へ、`--symlink-install` の ament_cmake_python が symlink を張ろうとして「existing path cannot be removed: Is a directory」で失敗した。volume 内の成果物を全削除してフルビルドし直して解決 (10 パッケージ成功、1min28s)。

## 症状

- いつ: イメージ再ビルド → コンテナ Recreate 直後の `colcon build --symlink-install --executor sequential`。
- どこで: `realsense2_camera_msgs` のビルド中 (ament_cmake_python の symlink 生成ステップ)。後続の `rfans_driver` は Aborted。
- 何が起きるか: 下記エラーで 1 package failed / 1 aborted / 4 not processed。
- 正常時: 全パッケージが Finished になる。

```
--- stderr: realsense2_camera_msgs
failed to create symbolic link '/workspace/build/realsense2_camera_msgs/ament_cmake_python/realsense2_camera_msgs/realsense2_camera_msgs'
because existing path cannot be removed: Is a directory
gmake[2]: *** [CMakeFiles/ament_cmake_python_symlink_realsense2_camera_msgs.dir/build.make:70: ...] Error 1
Failed   <<< realsense2_camera_msgs [1.61s, exited with code 2]
Aborted  <<< rfans_driver [1.60s]
```

## 切り分けの記録

### 1. 観測: エラーパスは build ディレクトリ内の「既存の実ディレクトリ」

エラーメッセージ自体が「symlink を作りたい場所に消せないディレクトリがある」と言っており、ソースではなくビルド成果物側の問題と判断。コンテナは直前に Recreate されたばかりなのに古いパスが存在する点が矛盾 → 永続化の仕組みを疑った。

### 2. docker-compose.yml の volume 定義確認 (当たり)

| 項目 | 観測値 | 備考 |
|------|--------|------|
| /workspace/build | named volume `ros_build` | コンテナ Recreate でも消えない |
| /workspace/install | named volume `ros_install` | 同上 |
| /workspace/log | named volume `ros_log` | 同上 |
| /workspace/src | ホスト `./src` の bind mount | ソースは常に最新 |

### 3. 該当なし

仮説 2 で確定したため追加の切り分けは行っていない。

## 根本原因

named volume は「ビルドキャッシュを保つ」ための設計で、`docker compose up --build` でイメージやコンテナを作り直しても意図的に生き残る。今回は (1) realsense-ros submodule の追加、(2) LIO-SAM の ROS1→ROS2 ブランチ切替、(3) 過去の非 `--symlink-install` ビルドの成果物、とビルドの前提が大きく変わったのに volume 内の旧成果物がそのまま残っていた。ament_cmake_python は既存パスが空でない実ディレクトリの場合、symlink への置換ができず即エラーになる。

## 修正

コード変更なし。volume 内の成果物をクリーンして再ビルド (成果物はすべて再生成可能なので削除は安全)。

```bash
docker exec rerobot_env bash -c "rm -rf /workspace/build/* /workspace/install/* /workspace/log/*"
docker exec rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install --executor sequential"
```

### なぜこの値 / この方法か

- 部分削除 (`build/realsense2_camera_msgs` のみ) でも当該エラーは直るが、LIO-SAM の ROS1→ROS2 切替直後で他パッケージにも stale 成果物の懸念があったため全クリーンを選択。
- `docker volume rm ros_build ...` でも同等だが、コンテナ停止が必要になるため exec での中身削除を選択。
- 破綻条件: なし (クリーン→フルビルドはいつでも安全。時間コストのみ)。

## 検証

| 項目 | 修正前 | 修正後 |
|------|--------|--------|
| colcon build 結果 | 4 finished / 1 failed (realsense2_camera_msgs) / 1 aborted / 4 not processed | 10 packages finished [1min 28s] |

検証手順:

```bash
docker exec rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install --executor sequential"
# Summary: 10 packages finished であること
```

## 教訓 / 今後の予防

1. 今後 submodule のブランチ切替・追加や `--symlink-install` 有無の変更など「ビルドの前提」を変えたときは、ビルド前に `/workspace/build` `/workspace/install` をクリーンする。`docker compose up --build` は named volume を消さない。
2. 「failed to create symbolic link … Is a directory」は ament_cmake_python の symlink⇄実体レイアウト切替の典型症状。見たらまず stale 成果物を疑う。
3. フォローアップ: 該当なし。
