<!-- claude: docs/report テンプレート。debug-report スキルから生成。-->
# Dockerfile の apt install が libgtsam-unstable-dev 不在で失敗する (exit 100)

- 日付: 2026-07-11
- 環境: ホスト Ubuntu (Linux 6.8.0-124-generic) / コンテナ rerobot_env (ベース `ros:jazzy-ros-base` = Ubuntu 24.04 noble) / ROS 2 Jazzy
- 対象ブランチ: `main` (HEAD: `2bf1642 chore: switch LIO-SAM submodule to ros2 branch`)
- 関連ファイル:
  - `Dockerfile` (41–60 行付近、LIO-SAM 依存ブロック)
  - `src/external/LIO-SAM/src/imuPreintegration.cpp` (gtsam_unstable の include 元、参照のみ)

## TL;DR

Ubuntu noble (24.04) に `libgtsam-unstable-dev` パッケージは存在せず、`docker compose up --build` の apt ステージが exit 100 で失敗した。この行は過去コミット fafaa66 で追加されたが、以降イメージを再ビルドしていなかったためレイヤーキャッシュに隠れており、今回 `ros-jazzy-imu-filter-madgwick` 追加で RUN 行が変わりキャッシュが無効化されて初めて顕在化した。gtsam_unstable を同梱する `ros-jazzy-gtsam` (GTSAM 4.2.0) への置換で解決。

## 症状

- いつ: madgwick 追加後の `docker compose up --build` 実行時 (それ以前はキャッシュヒットで apt が走らず成功していた)。
- どこで: Dockerfile の巨大な `RUN apt-get update && apt-get install -y ...` ステージ。
- 何が起きるか: exit code 100 で failed to solve。compose のログは折り返し表示で実エラー行 (`E:` 行) が読めない。
- 正常時: apt ステージを通過してイメージが生成される。

```
failed to solve: process "/bin/sh -c apt-get update && apt-get install -y ...
ros-jazzy-imu-filter-madgwick && rm -rf /var/lib/apt/lists/*"
did not complete successfully: exit code: 100
```

## 切り分けの記録

### 1. 仮説: 直前に追加した ros-jazzy-imu-filter-madgwick が存在しない → 外れ

唯一の変更点だったため最初に疑った。フレッシュな `ros:jazzy-ros-base` コンテナで `apt-cache policy` を実行。

| 項目 | 観測値 | 備考 |
|------|--------|------|
| ros-jazzy-imu-filter-madgwick | Candidate: 2.1.5-1noble.20260615.152736 | packages.ros.org に存在 → 無罪 |

### 2. パッケージリスト全体を dry-run → 真犯人が判明

Dockerfile の apt パッケージリスト全文を `apt-get install --dry-run` にかけた。

```
E: Unable to locate package libgtsam-unstable-dev
```

### 3. gtsam 系パッケージの実態調査 (当たり)

| 項目 | 観測値 | 備考 |
|------|--------|------|
| libgtsam-unstable-dev | パッケージ自体が noble に存在しない | apt-cache search でもヒットなし |
| libgtsam-dev (noble universe) | 4.2.0+dfsg-1build1 は存在 | ただし .deb 中の gtsam_unstable ヘッダは 0 件 |
| imuPreintegration.cpp:17 | `#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>` | 行削除だけでは lio_sam ビルドで詰む |
| ros-jazzy-gtsam | 4.2.0-4noble、gtsam_unstable 103 ファイル同梱 (IncrementalFixedLagSmoother.h 含む) + /opt/ros/jazzy/lib/cmake/GTSAM/GTSAMConfig.cmake | 置換先として成立 |

## 根本原因

`libgtsam-unstable-dev` は Ubuntu noble のリポジトリに存在しない (universe の gtsam は `libgtsam-dev`/`libgtsam4`/`python3-gtsam` のみで、gtsam_unstable はビルドされていない)。この行は fafaa66 で追加された時点から不正だったが、**Docker レイヤーキャッシュは「前回この RUN が通った」ことしか保証しない**ため、以降 RUN 行を変更するまで一度も apt が再実行されず、約 1 ヶ月間顕在化しなかった。madgwick 追加でキャッシュが無効化され、apt が再実行されて発覚した。

## 修正

`Dockerfile` の 2 行を 1 行に置換。イメージ再ビルド (`docker compose up --build`) が必要。

```diff
-    # 公式は ppa:borglab/gtsam-release-4.0 を使うが noble(24.04) 非対応のため
-    # universe の libgtsam-dev (GTSAM 4.x) を利用する。 # claude
-    libgtsam-dev \
-    libgtsam-unstable-dev \
+    # 公式は ppa:borglab/gtsam-release-4.0 を使うが noble(24.04) 非対応。
+    # universe の libgtsam-dev は gtsam_unstable ヘッダ
+    # (imuPreintegration.cpp が include する IncrementalFixedLagSmoother.h)
+    # を含まず、libgtsam-unstable-dev は noble に存在しない。
+    # → gtsam_unstable 同梱の ros-jazzy-gtsam (GTSAM 4.2.0) を使う。 # claude
+    ros-jazzy-gtsam \
```

### なぜこの値 / この方法か

- `ros-jazzy-gtsam` は /opt/ros/jazzy 配下に gtsam_unstable のヘッダ・ライブラリと GTSAMConfig.cmake を同梱しており、ROS 環境 source 後の `find_package(GTSAM)` がそのまま解決する (実測: dpkg -c で 103 件の gtsam_unstable エントリを確認)。
- `libgtsam-dev` (/usr) を併存させなかったのは、CMake が gtsam_unstable 無しの /usr 側 config を先に拾うと「ヘッダが見つからない」という分かりにくいエラーになるため。
- borglab PPA は noble 非対応 (Dockerfile 旧コメントの通り) のため採らず。ソースビルドは Docker ビルド時間が大幅に伸びるため採らず。
- 破綻条件: ROS distro を更新して `ros-<distro>-gtsam` の GTSAM バージョンが LIO-SAM の要求と乖離した場合は再評価。

## 検証

| 項目 | 修正前 | 修正後 |
|------|--------|--------|
| apt dry-run (フレッシュ jazzy ベース) | E: Unable to locate package libgtsam-unstable-dev | EXIT=0、9 upgraded / 1295 newly installed |
| docker compose up --build | exit code 100 | イメージ生成・rerobot_env 起動成功 |
| lio_sam の colcon build (Eigen3 修正後) | (ビルド不能) | Finished 50.4s |

検証手順:

```bash
# フレッシュなベースで依存解決を事前確認 (イメージ全ビルドより速い)
docker run --rm ros:jazzy-ros-base bash -c \
  "apt-get update -qq && apt-get install -y --dry-run <Dockerfile のパッケージリスト全文>"
# 本番
docker compose up --build
```

## 教訓 / 今後の予防

1. 今後 Dockerfile の apt 行を編集するときは、フレッシュなベースイメージで `apt-get install --dry-run` を通してから commit する。レイヤーキャッシュは既存行の正しさを保証しない。
2. compose のビルドログは折り返しで `E:` 行が埋もれる。apt の exit 100 を見たら、同じコマンドを `docker run --rm <base>` で再現して生のエラーを取る。
3. フォローアップ: 該当なし (lio_sam ビルド成功まで確認済み)。
