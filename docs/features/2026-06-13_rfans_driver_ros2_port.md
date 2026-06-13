# StarROS2 (rfans_driver) の ROS1 → ROS2 移植

- 日付: 2026-06-13
- 対象パッケージ: `rfans_driver`（`src/external/StarROS2`、git submodule `YamamotoSoya/StarROS2.git`）
- 対象 ROS: ROS 2 Jazzy（reRoBot コンテナ `rerobot_env`）

## 1. 目的・概要

`src/external/StarROS2` は Sure-Star 社の RFans/CFans LiDAR ドライバ `rfans_driver`。
ディレクトリ名は "StarROS2" だが**中身は ROS1 (catkin/roscpp) 実装**で、reRoBot が
ROS 2 Jazzy ワークスペースであるため**そのままではビルド・実行できなかった**。
本改造で、既存機能を維持したまま ROS 2 で動くよう ROS インターフェース層を移植した。

維持した機能:
- LiDAR 点群を `sensor_msgs/PointCloud2` として `/sdk_could` に publish
- `rfans_control` サービスでスキャン速度 / エコー種別を制御
- 距離・角度フィルタの**実行時調整**（旧 dynamic_reconfigure）
- pcap / isf ファイル再生による実機レス動作

スコープ外（意図的に未対応）:
- RViz 操作パネル（`teleop_pad`）の rviz2 移植 → ROS1 のまま残置・ビルド除外
- reRoBot 本体ワークスペースへのビルド統合（symlink / COLCON_IGNORE 解除）→ 別途手動

## 2. 移植の勘所

### 2.1 SDK 本体は無改造で再利用できる（最重要）

同梱の `lib/libstar.so` と `lib/libparallel_algorithm.so` は **ROS に一切依存しない**
（`objdump -p` の NEEDED は `librt/libdl/libpthread/libm/libc` のみ、x86-64）。
点群演算（座標変換 `CCalCoor`、補間 `Interpolation`、書き出し `Export`、`Reader`）も
すべて `ss::` 名前空間の SDK 呼び出しで、ROS には依存しない。

→ **移植対象は薄い ROS ラッパ層だけ**で、SDK 呼び出しと点群充填ロジックは
ロジック不変のまま流用した。

### 2.2 dynamic_reconfigure は ROS 2 に存在しない

ROS 2 には dynamic_reconfigure が無い。距離・角度フィルタの実行時調整は
**ノードパラメータ + `add_on_set_parameters_callback`** に置き換えた。
GUI は rqt_reconfigure か `ros2 param set` で代替できる。

## 3. データフロー（変更なし・トピック/サービス名は維持）

```
[LiDAR 実機 over UDP]  または  [pcap/isf ファイル]
        │
        ▼
[driver_node = RfansDriverNode (rclcpp::Node)]
   ├─ ss:: SDK で点群を計算（無改造）
   ├─ /sdk_could            (sensor_msgs/PointCloud2)  ← publish
   ├─ /rfans_driver/rfans_control (rfans_driver/srv/RfansCommand) ← service
   └─ パラメータ (min_range, max_range, ... )          ← 実行時フィルタ調整
```

## 4. ROS1 → ROS2 変換の対応表

driver 系コードに現れた API は機械的に以下で置換した。

| ROS1 | ROS2 (rclcpp) |
|------|---------------|
| `ros::init` / `ros::NodeHandle` | `rclcpp::init` / `rclcpp::Node` 継承クラスに集約 |
| `node.advertise<T>` | `create_publisher<T>` |
| `node.advertiseService` | `create_service` |
| サービス cb `bool(Req&, Res&)` | `void(shared_ptr<Request>, shared_ptr<Response>)` |
| `pub.publish(msg)` | `pub->publish(msg)` |
| `ros::Time::now()` | `now()` / `rclcpp::Clock().now()` |
| `ros::Rate` | `rclcpp::Rate`（ctor が explicit のため direct-init） |
| `ros::ok()` / `ros::spinOnce()` | `rclcpp::ok()` / `rclcpp::spin_some(get_node_base_interface())` |
| `nh.param<T>(n,var,def)` / `ros::param::get` | `declare_parameter<T>(n,def)` |
| `ros::this_node::getName()` | `get_name()` |
| `ROS_INFO/WARN/FATAL/DEBUG` | `RCLCPP_*(get_logger(), ...)` |
| `dynamic_reconfigure::Server` + `cfg/*.cfg` | パラメータ + `add_on_set_parameters_callback` |
| `rfans_driver/RfansCommand.h` | `rfans_driver/srv/rfans_command.hpp`（`rfans_driver::srv::RfansCommand`） |
| `rfans_driver/Packet.h` 等 | `rfans_driver/msg/packet.hpp`（`rfans_driver::msg::Packet`） |

## 5. ファイル別の変更点

### 5.1 メッセージ / サービス（`msg/`, `srv/`）
- `Packet.msg` / `RfansPacket.msg`: `time stamp` → `builtin_interfaces/Time stamp`
- `RfansScan.msg`: `Header header` → `std_msgs/Header header`
- `RfansPacket.msg`: **rosidl は snake_case フィールド必須**のため
  `udpCount`/`udpSize` → `udp_count`/`udp_size`（どちらもコードから未参照で安全）
- `srv/RfansCommand.srv`: `int32`/`bool` のみで ROS 2 でも有効、変更なし

### 5.2 ビルド定義
- `package.xml`: format 3 / `ament_cmake` + `rosidl` 化。`rclcpp`/`std_msgs`/`sensor_msgs`/
  `geometry_msgs`/`builtin_interfaces`/`rosidl_default_*` を宣言。**`libpcap-dev` 依存を明記**。
  dynamic_reconfigure・rviz 依存と rviz plugin export は削除。
- `CMakeLists.txt`: catkin → ament_cmake へ全面置換。`rosidl_generate_interfaces` で
  msg/srv 生成、`rosidl_get_typesupport_target` で自パッケージの型支援を `driver_node` に
  リンク。`star`/`parallel_algorithm`/`pcap` をリンク。`.so` は `<prefix>/lib` に install し、
  実行ファイルは `INSTALL_RPATH "$ORIGIN/.."` で実行時解決。

### 5.3 driver 本体（`src/`）
- 旧 `Rfans_Driver`（plain class、`ros::NodeHandle` 引数）を **`RfansDriverNode : public rclcpp::Node`**
  に統合。ROS1 版がグローバル変数（publisher `sdk_output`、フィルタ値 `min_range` 等、
  `temple_1`、`out_count`、file-global `reader`）と自由関数の reconfigure コールバックで
  状態を持っていたのを、**すべてノードのメンバに集約**した。
- フィルタ用パラメータ（`min_range`/`max_range`/`min_angle`/`max_angle`/`cfans_*`/
  `use_laserSelection`/`laserID`）を `declare_parameter` で宣言し、`onParamChange`
  （set-parameters callback）で実行時更新。rfans/cfans の角度選択排他は
  `recomputeEffectiveAngles()` に切り出して再現。
- `publisher.cpp` は `rclcpp::init` → ノード生成 → `spinOnce` ループ → `rclcpp::shutdown()`
  の薄い main に。
- `ioapi.h/.cpp`: ROS ヘッダ・型を ROS2 化。`IOSocketAPI`/`InputPCAP` が保持していた
  `ros::NodeHandle`・`ros::Rate` メンバを除去（`InputPCAP` のレートは scalar 化）。
- `ssFrameLib.cpp/.h`: `<ros/ros.h>` を除去、msg ヘッダパスを更新（フレーム整形のみで ROS 非使用）。
- `point_types.h`（`RFANS_XYZ_S`）は手書きヘッダで ROS 非依存、変更なし。

### 5.4 launch
- `launch/rfans_driver.launch.py` を新規作成（旧 `node_manager.launch` 相当）。
  旧 `<param>` 群をノードパラメータとして渡す。
- 旧 `.launch`（`node_manager.launch` / `multi_lidar.launch`）は ROS 2 では起動できないため
  残置（参考）。マルチ LiDAR は将来対応。

### 5.5 RViz 操作パネル（意図的に除外）
- `src/teleop_pad.{cpp,h}` / `plugin_description.xml` / `cfg/FilterParams.cfg` は
  **ROS1 のまま残置し、ビルド対象から外した**（CMakeLists でコメントアウト）。
  rviz2 (`rviz_common`) のプラグイン API・ノード取得方法は ROS1 と大きく異なり、
  移植コストが高いため別タスクとした。
- 表示確認用に **`rviz/rfans_driver.rviz`（rviz2 形式）を新規追加**（既存の
  `*_Rviz_cfg.rviz` は ROS1 形式で rviz2 では読めないため）。`/sdk_could` を
  Intensity 色付け、Fixed Frame=`world` で表示する。使い方は §6.4。

### 5.6 リポジトリ側
- `Dockerfile`: apt パッケージに **`libpcap-dev`** を追加（`pcap.h` / `-lpcap` のため）。

## 6. 使い方

> 前提: `rfans_driver` を colcon のビルド対象にするため、epos4 同様に
> `src/rfans_driver -> external/StarROS2` の symlink を張る（`src/external/COLCON_IGNORE` で除外されているため）。

### 6.1 ビルド

```bash
colcon build --symlink-install --packages-select rfans_driver
source install/setup.bash
```

### 6.2 起動（launch）

launch は `model` / `device_ip` / `rps` / `read_fast` / `read_once` / `repeat_delay` を
引数で上書きできる。**既定は R-Fans-16**（`model:=R-Fans-16`）。

```bash
# 既定（R-Fans-16, 192.168.0.3, 10Hz）でそのまま起動
ros2 launch rfans_driver rfans_driver.launch.py

# 接続先 IP・スキャン速度を変える
ros2 launch rfans_driver rfans_driver.launch.py device_ip:=192.168.1.10 rps:=20

# 機種を R-Fans-32 に切り替える
ros2 launch rfans_driver rfans_driver.launch.py model:=R-Fans-32

# 実機レス（pcap/isf 再生）。readfile_path を与えるとファイル再生モードになる
ros2 launch rfans_driver rfans_driver.launch.py \
  readfile_path:=/path/to/sample.pcap read_once:=false
```

> 注: `readfile_path` は launch の宣言引数には含めていないため、上記の再生例では
> 直接ノードに渡す形が確実: `ros2 run rfans_driver driver_node --ros-args
> -p model:=R-Fans-16 -p readfile_path:=/path/to/sample.pcap -p read_once:=true`

### 6.3 点群・トピック・サービスの確認

```bash
# ノードが上がっているか
ros2 node list                      # /rfans_driver があること

# 点群が publish されているか（Hz が出れば配信中／無反応ならデータ源が無い）
ros2 topic hz /sdk_could

# 点群の中身（width>0 なら点が入っている）
ros2 topic echo /sdk_could --once | head

# スキャン速度 / エコー種別の制御
ros2 service call /rfans_driver/rfans_control rfans_driver/srv/RfansCommand \
  "{cmd: 1, speed: 10, use_double_echo: false}"

# フィルタの実行時調整（旧 dynamic_reconfigure 相当。点群ストリーミング中のみ反映 → §8）
ros2 param set /rfans_driver max_range 50.0
ros2 param set /rfans_driver use_laserSelection true
#   rqt_reconfigure からも調整可
```

### 6.4 RViz2 で点群を表示

rviz2 形式の設定 `rviz/rfans_driver.rviz` を同梱（既存の `*_Rviz_cfg.rviz` は **ROS1 形式**で
rviz2 では読めないため新規作成）。Fixed Frame=`world`／`/sdk_could` を Intensity 色付けで表示する。
点群は `header.frame_id="world"` を持ち **TF を publish しない**ので、Fixed Frame を `world` に
合わせれば変換不要で表示できる。

```bash
# ホスト側（1回だけ）: コンテナに X11 描画を許可
xhost +local:docker

# コンテナ内で driver 起動後、別ターミナルで RViz2
rviz2 -d src/external/StarROS2/rviz/rfans_driver.rviz
#   symlink/install 後なら:
#   rviz2 -d $(ros2 pkg prefix --share rfans_driver)/rviz/rfans_driver.rviz
```

トラブル時:
- RViz2 起動時の `QStandardPaths: XDG_RUNTIME_DIR not set` / `Stereo is NOT SUPPORTED` は
  **無害**（前者は `export XDG_RUNTIME_DIR=/tmp/runtime-root` で抑制可）。
  `OpenGl version: ...` が出ていれば描画は機能している。
- `ros2 topic hz /sdk_could` で Hz が出るのに RViz が真っ暗 → ほぼ Fixed Frame の不一致
  （`world` か、PointCloud2 Display が Enabled かつ Topic が `/sdk_could` かを確認）。
- Hz が出ない → R-Fans-16 実機が未接続、または `readfile_path` 未指定（データ源なし）。

## 7. 検証結果（コンテナ `rerobot_env` / Jazzy, 2026-06-13）

一時ワークスペース（`/tmp/vws`、`src/` を当パッケージへ symlink）で確認:

- ✅ `colcon build` 成功（rosidl 生成 → コンパイル → リンク）
- ✅ `ros2 interface show rfans_driver/{srv/RfansCommand,msg/RfansScan}` が正常生成を表示
- ✅ `ldd driver_node` で `libstar.so` がパッケージ install 配下から解決
  （`INSTALL_RPATH "$ORIGIN/.."` が機能）
- ✅ ノード起動で `/rfans_driver`、`/sdk_could`、`/rfans_driver/rfans_control`、
  フィルタ用パラメータ群、`set_parameters` サービスを登録
- ✅ ビルド成果物に RViz パネル（`librviz_teleop_commander.so`）が無い＝除外を確認

## 8. 注意点・既知の制約

- **コンテナに `libpcap-dev` が必要**。本改造で Dockerfile に追加済みだが、既存の
  稼働コンテナには `apt-get install -y libpcap-dev`、もしくは `docker compose up --build`
  で反映する。未導入だと `fatal error: pcap.h` でビルド失敗。
- **デフォルトではビルドされない**。`src/external/COLCON_IGNORE` で除外されているため、
  epos4 と同様に `src/rfans_driver` への symlink が必要。
- **パラメータ更新は点群ストリーミング中のみ反映**される。ノードは点ループ内でしか
  spin しない設計を ROS1 から忠実に踏襲したため、実機/再生データが流れていないと
  `ros2 param set` が完了せずハングして見える（リグレッションではなく仕様）。
  恒久対策が要るなら MultiThreadedExecutor 化や定期 spin の導入を別途検討する。
- **submodule のため変更の永続化には submodule 側リポジトリ（`StarROS2.git`）への
  コミットが必要**。reRoBot 側はポインタ更新のみ。
- RViz 操作パネルは未移植。GUI で制御したい場合は当面 `ros2 service call` か
  `ros2 param set` を使う。

## 9. 今後の課題

- RViz teleop パネルの rviz2 (`rviz_common::Panel`) への移植
- `multi_lidar.launch` 相当（名前空間分離の複数 LiDAR）の `.launch.py` 化
- パラメータ更新をデータ非ストリーミング時にも反映させたい場合の executor 見直し
