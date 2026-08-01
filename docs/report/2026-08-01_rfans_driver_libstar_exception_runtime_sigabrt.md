<!-- claude: docs/report テンプレート。debug-report スキルから生成。-->
# rfans_driver の起動直後 SIGABRT (libstar.so の例外ランタイム横取り)

- 日付: 2026-08-01
- 環境: Ubuntu ホスト / コンテナ `rerobot_env` / ROS 2 Jazzy / FastDDS 2.14 / rfans_driver (StarROS2 submodule, プリビルド `libstar.so` 同梱)
- 対象ブランチ: `feat/workspace-split` (HEAD: `82edfef feat: update slam_toolbox configuration for improved map regeneration and scan matching thresholds`)
- 関連ファイル:
  - `ros2_ws_main/src/drivers/StarROS2/lib/libstar.so` (原因: ベンダー製プリビルド blob)
  - `ros2_ws_main/src/bringup/rerobot_bringup/launch/rerobot_bringup_3d.launch.py` (修正)
  - `ros2_ws_main/src/drivers/StarROS2/launch/rfans_driver.launch.py` (修正)

## TL;DR

`rerobot_bringup_3d` で点群 `/sdk_could` が出ない原因は、`rfans_driver` の `driver_node` が起動直後 (~90 ms) に無言で SIGABRT していたこと。プリビルドの `libstar.so` が古い C++ 例外ランタイム (`__cxa_throw`, `__cxa_begin_catch`, `_Unwind_RaiseException`) を export しており、同一 ROS ドメインに他ノードが先住していると FastDDS のポート衝突時の正常系例外の unwind が壊れて abort する。両 launch の rfans `Node` に `additional_env` で `LD_PRELOAD=libstdc++.so.6:libgcc_s.so.1` を追加して正規ランタイムを優先解決させ解消。修正後は他ノード稼働中でも生存し、点群 ~6 Hz / 13,035 点/メッセージを確認。

## 症状

- **いつ**: `ros2 launch rerobot_bringup rerobot_bringup_3d.launch.py` 起動時、および `ros2 launch rfans_driver rfans_driver.launch.py` 単独起動時 (他ノードが同一ドメインに残存している状態では 100% 再現)。
- **どこで**: `rerobot_env` コンテナ内の `driver_node` プロセス。
- **何が起きるか**: プロセス開始から約 90 ms で exit code -6 (SIGABRT)。**stdout / stderr とも完全に無出力**で、`terminate called ...` すら出ない。結果として `/sdk_could` トピックが存在しない。
- **正常時**: `rfans_driver` ノードが常駐し、`/sdk_could` (PointCloud2) が出続ける。

```
1785563371.0075965 [ERROR] [driver_node-3]: process has died [pid 4180, exit code -6,
  cmd '/workspace/install/rfans_driver/lib/rfans_driver/driver_node --ros-args -r __node:=rfans_driver
  --params-file .../config/params_3d.yaml --params-file /tmp/launch_params_26y4kyr3'].
```

なお同時に `device_container_node` も exit -6 で死んでいたが、これは別原因 (ホストで `can0` 未作成 = `can_up.sh` 未実行) であり、本レポートの対象外。

## 切り分けの記録

### 1. スタック診断 — ドライバ不在の確認

「点群が出ない」に対し、まず `ros2 node list` / `ros2 topic list` を確認。`rfans_driver` ノードと `/sdk_could` トピックが**存在しない**ことが判明 (点群のデータ経路以前の問題)。同時に、bringup_3d が起動しないはずの `/rviz` が残存していた — 後から見るとこれが発火条件だった。

| 項目 | 観測値 | 備考 |
|------|--------|------|
| ros2 node list | rfans_driver 不在、/motor1・/motor2 も不在 | 残存 /rviz, /epos4_* あり |
| launch.log | driver_node と device_container_node が exit -6 | 開始から ~90 ms |
| 手動起動 (stdout/stderr 分離取得) | 両方とも**完全に空**のまま core dump (exit 134) | terminate メッセージすら無し |

### 2. can0 の確認 (device_container 側の原因、当たり — ただし別問題)

ホストで `ip link show can0` → "Device \"can0\" does not exist"。`device_container_node` の即死はこれで説明できる (`can_up.sh` 未実行)。**ただし rfans_driver は Ethernet/UDP なので説明にならない** → 別原因を継続調査。

### 3. LiDAR 未接続が原因という仮説 (外れ)

「LiDAR が繋がっていないから」はこの症状の説明にならない: 接続が無い場合ドライバは起動して待ち受け、点群が空になるだけのはず。実際、後の検証で接続の有無に関係なく abort した。

### 4. gdb バックトレース (当たり)

無言 SIGABRT のため gdb (コンテナに `apt-get install gdb` で一時導入) で直接実行。

```
#4  __GI_abort ()
#6  __gcc_personality_v0 ()            from /lib/x86_64-linux-gnu/libgcc_s.so.1
#7  _Unwind_RaiseException_Phase2 ()   from /workspace/install/rfans_driver/lib/libstar.so  ← 注目
#9  __cxa_throw ()                     from /workspace/install/rfans_driver/lib/libstar.so  ← 注目
#12 eprosima::fastdds::rtps::UDPv4Transport::OpenAndBindInputSocket(...) from libfastrtps.so.2.14
```

FastDDS が投げた例外の `__cxa_throw` / unwinder が **libstar.so 内のコピー**に解決されている。`nm -D libstar.so` で裏取り: `T __cxa_throw`, `T __cxa_begin_catch` を export していた (古い libstdc++/libgcc の静的リンク漏れ)。

### 5. 発火条件の特定 — DDS ポート衝突

`UDPv4Transport::OpenAndBindInputSocket` の例外は「候補ポートが埋まっていたら投げて catch して次を試す」という FastDDS の**正常系**。つまり同一ドメインに先住ノードがいるときだけ通る経路。検証:

| 条件 | 結果 |
|------|------|
| ROS_DOMAIN_ID=151 (誰もいない) で単独起動 | **正常動作** (timeout まで生存, exit 0) |
| ROS_DOMAIN_ID=150 (他ノード稼働中) で起動 | 即 SIGABRT (100% 再現) |
| 150 + `LD_PRELOAD=libstdc++.so.6:libgcc_s.so.1` | **正常動作** |

## 根本原因

**シンボル横取り (symbol interposition)**。動的リンカは同名シンボルを「グローバル検索順で先に見つかった方」に解決する。プリビルドの `libstar.so` は古い libstdc++/libgcc を静的リンクしたまま `__cxa_throw` / `__cxa_begin_catch` / `_Unwind_RaiseException` 等を export しており、`driver_node` の依存順で本物の libstdc++ より先に解決される。その結果:

1. 起動時、FastDDS が DDS participant 用 UDP ポートを bind 試行 → 先住ノードが押さえていると `asio::system_error` を throw (正常系。catch して次ポートへ再試行する設計)
2. その throw が libstar.so 内の**古い** `__cxa_throw` / unwinder で処理される
3. 巻き戻し中に system の `__gcc_personality_v0` と不整合 → unwinder が続行不能と判断し **即 abort**。terminate ハンドラを経由しないため**エラーメッセージが一切出ない**

**なぜ今まで顕在化しなかったか**: 例外経路は「起動時に候補ポートが埋まっている」ときしか通らない。従来はまっさらな状態で bringup を一発起動しており衝突が起きなかった (= バグは潜伏)。今回はコンテナ内に前回セッションの RViz / epos4 系ノードが残ったまま起動したため、後発の driver_node が必ず衝突経路に入り、100% 再現になった。構成変更 (workspace 分割等) とは無関係。

## 修正

2 つの launch ファイルの rfans `Node` に `additional_env` で LD_PRELOAD を追加 (対象プロセス限定で正規ランタイムを最優先解決させる)。**要 colcon 再ビルド** (`rfans_driver` + `rerobot_bringup`、launch は install 空間へコピーされるため)。

- `ros2_ws_main/src/bringup/rerobot_bringup/launch/rerobot_bringup_3d.launch.py`
- `ros2_ws_main/src/drivers/StarROS2/launch/rfans_driver.launch.py` (submodule 側 — コミット時は submodule → 親 gitlink の順)

```python
        # claude: libstar.so (ベンダー blob) が古い C++ 例外ランタイム (__cxa_throw 等) を
        #   export しており、FastDDS がポート衝突時に投げる正常系例外の unwind を横取りして
        #   SIGABRT で即死する (他ノードが同一ドメインにいると 100% 再現)。正規の
        #   libstdc++/libgcc を先に解決させて無効化する (2026-08-01)。
        additional_env={
            "LD_PRELOAD": "/usr/lib/x86_64-linux-gnu/libstdc++.so.6:"
                          "/lib/x86_64-linux-gnu/libgcc_s.so.1"
        },
```

### なぜこの値 / この方法か

- **LD_PRELOAD は検索列の先頭に割り込む**ため、libstar.so の export より必ず先に本物が解決される。バイナリ非改変・可逆で、効果は gdb バックトレースの因果と 1:1 に対応する (対症ではなく機序を断つ)。
- **launch の `additional_env` に閉じたのは影響範囲の限定のため**。コンテナ全体の環境変数にすると全プロセスに波及し、無関係な挙動変化の疑いを常に持ち込む。
- **採らなかった選択肢**: (a) libstar.so の再ビルド — ソース非公開で不可能。(b) `objcopy` で export シンボルの局所化 — dynsym には効かない。(c) rfans_driver の CMake でリンク順操作 — 効果が ld の実装依存で不確実。
- **破綻条件**: ベースイメージ更新で libstdc++/libgcc のパスが変わったとき (パスをハードコードしているため)。その場合 launch がエラーになるので気付ける。メーカーが libstar.so を `--exclude-libs` 付きでビルドし直せばこの回避策ごと削除できる。

## 検証

| 項目 | 修正前 | 修正後 |
|------|--------|--------|
| driver_node 生存 (他ノード稼働中の DOMAIN 150) | 即 SIGABRT (100% 再現) | 常駐・正常動作 |
| `/sdk_could` 周波数 | トピック自体が不在 | 実測 平均 5.8–6.4 Hz (window 15–20) |
| `/sdk_could` 点数 | — | width=13,035 点/メッセージ |
| LiDAR パケット到着 (tcpdump) | 未計測 (当時未接続) | 192.168.0.3:2014 → broadcast:2014, 1206 B, 連続到着 |

検証手順:

```bash
# 1. LiDAR パケットが PC まで来ているか (ドライバより手前の切り分け)
docker exec rerobot_env tcpdump -i <ethernet_if> -c 5 -nn udp   # 192.168.0.3.2014 発が見えること

# 2. 他ノードがいる状態で起動して生存確認 (旧: ここで即死していた)
docker exec rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash \
  && export ROS_DOMAIN_ID=150 && ros2 launch rfans_driver rfans_driver.launch.py"

# 3. 点群の出力確認
ros2 node list | grep rfans          # /rfans_driver がいること
ros2 topic hz --window 20 /sdk_could # ~6 Hz
ros2 topic echo --once /sdk_could --field width
```

補足の観測 (異常ではないが記録):
- `bind message: Address already in use` が起動時に毎回出る (`StarROS2/src/ioapi.cpp:83`)。libstar.so 内部が先に UDP 2014 を bind しているためと推測 (推測: 点群が出ている実績と両立するので実害なしと判断)。点群が出ないのにドライバが生きているケースではここを再調査する。
- 単体 `rfans_driver.launch.py` は `frame_id: "world"` 設定。TF と繋がる `rfans` になるのは bringup_3d 経由 (params_3d.yaml) のみ。RViz/SLAM 用途では bringup_3d を使うこと。

## 教訓 / 今後の予防

1. **launch 前に前回セッションのノードを片付ける** (`./scripts/stop.sh`)。「昨日まで動いた/今日死ぬ」の差が残留プロセス起因のことがある — 今回の発火条件そのもの。
2. **無言の exit -6 (SIGABRT) は gdb バックトレース一択**。terminate メッセージを出す機構自体が壊れているサインで、ログをいくら眺めても情報は増えない。
3. **ベンダー製プリビルド `.so` を導入したら `nm -D <lib> | grep -E '__cxa|_Unwind'` を確認する**。C++ ランタイムシンボルを export している blob はプロセス全体の例外処理を乗っ取り得る。
4. センサ系の切り分けは**データの旅路を上流から順に**: 配線/IP (`ip addr`) → パケット到着 (`tcpdump`) → 受信プロセス生存 (`ros2 node list`) → 出力 (`ros2 topic hz`)。
5. フォロアップ TODO: `/sdk_could` が rps=10 設定に対し実測 ~6 Hz とやや低い。点数は正常なので保留とするが、SLAM で遅延が問題になったら再調査。
