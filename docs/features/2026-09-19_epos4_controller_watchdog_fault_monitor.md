<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# epos4_controller の指令ウォッチドッグ・フォルト監視・CAN リンク喪失検知

- 日付: 2026-09-19
- 対象パッケージ: `epos4_controller` (`ros2_ws_main/src/app/epos4_controller/`)、パラメータは `rerobot_bringup/config/params.yaml`
- 対象 ROS: ROS 2 Jazzy (reRoBot コンテナ `rerobot_env`)、ros2_canopen (lely) + EPOS4 Compact 50/5 ×2
- 関連文書: `docs/issue/2026-09-19_epos_stop_link_loss_rpdo_timeout_power.md` (背景の事象と原因分析)、`docs/issue/2026-08-11_joy_spin_epos_shutdown_usb_stall.md`、`docs/issue/2026-09-13_decel_epos_stop_12v_loss.md` §5〜6 (EMCY が記録されない問題)

## 1. 目的・概要

2026-09-19 の実走・再現実験で、モータ停止系の故障に「ソフトが塞いでいない層」が 3 つあることが確定した。

1. **PDO は届くのに指令が更新されない型の暴走** (14:57)。EPOS4 は補間周期 10 ms 内に目標速度 PDO が来ないと 0x8250 (RPDO timeout) で自衛停止するが、master の受信側だけが死んだ状態でゲームパッドの USB が瞬断すると、joy_node は publish を止め (ゼロを出さない)、`epos4_controller` は最後の非ゼロ Twist を 10 ms 周期で送り続ける。
2. **フォルトコードが残らない**。ros2_canopen は EMCY をログに出さず、EPOS4 の Error history 0x1003 は電源断で消える。同型事象 5 回 (07-31 ×2、08-11、09-13、09-19 午前) で 1 つもコードが取れていなかった。
3. **can0 が作り直されても master が追従しない**。USB 再列挙で `can0 (unregistered)` → canusb-up.service が can0 を再作成しても、lely は旧 ifindex の socket を掴んだまま。SDO は全滅、PDO も届かないのに、スタックは「動いている」ように見える (15:35 後の 5 分間)。

実装した機能:
- 指令ウォッチドッグ: `/robot_speed_cmd` が `cmd_timeout_s` (既定 0.5 s) 途絶し、目標が非ゼロなら目標を 0 にして既存のランプで停止
- フォルト監視: statusword (0x6041) を `monitor_period_s` (0.5 s) で SDO ポーリングし、Fault ビット立ち上がりで 0x603F / 0x1003:00〜05 / 0x2200:01 を即読み → コード名付き ERROR ログ + `/diagnostics`
- CAN リンク喪失検知: `/sys/class/net/<can_interface>/ifindex` の変化・消滅、両ノード `link_loss_polls` 回連続の SDO 無応答、のいずれかで ERROR (2 s スロットル) + 目標 0 固定 + `/diagnostics` ERROR

スコープ外 (意図的に未対応):
- **EPOS 側 consumer heartbeat (0x1016)**: PDO が届き続ける型 (項目 1) には無効で、PDO が止まる型は既に 0x8250 が止めるため優先度を下げた
- **lely master の can0 再バインド**: ros2_canopen 側の変更が必要で、本パッケージからはできない。検知して再 launch を促すに留めた
- **candump 常時記録**: EMCY フレームを直接残す最も確実な手段だがホスト側の運用 (scripts) の話なので別タスク
- **0x60C2 (補間周期) の拡大・0x6077 SDO ポーリングの停止**: bus.yml (submodule) 側の設定衛生。本文書の対象外

## 2. 設計の勘所

### 2.1 ウォッチドッグは「非ゼロのときだけ」効かせる

判断: 途絶 = 即ゼロ、ではなく「途絶 **かつ** 目標が非ゼロ」のときだけ 0 に落とす。teleop_twist_joy は `require_enable_button: true` で LB を離すとゼロを 1 回出して沈黙する。Nav2 も到達後は publish を止める。これらの通常運用で毎回 WARN が出ると意味が薄れる。

実測: joy_node は `autorepeat_rate: 20` (joy_teleop.yaml) でスティック保持中も 20 Hz で Joy を出し、teleop_twist_joy は Joy ごとに Twist を出すので、走行中に 0.5 s 途絶することはない。keyboard teleop は 20 Hz publish。Nav2 controller_server は 20 Hz。→ 0.5 s は通常運用で発動しない下限に近い値。

却下した代替案:
- 途絶で disable (励磁を切る): 坂で転がる。既存ランプで 0 へ落とす方が安全で、EPOS の状態機械も触らない
- joy_node 側でゼロを出す修正: 上流パッケージの改造になる。他の指令元 (Nav2・keyboard) には効かない

### 2.2 監視は SDO ポーリング、スレッドは init と同じワーカー

判断: statusword は TPDO1 に載っているが、ros2_canopen の Cia402Driver はそれを topic に出さない。`/diagnostics` 経由の EMCY は bus.yml の diagnostics 設定次第で、しかも順序が保証されない。確実なのは既存の `sdo_read` サービスで 0x6041 を読むこと。0.5 s × 2 ノード = 4 read/s は、ドライバ自身が 0x6077 を ~10 Hz × 2 ノードで常時ポーリングしている負荷に対して無視できる。

実測 (09-19 17:00〜): 2 分の稼働で device_container の SDO タイムアウト 0 件、TPDO (`joint_states`) 20 Hz 維持。

実装: ブロッキング SDO (future 待ち) は executor スレッドでは呼べないので、`run_init_sequence` を回している `init_thread_` の末尾から `run_monitor_loop()` に続ける。監視スレッドは `m*_value_` を直接触らず `std::atomic<bool> force_zero_` を立て、`timer_callback` (executor) が読んで 0 にする — 07-31 以来の「目標と出力は executor スレッド専用」という不変条件を守る。

却下した代替案:
- 監視専用スレッドを増やす: init 中に SDO を並列で叩くと init の読み戻し (`motor_ready`) と競合し得る。1 本に直列化した
- `/diagnostics` の EMCY を購読: 出力されるか・遅延がどれだけかが bus.yml と lely の設定に依存し、今日まで一度も観測されていない経路。証拠が無いものに依存しない

### 2.3 Fault 検知時に「電源を切る前に読む」ことを自動化する

判断: 0x1003 は Backup NO (揮発)。運用ルール「フォルト後は非常停止のみ → 0x1003 を読む → 電源断」は人間が守れないことが今日 2 回証明された (15:00、15:04 に主電源を切ってコード喪失)。Fault ビットの立ち上がりを検知した瞬間に 0x603F、0x1003:00〜05 (最大 5 件)、0x2200:01 を読んで ERROR ログに固定すれば、その後電源を切っても launch ログと bag (`/diagnostics`) に残る。

コード名テーブルは EPOS4 Firmware Specification §7.2 から主要 20 種 (0x8250 RPDO timeout、0x3220 Undervoltage、0x3210 Overvoltage、0x2310 Overcurrent、0x81FD CAN bus off、0x8120 CAN passive、0x8130 Heartbeat、0x5113 Logic supply 等)。未知のコードは "unknown" で数値のみ出す。

### 2.4 リンク喪失は 3 経路の OR、判定は「安全側に倒すだけ」

- `ifindex` 変化: USB 再列挙の決定的な署名。master は追従できないので ERROR 文に「`scripts/stop.sh` → 再 launch が必要」を含める
- `ifindex` 消滅: アダプタ切断中
- 両ノード連続 SDO 無応答 × `link_loss_polls` (3): CANUSB ストール・CAN 配線・EPOS 電源断・ドライバ未 activate を区別せずまとめる。監視用 SDO の待ちは `monitor_sdo_timeout_s` (0.5 s) に短縮し、検知まで約 1.5〜3 s

判断: リンク喪失時の唯一の動作は「目標を 0 に固定」で、これは誤検知でも害が無い (PDO が届いていれば止まるだけ、届いていなければ何も起きない)。復帰は SDO が返った時点で自動 (「CAN リンク復帰」WARN)。

## 3. データフロー

```
/robot_speed_cmd (geometry_msgs/Twist, joy/keyboard/Nav2)
        │ cmdSpeedCallback: last_cmd_time_ 更新, m*_value_ 更新           [executor]
        ▼
timer_callback 10 ms                                                    [executor]
   ├─ force_zero_ (atomic) が true → m*_value_ = 0
   ├─ now − last_cmd_time_ > cmd_timeout_s かつ目標 ≠ 0 → m*_value_ = 0, WARN
   └─ ramp_toward → /motor{1,2}/cia402_device_{1,2}/tpdo (COData 0x60FF)

init_thread_ : run_init_sequence → run_monitor_loop (0.5 s 周期)       [worker]
   ├─ read /sys/class/net/can0/ifindex  → 変化/消滅
   ├─ sdo_read 0x6041 ×2 ノード (canopen_interfaces/srv/CORead)
   │     Fault bit ↑ → sdo_read 0x603F, 0x1003:00..05, 0x2200:01 → RCLCPP_ERROR
   ├─ 両ノード連続失敗 ≥ link_loss_polls → link_lost
   ├─ force_zero_.store(link_lost) → (executor が目標 0)
   └─ /diagnostics (diagnostic_msgs/DiagnosticArray)
         ├ epos4_controller/can_link      OK / ERROR(理由文)   values: ifindex, force_zero, cmd_timed_out
         ├ epos4_controller/motor1(right) OK / ERROR(FAULT 0x…) / STALE(SDO 無応答 ×n)
         └ epos4_controller/motor2(left)  values: statusword, error_code, error_history, supply_voltage_V
```

## 4. 使い方

追加操作は不要。`rerobot_bringup.launch.py` (scripts/bringup2d.sh 等) で controller が上がれば自動で有効。

```bash
# 監視状態を見る (bag に含めるなら /diagnostics を記録対象に)
docker exec rerobot_env bash -c 'source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; export ROS_DOMAIN_ID=150; ros2 topic echo /diagnostics'
# controller のログだけ追う
docker exec rerobot_env bash -c 'tail -f $(ls -t /root/.ros/log/epos4_controller_*.log | head -1)'
```

| パラメータ (`epos4_controller_node`) | 既定 | 説明 |
|---|---|---|
| `cmd_timeout_s` | 0.5 | 指令途絶の判定秒。0.0 で無効 |
| `monitor_period_s` | 0.5 | statusword ポーリング周期 [s] |
| `monitor_sdo_timeout_s` | 0.5 | 監視用 SDO 読みの待ち [s] |
| `link_loss_polls` | 3 | 両ノード連続 SDO 無応答でリンク喪失と判定する回数 |
| `can_interface` | "can0" | ifindex を監視するインタフェース名 |

ログの読み方 (実機で出たもの):
```
[WARN]  指令ウォッチドッグ: /robot_speed_cmd が 0.50 s 途絶 (最後の目標 m1=-138 m2=-138 rpm) → 0 へランプ
[ERROR] motor1(right) FAULT: statusword=0x0208 error=0x3220 (Undervoltage (...)) history[1]=[ 0x3220 ] Vcc=8.6 V — コードは記録済み。...
[ERROR] CAN リンク喪失: can0 が消滅 (USB アダプタ切断?) → 目標速度を 0 に固定 (EPOS 側は 0x8250 で自衛)
[ERROR] CAN リンク喪失: can0 が作り直された (ifindex 36 → 37): master は旧 socket のまま → scripts/stop.sh → 再 launch が必要 → ...
[ERROR] CAN リンク喪失: 両ノードが SDO 無応答 ×3 (CANUSB ストール / CAN 配線 / EPOS 電源断) → ...
```

実機検証 (2026-09-19 17:00〜17:45、ロボット静止、ユーザ操作):
1. 非常停止押下 → 0.5 s 以内に両モータ `FAULT … 0x3220 … Vcc=8.6 / 8.1 V` (RSD 出力の放電中の値) を記録 ✅
2. CANUSB 抜き差し → 「can0 が消滅」→「作り直された (36 → 37)」を 2.2 s 間隔で ERROR ✅ (`/diagnostics` の can_link も同文)
3. 非常停止押下中 (Vcc 0 V、車輪は動かない) に `ros2 topic pub --once /robot_speed_cmd … x: 0.02` → 0.5 s 後に WARN、目標 0 ✅
4. 副作用: device_container の SDO タイムアウト 0 件、`joint_states` 19.99 Hz

## 5. 変更ファイル一覧

- `ros2_ws_main/src/app/epos4_controller/src/epos4_controller.cpp` — ウォッチドッグ (timer_callback / cmdSpeedCallback)、`read_sdo` の subindex・timeout 引数化、`run_monitor_loop` / `poll_motor` / `publish_diagnostics` / `epos4_error_name` / `read_can_ifindex` 追加、init サービス不在時も監視を開始するよう分岐変更 (タグ `claude_watchdog`)
- `ros2_ws_main/src/app/epos4_controller/CMakeLists.txt`, `package.xml` — `diagnostic_msgs` 依存追加
- `ros2_ws_main/src/bringup/rerobot_bringup/config/params.yaml` — 新パラメータ 5 つ (`epos4_controller_node`)

## 6. 既知の制限

- **非常停止押下中 (+Vcc 無し) に launch すると監視開始が遅い**: init の再試行 (5 回 × init/enable/csv 各 5 s タイムアウト + recover) が両モータで約 3.5 分かかり、その間 `run_monitor_loop` は始まらない。ウォッチドッグ (executor 側) は起動直後から有効。改善案: init の service timeout を短縮するか、監視を別スレッドにして init と並走させる
- **リンク喪失の検知遅れ ≈ 1.5〜3 s** (SDO 待ち 0.5 s × 3 回)。PDO が届き続ける型の暴走は、この検知よりウォッチドッグ (0.5 s) の方が先に効くことを期待している (パッド喪失なら Twist が止まる)。**指令元が生きたまま master の受信側だけ死ぬ**場合 (14:57 の SDO 死から 13 s 後にパッドが切れるまで) は、指令が続く限り走り続ける — これは仕様上「正しい指令が届いている」状態で、止める根拠が無い
- `/diagnostics` は urg_node も publish しており、rate は合算 (~4 Hz) になる
- `read_can_ifindex` はコンテナが `network_mode: host` であることに依存 (別ネットワーク名前空間では /sys が見えない → 常に −1 = 「消滅」誤検知。その場合 `can_interface` を空にする等の無効化手段は未実装)
- driver の `sdo_read` サービス呼び出しごとに device_container 側が INFO ログ (`SDO Read Call index=0x6041`) を 1 行出す (~4 行/s、約 170 KB/h)
- 監視の SDO 読みは master の SDO クライアントを共有するので、再 launch を伴わない CANUSB の半死状態 (送信可・受信不可) では driver 未 activate と区別できず、同じ「SDO 無応答」として報告する (意図どおりだが原因分離はできない)
