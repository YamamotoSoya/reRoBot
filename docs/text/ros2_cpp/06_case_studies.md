<!-- claude: ROS 2 C++ 構造読本 第6章 (2026-08-12)。実際に起きた 4 事故を言語機構の
     観点から解剖。事実は一次資料 (docs/report, docs/claude/knowledge) に従う。 -->

# 第6章 事例集 — 実際に壊れたとき何が起きていたか

前章までの 5 つの機構が「壊れるとどう見えるか」を、本プロジェクトで実際に起きた
事故で確かめる。各事例は **症状 → 機構 → 修正 → 教訓** の 4 部で解剖する。
事実関係はすべて一次資料 (各事例の冒頭に明記) に基づく。

| 事例 | 一言でいうと | 例証する機構 | 状態 |
|---|---|---|---|
| [A](#事例a-init-連射レースで片輪が動かない) | init 連射レースで片輪が動かない | コンストラクタと spin の時系列・future・スレッド (第5章) | 2026-06-04 解決 |
| [B](#事例b-velocity-は値があるのに常に-0) | velocity は「値があるのに常に 0」で EKF 停止 | 防御的分岐の沈黙フォールバック (第4章の読解力) | 2026-08-11 解決 |
| [C](#事例c-起動-90-ms-で無言の-sigabrt) | 起動 90 ms で無言の SIGABRT | 例外はランタイム機構・シンボル横取り (第2章 2.4 の裏面) | 2026-08-01 解決 (回避策) |
| [D](#事例d-パラメータ既定値で距離表示が-5-倍) | パラメータ既定値で距離表示が 5 倍 | declare_parameter の静かな既定値 | 2026-08-11 対策 |
| (E) | YAML キー 1 個 (`ros__parameters`) の typo でノード即死 | パラメータの結線 (事例D 内で言及) | 恒常の罠 |

## 事例A: init 連射レースで片輪が動かない

- 一次資料: `docs/report/2026-06-04_epos4_controller_init_race_dead_wheel.md`
- 対象コード: `epos4_controller.cpp` (修正後の現行コードが :121-127, :353-370, :418-449)

### 症状

teleop で速度指令を与えると**片輪だけ回らない**ことが頻発する。エラーは一切
出ない。指令 (TPDO 目標速度) は両モータへ正しく届いている。そして間欠的 —
report の実測 (修正前、再起動 ×8):

```
iter 1: dLEFT=-1.0707   dRIGHT=+0.0000     <- 右輪 無回転
iter 2: dLEFT=-1.7410   dRIGHT=+0.0000     <- 右輪 無回転
iter 4: dLEFT=-1.6045   dRIGHT=+0.0000     <- 右輪 無回転
iter 7: dLEFT=-1.8054   dRIGHT=-1.7962     <- 両輪OK ← 「たまに動く」
iter 8: dLEFT=-1.9757   dRIGHT=+0.0000     <- 右輪 無回転
```

当時この現象は「本当に原因不明。動くか動かないかは確率。build のバグかも?」と
記録されていた。**「確率」「再現しない」は race condition の症状名である**
([第5章 5.5](05_spin_concurrency.md)) — ここからその決定的な機構を特定する。

### 機構

旧コードはコンストラクタで 6 つの状態遷移サービスを連射していた:

```cpp
// 旧実装 (report より): 応答を待たず 6 連射
call_trigger_service(m1_client_driver_init_, "init");
call_trigger_service(m1_client_driver_enable_, "enable");
call_trigger_service(m1_client_driver_csv_mode_, "cyclic_velocity_mode");
call_trigger_service(m2_client_driver_init_, "init");
// ... (m2 も同様)
```

言語機構としての問題は 2 段になっている:

1. `call_trigger_service` は内部で `async_send_request` を呼んで**即 return** する
   (撃ちっぱなし)。前段の遷移完了を確認せず次を撃つので、ドライバ側で遷移が
   取りこぼされる余地が生まれる
2. さらに**コンストラクタは spin より前に走る** ([第5章 5.3](05_spin_concurrency.md))
   ので、待とうにも待てない。応答 future を ready にする executor がまだ存在しない —
   旧実装が「撃ちっぱなし」だったのは、実は**それしか書きようがなかった**からである

失敗したモータは init (homing) モードに取り残され (SDO 直読みで statusword
`0x0240` = Switch On Disabled / mode `6` = Homing)、目標速度を受け付けない。
どちらの輪が落ちるかは「2 番目に初期化される側が落ちやすい」— 6 サービスが
数 µs 間隔で同一 CAN バス上の 2 ドライバへ殺到するため、先発の遷移トラフィックに
後発の enable/csv が重なる。タイミング勝負だから間欠的に見えた、という筋である。

### 修正

「spin が始まったあとの世界で、待ちながら順に撃つ」に構造を変えた (現行コード):

```cpp
init_thread_ = std::thread(&Epos4_Control2_Node::run_init_sequence, this);  // :127
```

- **逐次化**: ワーカースレッド内の `call_trigger_sync` (:353-370) が各応答の
  future を `wait_for` で待ってから次を発行 (executor が配達係になれるので待てる)
- **検証**: 遷移サービスの戻り値ではなく、実機の真値 (statusword 0x6041 /
  mode 0x6061) を SDO で読んで Operation Enabled + CSV を確認 (`motor_ready` :398-413)
- **リトライ**: 未達なら recover して最大 5 回やり直す (`init_motor` :418-449)

検証結果 (report): 修正前 約 7/8 で片輪失敗 → 修正後 **10/10 回 attempt 1 成功**
(リトライ発動ゼロ = 逐次化だけでレースは消えた)。

### 教訓

- **コンストラクタ内で発行した非同期要求の完了は、そのコンストラクタ内では絶対に
  確認できない**。初期化シーケンスはワーカースレッド (or 起動後のコールバック) に置く
- 「成功した」をサービスの戻り値だけで判断せず、実機の真値で検証する
- 「動くかどうかは確率」と言いたくなったら race condition を疑い、
  **「実行の流れは何本か・共有データは何か」を数える** (第5章 5.5 の判定樹形図)

## 事例B: velocity は「値があるのに常に 0」

- 一次資料: `docs/claude/knowledge/2026-08-11_ekf_odometry_code_walkthrough.md`
  (体系版: [EKF センサ融合読本](../ekf_fusion/00_index.md))
- 対象コード: `epos4_odometry.cpp:151-158` (修正後)

### 症状

EKF を導入した日 (2026-08-11)、`/odometry/filtered` の位置が進まない。
実走 bag で確認すると `/odom` の **pose は 20 m 動いているのに twist は常に
ほぼ 0**。SLAM だけで運用していた時期 (位置しか使わない) は完全に無症状だった。

### 機構

旧コードはこういう防御的分岐を持っていた:

```cpp
// 旧実装のロジック (要旨): velocity 配列が入っていればそれを信用する
if (!msg->velocity.empty()) {
  v = msg->velocity[0];      // ← 常にこちらを通り、値は常に 0
} else {
  v = 位置差分 / dt;          // ← 一度も通らない
}
```

`empty()` が答えるのは「**配列に要素があるか**」だけで、「**その値に意味があるか**」
ではない。ドライバは velocity 配列を常に確保して publish する (要素はある) が、
bus.yml で 0x606C (velocity actual) が PDO マッピングされていないため**値は常に 0**
(意味がない)。データの意味論は型にも empty() にも現れず、コードの外 (bus.yml) に
あった — 防御的に書いたはずの分岐が、**静かに間違った側へ倒れ続ける
フォールバック**になっていた。

### 修正

分岐を廃止し、常に位置差分から速度を導出する (現行 odometry:151-158)。修正後の
コードには機構の説明と実証 (bag での確認) がコメントとして残されており、
「意味論をコメントで明示する」の見本になっている:

```cpp
// The driver's joint_states.velocity is ALWAYS zero (0x606C is not PDO-mapped
// in bus.yml) — confirmed from the 2026-08-11 ekf_test bag, ...
double v_lin = d_s / dt;     // :157
double v_ang = d_theta / dt;
```

### 教訓

- 「値が存在する」と「値に意味がある」は別。**意味論はコードから読めない**ので、
  出所 (この場合 bus.yml の PDO マッピング) まで遡って確認する
- フォールバック分岐を書くなら「いまどちらを通っているか」を起動時に 1 度ログする。
  沈黙する分岐は、下流の消費者が変わった日 (SLAM → EKF) に突然発症する

## 事例C: 起動 90 ms で無言の SIGABRT

- 一次資料: `docs/report/2026-08-01_rfans_driver_libstar_exception_runtime_sigabrt.md`
- 対象: `rfans_driver` (StarROS2 submodule) のベンダー製プリビルド `libstar.so`

### 症状

3D bringup で点群トピックが出ない。`driver_node` プロセスが起動から約 90 ms で
exit code -6 (SIGABRT)。**stdout / stderr とも完全に無出力** — `terminate called ...`
のエラーメッセージすらない。他ノードが同一 ROS ドメインに残存しているときだけ
100% 再現し、まっさらな環境では動く。

### 機構

C++ の例外 (`throw` / `catch`) は構文糖ではなく、**ランタイム関数群
(`__cxa_throw`、スタック巻き戻しの unwinder) で実装される機構**である。そして
動的リンカは同名シンボルを「検索順で先に見つかった方」に解決する (シンボル横取り)。

ベンダー blob `libstar.so` が古い libstdc++/libgcc を静的リンクしたまま
`__cxa_throw` 等を export していたため、gdb バックトレースでこうなっていた:

```
#7  _Unwind_RaiseException_Phase2 ()  from .../libstar.so   ← 本物でない unwinder
#9  __cxa_throw ()                    from .../libstar.so   ← 本物でない throw
#12 eprosima::fastdds::rtps::UDPv4Transport::OpenAndBindInputSocket(...)
```

発火条件が絶妙で、FastDDS は起動時「候補ポートが埋まっていたら例外を投げ、catch
して次のポートを試す」という**正常系で例外を使う**。先住ノードがいるときだけ
この経路に入り、新旧混在のランタイムで巻き戻しが破綻 → terminate ハンドラを
経由せずに abort → だから**無言**。odometry:52-55 の `throw std::runtime_error`
(コンストラクタで投げて起動を失敗させる) が健全な例外の使い方であるのと対照的に、
これは例外そのものではなく「例外を support する言語ランタイムが壊された環境」の
事故である。

### 修正

launch の該当 Node に `LD_PRELOAD=libstdc++.so.6:libgcc_s.so.1` を追加し、正規の
ランタイムを検索列の先頭で解決させた (バイナリ非改変・対象プロセス限定)。
修正後は他ノード稼働中でも常駐し、点群 ~6 Hz を確認。

### 教訓

- **無言の SIGABRT は gdb バックトレース一択**。エラーメッセージを出す機構ごと
  壊れているサインなので、ログをいくら眺めても情報は増えない
- ベンダー製プリビルド .so を導入したら `nm -D <lib> | grep -E '__cxa|_Unwind'` —
  C++ ランタイムシンボルを export する blob はプロセス全体の例外処理を乗っ取り得る
- 例外は「言語の構文」ではなく「プロセスで 1 セットのランタイム機構」と理解して
  おくと、この種の事故の存在自体が想像できるようになる

## 事例D: パラメータ既定値で距離表示が 5 倍

- 一次資料: `docs/claude/knowledge/2026-08-11_ekf_odometry_code_walkthrough.md` ④
- 対象コード: `teleop_keyboard.cpp:68-78` (対策後)

### 症状

teleop の走行距離表示が実際の 5 倍 (実走 10 m が 51 m 表示)。エラーも警告もなし。

### 機構

`declare_parameter("gear_ratio", 1.0)` (teleop:50) の第 2 引数は既定値で、
`--params-file` を渡し忘れると**黙って 1.0 が採用される**。この機体の実ギヤ比は
5.0 なので距離が 5 倍になる。パラメータは「宣言 (コード)・供給 (YAML)・結線
(--params-file 引数)」の 3 点が揃って初めて意図の値になるが、結線が欠けたときの
挙動が「エラー」ではなく「静かな既定値」なのである。

### 修正 (緩和)

起動時に採用値を必ずログし、既定値のままなら WARN を出す (teleop:68-78):

```cpp
RCLCPP_INFO(get_logger(), "teleop params: tire_diam=%.3f m, gear_ratio=%.2f", ...);
if (gear_ratio_ == 1.0) {
  RCLCPP_WARN(get_logger(),
    "gear_ratio=1.0 (default) — did you forget --params-file? ...");
}
```

**併記 (事例E)**: 逆方向の極端が `ros__parameters` (アンダースコア 2 個) の
typo である。`ros_parameters` と書くと YAML 全体が解釈されず、ノードは起動時に
`RCLInvalidROSArgsError` で**即死**する (CLAUDE.md 記載の恒常の罠)。静かな既定値
(D) と即死 (E) は、同じ「コードと YAML の結線失敗」の両極端であり、即死のほうが
まだ親切 — D の対策 (せめて WARN を出す) はその発想である。

### 教訓

- **起動時に採用値をログする**のは、パラメータ事故への最も安い防御
- `declare_parameter` の既定値は「安全なフォールバック」に見えて、機体固有値
  (ギヤ比・トレッド幅) では「静かに間違う既定」になる。既定値で動いて困るものは
  検知の仕掛けを添える

## 章のまとめ — 4 事例を貫く一本の線

```
4 事例を貫く線 — 「エラーが出ない失敗」
├── A: レース ............ エラーなし。タイミングで結果が変わるだけ
├── B: 沈黙フォールバック . エラーなし。twist が 0 という「正常な形の嘘」が流れ続ける
├── C: 無言 abort ........ エラーメッセージを出す機構ごと壊れる
└── D: 静かな既定値 ...... エラーなし。数字が 5 倍なだけ
```

C++/ROS 2 の失敗は「例外や assert で騒ぐ」形より、**静かに違う値が流れる・静かに
違う経路を通る**形が圧倒的に多い。機構を学ぶ実利は、書けるようになること以上に、
**「このコードはどこで沈黙し得るか」の当たりを付けられる**ようになることである。

→ [第7章 実務編](07_reference.md)
