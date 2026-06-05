# モータ脱力(フリー)モードの追加

- 日付: 2026-06-05
- 対象パッケージ: `epos4_controller`, `epos4_teleop`
- 関連: [init レース / dead wheel レポート](../report/2026-06-04_epos4_controller_init_race_dead_wheel.md)

## 1. 目的・概要

teleop のキーボードから **モータを「脱力(フリー)」状態にトグルできる**ようにした。
脱力中は両 EPOS4 を非励磁(de-energize)にするため、巻線にトルクがかからず
**車輪を手で自由に回せる**。もう一度トグルすると `enable` + cyclic synchronous
velocity (CSV) モードを掛け直して通常の駆動状態に復帰する。

用途:
- ロボットを手で押して移動位置を直したいとき
- 調整・搬送・緊急時に車輪をフリーにしたいとき
- モータをホールドトルクで固めたくないとき

## 2. CiA 402 的な「脱力」の意味

EPOS4 は CiA 402 ステートマシンで動く。通常駆動は **Operation Enabled** 状態で、
このとき指令速度 0 でもサーボはその場を保持しようと励磁し続ける(ホールドトルク)。

`disable` サービスを呼ぶと **Operation Enabled → Switched On** へ遷移し、巻線が
非励磁になる。これが「脱力」。復帰には `enable`(Operation Enabled へ)と
`cyclic_velocity_mode`(mode of operation = 9 / CSV)の再投入が必要。

```
通常駆動 (Operation Enabled, CSV)
    │  disable         ← 脱力 ON
    ▼
Switched On (非励磁=フリー)
    │  enable + cyclic_velocity_mode  ← 脱力 OFF(復帰)
    ▼
通常駆動 (Operation Enabled, CSV)
```

## 3. データフロー(新設チャネル)

teleop とコントローラ間は従来 `geometry_msgs/Twist`(`/robot_speed_cmd`)の 1 本のみ
だった。脱力はライフサイクル操作なので、その所有者である `epos4_controller` に
**新トピック `/robot_free_mode`(`std_msgs/Bool`)** を生やし、teleop は単に
トグルを publish するだけにした(責務分離)。

```
[teleop_keyboard]
    │ std_msgs/Bool on /robot_free_mode   (true=脱力 ON, false=復帰)
    ▼
[epos4_controller]
    │ /motor{1,2}/.../disable  または  init→enable→cyclic_velocity_mode
    ▼
[EPOS4 #1 + #2]
```

## 4. なぜワーカースレッド経由なのか(重要な設計判断)

脱力トグルは付属リファレンス `maxon_epos4_ros2/src/epos4_vel.cpp` と同じ方式、
すなわち **`call_trigger_service`(`async_send_request` の投げっぱなし)** で
サービスを叩く。非同期なので executor をブロックせず、**専用スレッドも検証ループも
不要**。コールバック内から直接サービスを発行して完結する。

> 補足: 起動時の `init` 処理だけは別物。あちらは「init→enable→CSV を**検証付きで
> 逐次化**して dead-wheel レースを潰す」のが目的なので、ブロッキングな
> `call_trigger_sync` を使い `init_thread_`(別スレッド)で回している。脱力トグルは
> そこまでの確実性を要求しないので、リファレンス流の非同期で軽く済ませる。

## 5. `epos4_controller.cpp` の変更点(関数ごとの役割)

### 追加したもの

| 名前 | 型 | 役割 |
|------|----|------|
| `free_mode_subscription_` | `Subscription<std_msgs::msg::Bool>` | `/robot_free_mode` の受信口 |
| `free_mode_` | `bool`(plain) | 現在脱力中か。true の間は速度指令を無視。読み書きとも executor スレッド上なので atomic 不要 |

### `freeModeCallback(std_msgs::msg::Bool::SharedPtr msg)`
脱力トグルの実装。`msg->data` を `free_mode_` に反映し、ターゲット速度を 0 に
落としてから、状態に応じてサービスを発行する。

- **脱力 ON**: `disable` を両モータへ**非同期**で(`call_trigger_service`)。励磁を
  切るだけで順序は問わないので投げっぱなしでよい。→ 非励磁=フリー。
- **復帰**: `enable` → `cyclic_velocity_mode` を**短命スレッド `reenable_thread_`
  上で逐次**に効かせる(`call_trigger_sync` を順番に)。
  - 並行に投げると mode 設定がレースして**片輪が CSV に入りきらず遅れる**ため
    (§11 参照)。`init`(homing)は呼ばない(再 init は復帰失敗の原因。§9 参照)。
  - executor は塞がない(sync 呼び出しはスレッド側、応答は executor が捌く。
    起動時 `init_thread_` と同じ理由)。再トグル時は前スレッドを `join()` してから張り直す。
  - 最後に `motor_ready()`(SDO で statusword/mode を読む既存関数)で**実ドライブ状態を検証**し、
    両輪 Operation Enabled & CSV なら完了ログ、そうでなければ retry を促す WARN を出す。
    サービスの戻り値では判定しない(no-op 遷移で偽陰性になるため。§11 参照)。

### `cmdSpeedCallback(...)`(既存関数への追記)
先頭に**脱力中ガード**(`if (free_mode_) return;`)を追加。脱力中は速度指令を反映せず、
ターゲットは 0 のまま。復帰直後に古い指令で飛び出すのも `freeModeCallback` 側の
ゼロリセットで防いでいる。

> 補足: `timer_callback`(100 Hz の TPDO publish)は脱力中も 0 速度を出し続けるが、
> drive が非励磁なので無視される。特別な分岐は不要。

## 6. `teleop_keyboard.cpp` の変更点(関数ごとの役割)

### 追加メンバ / パラメータ

| 名前 | 役割 |
|------|------|
| `free_publisher_` | `/robot_free_mode`(`std_msgs/Bool`)への publisher |
| `free_mode_`(`bool`) | teleop 側が把握している脱力状態(表示用) |
| パラメータ `free_topic` | 送信先トピック名(既定 `/robot_free_mode`) |

### `printHelp()`(追記)
キー一覧に `f : toggle 脱力(free) mode` を追加。

### `handleKey()` に `case 'f'`(新規)
`f` キーで脱力状態をトグルする。

1. `free_mode_` を反転。
2. **脱力に入るとき**は `linear_ = angular_ = 0` に指令をリセット
   (復帰後に古い指令で動き出さないように、送信側でも保険を掛ける)。
3. `std_msgs::msg::Bool{data = free_mode_}` を `free_publisher_` で publish。

### `render()`(追記)
ステータス行の先頭に現在モードを表示:`[FREE/脱力]` または `[  DRIVE  ]`。
ひと目で励磁状態が分かるようにした。

### ビルド依存の追加
`std_msgs/msg/bool.hpp` を使うため、`CMakeLists.txt` と `package.xml` に
`std_msgs` 依存を追加(teleop は従来未宣言だった)。
`epos4_controller` 側は既に `std_msgs`(Float64)に依存済みのため変更不要。

## 7. 使い方

```bash
# 通常どおりスタックを起動
ros2 launch rerobot_bringup rerobot_bringup.launch.py

# teleop 起動
ros2 run epos4_teleop teleop_keyboard --ros-args --params-file src/epos4_teleop/config/params.yaml
#  f キー : 脱力 ON/OFF をトグル(ステータス行で [FREE/脱力] / [  DRIVE  ] を確認)
```

teleop を介さず直接トグルしたい場合:

```bash
ros2 topic pub --once /robot_free_mode std_msgs/msg/Bool "{data: true}"   # 脱力 ON
ros2 topic pub --once /robot_free_mode std_msgs/msg/Bool "{data: false}"  # 復帰
```

## 8. 注意点・既知の制約

- 復帰は起動時と同じ `init_motor()`(init→enable→CSV + SDO 検証 + リトライ)を再利用する。
  init は CSV では不要な homing を呼ぶため `Homing failed` ログが出るが**仕様どおり想定内**
  (起動時と同じ挙動)。続く enable/CSV は成功する。
- トランジション中(数百 ms 程度)に再度トグルしても `mode_busy_` で弾かれ、警告ログのみ。
- 脱力中の `joint_states.position` は引き続き更新されるので、手で押した移動分は
  オドメトリに反映され得る(脱力=エンコーダが止まるわけではない)。

## 9. 「脱力後に復帰できない」不具合と修正(2026-06-05 追記)

### 症状
脱力(`disable`)した後にもう一度トグルしても、**コントローラ操作に戻らない**
(速度指令を送っても車輪が動かない)。

### 原因
初版の復帰経路は起動時と同じ `init_motor()`(`init`→`enable`→`cyclic_velocity_mode`)
を再投入していた。しかし `init` は CiA 402 の **homing** を起動するサービスである。

- 起動時: ドライブは NMT START 直後の **"Switch On Disabled"** 状態。ここで `init` を
  呼ぶと homing が「空振り」して `Homing failed` を出すだけの実質 no-op で、続く
  `enable`/`cyclic_velocity_mode` が実際に Operation Enabled / CSV へ持っていく
  (CLAUDE.md にも明記の挙動)。
- 復帰時: `disable` 後のドライブは別のサブ状態にあり、ここで再び `init` を投げると
  **homing モードへ遷移しようとして Operation Enabled / CSV に戻れない**。結果、
  `init_motor` の検証(SDO で statusword=0x27 & mode=9 を確認)が 5 回とも失敗して諦める。

結果、ドライブは Operation Enabled に居らず **「指令は流れるのに車輪は動かない」**
= 復帰できない症状になっていた。

参考: 付属リファレンス `maxon_epos4_ros2/src/epos4_vel.cpp` のメニューでも、`init`(1)は
最初の一度だけで、運転の再開は `enable`(3)→`cyclic_velocity_mode`(8)で行っている。

### 修正
復帰時は **`init`(homing)を呼ばず** `enable` → `cyclic_velocity_mode` のみ発行する。
このルールは最小構成化の後も維持している(`freeModeCallback` の復帰側参照)。

| | 初版(復帰失敗) | 現行 |
|--|------------------|------|
| 復帰時に呼ぶサービス | `init`→`enable`→`csv` | `enable`→`csv` |
| homing の再起動 | あり(これが原因) | なし |

> 最小構成化(§10)で検証付きリトライ(`reenable_motor`)は撤去したが、
> 「復帰で init を呼ばない」という肝は残してある。

## 10. 最小構成化(2026-06-05 さらに追記)

初版〜§9 修正までで、脱力トグルに**専用ワーカースレッド + SDO 検証 + リトライ**
(`mode_thread_` / `mode_busy_` / `apply_free_mode` / `request_free_mode` /
`reenable_motor`)を盛り込んでいたが、起動時の dead-wheel 対策レベルの確実性は
脱力トグルには過剰だった。リファレンス `epos4_vel.cpp` に倣い、**非同期サービス
発行(`call_trigger_service`)1本**に集約して大幅に削減した。

### 撤去したもの
- `mode_thread_`(`std::thread`)/ `mode_busy_`(`std::atomic`)
- `request_free_mode()` / `apply_free_mode()` / `reenable_motor()`
- `free_mode_` の `std::atomic` 化(→ plain `bool`)
- デストラクタの `mode_thread_.join()`

### 残したもの(要件は維持)
- `/robot_free_mode`(Bool)購読と `freeModeCallback`(約15行に集約)
- 脱力 ON=`disable`、復帰=`enable`→`cyclic_velocity_mode`(**init は呼ばない**)
- `cmdSpeedCallback` の脱力中ガードと、トグル時のターゲット 0 リセット

### トレードオフ
非同期・投げっぱなしなので **SDO による成否検証はしない**(リファレンスと同じ割り切り)。
復帰が稀に失敗した場合はもう一度 `f` を 2 回(OFF→ON→OFF)押せば再投入できる。
起動時の確実な初期化(`init_thread_` + 検証 + リトライ)はそのまま温存しているので、
電源投入直後の dead-wheel 対策には影響しない。

## 11. 復帰後に左輪が遅れる不具合と修正(2026-06-05 さらに追記)

### 症状
脱力 → 復帰した後、**m1_wheel_link(左車輪)の動作が遅れる**ようになった
(指令への追従が右車輪より一拍遅く、ロボットが曲がる)。最小構成化(§10)で
復帰を全部**非同期で並行発行**に変えたのが原因。

### 原因 — mode 設定のレース
§10 の復帰は `enable m1 / enable m2 / csv m1 / csv m2` の 4 本を
`call_trigger_service`(`async_send_request`)で**ほぼ同時に投げて**いた。

- m1 と m2 は別ドライバノードなので**並行処理**される。
- EPOS4 の状態遷移はサービスが返った後も**数 sync 周期かけて落ち着く**。
- そのため `csv m1` が `enable m1` の遷移完了前に処理されると、**motor1 だけ CSV
  (mode 9)に入りきらず**、加速ランプの掛かるモード(profile velocity 等)のまま
  残る → 指令への追従が「遅れる」。
- 先に投げた m1 が一番レースに負けやすいので、症状が**左輪に偏って**出る。

これは本リポジトリのコミット `957b7c0`(*ensure sequential service calls to
prevent race conditions*)で起動時 init を逐次化したのと**同じ罠**。最小化の際に、
必要な「逐次性」まで一緒に落としてしまっていた。

### 修正 — 復帰だけ逐次化
復帰の enable→csv を**短命スレッド `reenable_thread_` 上で `call_trigger_sync` で
順番に**効かせるようにした(各サービスの応答を待ってから次へ)。

```
[reenable_thread_]
  m1 enable  → 応答待ち
  m1 csv     → 応答待ち
  m2 enable  → 応答待ち
  m2 csv     → 応答待ち
```

- 検証ループ・リトライ・atomic・`mode_busy_` は**戻していない**(最小構成の精神は維持)。
  追加したのはスレッド 1 本(`reenable_thread_`)とデストラクタでの `join()` のみ。
- sync 呼び出しはスレッド側なので executor は塞がらず、応答 future は executor が捌く
  (起動時 `init_thread_` と同じパターン)。
- 脱力 ON 側の `disable` は励磁を切るだけで順序不問なので、引き続き非同期のまま。

| | §10(遅れ発生) | 現行 |
|--|----------------|------|
| 復帰の発行方法 | 4 本を非同期で並行 | enable→csv をスレッドで逐次 |
| mode 設定のレース | あり(左輪が遅れる原因) | なし(順番に確定) |
| 追加コスト | なし | スレッド 1 本のみ |

### 教訓
「最小構成化」と「必要な逐次性」は別物。複数ドライブの mode/state を切り替える操作は、
非同期で並行に投げると**速い者勝ちのレース**になり、負けた輪が別モードに取り残される。
de-energize(disable)のような順序不問の操作は非同期で軽く、enable→mode 設定のような
**順序・落ち着きが要る操作は逐次**で、と切り分けるのが妥当。

### 成否判定はサービス戻り値ではなく実状態で(実機検証で判明)
逐次化の最初の版では、`call_trigger_sync` の戻り値を `&=` で集めて完了/失敗ログを出していた。
ところが実機では**正常に復帰しているのに `a re-enable step failed` の WARN が出た**。
原因は、`enable`/`cyclic_velocity_mode` が「すでにその状態」の **no-op 遷移**になったとき、
cia402 ドライバが `success=false` を返すことがあるため(偽陰性)。
→ 戻り値での判定をやめ、`motor_ready()`(SDO で 0x6041/0x6061 を読む)で**実ドライブ状態**を
確認してログを出すように変更した。

### 実機検証ログ(rerobot_env / can0 + EPOS4 実機, 2026-06-05)
| タイミング | motor1(左) | motor2(右) |
|-----------|-----------|-----------|
| 起動直後 | `sw=0x1237 mode=9`(Op Enabled, CSV) | 同左 |
| 脱力 ON | `sw=0x0223`(Switched On=非励磁/フリー) | 同左 |
| 復帰後 | `sw=0x1237 mode=9`(Op Enabled, CSV) | 同左 |

復帰後に**両輪とも CSV(mode 9)+ Operation Enabled** に揃うことを SDO 直読みで確認。
遅れの原因だった「片輪だけ CSV 外」は再現せず、`脱力モード OFF: both motors re-enabled (CSV)`
が出力される。なお速度指令は出していないので車輪は自走しない状態での検証。
最終的な走行フィーリングは teleop で手動確認すること。
