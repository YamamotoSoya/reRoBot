<!-- claude: ROS 2 C++ 構造読本 第7章 (2026-08-12)。記法逆引き表・読解チェックリスト・
     ビルド 3 点対応。繰り返し参照する実務ページ。 -->

# 第7章 実務編 — 読めない記法に出会ったらどこを引くか

この章は通読するページではなく、コードを読んでいて手が止まったときに引く
リファレンスである。逆引き表 → 該当章 → 実例、の順にたどれるようにしてある。

## 7.1 記法逆引き表

| 見た目 | 読み方 | 章 | reRoBot の実例 |
|---|---|---|---|
| `class X : public Y` | X は Y を継承 (X is a Y) | [1](01_class_inheritance.md) | controller:22 |
| `X() : Y("...")` | メンバ初期化リスト — 親を先に組み立てる | [1](01_class_inheritance.md) | odometry:22 |
| `override` | 親の仮想関数の上書き検査マーク | [1](01_class_inheritance.md) | teleop:99 |
| `A::B::C` | 名前空間/クラス「の中の」名前 | [2](02_namespace_and_types.md) | `geometry_msgs::msg::Twist` |
| 先頭の `::` | グローバル (C の世界) の名前 | [2](02_namespace_and_types.md) | teleop:137 `::read` |
| `using X = ...;` | 型の別名 (typedef 後継)。新名 = 実体の向き | [2](02_namespace_and_types.md) | odometry:91-95 |
| `f<T>(...)` / `C<T>` | 「T 用の」実体をコンパイル時に生成 | [2](02_namespace_and_types.md) | controller:83 |
| `10ms` / `5s` | chrono リテラル (`using namespace std::chrono_literals` 前提) | [2](02_namespace_and_types.md) | controller:119, :355 |
| `X::SharedPtr` | `std::shared_ptr<X>` の別名 | [3](03_ownership_raii.md) | controller:153 |
| `std::make_shared<T>(...)` | 共有所有のオブジェクト生成 (new は書かない) | [3](03_ownership_raii.md) | controller:520 |
| `std::make_unique<T>(...)` | 所有者 1 人のオブジェクト生成 | [3](03_ownership_raii.md) | odometry:61 |
| `std::lock_guard<std::mutex>` | スコープ = ロック区間 (RAII) | [3](03_ownership_raii.md) / [5](05_spin_concurrency.md) | teleop:169 |
| `std::bind(&X::f, this, _1)` | メンバ関数 + this + 引数予約席を束ねる | [4](04_callbacks_binding.md) | controller:85 |
| `[this](...) { ... }` | ラムダ — this を持ち込んでその場で関数を作る | [4](04_callbacks_binding.md) | teleop:84 |
| `double & r = ...` | 参照 = null なし・自動参照剥がしの固定ポインタ | [4](04_callbacks_binding.md) | teleop:209 |
| `const T &` 引数 | コピーせず読むだけで借りる | [4](04_callbacks_binding.md) | odometry:100 |
| `std::thread(&X::f, this)` | スレッド起動 (bind 内蔵) | [4](04_callbacks_binding.md) / [5](05_spin_concurrency.md) | controller:127 |
| `std::atomic<bool>` | スレッド間で安全な変数 (合図用) | [5](05_spin_concurrency.md) | controller:201 |
| `future.wait_for(t)` | 非同期応答の到着待ち (spin 中の別スレッドでのみ可) | [5](05_spin_concurrency.md) | controller:363 |
| `std::optional<T>` / `std::nullopt` | 「値がない」を型で表す (C の -1 番兵の代替) | — (下記 7.4) | controller:373-391 |

## 7.2 コード読解チェックリスト

新しい ROS 2 C++ ファイルを開いたら、上から順に読まずにまずこの 7 点を拾う。
このリスト自体が本書の要約になっている:

```
□ 1. 親クラスは何か (class X : public ...) — ほぼ rclcpp::Node のはず (第1章)
□ 2. コンストラクタで何を登録しているか — pub / sub / srv / timer の一覧が
     そのまま「このノードの入出力仕様」になる (第1章 1.3)
□ 3. private のコールバックはいくつか — 登録 (2.) と 1:1 対応しているか (第4章)
□ 4. デストラクタはあるか — あるなら「自動で片付かない資源」は何か (第3章 3.4)
□ 5. std::thread / atomic / mutex はあるか — 実行の流れは何本か。
     共有データと排他の対応が取れているか (第5章 5.5)
□ 6. パラメータの宣言と既定値 — 既定値のまま動くと困るものはないか (第6章 事例D)
□ 7. main() は定型 4 行 (init → make_shared → spin → shutdown) か —
     定型から外れた行 (teleop の TerminalGuard など) には必ず理由がある
```

## 7.3 ビルド 3 点対応 — 新しい #include を足したら

[第2章 2.6](02_namespace_and_types.md) の 3 層を、作業手順として引ける形にしたもの。
新しいライブラリのヘッダを 1 つ include したら、以下をセットで触る:

```
#include "message_filters/subscriber.h" を足す (例: 2026-08-11 の odometry 改修)
├── CMakeLists.txt
│   ├── find_package(message_filters REQUIRED) .................. :18
│   └── ament_target_dependencies(epos4_odometry ... message_filters) :39
│       └── ⚠️ 実行ファイル単位。同居する他の add_executable には効かない
└── package.xml
    └── <depend>message_filters</depend> ........................ :21
```

エラーからの逆引き:

| エラーの見え方 | 欠けている層 |
|---|---|
| コンパイル時 `fatal error: xxx.h: No such file or directory` | `find_package` / `ament_target_dependencies` |
| リンク時 `undefined reference to ...` | `ament_target_dependencies` に依存はあるが**対象の実行ファイルが違う**、など |
| 自マシンでは通るが他環境で `find_package` 失敗 | `package.xml` の `<depend>` (rosdep が依存を導入できない) |
| `ros2 run` で `Package 'xxx' not found` / 実行ファイルが見つからない | `install(TARGETS ...)` 漏れ、または `source install/setup.bash` 忘れ |
| 起動直後 `RCLInvalidROSArgsError: Cannot have a value before ros__parameters` | ビルドではなく YAML — `ros__parameters` のアンダースコアは 2 個 (第6章 事例E) |

補足: `package.xml` の `<exec_depend>` は「ビルドには不要、実行時にだけ必要」な
依存 (例: `epos4_teleop/package.xml:17` の `<exec_depend>epos4_controller</exec_depend>` —
teleop はコントローラのヘッダを使わないが、一緒に動いて初めて意味がある)。

## 7.4 本書に登場しなかった頻出記法

reRoBot の 3 ファイルには出てこないが、他所の ROS 2 コード (Nav2、slam_toolbox、
サンプル集) で高頻度に出会うものを最小例で:

```cpp
// range-based for — 配列・vector の全要素を順に。C の for(i=0;...) の 9 割はこれで書ける
for (const auto & p : msg->position) { sum += p; }
```

```cpp
// enum class — 値が Mode:: に閉じる C enum の改良版。裸の Idle が名前空間を汚さない
enum class Mode { Idle, Drive, Free };
Mode m = Mode::Idle;
```

```cpp
// std::optional — 「失敗 = 値なし」を型で表す。C の「-1 を返す」「NULL を返す」の代替。
// 実は controller:373-391 の read_sdo が使っている: 失敗は std::nullopt、
// 呼び手は has_value() で確認してから value() を取る (controller:403, :407)
std::optional<uint32_t> read_sdo(...);
```

```cpp
// 構造化束縛 — 複数の戻り値をまとめて受ける
auto [it, inserted] = my_map.insert({key, value});
```

いずれも本書の道具 (`<>` は「〜用の」、`::` は「の中の」、`&` は別名) だけで
読み下せるはずである。読めなかったら該当章へ戻ってほしい。

→ [目次に戻る](00_index.md)
