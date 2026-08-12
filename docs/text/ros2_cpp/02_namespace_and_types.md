<!-- claude: ROS 2 C++ 構造読本 第2章 (2026-08-12)。namespace / using / テンプレート型名の
     読解を C のプレフィックス命名から積み上げる。 -->

# 第2章 名前空間と型名の読解 — その長い型名はどこから来るのか

`message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::JointState, ...>` —
ROS 2 のコードが読みにくい最大の理由は、この種の長い名前である。だが長い名前は
暗記対象ではなく、**少数の規則で機械的に分解できる合成物**にすぎない。この章では
名前空間の 4 つの書き方と、テンプレート型名の読み下し方を身につける。

## 2.1 C のプレフィックス命名 → namespace

**C ではこう書いていた**: 名前の衝突 (別ライブラリに同名関数がいる) を避ける
手段は命名規約しかなく、モジュール名を関数名の頭に人力で貼り付けていた。

```c
int can_open(const char * dev);     /* can モジュールの open */
int epos4_init(int node_id);        /* epos4 モジュールの init */
```

**C++ ではこう書く**: そのプレフィックスを言語機能に昇格させたのが namespace。

```cpp
namespace epos4
{
int init(int node_id);
}

epos4::init(1);   // :: は「〜の中の」
```

namespace はフォルダ、`::` はパス区切り、と読めばよい。標準ライブラリの関数・型が
全部 `std::` で始まるのは「std という 1 つのフォルダに収まっている」ということで、
入れ子も普通にある: `message_filters::sync_policies::ApproximateTime` は
message_filters フォルダの中の sync_policies フォルダの中の ApproximateTime である。

## 2.2 4 つの書き方の使い分け

名前空間の中の名前を使うとき、C++ には 4 つの書き方があり、reRoBot には
④まで全部の実物がある。使い分けの判断ごと樹形図にする:

```
名前空間の中の名前を使う 4 つの書き方
├── ① フル修飾で毎回書く .......... std::string, rclcpp::Node など
│   └── 基本形。名前の出所が常に明示され、読み手に最も親切
├── ② using namespace X; .......... X の中身をすべて裸の名前で使えるようにする
│   ├── ❌ using namespace std; — 衝突の温床。reRoBot には存在しない
│   └── ✅ using namespace std::chrono_literals; — 10ms / 5s と書くためだけの慣用句
├── ③ using 新名 = 長い型; ........ 型に別名を付ける (C の typedef の後継)
│   └── odometry:91-95 (本章 2.5 のボス戦で使う)
└── ④ namespace { ... } 無名名前空間 「このファイル限定」の宣言
    └── teleop:16-42 — C の static (内部リンケージ) の C++ 流
```

**② が chrono_literals にだけ許される理由**: `10ms` (controller:119) や `5s`
(controller:355) は「ユーザ定義リテラル」で、数値の後ろに単位サフィックスを付ける
機能である。サフィックスは `std::chrono_literals::10ms` のように修飾して書けない
言語仕様のため、using namespace で導入するのが公式の使い方。この namespace には
リテラル演算子しか入っておらず、衝突の実害がない。同じファイルに冗長形も同居して
いるので対比できる — `epos4_controller.cpp:233` の
`std::this_thread::sleep_for(std::chrono::milliseconds(100));` と
`:432` の `std::this_thread::sleep_for(200ms);` は同じ意味である。

**③ typedef との対比**: C の typedef は関数ポインタが絡むと呪文になった。

```c
typedef unsigned int u32;                   /* C: これは読める */
typedef void (*trigger_cb_t)(int, void *);  /* C: 新しい名前がどれか探すことになる */
```

```cpp
using u32 = unsigned int;                            // 常に「新名 = 実体」の向き
using JointStateMsg = sensor_msgs::msg::JointState;  // テンプレートが絡んでも同じ形
```

`using 新名 = 実体;` は代入と同じ向きで必ず読める。これが typedef より好まれる理由の
ほぼすべてである。

**④ 無名 namespace**: teleop は TerminalGuard をこう包んでいる:

```cpp
namespace
{
struct TerminalGuard { ... };
}  // namespace                              // teleop_keyboard.cpp:16-42
```

意味は「この struct はこのファイルの外から見えない」。C でファイル私物の関数に
`static` を付けたのと同じ内部リンケージで、他の翻訳単位に同名の TerminalGuard が
いてもリンク時に衝突しない。ROS 2 ノードの .cpp は 1 ファイル 1 実行ファイルが
多いので出番は少ないが、見かけたら「ファイル私物の道具」という意思表示として読む。

## 2.3 ROS 2 の型名はディレクトリ構造

`geometry_msgs::msg::Twist` の 3 階層は飾りではなく、**メッセージパッケージの
ディレクトリ構造そのもの**である:

```
geometry_msgs (パッケージ)
└── msg/
    └── Twist.msg ──(ビルド時にコード自動生成)──→ C++ 型:      geometry_msgs::msg::Twist
                                                  ヘッダ:      #include "geometry_msgs/msg/twist.hpp"
                                                  CLI 表記:    geometry_msgs/msg/Twist
```

覚える規則は 2 つだけ:

- **ヘッダパスは snake_case、型名は CamelCase**: `co_read.hpp` ↔ `CORead`
- **#include 1 行と型 1 個が 1:1 対応**: controller の `:10-18` のインクルード群は
  そのまま「このノードが話すメッセージ・サービス型の一覧表」になっている

サービス型は中にもう 1 段ネストがある。「呼ぶときに渡すもの」と「返ってくるもの」が
型の中に部屋を持っている:

```
std_srvs::srv::Trigger (controller が 16 本のクライアントで使うサービス型)
├── ::Request ........ 空 (引数なしで「実行して」と言うだけ)
└── ::Response
    ├── success (bool)
    └── message (string)
```

`std::make_shared<std_srvs::srv::Trigger::Request>()` (controller:344) という長い
名前は「Trigger サービスの Request 型」を外から指しているだけで、部品はすべて
この樹形図の中にある。

## 2.4 先頭の `::` — グローバル名前空間

```cpp
ssize_t n = ::read(STDIN_FILENO, &ch, 1);   // teleop_keyboard.cpp:137
```

**先頭に** `::` を付けると「どの namespace にも入っていない、一番外側の名前」を
明示する。ここで呼んでいるのは POSIX の C 関数 `read(2)` — C の関数はすべて
この「無印の名前空間」に生きていて、C++ からそのまま呼べる。

わざわざ `::` を付ける理由は 2 つ。(1) クラスのメンバ関数や using で導入された
同名の何かに解決を横取りされないため、(2) 読み手に「これは OS の C 関数だ」と
伝えるため。**C の世界は消えたのではなく、グローバル名前空間として C++ の中に
まるごと残っている**、というのがこの 1 行の教えである。

## 2.5 テンプレート — 読む側 100%・書く側 0%

C で「型ごとに同じ処理」が必要なとき、手段はコピペ (`min_int` / `min_double`) か
マクロだった。C++ のテンプレートは「型を引数に取る関数・クラス」で、呼ぶ側が
`<>` で型を渡すと、コンパイラがその型専用の実体をコンパイル時に生成する。

先に割り切りを言っておく: **ROS 2 アプリを書く側は、テンプレートを定義しない。
呼ぶだけである**。実際、reRoBot の 1,072 行に `template<...>` の定義は 1 つもない。
必要なのは `<>` を読み下す力だけで、読み方は「**〜用の**」である。

```cpp
create_subscription<geometry_msgs::msg::Twist>("/robot_speed_cmd", ...)
// 「Twist 用の subscription を作る」 — epos4_controller.cpp:83
```

同じ関数が `:89` では `<std_msgs::msg::Bool>` で呼ばれている。呼ぶたびに別の実体が
生成されるので、C なら `create_subscription_twist` / `_bool` を手書きしていた仕事が
`<>` の指定 1 つで済んでいる。

メンバ宣言に現れる形も同じ規則で分解できる:

```
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr        (epos4_controller.cpp:153)
│              │                        │
│              │                        └ その Client 型の中に定義された別名
│              │                          = std::shared_ptr<rclcpp::Client<...>> (第3章 3.3)
│              └ 扱うサービス型 = 「Trigger 用の」
└ rclcpp のサービスクライアント (テンプレートクラス)
```

### ボス戦: Synchronizer の型を解体する

本リポジトリで最も長い型は odometry にいる。書き方③ (using エイリアス) が
3 段の踏み台になっていて、上から順に読めるようになっている:

```cpp
using JointStateMsg = sensor_msgs::msg::JointState;              // :91 ① 素材
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  JointStateMsg, JointStateMsg>;                                 // :92 ② 方針: JointState 2 本を時刻で揃える
using Synchronizer =
  message_filters::Synchronizer<SyncPolicy>;                     // :94 ③ その方針で動く同期器
```

使う場所 (odometry:75-76) は `std::make_shared<Synchronizer>(SyncPolicy(10), m2_sub_, m1_sub_)` —
エイリアスのおかげで 1 行に収まる。もしエイリアスなしで書いたら:

```cpp
message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::JointState, sensor_msgs::msg::JointState>> sync_;
```

この対比が示すように、using エイリアスは単なる省略記法ではなく **読む順序を設計する
道具**である。①素材 → ②方針 → ③完成品の 3 行は、そのままこの同期機構の説明文に
なっている。長い型に出会って読めないときは「エイリアスを 3 つ切るならどこで切るか」を
考えると、型の構造そのものが見えてくる。

> **要点**: `<>` は「〜用の」、`::` は「〜の中の」。この 2 つの読みだけで、
> ROS 2 の型名はどれだけ長くても左から機械的に分解できる。

## 2.6 名前解決の最外周 — #include / find_package / package.xml

`message_filters::Subscriber` と書けるためには、コンパイラの名前解決より外側でも
「その名前はどこから来るか」が 3 つの層で解決されている必要がある。odometry に
同期機構を追加したとき (2026-08-11、`claude_sync` タグ) に触った 3 箇所がそのまま
実例になっている:

| 層 | 問い | 書く場所 | 実例 (epos4_controller パッケージ) |
|---|---|---|---|
| コンパイラ | 型の宣言はどのヘッダ? | ソースの `#include` | `epos4_odometry.cpp:8-10` |
| CMake | ヘッダ・ライブラリはどのパッケージ? | `find_package` + `ament_target_dependencies` | `CMakeLists.txt:18` と `:39` |
| パッケージ管理 | このパッケージは何に依存? | `package.xml` の `<depend>` | `package.xml:21` |

1 層でも欠けるとそれぞれ違う顔のエラーになる (逆引き表は [第7章 7.3](07_reference.md))。
覚え方としては、**「namespace はコンパイラにとっての名前解決、find_package は
ビルドシステムにとっての名前解決」** — 同じ「この名前はどこから来る?」という問いが、
スコープを変えて 3 回問われている、というのが統一的な見方である。

なお CMakeLists.txt には「1 パッケージから 2 実行ファイル」の実例もある:
`add_executable(epos4_controller ...)` (:21) と `add_executable(epos4_odometry ...)`
(:31) は依存リストが違い (odometry だけ nav_msgs / tf2 / message_filters を使う)、
`ament_target_dependencies` が実行ファイル単位なのはそのためである。

## 2.7 この章のまとめ

```
名前空間と型名 まとめ
├── namespace = C のプレフィックス命名の言語機能化。:: = 「の中の」
├── 書き方は 4 つ
│   ├── 基本はフル修飾
│   ├── using namespace は chrono_literals (10ms/5s) の慣用句だけ
│   ├── using 新名 = 実体; — typedef 後継。読む順序を設計する道具
│   └── 無名 namespace = C の static (ファイル私物)
├── ROS 2 の型名はパッケージのディレクトリ構造
│   ├── snake_case パス ↔ CamelCase 型名が 1:1
│   └── srv は ::Request / ::Response を内蔵
├── 先頭の :: = グローバル (C の世界) の明示 ......... ::read (teleop:137)
├── テンプレートは呼ぶだけ。<> は「〜用の」と読む
└── 名前解決は 3 層 — #include / find_package + ament_target_dependencies / <depend>
```

型名が読めるようになったので、次はその型の値を「誰が持ち、誰が片付けるか」—
ROS 2 のコードに new も delete も 1 つもない理由に進む。

→ [第3章 所有権と RAII](03_ownership_raii.md)
