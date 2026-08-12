<!-- claude: ROS 2 C++ 構造読本 第1章 (2026-08-12)。C の struct+関数を足場に class /
     メンバ初期化リスト / 継承 / override を reRoBot の 3 ノードで解説。 -->

# 第1章 クラスと継承 — なぜノードは rclcpp::Node を継承するのか

reRoBot の自作ノードは 3 つとも、本体の最初の行が `class X : public rclcpp::Node` で
始まる。この 1 行の意味が正確に読めれば、ROS 2 ノードの骨格は 8 割読めたことになる。
この章では C の struct + 関数の書き方から出発して、class・メンバ初期化・継承・仮想関数を
「ROS 2 ノードを読む/作るのに必要な範囲」だけ積み上げる。

## 1.1 C の struct + 関数 → C++ の class — 隠れた this

**C ではこう書いていた**: データ (struct) とそれを操作する関数は別々で、
「どの実体を操作するか」を毎回第 1 引数のポインタで渡す。

```c
typedef struct {
  double value;
} Motor;

void motor_set(Motor * m, double v) { m->value = v; }

Motor m1;
motor_set(&m1, 3.0);   /* どの Motor か、を人間が毎回渡す */
```

**C++ ではこう書く**: 関数を struct の中に入れる。すると「どの実体か」は言語が
自動で渡すようになる。この隠れた第 1 引数を `this` と呼ぶ。

```cpp
class Motor
{
public:
  void set(double v) { value_ = v; }   // value_ は this->value_ の省略形
private:
  double value_ = 0.0;                 // 宣言と同時に初期値 (C の struct には書けない)
};

Motor m1;
m1.set(3.0);   // motor_set(&m1, 3.0) と同じこと。&m1 が this になる
```

つまり **メンバ関数とは「第 1 引数に this を取る普通の関数」の別記法**である。
この見方は第4章 (std::bind に this を渡す理由) でそのまま効いてくる。

**ROS 2 ではこう現れる** — controller の実物:

- `epos4_controller.cpp:151` — `double m1_value_ = 0.0;` メンバ宣言と同時の初期値。
  末尾の `_` は「メンバ変数」の印という C++ の命名慣習 (ローカル変数と見分けるため)
- `epos4_controller.cpp:310` — コールバック内の `m1_value_ = invert_right_ ? -rpm_right : rpm_right;`
  は `this->m1_value_ = ...` の省略形。「どの実体の変数か」を書かずに済んでいるのは
  this が黙って効いているから

`public:` / `private:` は「使う人に見せる面」と「実装の中身」の仕切り。3 ノードの
仕切り方を並べると、ROS 2 ノードの重要な性質が見える:

| ノード | public にあるもの | private にあるもの |
|---|---|---|
| Epos4_Control2_Node (`epos4_controller.cpp:22`) | コンストラクタ + デストラクタ | コールバック・補助関数 12 本 + メンバ 30 本超 |
| Epos4OdometryNode (`epos4_odometry.cpp:19`) | コンストラクタのみ | コールバック 2 本 + メンバ 19 本 |
| TeleopKeyboardNode (`teleop_keyboard.cpp:44`) | コンストラクタ + デストラクタ | キー処理・描画 5 本 + メンバ 21 本 |

**public はコンストラクタ (とデストラクタ) だけ**。ROS 2 のノードは他のコードから
メンバ関数を直接呼ばれることがなく、話しかけられるのはトピック・サービス経由
(第5章) だからで、「公開 API がほぼ空」はノードの正しい姿である。

## 1.2 継承 — 親の機能を「自分のもの」として使う

C で「既製の機能一式を持つ部品」を再利用するには、構造体の埋め込みと関数の
手書き委譲しかなかった。C++ の継承は、これを 1 行の宣言にしたもの:

```cpp
class Epos4OdometryNode : public rclcpp::Node   // epos4_odometry.cpp:19
```

読み方は「Epos4OdometryNode は rclcpp::Node **である** (is-a)」。この宣言 1 つで
2 つのことが同時に起きる。

### (a) Node として扱ってもらえる

```cpp
rclcpp::spin(node);   // epos4_odometry.cpp:260
```

`rclcpp::spin` は「Node (の shared_ptr) なら何でも」受け付ける関数で、
Epos4OdometryNode という型の存在を知らない。**「Node である」ことだけを頼りに
動く仕組みに自作クラスを差し込める** — ポリモーフィズム (多態) の入り口である (1.4 で戻る)。

### (b) 親のメンバが自分のものになる

odometry のコンストラクタは `declare_parameter` や `create_publisher` を連打するが、
**これらの関数はこのファイルのどこにも定義されていない**:

```
Epos4OdometryNode が呼んでいる関数はどこで定義されているか
├── 自分で定義した関数 (epos4_odometry.cpp 内に本体がある)
│   ├── onJointStates() ......................... :99
│   └── publishWheelJointStates() ............... :205
└── rclcpp::Node から相続した関数 (このファイルに定義がない)
    ├── declare_parameter() / get_parameter() ... :24-51 で計 18 回
    ├── create_publisher() ...................... :57, :60
    ├── get_logger() ............................ :53, :85
    └── now() ................................... :116
```

継承コードの第一の読解ルール: **「定義がどこにも見当たらないのに呼べている関数」を
見つけたら、まず親クラスを疑う**。controller の `create_client` ×20 本 (:28-77)、
`create_wall_timer` (:118) もすべて Node 由来である。

### (c) コンストラクタは親から先に組み立てる — メンバ初期化リスト

```cpp
Epos4OdometryNode() : Node("epos4_odometry_node")   // epos4_odometry.cpp:22
```

`:` 以降は**メンバ初期化リスト**という専用の場所で、ここでは「親クラス Node の
コンストラクタにノード名を渡して、親の部分を先に組み立てる」ことをしている。
組み立て順は常に **親 → メンバ変数 → 自分の `{}` 本体**。

C の感覚と違うのは、これが「代入」ではなく「誕生時の初期化」であること。
rclcpp::Node には「名前なしで作る」コンストラクタが存在しないため、この記法は
省略できない (省略するとコンパイルエラー)。逆に言えば、本体 `{}` の 1 行目に
到達した時点で親は完成済みだから、いきなり `create_publisher` (親の機能) を呼べる。

## 1.3 ROS 2 の「継承して作る」はほぼ 1 パターン

継承は一般には設計判断の難しい機能 (何を親にするか、多重継承は、など) だが、
**ROS 2 アプリでは考えることがない: rclcpp::Node を public 継承する、以上**。
3 ノードすべてが同じ骨格であることを確認してほしい:

```
ROS 2 ノードの定型 (reRoBot の 3 ノードすべてこの形)
├── class MyNode : public rclcpp::Node
├── public:
│   ├── コンストラクタ ..... 「登録」だけする (パラメータ宣言・pub/sub・タイマ・クライアント)
│   └── (デストラクタ) ..... 手動の後始末があるノードだけ (第3章 3.4)
├── private:
│   ├── コールバック関数 ... コンストラクタで登録したもの。呼ぶのは spin (第5章)
│   └── メンバ変数 ......... 登録の控え (SharedPtr) + パラメータ + 状態変数
└── main() ................. init → make_shared → spin → shutdown の定型 4 行
```

この骨格を最小の完動コードにすると次のようになる。**新しいノードはこれを写して
肉付けすれば作れる**:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalNode : public rclcpp::Node
{
public:
  MinimalNode() : Node("minimal_node")
  {
    sub_ = create_subscription<std_msgs::msg::String>(
      "/chatter", 10,
      std::bind(&MinimalNode::onMsg, this, std::placeholders::_1));
  }

private:
  void onMsg(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "heard: %s", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalNode>());
  rclcpp::shutdown();
  return 0;
}
```

まだ読めない部品 — `<std_msgs::msg::String>` (第2章)、`SharedPtr` と `make_shared`
(第3章)、`std::bind` (第4章)、`spin` (第5章) — は後続章がそれぞれ解体する。
この章の主張は 1 つ: **どんな大きさのノードも、この骨格の肉付けにすぎない**。
523 行ある controller も、コンストラクタが長い (:25-134 = 登録が多い) だけで
骨格は上とまったく同じである。

> **要点**: ROS 2 ノードを「作る」とは、Node を継承したクラスのコンストラクタに
> 登録を並べ、private にコールバックを書くこと。継承の設計に迷う場面はない。

## 1.4 仮想関数と override

1.2(a) の「Node として扱ってもらえる」には続きがある。Node 型として持たれている
のに、破棄やイベントの際には**実体の型の関数**が呼ばれてほしい。それを実現する
仕組みが仮想関数で、最小例で見るのが早い:

```cpp
struct Animal
{
  virtual ~Animal() {}                            // 仮想デストラクタ (後述)
  virtual void speak() { std::puts("..."); }      // virtual = 上書き可能の宣言
};

struct Dog : Animal
{
  void speak() override { std::puts("wan"); }     // override = 上書きの検査マーク
};

std::shared_ptr<Animal> a = std::make_shared<Dog>();
a->speak();   // "wan" — 変数の型は Animal でも、実体 Dog の speak が選ばれる
```

`override` は「親の仮想関数を上書きしているはず。もし違ったらコンパイルエラーに
してくれ」という検査マーク。書かなくても動くが、親の関数名を打ち間違えたときに
「上書きし損ねた無関係な新関数」が静かに生まれる事故を防ぐ。

実コードで唯一の登場箇所が teleop のデストラクタである:

```cpp
~TeleopKeyboardNode() override   // teleop_keyboard.cpp:99
{
  auto stop = geometry_msgs::msg::Twist();
  cmd_publisher_->publish(stop);   // 死ぬ前に停止指令を 1 発
}
```

これが正しく動くのは **rclcpp::Node のデストラクタが virtual だから**。spin の中で
ノードは `shared_ptr<Node>` として (= 親の型で) 保持されるが、仮想デストラクタの
おかげで、破棄時にはちゃんと ~TeleopKeyboardNode (停止 Twist の送信) が先に呼ばれ、
それから親の後始末が走る。もし virtual でなかったら子の後始末は黙って飛ばされる —
「親のポインタで持つ可能性のあるクラスのデストラクタは virtual にする」という
C++ の定番規則は、この事故の予防である。

ROS 2 アプリを書く側の実態としては、**自分で仮想関数を設計する機会はほぼない**。
rclcpp が用意した仮想の仕組みに乗るだけであり、必要なのは `override` の意味が
読めることだけである。

最後に逆側の注意: **クラス = 継承ではない**。teleop の `TerminalGuard`
(`teleop_keyboard.cpp:18`) は何も継承しないただの struct で、それで正しい
(役割は第3章)。継承するのは「既製の枠組み (Node) の一員になりたいとき」だけで、
単なる道具クラスには要らない。なお `struct` と `class` の違いは既定の可視性
(public / private) だけ — TerminalGuard が struct なのは「全部公開の小さな道具」だからである。

## 1.5 この章のまとめ

```
クラスと継承 まとめ
├── メンバ関数 = 第 1 引数に this を取る関数の別記法
│   └── m1_value_ = ... は this->m1_value_ = ... の省略 (controller:310)
├── : public rclcpp::Node = 「Node である」宣言
│   ├── spin に渡せる (is-a) ................. odometry:260
│   └── create_* / declare_parameter / get_logger は親由来
│       └── 定義が見つからない関数はまず親を疑う
├── X() : Node("名前") = メンバ初期化リスト
│   └── 親 → メンバ → 本体の順で組み立てる。Node は名前必須なので省略不可
├── ROS 2 の継承は 1 パターンだけ
│   └── 「登録するコンストラクタ + private コールバック」の骨格を写せば作れる
└── virtual / override
    ├── 親の型で持たれても実体の関数が呼ばれる仕組み
    └── 自分で設計はしない。読めれば足りる (teleop:99 が唯一の実例)
```

次章では、この章で読み飛ばした「長い名前」—
`rclcpp::Subscription<std_msgs::msg::String>::SharedPtr` のような型名が
どんな部品でできているかを解体する。

→ [第2章 名前空間と型名の読解](02_namespace_and_types.md)
