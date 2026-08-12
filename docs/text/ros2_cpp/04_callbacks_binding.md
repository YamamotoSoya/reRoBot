<!-- claude: ROS 2 C++ 構造読本 第4章 (2026-08-12)。std::bind / ラムダ / メンバ関数
     ポインタの 3 方式と参照 (&) を C の関数ポインタ + void* から積み上げる。 -->

# 第4章 コールバックと束縛 — 関数を「渡す」とき何が一緒に運ばれるのか

`std::bind(&Epos4_Control2_Node::cmdSpeedCallback, this, std::placeholders::_1)` —
ROS 2 コードで最初に「読めない」と感じるのは大抵この行である。正体は
「あとで呼んでもらう関数に、必要な材料を先に括り付けておく」だけの仕組みで、
C のコールバック API を思い出すと構造が透けて見える。

## 4.1 C の方式 — 関数とデータを別々に渡す

**C ではこう書いていた**: コールバック API は「関数ポインタ」と「そのとき使う
データ (`void * user_data`)」の 2 個口で、両者の対応は人間が管理していた。

```c
/* C: 関数と「そのとき使うデータ」を別々の引数で渡す */
void on_speed_cmd(const Twist * msg, void * user_data)
{
  Controller * self = (Controller *)user_data;   /* 型情報は void* で一度消える */
  self->target = msg->linear_x;
}

subscribe("/robot_speed_cmd", on_speed_cmd, &controller);
```

**C++ ではこう書く**: 関数とデータを、渡す前に 1 個の「呼び出せるオブジェクト」に
束ねてしまう (この束をクロージャと呼ぶ)。`void *` が消え、型は保たれたまま。
束ね方が 3 通りあり、reRoBot に全部の実物がある:

| 方式 | 見た目 | reRoBot の実例 |
|---|---|---|
| 1. std::bind | `std::bind(&Class::method, this, _1)` | controller:85, :119 / odometry:80 |
| 2. ラムダ | `[this](引数) { ... }` | teleop:84-90 / controller:492 |
| 3. メンバ関数ポインタ直渡し | `std::thread(&Class::method, this)` | controller:127 |

3 方式はどれも「メンバ関数 = this を第 1 引数に取る関数」([第1章 1.1](01_class_inheritance.md))
の応用である。順に見る。

## 4.2 方式1: std::bind — メンバ関数と this を束ねる

**適用条件**: すでにメンバ関数として書いてある処理を、そのまま登録したいとき。

**最小コードと読み方**:

```cpp
std::bind(&MyNode::onMsg,         // ① どの関数か (メンバ関数ポインタ)
          this,                   // ② どの実体で呼ぶか (隠れ第 1 引数を確定)
          std::placeholders::_1)  // ③ 「呼ばれるとき渡される第 1 引数」の予約席
```

bind は「引数の一部を先に埋めた、新しい呼び出せるオブジェクト」を作る。
② で this を埋めるのは、メンバ関数の隠れ第 1 引数を確定させる作業 — C 版で
`&controller` を user_data に渡していたのと同じことを、型を失わずにやっている。

**プロジェクト実例** — プレースホルダの数は「呼び出し側 (rclcpp) があとから
渡してくる引数の数」で決まる。reRoBot には 0 / 1 / 2 個の 3 段が全部ある:

| 実例 | プレースホルダ | 呼び出し側があとから渡すもの |
|---|---|---|
| タイマ (controller:118-119) | なし | なし (満了したという事実だけ) |
| subscription (controller:83-85) | `_1` | 受信メッセージ 1 個 |
| synchronizer (odometry:79-82) | `_1, _2` | 時刻の揃った左右ペア 2 個 |

```cpp
// 0 個: タイマは引数なしでコールバックを呼ぶ
topic_timer_ = this->create_wall_timer(
  10ms, std::bind(&Epos4_Control2_Node::timer_callback, this));

// 1 個: 購読はメッセージ 1 個を渡してくる
cmd_speed_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
  "/robot_speed_cmd", 10,
  std::bind(&Epos4_Control2_Node::cmdSpeedCallback, this, std::placeholders::_1));

// 2 個: 同期器は揃ったペアを 2 個渡してくる
sync_->registerCallback(
  std::bind(&Epos4OdometryNode::onJointStates, this,
            std::placeholders::_1, std::placeholders::_2));
```

**落とし穴**: プレースホルダの数や関数のシグネチャを間違えると、コンパイルエラーが
テンプレートの奥地から数十行噴き出す。全部読もうとしないこと — **最初の 1 行
(どのファイルのどの行で起きたか) と、自分のコードのファイル名を含む行だけ**拾えば
「登録しようとした関数と、期待される引数が合っていない」ことはほぼ特定できる。

## 4.3 方式2: ラムダ式 — その場で関数を作り、値を焼き込む

**適用条件**: 処理がその場の数行で済むとき、または「登録の時点で決まる追加引数」を
焼き込みたいとき。

**記法の分解**:

```cpp
[キャプチャ] (仮引数) { 本体 }
```

キャプチャは「本体がスコープ外の変数を使うための持ち込みリスト」。`[this]` は
自分の実体へのポインタを持ち込む = 本体の中で自分のメンバに触れる、という意味で、
方式1 で bind に this を渡したのと同じ役割を果たす。

**プロジェクト実例** — teleop は同じ関数を「引数だけ変えて」2 トピックに登録する
ためにラムダを使っている:

```cpp
m1_subscription_ = create_subscription<sensor_msgs::msg::JointState>(   // teleop:83-90
  m1_topic, 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
    updateDistance(msg, /*is_left=*/false);  // claude_swap: motor1 -> RIGHT
  });
m2_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
  m2_topic, 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
    updateDistance(msg, /*is_left=*/true);   // claude_swap: motor2 -> LEFT
  });
```

`is_left` は「登録時に確定し、呼び出し時には変わらない引数」で、これをラムダ本体に
書き込む (焼き込む) のが定石。bind でも
`std::bind(&TeleopKeyboardNode::updateDistance, this, std::placeholders::_1, false)`
と書けるが、ラムダは上から普通の関数呼び出しとして読み下せる分だけ読みやすい。
`/*is_left=*/false` は引数名を示すコメント慣習 — C++ に名前付き引数がないための
可読性の工夫で、真似する価値がある。

もう 1 つの用途が「一連の手順を丸ごと包んでスレッドに渡す」(controller:492-512)。
脱力モード復帰の enable→CSV 逐次実行を `[this] { ... }` に包んで `std::thread` へ
渡している (中身の意味は第5章)。

**落とし穴**: `[&]` (スコープの変数を全部参照で持ち込む) で作ったラムダを、
そのスコープより長生きさせる (登録する・スレッドに渡す) と、持ち込んだ参照の先が
死んでいる (dangling)。**登録するラムダは `[this]` + メンバ変数に限る**のが安全側の
習慣である (this 自体の寿命は 4.6)。

## 4.4 方式3: メンバ関数ポインタ直渡し

```cpp
init_thread_ = std::thread(&Epos4_Control2_Node::run_init_sequence, this);  // controller:127
```

`std::thread` のコンストラクタは「(関数, 引数...)」を受けて内部で束ねる —
bind 相当の機能を内蔵しているので、bind を書かずに材料 (`&Class::f` と `this` の
ペア) を直接渡せる。方式1 の ①② と同じ部品が、包み紙なしで並んでいるだけである。

## 4.5 参照 (&) — 引数リストを読む最後の部品

コールバックのシグネチャには `&` が多用される。C のポインタとの対応で一気に読める:

```c
double * p = &x;   *p = 3.0;    /* C: 番地を取り、明示的に * で参照剥がし */
```

```cpp
double & r = x;    r = 3.0;     // C++: r は x の別名。以後 * は不要
```

参照とは「**null がなく、付け替え不可で、自動的に参照剥がしされるポインタ**」。
宣言時に必ず実体と結ばれ、以後はその実体そのものとして振る舞う。

**`const T &` = コピーせず読むだけ** — odometry のコールバックがこの形:

```cpp
void onJointStates(
  const JointStateMsg::ConstSharedPtr & left_msg,    // odometry:99-101
  const JointStateMsg::ConstSharedPtr & right_msg)
```

shared_ptr を値で受けるとコピーのたびに参照カウントの増減が走る。`const &` なら
借りるだけでコストゼロ。一方 controller:290 は値受け (`const Twist::SharedPtr msg`) —
どちらも正しく動き、この頻度では性能差も出ないが、**2 つの流儀が読み分けられる**
ことが大事である (ConstSharedPtr は「指す先も書き換えない」側の別名)。

**参照エイリアス技法** — teleop に、C 経験者にこそ刺さる `&` の使い方がある:

```cpp
double & prev = is_left ? prev_pos_left_ : prev_pos_right_;   // teleop:209-212
bool & have_prev = is_left ? have_prev_left_ : have_prev_right_;
double & dist = is_left ? dist_left_ : dist_right_;
```

C で書けばこうなる:

```c
double * prev = is_left ? &prev_pos_left : &prev_pos_right;
/* ... 以後ずっと *prev = ...; と * が付いて回る */
```

参照版は、束縛の 3 行だけが「どちら側か」を知っていて、以降のロジック
(teleop:214-225) は左右をまったく意識せず `prev = pos_rad;` `dist += ...;` と
書ける。`if (is_left) { 左用の式一式 } else { 右用の式一式 }` という全文コピペを
消すための、参照ならではの整理術である。

## 4.6 束ねたものの寿命 — 次章への橋

方式1〜3 が包んだ `this` は生のポインタである。ではノードが死んだ後にコールバックが
呼ばれたら? — 答え: 呼ばれない。登録したコールバックは **subscription オブジェクト
(= メンバに持っている SharedPtr の実体) の中に保存されて**おり、ノードが破棄される
ときメンバも一緒に破棄されるから、コールバックの寿命はノードの寿命と一致する。
第3章の「登録の控えをメンバに持つ」は、寿命を揃えるための設計だったのである。

残った問いは 1 つ。**保存されたコールバックを、誰が・いつ・どのスレッドで呼ぶのか**。
それが `rclcpp::spin` の正体である。

## 4.7 この章のまとめ

```
コールバックと束縛 まとめ
├── C: 関数ポインタ + void* の 2 個口 → C++: 関数と材料を 1 個に束ねる (クロージャ)
├── 方式1 std::bind(&Class::f, this, _1)
│   ├── this = 隠れ第 1 引数の確定 (第1章 1.1 の回収)
│   └── _1, _2 = あとで渡される引数の予約席。数は呼び出し側が決める (0/1/2 の実例あり)
├── 方式2 ラムダ [this](args){ ... }
│   ├── キャプチャ = 持ち込みリスト。登録用は [this] に限るのが安全
│   └── 登録時に決まる引数を焼き込める (is_left の 2 連登録)
├── 方式3 std::thread(&Class::f, this) — bind 内蔵 API には材料を直接渡す
├── 参照 & = null なし・付け替え不可・自動参照剥がしのポインタ
│   ├── const T & = コピーせず読むだけ (コールバック引数の既定形)
│   └── 参照エイリアス = 左右分岐を束縛 3 行に閉じ込める (teleop:209-212)
└── コールバックの寿命 = それを保存する subscription メンバの寿命 = ノードの寿命
```

→ [第5章 spin と並行性](05_spin_concurrency.md)
