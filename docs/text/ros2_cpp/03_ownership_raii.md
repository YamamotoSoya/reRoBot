<!-- claude: ROS 2 C++ 構造読本 第3章 (2026-08-12)。RAII とスマートポインタを
     TerminalGuard と 3 ノードのデストラクタ比較で解説。 -->

# 第3章 所有権と RAII — なぜこのコードには delete も free もないのか

reRoBot の自作 C++ 1,072 行には、`new` も `delete` も `malloc` も `free` も
1 回も出てこない。C の感覚では「じゃあ誰がメモリを片付けているのか」が謎になる
はずで、その答えが本章の主題 — **RAII とスマートポインタ**である。これは記法の
話ではなく「後始末を人間の注意力から言語の仕組みへ移す」という設計思想であり、
ROS 2 の API 全体がこの思想の上に建っている。

## 3.1 C の後始末問題 — 復元コードを全出口に書く暮らし

**C ではこう書いていた**: 資源 (メモリ・ファイル・端末設定…) を獲得したら、
**すべての出口**で解放コードを書く義務が人間に課される。

```c
int calibrate(void)
{
  struct termios saved;
  tcgetattr(STDIN_FILENO, &saved);            /* 端末設定を退避して raw モードへ */
  enable_raw_mode();

  if (step1() != 0) { tcsetattr(STDIN_FILENO, TCSANOW, &saved); return -1; }
  if (step2() != 0) { tcsetattr(STDIN_FILENO, TCSANOW, &saved); return -1; }

  tcsetattr(STDIN_FILENO, TCSANOW, &saved);   /* 正常系の出口にも書く */
  return 0;
}
```

出口が増えるたび復元コードが増え、1 箇所忘れると「エコーが戻らない端末」のまま
シェルに帰ることになる。`goto cleanup` 慣用句はこの苦労の緩和策であって解決策では
ない。Arduino で「ループの途中で return してピンが HIGH のまま残る」のと同じ種類の、
**正常系では顕在化せず、異常系の出口でだけ漏れる**問題である。

## 3.2 RAII — TerminalGuard の解剖

C++ の答えは RAII (Resource Acquisition Is Initialization) という技法:
**資源の獲得をコンストラクタに、解放をデストラクタに置き、資源の寿命を変数の
寿命に一致させる**。デストラクタは「スコープを抜けるとき必ず呼ばれる」
(return でも、途中の例外でも) ので、出口の数え上げが不要になる。

teleop の TerminalGuard は、3.1 の C コードとまったく同じ仕事の RAII 版である:

```cpp
struct TerminalGuard                       // teleop_keyboard.cpp:18-41
{
  TerminalGuard()                          // 獲得: 端末を raw モードへ
  {
    if (tcgetattr(STDIN_FILENO, &saved_) == 0) {
      saved_valid_ = true;
      termios raw = saved_;
      raw.c_lflag &= ~(ICANON | ECHO);
      // ...
      tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
  }
  ~TerminalGuard() { restore(); }          // 解放: スコープを抜ければ必ず復元
  void restore()
  {
    if (saved_valid_) {                    // 2 回呼ばれても安全 (冪等)
      tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
      saved_valid_ = false;
    }
  }
  termios saved_{};
  bool saved_valid_ = false;
};
```

使う側は main に 1 行置くだけ:

```cpp
int main(int argc, char ** argv)           // teleop_keyboard.cpp:276-286
{
  rclcpp::init(argc, argv);
  TerminalGuard term_guard;                // ← スタックに置くだけで契約成立
  auto node = std::make_shared<TeleopKeyboardNode>();
  rclcpp::spin(node);
  term_guard.restore();                    // 明示復元 (冪等なので二重でも安全)
  std::printf("\n");
  rclcpp::shutdown();
  return 0;
}
```

`q` キーでも Ctrl-C でも途中の例外でも、main を抜ける瞬間に `~TerminalGuard` が
走って端末が直る。3.1 で全出口に書いた復元コードが、**宣言 1 行に圧縮された**。
これが本書で最初に読む「自作クラスを作る理由」であり、クラスは「データの入れ物」
だけでなく「約束 (必ず復元する) の入れ物」にもなる、という例である。

## 3.3 スマートポインタ — 所有権を型にする

malloc/free の泣き所は、「誰が free するか」(= 所有権) がコメントと口約束でしか
表現できないことだった。C++ のスマートポインタは、**所有権のルールそのものを
型にした RAII オブジェクト**である。使い分けは 2 択 + 「持たない」しかない:

```
動的に作ったオブジェクトを誰が片付けるか
├── 所有者は 1 人 → std::unique_ptr<T> (make_unique で作る)
│   ├── 持ち主がスコープを抜けた瞬間に delete が自動で走る
│   ├── コピー不可 — 「所有者 1 人」を型が強制する (破ればコンパイルエラー)
│   └── 実例: tf_broadcaster_ (odometry:61 で作り、:233 で保持)
├── 所有者が複数 (or 1 人と言い切れない) → std::shared_ptr<T> (make_shared で作る)
│   ├── 参照カウント方式: コピーで +1、破棄で −1、0 になった瞬間 delete
│   └── 実例: ノード本体 (main の make_shared)、pub/sub/client/timer の全メンバ
└── 所有しない (借りて見るだけ)
    ├── 参照 & — null なし・付け替えなし (第4章 4.5)
    └── 生ポインタ — C API との境界にだけ残る
```

**`SharedPtr` の正体**: メンバ宣言に並ぶ `rclcpp::Subscription<T>::SharedPtr`
(controller:190 など) は、`std::shared_ptr<rclcpp::Subscription<T>>` の別名である
(第2章 2.5 の分解図の `::SharedPtr`)。rclcpp がすべての型に用意している短縮名で、
新しい概念ではない。

**なぜ ROS 2 は shared_ptr だらけなのか**: `create_subscription` が作った
subscription オブジェクトは、(1) 自分のメンバ `sub_` (登録の控え) と、
(2) rclcpp 内部 (executor がコールバックを呼ぶための参照 — 第5章) の
**両方から参照される**。所有者が 1 人と言い切れないから shared が既定になる。
逆に `tf_broadcaster_` は自分しか触らないので unique で足りる — odometry は
この使い分けを 1 ファイル内で見せてくれる。

⚠️ C の感覚との違い 1 点: `auto p2 = p1;` (shared_ptr のコピー) は**実体の複製では
ない**。同じ実体を指す 2 本目の手ができてカウントが 2 になるだけ — 意味論は
C のポインタコピーと同じで、そこに「最後の 1 本が離れたら自動 delete」が付く。

## 3.4 デストラクタの有無が教えること

RAII を理解すると、「デストラクタが書いてあるかどうか」がコードの読解情報になる。
3 ノードを並べると答え合わせができる:

| ノード | デストラクタ | 理由 |
|---|---|---|
| Epos4OdometryNode | **ない** | メンバ全部がスマポ・値型 → 自動で片付く。書く必要がない |
| Epos4_Control2_Node (`controller:136-147`) | **ある** | `std::thread` はスマポが面倒を見ない資源。join を手書きする (下記) |
| TeleopKeyboardNode (`teleop:99-104`) | **ある** | 資源ではなく振る舞い: 死ぬ前に停止 Twist を 1 発送る (第1章 1.4) |

controller のデストラクタが「手動の後始末」の見本である:

```cpp
~Epos4_Control2_Node()                     // epos4_controller.cpp:136-147
{
  stop_init_.store(true);                  // ワーカースレッドに停止を依頼し
  if (init_thread_.joinable()) {
    init_thread_.join();                   // 終わるのを待って回収 (第5章 5.4)
  }
  if (reenable_thread_.joinable()) {
    reenable_thread_.join();
  }
  shutdown_node();
}
```

join しないまま thread 変数が壊されるとプログラム全体が `std::terminate` で即死する
仕様なので、スレッドを持つクラスはデストラクタを書く義務が生じる。

ここから読解ルールが 2 つ出る:

- **デストラクタが書いてある = 自動では片付かない何か (スレッド・外部状態・
  最後に送るべきメッセージ) を持っている印**。何を持っているのかを探しに行く
- 逆に、デストラクタのないクラスに「解放漏れでは?」という心配は不要。
  メンバが全部スマポ・値型なら、**何も書かないのが正解の形** (odometry がそれ)

## 3.5 lock_guard — ロックも資源

RAII はメモリと端末だけの話ではない。「獲得したら必ず返すもの」全般に同じ型で
効き、mutex のロックがその代表である:

```cpp
case 'r': {
  std::lock_guard<std::mutex> lk(dist_mutex_);   // 獲得 = ロック (teleop:169)
  dist_left_ = 0.0;
  dist_right_ = 0.0;
} break;                                         // } を抜けた瞬間に必ずアンロック
```

`case` の中に `{ }` をわざわざ作っているのは、**ブロック = ロック区間**として
範囲を最小に絞るため。unlock の書き忘れや、例外で飛ばして持ちっぱなし、が
構文的に不可能になる。3.2 の TerminalGuard と完全に同じ原理で、対象が端末設定から
mutex に変わっただけである (この mutex が何を守っているかは第5章 5.5)。

## 3.6 この章のまとめ

```
所有権と RAII まとめ
├── RAII = 獲得をコンストラクタに、解放をデストラクタに
│   ├── 出口の数え上げ (C の全 return に fclose) が不要になる
│   └── TerminalGuard = termios の RAII 化 (teleop:18-41 + main:279)
├── スマートポインタ = 所有権ルールの型化
│   ├── unique_ptr: 所有者 1 人。コピー不可 ......... tf_broadcaster_
│   ├── shared_ptr: 参照カウント。0 で自動 delete ... ノード本体・pub/sub 全部
│   └── X::SharedPtr は std::shared_ptr<X> の別名にすぎない
├── new / delete / free が 0 回 — メモリ管理コード自体が消えている
├── デストラクタの有無は読解情報
│   ├── ある = 自動で片付かない資源の印 (controller のスレッド)
│   └── ない = 全メンバがスマポ・値型で正解の形 (odometry)
└── lock_guard = ロックの RAII。{ } がロック区間
```

所有の次は「渡し方」— メンバ関数を `create_subscription` に渡すとき、
this や引数はどう束ねられて運ばれるのか。ユーザが最も読めないと感じている
`std::bind(&X::f, this, _1)` を次章で解体する。

→ [第4章 コールバックと束縛](04_callbacks_binding.md)
