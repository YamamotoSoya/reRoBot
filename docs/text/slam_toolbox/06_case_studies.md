<!-- claude: slam_toolbox 読本 第6章 (2026-08-12) -->

# 第6章 事例集 — reRoBot で実際に起きた SLAM の事故と発見

理論は前章まで。この章は原則を破る (または知らない) とどう壊れるかの実録である。

| 事例 | 一言でいうと | 破った原則 | 状態 |
|---|---|---|---|
| [A](#事例a) | scan queue full で地図が出ない | 隠しパラメータ scan_queue_size=1 (第2章 2.4) | ✅ 修正済 (2026-05-28) |
| [B](#事例b) | 地図が「絶妙にずれる」 | prior の品質が SLAM の下限 (第1章 1.4) | ⚠️ 未解決 (複合要因) |
| [C](#事例c) | 2D SLAM が 3D SLAM の測定器になった | — (原則の実証例) | ✅ 診断完了 (2026-08-12) |
| [D](#事例d) | autostart が効かず起動しない | lifecycle は誰かが発火する (第2章 2.5) | ⚠️ 回避中 (原因未特定) |

## 事例A

**scan queue full — 40 Hz のスキャンがほぼ全滅していた**

- 一次資料: `docs/report/2026-05-26_slam_toolbox_scan_queue_full.md` (177 行)
- 時刻視点の解剖: [タイムスタンプ読本 第6章 事例D](../timestamp/06_case_studies.md) (相互補完)

### 症状

slam_toolbox を起動した瞬間から `Message Filter dropping message ... queue is full` が
約 2.5 秒間隔でログに流れ、`/map` が生成されない。bringup 単体では無警告 —
ただしこれは「正常」ではなく、**このログを吐く主体 (MessageFilter を持つノード) が
居ないだけ**だった。

### 機構

レート差 × キュー長 1 の必然:

```
/scan .............. 39.96 Hz (25 ms 間隔) ← urg_node
odom→base_link TF .. 20.00 Hz (50 ms 間隔) ← epos4_odometry (PDO sync 50 ms 由来)

scan(stamp=t) 到着時点で TF buffer の最新は t より最大 ~50 ms 過去
→ tf2 は外挿しないので次の TF を待つ
→ 待っている 25 ms の間に次の scan が到着
→ scan_queue_size: 1 では古い方が即 drop
```

さらに観測を難しくしたのが**ログの rate-limit**: drop 警告は tf2_ros 側で約 2.5 秒に
1 行に間引かれる。ログの見た目は「たまに落ちる」だが、実際の drop は scan レートと
同程度 = ほぼ全滅だった。数字の裏取り (`ros2 topic hz`、stamp の系統差 ±数 ms、
`ros2 param dump` で queue_size=1 の確認) は一次資料が詳しい。

### 修正

`scan_queue_size: 10` + `transform_timeout: 0.2 → 0.5` (git 33af0a0)。
計算上は `ceil(50 ms / 25 ms) = 2` で足りるが、`/odom` の瞬断 (epos4_odometry の
ApproximateTime 同期がペアを取り逃す時間帯) も吸えるよう 250 ms 分の 10 とした。
検証: drop ログ 12+ 件 → 0 件、TF map→odom が identity で配信開始、/map publish 開始。

### 教訓

- **scan レート > TF レートの構成では scan_queue_size (既定 1) は必ず溢れる**。
  しかもこのパラメータは上流のサンプル yaml に載っていない (第4章 4.9)
- 「上位ノードが居ないからエラーが出ない」を正常と誤認しない
- rate-limit されたログは発生頻度の証拠にならない — 件数はレートと窓から逆算する

## 事例B

**地図が「絶妙にずれる」 — SLAM パラメータを触る前に容疑者の序列を守る**

- 一次資料: `docs/issue/2026-07-07_monthly_2026_6_todo_triage.md` T11、
  `docs/monthly/2026_6` (「slamはできたが絶妙にずれる」「9号館のmapに存在しない空間」)

### 症状

9 号館で取得した地図が大枠では正しいが微妙に歪む。壁の二重化・存在しない空間の発生。
破綻ではなく「絶妙な」ずれ、というのが本人の表現。

### 機構

単独犯ではなく、寄与順に並ぶ複合要因と推定されている:

```
容疑者リスト (寄与が大きい順)
├── ① オドメトリ精度 — 特に yaw
│   └── yaw の積分は tread_width (0.41 m) の正確さに直結 (差動二輪の宿命)。
│       第1章 1.4 の通り、縮退環境では解がほぼ prior で決まる = prior の歪みは地図に直写
├── ② LiDAR FOV ±90° 制限 (第5章 5.5)
│   └── 拘束が減り、廊下・交差点でマッチングが甘くなる
├── ③ IMU 未融合 (SLAM 時は EKF なし — 第5章 5.4)
└── ④ ループ閉じ込みパラメータ — ほぼデフォルトのまま
    └── ただし「①〜③を直してから」。先に触ると原因を隠す
```

歴史的な注意もある: 2026-07-29 のエンコーダスケーリング 4 倍問題の修正
(`docs/issue/2026-07-07_wheel_odometry_encoder_scaling_4x.md`) 以前に取った地図は
そもそも距離が狂った prior で作られている。

### 修正

未解決。切り分けの定石は「①から順に、1 つずつ潰して再取得」:
tread_width は 360° その場旋回の実測 → ② FOV を ±135° に拡大 → ③ EKF 併用で SLAM →
それでも残るなら④に初めて手を付ける。

### 教訓

- **SLAM の「ずれ」は SLAM のパラメータで直すな、が第一手** — prior (オドメトリ) と
  入力 (FOV) の品質を先に確定させる
- チューニングの順序を誤ると「パラメータで症状を隠した状態」になり、後段の調整が全部
  砂上になる

## 事例C

**2D SLAM が 3D SLAM の測定器になった — odom prior の効果の実証実験**

- 一次資料: `docs/issue/2026-08-12_glim_horizontal_drift.md`
  (画像: `docs/issue/img/2026-08-12_glim_drift/02_2d_slam_clean.png`)
- 3D 側の解剖: [GLIM 読本 第6章 事例A](../glim/06_case_studies.md)

### 症状

(こちらは slam_toolbox の事故ではなく、活躍の記録である)
3D SLAM (GLIM) が同じ 235 s の bag で水平方向に数 m 塗れる地図を出した。
「入力データ (R-Fans) が悪いのか、GLIM 側が悪いのか」を切り分ける必要が生じた。

### 機構

R-Fans の水平リング (仰角 −1° の laserid 7 とその対称) を LaserScan に変換し、
slam_toolbox に食わせた。結果は壁がシャープに閉じた綺麗な 2D 地図 (占有 9,250 セル) —
**同じセンサの同じ走行データで 2D は解けた**。これで入力は無罪、差分は手法側と確定した。

決定的な差は第 1 章 1.4 の構図そのものである:

```
同じデータ・同じ環境での差
├── slam_toolbox = スキャンマッチング + 【車輪オドメトリ prior】 + 3 自由度
│   └── 廊下で縮退する前進方向を車輪が直接アンカー → 解ける
└── GLIM (LIO) = スキャンマッチング + IMU、【車輪 odom 不使用】 + 6 自由度
    └── IMU は姿勢・重力には強いが、前進方向の縮退は救えない → 流れる
```

おまけの発見として、prior を車輪 yaw から IMU yaw に替えても結果はほぼ同等
(9,395 セル) — slam_toolbox は prior の出所に対して頑健だった (探索窓の中に真値が
入ってさえいれば相関マッチングが吸収する、という第 3 章 3.1 の性質の実証)。

### 修正

(診断としては完了) 3D 側の対策は GLIM 読本 事例A に譲る。

### 教訓

- **切り分けに「もう 1 本の独立した処理系」を使う**のは強力 — 入力を共有して手法だけ
  替えれば、差分が手法に帰着する
- 2D SLAM の頑健さの源泉は odom prior。逆に「odom prior の無い SLAM」を導入するときは
  縮退環境での挙動を最初に疑う

## 事例D

**autostart 不発 — 起動したのに何も起きないノード**

- 一次資料: `docs/issue/2026-07-07_monthly_2026_6_todo_triage.md` T7 (未解決)・T4・C2

### 症状

yaml に `autostart: true` を書いたのに、slam_toolbox が unconfigured のまま何もしない。
エラーも出ない。/map も TF も出ない。

### 機構

第 2 章 2.5 の通り、lifecycle node は誰かが遷移を発火するまで働かない。
Jazzy の apt 配布版 (2.8.5) では autostart による自己遷移が機能しなかった。
原因は未特定で、候補は ①配布バイナリが autostart 対応前 ②configure のみで activate
しない実装 ③activate 直後に入力不備で inactive に戻る、の 3 つ (T7 に調査手順あり)。

### 修正

(回避) launch 側から EmitEvent + OnStateTransition で CONFIGURE→ACTIVATE を明示発火
(第 5 章 5.2)。公式 launch と同じパターンなので上流的にも正攻法である。
yaml の autostart: true は保険として残置 (`slam_toolbox.yaml:21-25`)。
将来 nav2_lifecycle_manager 方式に整理する案が T4 にあるが、
**bond_timeout: 0.0 を忘れると activate 後に kill される**罠がある (第4章 4.9)。

### 教訓

- lifecycle node の「無反応」はエラーではなく状態。まず `ros2 lifecycle get` (第 7 章)
- 「パラメータに書いたのに効かない」ときは、そのパラメータを読む主体とバージョンを疑う

## 章のまとめ — 事例を貫く一本の線

```
事故の共通構造
├── 見えない既定値が地雷になる (A: scan_queue_size=1, D: autostart 不発)
│   └── 対策: 「動いている」の根拠を必ずコマンドで確認する (第7章)
├── SLAM の品質問題は SLAM の外に原因がある (B: オドメトリ/FOV, C: prior の有無)
│   └── 対策: パラメータチューニングは容疑者リストの最後
└── 独立系での比較が最強の切り分け (C)
```

→ [第7章 実務チェックリスト](07_checklist.md)
