<!-- claude: docs/report テンプレート。debug-report スキルから生成。-->
# rfans_driver の R-Fans-16(dataID 0x37)で「no matching device model」「点群が z=0 の平面(2D)」になる

- 日付: 2026-06-13
- 環境: reRoBot コンテナ `rerobot_env` / ROS 2 Jazzy / `rfans_driver`(submodule `src/external/StarROS2`)。LiDAR は R-Fans-16 実機(UDP port 2014, data_level=3 固定)。点群描画は RViz2(fixed_frame=`world`)。
- 対象ブランチ: `main` (HEAD: `9a158c0 Applicate clang-format ...`)。submodule `StarROS2` は `master` (HEAD: `a0320ba modify for ros2 (issue : z=0 and low fps)`)
- 関連ファイル:
  - `src/external/StarROS2/src/rfans_driver.cpp`(LED 本数自動判定 / `calcXYZ` 呼び出し / パラメータ宣言)
  - `src/external/StarROS2/src/rfans_driver.h`(remap メンバ変数)
  - `src/external/StarROS2/launch/rfans_driver.launch.py`(launch 引数)
  - `src/external/StarROS2/lib/libstar.so`(プリビルド SDK。`calcTheta` / 縦角テーブルの解析対象)
  - `src/external/StarROS2/src/ssFrameLib.h`(`RFANS_GM_16_FLAG=0x3732` 等)

## TL;DR

R-Fans-16 実機が送るパケット `dataID=0x37` が、(a) ドライバの LED 本数判定テーブルにも、(b) SDK(`libstar.so`)の縦角テーブル選択 `CalMultiLidarCoor::calcTheta(dataid,laserid)` のジャンプテーブルにも**未登録**だったことが単一の根本原因。結果、起動時に「no matching device model」、かつ縦角 θ=0 で全点 z=0 の平面点群、が同時に発生していた。対処は、(a) ドライバの 16本テーブルへ `0x37` を追加 + `model` 引数フォールバック、(b) `calcXYZ()` 直前で `dataID 0x37 → 0x57` に remap(launch 引数 `theta_remap_to=87` で可変)。`0x57` は SDK 内の GM-16 縦角テーブル `VAngle_V6B_16G`(±15°/2°間隔)へ分岐するため、θ・方位・360° がまとめて正常化する。

## 症状

- **いつ**: `ros2 launch StarROS2 rfans_driver.launch.py` で R-Fans-16 実機を起動した直後から継続的に。
- **どこで**: `rfans_driver`(driver_node)。点群は `/sdk_could`(PointCloud2)。
- **何が起きるか**:
  1. 起動ログに `no matching device model` が出続ける(本数自動判定が既知テーブルに当たらず 32 にフォールバック)。
  2. RViz で点群は出るが**全点 z=0 の平面(2D)**。AxisColor(Z)にすると全点同色。
  3. (回避策の試行で)未登録 dataID へ remap すると、**方位 180° 分しか出ず円形に破綻**。
- **正常時**: R-Fans-16 は縦 ±15° に 16 ライン。建物の壁・天井が 3D 構造として立ち、方位は全周 360° 見えるべき。

修正後にユーザが取得した本数判定ログ抜粋(LED_num フォールバック導入後):

```
[driver_node-1] [WARN] [1781341064.341386753] [rfans_driver]: no matching device model (dataID=0x37, points=15008) -> falling back to LED_num=16 from model='R-Fans-16'
```

## 切り分けの記録

### 1. 「no matching device model」は model 引数の誤りではない(観測)

ソースを追うと、LED(レーザ)本数は launch の `model:=R-Fans-16` ではなく、**受信パケットの `dataID` から自動判定**していた(`rfans_driver.cpp` の `Asyn_date`/`syn_date` 内、`packets.points.at(20).dataID` を既知テーブルと照合)。`dataID` は `LasShot_S.dataID`(`star/Shot.h`)で、SDK デコーダが埋める「データレベル/形式 ID」。

実測のため `else` ブランチに dataID と点数を出すログを追加した(`RCLCPP_INFO`→`RCLCPP_WARN` に格上げ)。

| 項目 | 実測値 | 備考 |
|------|--------|------|
| dataID | `0x37` | 既知テーブル(16本: 0x57,0x58,…,0x90 / 32本: 0x50,… / 8本: 0x80,0x81)のいずれにも非該当 |
| points/scan | 15008 | デコード自体は流れている(`dataID=0` ではない=形式不一致ではない) |
| model 引数 | `R-Fans-16` | 本数判定には未使用 |

→ 推測: `0x37` は既知テーブル外の形式(`ssFrameLib.h` の `RFANS_GM_16_FLAG=0x3732` から GM-16 系と推定)。本数は model 引数フォールバックで 16 に矯正でき、点群は出るようになった。

### 2. z=0(2D)は移植バグではなく既知問題(観測)

本数を 16 に直しても点群は平面のまま。`a0320ba` のコミットメッセージが `issue : z=0 and low fps` と明記。元 ROS1 初版 `640892e` の R-Fans 初期化を確認したところ、現行と**完全に同一**(`configure.deviceVersion().type = lidar::RFans` のみ、`CalCoor::setDeviceType()` 未使用)。よって移植リグレッションではなく以前からの未解決問題。

PointCloud2 のフィールド配置(`InitPointcloud2`)と点構造体 `RFANS_XYZ_S`(`point_types.h`)を突き合わせ、`z` は offset 8 の float で整合、`point_step=41`(packed)も一致 → **Z データ自体が 0**(フィールドマッピングのズレではない)と確定。

### 3. ビーム分離は正常 / 縦角だけ未適用(観測)

RViz で Color Transformer=Intensity, Channel=`laserid` に設定 → **ほぼリング状に色分け**された。

| 観測 | 解釈 |
|------|------|
| laserid がリング状に変化 | 16 本のビーム分離(lidarID 0〜15)は成立している |
| z は依然 0 | 各ビームの**仰角(θ)だけ**が当たっていない |

→ 幾何上 `Z=R·sinθ`、`水平=R·cosθ`。θ=0 なら Z=0・水平=R(=平面リング)で症状と一致。

### 4. 当たらなかった回避策: 未登録 dataID への remap

`calcXYZ()` 直前で `dataID 0x37 → 0x90` に書き換えて SDK に縦角を引かせる、という当て推量を試行 → **円形に破綻 + 方位 180° のみ**。後述(根本原因)の通り `0x90` も SDK 未登録でジャンプテーブルの DEFAULT に落ち θ=0、さらに `calcRFansXYZ` 内の方位処理まで乱す。**「適当な既知っぽい dataID」への remap は逆効果**と判明。

### 5. 最終的に当たった仮説: SDK の dataID→縦角テーブルに 0x37 が無い(バイナリ解析で確証)

`lib/libstar.so` を解析。詳細は次節。`calcTheta` のジャンプテーブルで `0x37 → DEFAULT(θ=0)`、正しい GM-16 縦角テーブル `VAngle_V6B_16G` へは `0x57` が分岐すると判明 → `0x37→0x57` remap に変更。

## 根本原因

**単一原因: 実機の `dataID=0x37` が、ドライバと SDK の双方で「未知の形式」だった。**

### SDK 側(z=0 の真因)

座標計算はプリビルド `lib/libstar.so` 内の `ss::calc::CalMultiLidarCoor` が担う。R-Fans の縦角は `calcTheta(unsigned short dataid, unsigned short laserid)` が引く。逆アセンブル(`objdump -d`, 関数先頭 `0x191840`)すると、`dataid` を使った**ジャンプテーブル方式の switch**だった:

```asm
191840: sub    $0x24,%esi              ; index = dataid - 0x24
191843: cmp    $0xdb,%si               ; 範囲 [0x24, 0x24+0xdb=0xFF]
191848: ja     191a50                  ; 範囲外 → DEFAULT(θ=0)
19184e: lea    0xd060f(%rip),%rax       ; jump table @ .rodata 0x261e64
191858: movslq (%rax,%rsi,4),%rcx       ; target = 0x261e64 + int32[index]
19185c: add    %rcx,%rax
19185f: jmp    *%rax
```

各分岐は名前付きの縦角テーブル(`.rodata` の `VAngle_*` シンボル)を読む。ジャンプテーブル(0x261e64 の int32×220)を読み出して `dataid → テーブル` を復元した結果:

| dataID | 分岐先テーブル | 本数 |
|--------|---------------|------|
| **0x37** | **DEFAULT(θ=0)** | — ← 本件 |
| 0x90 | DEFAULT(θ=0) | — ← 当て推量で踏んだ罠 |
| 0x57 / 0x5C / 0x8D | `VAngle_V6B_16G` | 16(GM) |
| 0x58 / 0x4F / 0x8C / 0x96 / 0xAA | `VAngle_V6B_16M` | 16 |
| 0x55 / 0x9F | `VAngle_16E1` | 16 |
| 0x56 / 0x8F | `VAngle_16E2` | 16 |
| 0x5B | `VAngle_V6K_16M` | 16 |
| 0x24 | `VAngle_V6K_16M_0X24` | 16 |

`.rodata` から double で抽出した GM-16 テーブル(これが正解の縦角):

```
VAngle_V6B_16G = [-15,-13,-11,-9,-7,-5,-3,-1, 1, 3, 5, 7, 9, 11, 13, 15] (deg)
```

`0x37` はテーブルの**有効範囲内(0x24〜0xFF)だが、その index のエントリが DEFAULT に向いている** → `calcTheta` が常に 0 を返す → 全ビーム θ=0 → z=0。SDK には `theta_16` という汎用シンボルは無いが、形式別の 16本テーブル(`VAngle_*_16*`)は持っている。つまり SDK は 16本機自体は扱えるが、**`0x37`(GM-16 の一形式)という dataID を未登録**だっただけ。

### ドライバ側(「no matching device model」の真因)

`Asyn_date`/`syn_date` の本数自動判定 `if/else` の列挙に `0x37` が無く、最後の `else` に落ちて警告 + `LED_num=32`(誤り)。`LED_num` は `col`(点群の列インデックス)計算 `col = hangle/(scanSpeed*LED_num*360/PonintFrequency)` に効くため、32 誤判定で格子構造が歪む。

### なぜ今まで顕在化しなかったか

元 ROS1 実装(`640892e`)から R-Fans の初期化は `deviceVersion().type=lidar::RFans` のみで、特定機種(16/32, GM 形式)を SDK に伝える経路が無かった。同梱 `libstar.so` がこの個体の `0x37`(GM-16)世代を知らない版だったため、移植前から潜在的に z=0 だったと考えられる(コミットメッセージ `issue: z=0` がそれを裏付ける)。実機で 3D 検証された記録が無く、表面化していなかった。

## 修正

すべて submodule `src/external/StarROS2` 内。**`colcon build --packages-select rfans_driver` での再ビルドが必要**(コンテナ Jazzy 内)。

### (a) 本数判定の修正 — `src/external/StarROS2/src/rfans_driver.cpp`

`Asyn_date`/`syn_date` 双方の 16本側 `else if` に `0x37` を追加。さらに、自動判定が外れても `model` 引数から本数を採るフォールバック関数を追加し、`else` の `RCLCPP_INFO`→`RCLCPP_WARN`(dataID/点数/採用値出力)に格上げ。

```cpp
// 16本側テーブルに追記(2 関数とも)
||packets.points.at(20).dataID==0x90
||packets.points.at(20).dataID==0x37)   // claude: 実機 R-Fans-16 が送る dataID
{
  LED_num=16;

// 既知テーブルに当たらないときのフォールバック
static int ledNumFromModelName(const std::string& name) {
  if (name.find("-128") != std::string::npos) return 128;
  if (name.find("-16")  != std::string::npos) return 16;
  if (name.find("-32")  != std::string::npos) return 32;
  return 32;  // 未知名は従来どおり 32
}
```

### (b) z=0 の修正 — `calcXYZ()` 直前で dataID remap(2 箇所)

```cpp
calout.lasShot = *iter;
// claude: 未知 dataID を SDK 既知の R-Fans-16 値へ書き換え、calcTheta に
//         正しい縦角を引かせて z を埋める(z=0/2D 回避)。to=0 で無効。
if (theta_remap_to_ != 0 && calout.lasShot.dataID == theta_remap_from_) {
  calout.lasShot.dataID = theta_remap_to_;
}
calCoor.calcXYZ(&calout);
```

### (c) パラメータ化 — `rfans_driver.h` / `setupNodeParams` / launch

```cpp
// rfans_driver.h(既定値)
int theta_remap_from_ = 0x37;   // 書き換え元(実機 GM-16)
int theta_remap_to_   = 0x57;   // 書き換え先 → VAngle_V6B_16G

// setupNodeParams()
theta_remap_from_ = declare_parameter<int>("theta_remap_from", 0x37);
theta_remap_to_   = declare_parameter<int>("theta_remap_to",   0x57);
```

```python
# rfans_driver.launch.py(10進。0x37=55, 0x57=87)
theta_remap_from = DeclareLaunchArgument("theta_remap_from", default_value="55")
theta_remap_to   = DeclareLaunchArgument("theta_remap_to",   default_value="87")
# parameters dict に ParameterValue(..., value_type=int) で配線
```

### なぜこの値 / この方法か

- **なぜ remap(SDK 既知 dataID への偽装)か**: 縦角を自前計算する案もあったが、SDK の X,Y(方位)は素の 0x37 で正しく出ていた。`0x57` のような**実在する 16本 GM dataID**へ書き換えれば、θ だけでなく方位・360°処理も SDK の自己整合した正規経路に乗るため、自前計算の laserid 並び/`cosθ` 補正の当て推量を避けられる。
- **なぜ当て先 0x57 か**: バイナリ解析で `0x57 → VAngle_V6B_16G`(±15°/2°対称)と確定。実機は GM-16(`RFANS_GM_16_FLAG=0x3732` 系)なので、同じ GM テーブルへ飛ぶ `0x57/0x5C/0x8D` が物理ビーム配置に最も合う公算が高い。RViz で「それっぽい 3D」が確認できた。
- **なぜ 0x90 を採らなかったか**: 0x90 はジャンプテーブル DEFAULT に落ち θ=0、さらに方位が 180° に破綻した(実測)。**テーブル外の値への remap は禁忌**。
- **破綻条件 / 再評価トリガー**: 別個体・別ファームで `dataID` が 0x37 以外になったら `theta_remap_from` を更新する必要あり。各リングの仰角間隔が**不均等**に見えたら、対称表 `V6B_16G`(0x57)ではなく非対称表 `V6B_16M`(0x58=88)等へ `theta_remap_to` を切り替える(リビルド不要、launch 引数で可)。

## 検証

| 項目 | 修正前 | 修正後 |
|------|--------|--------|
| 起動ログ | `no matching device model`(LED_num=32 にフォールバック) | 本数判定が 16 を返し警告消失(0x37 をテーブル登録) |
| LED_num | 32(誤) | 16 |
| 点群 Z | 全点 z=0(平面・AxisColor 単色) | 高さ方向に広がり、3D 構造が「それっぽく」見える(ユーザ確認) |
| 方位範囲 | (0x90 remap 試行時)180° のみ・円形破綻 | 0x57 remap で破綻解消 |

検証手順(Jazzy コンテナ `rerobot_env` 内):

```bash
colcon build --symlink-install --executor sequential --packages-select rfans_driver
source install/setup.bash
ros2 launch StarROS2 rfans_driver.launch.py          # 既定 0x37→0x57

# RViz2: Fixed Frame=world, PointCloud2(/sdk_could) を追加
#  - Color Transformer=AxisColor(Z) … 高さ方向の色グラデーションを確認
#  - Color Transformer=Intensity, Channel=laserid … 16 リング分離を確認

# 当て先の微調整(リビルド不要)
ros2 launch StarROS2 rfans_driver.launch.py theta_remap_to:=92   # 0x5C(同 V6B_16G)
ros2 launch StarROS2 rfans_driver.launch.py theta_remap_to:=88   # 0x58(V6B_16M, 非対称)
ros2 launch StarROS2 rfans_driver.launch.py theta_remap_to:=0    # remap 無効(素の 0x37=z0 に戻す)
```

注: 現時点の確認は RViz 目視での定性評価(「それっぽい 3D」)まで。各リング仰角の定量検証(実測壁面との角度照合)は未実施。

## 教訓 / 今後の予防

1. **LiDAR の異常は「パケット dataID」をまず実測する**。本数判定も縦角選択も dataID をキーにしている。`else`/DEFAULT に落ちる症状では、推測せず該当箇所に `dataID` を出すログを仕込んで実値を取る(本件は `0x37`)。
2. **プリビルド SDK が絡む不具合は、ソースが無くてもバイナリで詰められる**。エクスポートシンボル(`nm -CD`)→ 関数逆アセンブル(`objdump -d`)→ ジャンプテーブル / `.rodata` 定数の抽出、で「どの入力がどのテーブルに飛ぶか」「正解の値は何か」を確定できる。本件はこれで `0x57 → VAngle_V6B_16G` を特定した。
3. **dataID の remap は「テーブルに実在する値」にしか向けてはいけない**。範囲内でも DEFAULT に落ちる値(0x90 等)へ remap すると θ=0 のうえ方位処理まで壊れる(180°/円形破綻)。当て先は逆アセンブルで分岐先テーブルを確認してから選ぶ。
4. **「2D に見える点群」= まず Z データ自体が 0 か、PointCloud2 フィールド配置のズレかを切り分ける**。構造体 offset と `point_step` を実値突き合わせし、次に RViz の `laserid` 色分けでビーム分離の生死を見ると早い。
5. フォローアップ TODO:
   - 各リング仰角の**定量検証**(既知壁面との角度照合)で `0x57`(対称 V6B_16G)か `0x58`(非対称 V6B_16M)かを最終確定する。
   - コミットメッセージにあった **low fps** 問題は本件と別 → 解決済み(別レポート `docs/report/2026-06-13_rfans_driver_low_fps_per_point_spin.md`。点ごと spin_some が原因、~1〜2→~20 Hz に改善)。
   - 恒久対応として、`0x37` を SDK の `calcTheta` 経路に正式に乗せる方法(ベンダー `sdk@isurestar.com` 提供の対応 SDK 入手、または `dataID` 正規化を 1 箇所に集約)を検討。現状は remap という対症療法。
