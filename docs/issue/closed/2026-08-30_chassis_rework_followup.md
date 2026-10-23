<!-- claude: 2026-08-30 車体改修のフォローアップ作業リスト。実測値が入り次第、該当行を消し込むこと。-->
# 車体改修 (2026-08-30) フォローアップ — 実測待ちの編集箇所一覧

- **ステータス: ほぼ完了 (08-31)** — 残は ④回転方向の浮かせ確認・⑤実機検証のみ。完了した項目はチェックを付けて消し込む。
- **背景**: 2026-08-30 に減速機 5:1 → **92.25:1**・タイヤ φ0.15 → **φ0.256 m** (08-31 実測補正、当初 0.25 と申告) に交換、トレッド幅は **0.529 m** (08-31 実測)、センサ搭載位置も変更 (未実測)。モータ/エンコーダは同一個体 (bus.yml の 2π/1024 は据え置きで確定)。
- 完了済み: EPOS4 ハードニング (0x60C5/0x6085/0x6084 = 30000 rpm/s・電流 10 A、SDO 裏取り済み — `docs/reference/2026-07-27_epos4_studio_startup_settings.md` に記録)、params の gear_ratio 92.25 / tire_diam 0.256 / tread_width 0.529 / slew limiter 15000 rpm/s への追従、センサ 3 種の接続確認。
- 新チャシスの換算メモ: 車体 1 m/s² ≈ **6882 rpm/s** (モータ軸)、最高速 ≈ **1.12 m/s** (モータ 7720 rpm 制約)、タイヤ 1 回転 = 94,464 inc = 手回しで距離表示 ≈ **0.80 m**。

⚠️ 行番号は 2026-08-30 時点。ズレたら joint 名・キー名で探すこと。URDF・params は symlink install のため**編集後の再ビルド不要**。

## ① トレッド幅 (0.529 m 実測済み 08-31)

- [x] `ros2_ws_main/src/bringup/rerobot_bringup/config/params.yaml:9` — `tread_width` 0.529 (epos4_controller_node、08-31 ユーザ反映済み)
- [x] `ros2_ws_main/src/bringup/rerobot_bringup/config/params.yaml:25` — `tread_width` 0.529 (epos4_odometry_node、08-31 ユーザ反映済み)
- [x] URDF ×4: 車輪 joint `m1_wheel` / `m2_wheel` の origin **y=±0.2645** (✅ 08-31: flat はユーザ、他 3 ファイルへは Claude が伝播)

## ② タイヤ径 φ0.256 m の URDF 反映 (✅ 08-31 完了)

- [x] URDF ×4: 車輪 cylinder `radius` **0.128** / 車輪 joint origin `z` **0.128**

## ③ センサ位置 (✅ 08-31 実測反映 — flat をユーザが実測、Claude が全プリセット + GLIM に伝播)

- [x] `laser_joint`: **(0.067, 0, 0.09406)** + **上下反転取付 → rpy=(π,0,0)** (ユーザ申告 08-31。⚠️ 反転軸が roll か pitch かで前方が 180° 変わる — /scan で「前方障害物が +X」を要確認、ズレたら pitch=π へ)
- [x] `rfans_joint`: flat 実測 **(0, 0, 0.80246)**。tilted45/15 は **flat 実測 + 旧オフセットの換算値** (45°: (0.055, 0, 0.82746) / 15°: (0.038, 0, 0.80746)) — ⚠️ **傾け取付へ実際に切り替える時は要再実測**
- [x] `imu_joint`: **(0, 0, 0.74196)**、rpy (0,0,π/2) は不変
- [x] GLIM `T_lidar_imu` 再計算済み (numpy 検算・回転は既存値一致で妥当性裏取り): flat t=(0, 0, −0.0605) / tilted45 t=(−0.0215668, 0, −0.0993485) / tilted15 t=(−0.0197525, 0, −0.0731033)
- [x] **live を flat プリセットに切替** (cp 2 発セット: rerobot.urdf + config_sensors.json。check_urdf ×4 / JSON パース ×4 OK、live⇔flat の一致確認済み)

## ④ 浮かせテストで回転方向が逆だったら (減速機交換で反転の可能性)

- [ ] `params.yaml:13-14` / `:31-32` — `invert_left` / `invert_right` (controller / odometry、現在どちらも true)
- [ ] `ros2_ws_main/src/app/epos4_teleop/config/params.yaml:5-6` — teleop 側 invert (距離表示の符号用、現在 false)

## ⑤ 実機検証 (編集後)

- [ ] 浮かせ: teleop 前進 → 両輪前転か (④の判定)。車輪 1 回転手回し → 距離表示 ≈ 0.80 m (較正確認)
- [ ] 次回 EPOS 電源再投入時: SDO 再読で NVM 永続確認 (0x60C5=30000 等が残っているか。raw SDO 手順はタイムライン 08-30)
- [ ] 接地: 低速から 10 m 直進・360° 旋回 (旧チャシスの接地検証残タスクを新チャシスで実施)

## 横断規約 (CLAUDE.md 由来)

1. **URDF は 4 兄弟** — `rerobot.urdf` (live) + `rerobot.{tilted45,tilted15,flat}.urdf`。**rfans_joint 以外**の変更は 4 ファイル全部に同じ変更を入れる (行番号は ±1 ズレあり — joint 名で探す)。
2. **GLIM config も 4 兄弟** — `config_sensors.json` + `.{tilted45,tilted15,flat}.json`。T_lidar_imu はプリセット毎に回転が違うため、imu 位置変更時は 4 つとも並進を個別再計算。

## 関連

- `docs/claude/PROJECT_STATE.md` §10-0 (残タスクの正) / タイムライン 08-30
- `docs/reference/2026-07-27_epos4_studio_startup_settings.md` — EPOS4 NVM の現在値 (08-30 更新済み)
- `docs/issue/2026-08-11_joy_spin_epos_shutdown_usb_stall.md` — ハードニングの動機となった事故
