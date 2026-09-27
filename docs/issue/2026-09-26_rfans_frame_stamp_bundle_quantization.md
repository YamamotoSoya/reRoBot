<!-- claude: docs/issue — 未解決問題の調査記録。解決したらステータスを更新すること。2026-09-26 作成。 -->
# R-Fans フレーム stamp が UDP bundle 到着 (41 ms) に量子化され、隣接間隔が 83 / 123 ms で交互する

- **ステータス: 原因特定済み・ドライバ修正実装済み (submodule 未コミット)・bag 再生で検証済み・GLIM 効果測定済み (警告 1,597 → 0、z はノイズ床内)。実機 (live) での確認は未。**
- 日付: 2026-09-26。bag `2026-09-25_0751` の R-Fans 欠落確認 (欠落ゼロ) の副産物として発覚。
- 環境: `surestar_rfans_ros2` (ros2_ws_main/src/drivers、commit 0f4566a 時点)、`use_gps: false`、data_level 3 (processPacketUserSimple 経路)、R-Fans-16 (V6K-16G) 10 Hz。
- 関連: `docs/issue/2026-08-14_rfans_driver_renewal.md` (08-14 の stamp 巻き戻し)、`docs/issue/2026-09-10_glim_z_drift_not_vangle.md` (残留時刻オフセット −15 ms の議論)、GLIM ログの `imu_rate stamp does not cover the scan duration range (~17 ms)` 警告。

## 1. 観測 (bag 2026-09-25_0751、14,131 フレーム)

| 項目 | 値 |
|---|---|
| フレーム stamp 間隔 | **83 ms: 7,844 回 / 123 ms: 6,286 回** (100 ms 前後はほぼ無し)、隣接間隔の相関 −0.80 (交互) |
| 受信 (bag 記録時刻) 間隔 | 同じく 83 / 123 ms の交互 → stamp の付け方の問題ではなく、フレームの完成・publish 自体がこの刻み |
| 受信 − stamp | 常に 100 ms (5〜95% で 100〜101 ms) = 08-14 の「span 分巻き戻し」 |
| `/rfans_driver/rfans_packets` (bundle) | 24.43 Hz = **40.9 ms 間隔**、1 bundle = UDP 32 個 × 1206 B、bundle の `stamp` は先頭 UDP の到着時刻 (受信 − stamp = 39.7 ms) |
| bundle 内 `gps_timestamp` (機器 µs カウンタ) | bundle ごとに +40,919 µs → 機器時計とも 40.9 ms/bundle で整合 |

## 2. 機構

```
UDP 1 個 (1.28 ms) ×32 → bundle (40.9 ms) を ioapi.cpp revPacket が作り、先頭 UDP の recvfrom 直後に pkt.stamp = now()
→ bufferDecode.cpp Depacket が bundle 単位で点を積み、方位角が 1 周したところでフレーム確定
→ フレーム確定は「最後の点を含む bundle を処理した時」= bundle 到着 (40.9 ms 刻み) に量子化
→ publishCloud: stamp = now() − (tmax − tmin)   ← now() が 41 ms 刻みで揺れる
→ 1 周 100 ms は bundle 2.44 個分なので、フレーム完成の遅れが 2 bundle / 3 bundle と交互 → stamp 間隔 83 / 123 ms
→ stamp は真のスキャン開始から ±20 ms 交互にずれる。per-point `time` はこの stamp 基準の相対値なので、スキャン全点が一緒にずれる
```

影響: GLIM の deskew と IMU 因子で LiDAR–IMU の時刻整合が毎フレーム ±20 ms 揺れる (0.5 m/s で 1 cm、30°/s 旋回で 0.6°/フレーム)。交互なので長区間では相殺しやすく、z ドリフト A の主因ではないと見るが、GLIM の `imu_rate stamp does not cover the scan duration range` (≈17 ms) 警告と、09-10 issue の残留オフセット −15 ms はこの量と一致する。

## 3. 修正 (実装済み、`src/bufferDecode.cpp`、タグ `claude: frame-stamp anchor (2026-09-26)`)

- bundle 先頭パケットについて `offset = bundle.stamp (PC 到着) − gps_timestamp × 1e-6` を直近 32 bundle 分リングに保持し、**最小値 (= 遅延最小)** を機器時計→PC 時刻の変換に使う。
- `publishCloud` の stamp を `tmin (フレーム先頭点の機器 µs) × 1e-6 + offset` に変更。フォールバック: リングが 4 未満 / 機器カウンタの 1 時間巻き戻り (リング reset) / 値が最新 bundle 到着の −0.5〜+0.005 s 外 → 従来の `now() − span`。
- フックは data_level 2 / 3 (`processPacketUser` / `processPacketUserSimple`、**実機は 3**) と V6 分岐 ×2 の bundle 先頭 (i==0)。use_gps=true 経路は不変。
- 機器時計と PC 時計の相対ドリフトは 1.3 s 窓内で無視できる。stamp 精度は「先頭 UDP の到着ジッタ (≈1 ms)」と「per-point time の float32 量子化 (~256 µs)」で決まる。

### 検証 (bag 再生、09-26)

`ros2 bag play ... --topics /rfans_driver/rfans_packets --rate 4` を修正後の `calculation_node` に流し、`/rfans_driver/rfans_points` の stamp 間隔を 60 s 計測:

| | 修正前 (bag 記録値) | 修正後 (再生) |
|---|---|---|
| stamp 間隔 中央値 / 5% / 最小 | 83 or 123 ms (双峰) | **100.1 / 99.7 / 99.5 ms** |
| 隣接相関 | −0.80 | +0.04 |
| ログ | — | `frame stamp anchored to device clock (offset ..., 5 bundles)` |

(再生時の 95% 300 ms・最大 600 ms は 4 倍速で計測側 best-effort 購読が取りこぼしたもの = 100 ms の整数倍。stamp 誤りではない。)

### 3.1 再生で見つかった 2 つ目の問題と修正 (09-26)

録り直し (packets + imu を 1 倍速再生 → 修正版 calculation_node → record) の bag を GLIM に流すと、**先頭 1 フレームだけ stamp が「再生日の壁時計」**で、GLIM がそれ以降の全フレームを「timestamp rewind」として捨てた (odom 0 行)。原因は anchor リングが揃う前 (先頭 <4 bundle) のフォールバック `now() − span` — live では now() と anchor が同じ時計なので無害だが、再生では別日になる。修正: フォールバックの基準を `最新 bundle の到着 stamp` (live = 壁時計、再生 = 記録時刻) に変更 (同ファイル、タグ `claude (2026-09-26): fallback`)。既に録った bag は先頭の異常フレームを除外して書き直す (`filter_epoch_bag.py`)。
教訓: 再生で検証するときは stamp 間隔だけでなく **先頭フレームの絶対時刻**も見る。

## 4. 残作業

1. **実機 live で確認** (次回 bringup 時): `ros2 topic echo /rfans_driver/rfans_points --field header.stamp` の間隔が 100 ± 1 ms、ログに anchored 行、`imu_rate stamp does not cover` 警告が GLIM から消えるか。
2. ✅ **GLIM への効果** (09-26): 録り直し bag (`exp_2026-09-25_tareoff_0751/restamp_bag_f`、14,129 フレーム) × live config → **`imu_rate stamp does not cover the scan duration range` 警告 1,597 → 0**、IMU 回転予測優位率 0.86 → 0.89、odom 段 A 1.04 → 1.46° / global z_end +20.7 → +13.2 m (いずれも run 間ノイズ床 内)。時刻整合の警告源としては完全解消、z ドリフトのレバーではない。
3. submodule のコミット + 親の gitlink 更新 (bus.yml と同じ運用)。過去 bag の stamp は修正できない (rfans_packets から再生成すれば可: 上記 2 の手順)。
4. 08-14 の `docs/issue/2026-08-14_rfans_driver_renewal.md` の stamp 節に本 issue への参照を追記 (未)。
