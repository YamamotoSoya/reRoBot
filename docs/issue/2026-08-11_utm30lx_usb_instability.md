<!-- claude: docs/issue — 未解決問題の調査記録。解決したらステータスを更新すること。-->
# UTM-30LX の USB 接続が不安定 → 現在完全に不通の問題

- **ステータス: 調査中** (ハード点検待ち — 電源 LED / 12 V 供給 / USB コネクタの確認が次アクション)
- 日付: 2026-08-11
- 環境: HOKUYO UTM-30LX (native USB, Hokuyo VID 15d1 → cdc_acm `/dev/ttyACM*`)、ハブ 1-2 経由
- 関連ファイル:
  - `/etc/udev/rules.d/99-hokuyo-devices.rules` — `15d1:0000 → /dev/ttyUSB-utm-30lx` の symlink は**整備済み** (挿せば自動で生える。CLAUDE.md/launch にあった「手動 ln -sf が必要」は古い記述で 08-11 に訂正済み)
  - `ros2_ws_main/src/bringup/rerobot_bringup/launch/rerobot_bringup.launch.py` (`serial_port` 引数)

## 症状

urg_node が接続不可。調査の結果、**デバイスが USB バス上に存在しない** (lsusb に Hokuyo 15d1 なし、
symlink も当然生えない)。ソフトウェアでは復旧不可能な状態。

## kernel log による履歴 (2026-08-11) — 人手によらない自然断が 3 回

| 時刻 | イベント |
|---|---|
| 13:37:05 | 接続 (devnum 10, ttyACM1) |
| 14:50:21 | **切断 (1 回目)** — 約 2 時間不在 |
| 16:43:06 | 再接続 → **3 秒で切断 → 再接続** (二重列挙 = 接触不安定の兆候) |
| 16:53:07 | **切断 (3 回目)** — 急回転事故 (16:50) の直後。以降バス上に不在 |

ユーザ確認: ハブ・ケーブルには誰も触れていない → 抜け落ちではなく**自然断**。
急回転の振動 or 電源系の瞬断がとどめの可能性。

## 疑い (優先順)

1. **UTM の電源断** — UTM-30LX は外部 12 V 給電。EPOS 事故 (同 issue 参照) の電流スパイクで
   ヒューズ/供給ラインが落ちた可能性。→ **本体 LED を見る**のが最速の切り分け
2. USB コネクタ/ケーブルの接触不良 — 14:50 と 16:43 の挙動はこれと整合。振動で悪化
3. UTM 本体の USB インタフェース故障 (最悪ケース)

## 切り分け手順

1. UTM 本体の電源 LED 確認 → 消灯なら 12 V ライン/ヒューズを追う
2. 点灯していれば USB を挿し直し → `ls -l /dev/ttyUSB-utm-30lx` で symlink が生えるか
3. 生えれば `ros2 launch rerobot_bringup rerobot_bringup_2d.launch.py` で /scan 確認
4. 再発するようなら USB ケーブル交換・ハブを介さず PC 直挿しで様子見
