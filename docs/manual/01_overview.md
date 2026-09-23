<!-- claude: 運用手引き 第1章 テンプレ (2026-09-22) -->

# 第1章 全体像

![reRoBot 配線図 (harness v2)](img/harness_v2.png)

## 機体の最高速メモ (2026-09-24 時点)

| 区分 | 上限 | 決めている場所 | 備考 |
|---|---|---|---|
| **ハード限界 (直進)** | **1.12 m/s (4.0 km/h)** | モータ Max speed 7720 rpm ÷ 減速比 92.25 × タイヤ周長 π×0.256 m | 物理上限。これ以上は指令しても出ない |
| **ハード限界 (旋回)** | **4.2 rad/s** (その場旋回) | 上記 × 2 ÷ トレッド 0.529 m | 左右逆転フル速のとき。実用ではもっと低い |
| EPOS4 Max profile velocity | 7720 rpm = 1.12 m/s | EPOS4 内部設定 (0x607F)、EPOS Studio で書込 | ハード限界と同値なので、実質「上限なし」の意味。詳細 [docs/reference/2026-07-27_epos4_studio_startup_settings.md](../reference/2026-07-27_epos4_studio_startup_settings.md) |
| controller | 速度上限なし (加減速のみ制限) | `params.yaml` `max_motor_accel/decel_rpm_per_s: 15000` = 車体 2.2 m/s² | 速度の頭打ちは持たず、Twist をそのまま rpm へ変換する。1.12 m/s から停止まで約 0.5 s |
| キーボード teleop | 1.0 m/s / 2.0 rad/s | `epos4_teleop/config/params.yaml` `max_linear` / `max_angular` | `+/-` で刻み幅を変えても上限はここで止まる |
| ゲームパッド (通常 / turbo) | 0.5 / 1.0 m/s、0.3 / 1.0 rad/s | `joy_teleop.yaml` `scale_linear[_turbo].x` / `scale_angular[_turbo].yaw` | スティック最大時の値。RB 押下で turbo |
| Nav2 (自律走行) | 0.4 m/s、方位合わせ旋回 1.0 rad/s | `nav2_params.yaml` `desired_linear_vel` / `rotate_to_heading_angular_vel` | RPP controller。曲率に応じてさらに減速する設定 (`use_regulated_linear_velocity_scaling`) |

要点: **ソフト側でいちばん速いのは joy turbo とキーボードの 1.0 m/s** で、ハード限界 1.12 m/s の 9 割。Nav2 は 0.4 m/s と控えめ。
速度を上げたいときは各行の「決めている場所」を触るが、controller には頭打ちが無いので、**teleop / Nav2 側の値が 1.12 m/s を超えても機体はそれ以上出ない** (EPOS4 で飽和する)。

---

← [目次](00_index.md) | → [第2章 初回セットアップ](02_setup.md)
