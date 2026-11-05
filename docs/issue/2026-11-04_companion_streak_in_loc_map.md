<!-- claude: docs/issue — 未解決問題の調査記録。解決したらステータスを更新すること。-->
# 自己位置推定用 2D 地図に「随伴者の筋」が焼き付いている (10-09 地図 bag)

<!-- claude: 2026-11-04 新設。発見の経緯: 「GLIM 地図の壁の帯は GLIM の平面拘束が弱いせいか」の調査 (docs/trial/analysis/2026-11-03_slamtoolbox_direct_2dmap.md §10) の critic 査読で指摘され、companion_check.py で確認 -->

**ステータス (2026-11-04): 筋の存在は確定 (両 bag のセンサ座標で直接観測)、AMCL への影響 (横バイアス・台地の形) は仮説で未検証。critic 査読済み。対策 = 地図生成時にセンサ座標の除外箱を足して再生成 → 同一 bag A/B (カード提案済み)。**

## 1. 現象

- 地図作成 bag `bags/raw/2026-10-09_1643_5goukan_chal` の `/rfans_driver/rfans_points` をセンサ座標のまま数えると、**95 % のスキャンで LiDAR の真後ろ (方位 171°、p10〜p90 159〜178°) 距離 1.17 m (0.96〜1.43 m)、高さ 0.54〜1.08 m (base_link) に約 1400 点の塊**がある (`bags/exp/2026-11-03_slamtoolbox_2d/companion_check.py` / `companion_check.png`)。z の上限 1.1 m は R-Fans の +15° リングが 1.2 m 先で届く高さで、物体の背丈ではない。URDF の車体後端は −0.51 m なので自車体ではなく、歩いて付いてきた人 (随伴者) か牽引物。bag 全長で固定。
- 現行の自己位置推定用地図 `maps/2d/glim/2026-10-09_1643_5goukan_chal_fixed2_filtered/raw_traj/0.3-100.0_r180/map.pgm` (`glim_traj_to_2dmap`、`--map_only`、`range_min 0.5`) にはこの塊が**軌跡に沿った幅約 0.4 m の連続した黒い帯**として残っている (`companion_streak_mapcrop.png`: T=199/399/1200 の切り出しで、自由空間の真ん中を赤い軌跡と重なって走る)。
- Nav2 用の本体地図 `nav2/map.pgm` は `map_to_keepout --path_clear_radius 0.4` で軌跡沿いを掃除しているため筋は消えている (そもそもこの掃除が要った理由がこれだった可能性)。keepout も同様。**筋が残っているのは AMCL に渡す raw_traj 系の地図だけ**。
- 評価 bag `2026-11-01_2114` でも `/rfans/scan` の後方 0.6〜2.0 m に 80 % のスキャンでビームがある (中央値 76 本、最小距離の中央値 1.49 m、1.2〜1.8 m で揺れる = パッド操作者が後ろを歩いている)。

## 2. これまでの解析に与えていた影響

- `wall_band_survey.py` の 26 窓のうち 6 窓 (T=149/199/250/299/399/1250、点の 70〜99 %) は壁ではなくこの筋だった。窓の中心を「11-01 bag の range > 1 m の最寄り終点で地図が占有の点」で選んでいたため、11-01 の随伴者 (1.2 m 後方) が最寄り終点になり、同じ車線を走った 10-09 の筋に当たる、という選択効果。
- `compare.py` / `plateau_fig.py` の拡大図の中心 (T=430 など) も同じ選び方なので、随伴者の位置に引かれている。台地幅の数値 (±1 m × ±10° の尤度) はスキャン全体で計算しているので中心の選び方には依存しないが、**地図側に筋が入っている状態の値**である。

## 3. AMCL への影響 (仮説、未検証)

11-01 の live scan に随伴者の点 (後方 1.2〜1.8 m、数十本) があり、地図に幅 0.4 m の筋が車線沿いにあるので、likelihood_field は「随伴者を筋に合わせる」方向に働きうる。効くとすれば **車線方向の横位置を地図作成時の車線に固定するバイアス** で、10-12 以来未解明の「2D 床 0.5 m = 共通横バイアス」(`docs/issue/2026-10-11_loc_eval_2d_corner_loss_vs_3d.md`)、植え込み通路の退化、台地の形に関係しうる。scan_all 2000 本のうち数十本なので効き目は限定的かもしれない — A/B で測るまで断定しない。

## 4. 対策候補

1. **地図生成時にセンサ座標の除外箱を足す** (`glim_traj_to_2dmap` に `--exclude_box x_min x_max y_min y_max` 等。今回は x_s −2.0〜−0.6 m、|y_s| ≤ 0.6 m、全 z)。現行と同条件で再生成し、`compare.py` で台地幅、`replay_amcl.sh` で横ずれを同一 bag A/B。
2. 既存地図への後処理: 軌跡から半径 0.4〜0.6 m の占有を消す (`map_to_keepout --path_clear_radius` 相当を raw_traj 地図にも適用)。軌跡沿いの本物の構造 (狭い通路の壁) も消すので 1 より粗い。
3. 今後の地図作成走行では随伴者を 3 m 以上離す、または後方 ±30° を地図作成から除外する運用。
4. live 側: AMCL に渡す scan から後方 0.6〜2.0 m を落とすフィルタ (随伴者がいる運用のとき)。

## 5. 再現

```bash
# main コンテナ内
python3 /workspace/bags/exp/2026-11-03_slamtoolbox_2d/companion_check.py          # 両 bag の後方点数 (約 4 分)
python3 /workspace/bags/exp/2026-11-03_slamtoolbox_2d/companion_streak_mapcrop.py  # critic 作成、地図切り出し + 10-09 軌跡
```
