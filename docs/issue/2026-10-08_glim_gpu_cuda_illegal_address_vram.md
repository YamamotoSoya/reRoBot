<!-- claude: 2026-10-08 作成 -->
# GLIM GPU 版が長尺 bag の終盤で cudaErrorIllegalAddress 落ち — GPU プリセットの点数が重い (回避済み・原因は本命段階)

**ステータス: 回避策で解決 / 原因は本命段階 (確定ではない)** — `config_global_mapping_gpu.json` の `randomsampling_rate` を上流既定 1.0 から CPU 版と同じ 0.2 に下げると、同じ bag が完走した (GPU メモリのピーク 8064 MB 上限到達 → 4728 MB)。「GPU 係数のメモリ消費が 8 GB を超えた」が本命だが、**前回実行の stderr が残っておらず `cudaErrorMemoryAllocation` 警告の有無が未確認**のため確定とは書かない。critic 査読済み (§5 に棄却できなかった代替仮説)。

- 症状: `glim_rosbag` (GPU 版) が 86 分の bag の **97% 地点**で `cudaErrorIllegalAddress` を連発し `[ros2run]: Aborted`。**dump は 1 バイトも保存されない** (保存処理は終了時にしか走らないため、途中経過は全損)
- 対象 bag: `bags/raw/2026-10-03_1126_tsukuba` (mcap 48 GB、5149 s = 86 分、`/rfans_driver/rfans_points` 51378 件、`/imu/data` 1029105 件)
- 環境: `glim_gpu_env` (`koide3/glim_ros2:jazzy_cuda12.5` ベース、2026-10-07 構築)、GTX 1080 (VRAM 8 GB、compute capability 6.1)、driver 580.173.02、ホストは QEMU ゲスト + GPU パススルー
- 関連: GPU 構成の導入は PROJECT_STATE 10-07 の行、手順は `docs/manual/07_glim.md` §GPU mode

## 1. 何が起きたか (観測)

ユーザ端末:

```
warning: cudaErrorIllegalAddress
       : an illegal memory access was encountered     ← 連発
terminate called after throwing an instance of 'thrust::...::system_error'
  what():  parallel_for failed: cudaErrorIllegalAddress
[ros2run]: Aborted
```

コンテナ内 `/tmp/glim_global.log`:

```
[2026-10-07 17:15:26] [global] [error] an exception was caught during global map optimization!!
[2026-10-07 17:15:26] [global] [error] parallel_for failed: cudaErrorIllegalAddress
```

ホストのカーネルログ (`journalctl -k`。2026-09-01 以降この 1 件のみ):

```
NVRM: Xid (PCI:0000:01:00): 31, pid=1191280, name=glim_rosbag,
  MMU Fault: ENGINE GRAPHICS GPC0 ... faulted @ 0x0_00003000
  Fault is of type FAULT_PDE ACCESS_TYPE_WRITE
```

`/tmp/glim_mem.log` (GLIM 自身のメモリ監視。**80% を超えた時だけ記録する**実装):

| 時刻 (UTC) | GPU メモリ |
|---|---|
| 17:03:00〜17:14:32 | 7136/8107 MB で **12 分間 1 MB も動かず平坦** |
| 17:15:07 | 7488/8107 MB (92.4%) |
| 17:15:17 | 7584/8107 MB (93.6%) |
| 17:15:22 | 8064/8107 MB (**99.5%**) |
| 17:15:26 | クラッシュ |

**読み取れること**: 落ちたのは **global mapping の最適化スレッド** (odometry でも sub mapping でもない)。フォルトアドレス 0x3000 はほぼ先頭番地 = **ヌル近傍への書き込み**であり、「確保できなかった」そのものではない。VRAM の急増は**最後の 45 秒に集中**しており、単調な地図成長ではなく**イベント的なバースト**。クラッシュ地点 (bag の 97%) はつくば周回の復路で出発点付近に戻る = **ループ閉じ込みが集中する区間**。

## 2. なぜ「VRAM 枯渇が原因」と即断できないか (critic 指摘)

1. **`/tmp/glim_mem.log` の値は GLIM の CUDA 確保量ではない**。実装 (`glim/src/glim/viewer/memory_monitor.cpp`) は `cudaMemGetInfo` 相当でデバイス全体を読むだけで、同一プロセス内の `libstandard_viewer.so` の OpenGL バッファ・別プロセス (RViz2 等)・`cudaMallocAsync` のメモリプール予約を区別しない。
2. **80% 未満は記録されない**ので、成長曲線そのものが観測されていない。「地図の成長で枯渇」は推測。
3. **12 分間の完全な平坦**は単調成長では説明できない (プール予約の張り付き、または GL バッファ固定の方が自然)。
4. **桁が合わない**。4〜5 分の試走はサブマップ 25 個で 280 MiB (大半は CUDA コンテキスト)。線形外挿では 7 GB に届かない → 超線形な消費者が必要。
5. **OOM なら普通は `cudaErrorMemoryAllocation`**。`cudaErrorIllegalAddress` と繋ぐには中間項が要る (§3)。

## 3. OOM と不正アクセスを繋ぐ経路 (実装確認済み)

`gtsam_points/src/gtsam_points/cuda/check_error.cu`:

```cpp
void CUDACheckError::operator<<(cudaError_t error) const {
  if (error == cudaSuccess) return;
  std::cerr << "warning: " << cudaGetErrorName(error) << std::endl;  // 警告のみ。続行する
}
```

gtsam_points の `cudaMallocAsync` は**すべて**この経路。したがって**確保に失敗しても処理は止まらず、ヌルポインタのままカーネルが起動する**。Xid 31 の低位アドレス write はこの経路の指紋と整合する。

⚠️ ただし **その警告が実際に出ていたかは未確認**。spdlog のファイル群には出ず、ユーザ端末のスクロールバックにしか残らない。ここが確定を阻む唯一の欠落。

## 4. 消費源の候補 (量的に桁が合うのはこれだけ)

`glim/src/glim/mapping/global_mapping.cpp::create_matching_cost_factors()` は、新しいサブマップごとに**過去の全サブマップ**を走査し、`max_implicit_loop_distance` (100 m) 以内かつ `min_implicit_loop_overlap` (0.2) 以上のものすべてに対し、**voxelmap レベル数ぶん** `IntegratedVGICPFactorGPU` を作る。

1 係数の GPU メモリは `source_inliers` = `sizeof(int) × 点数`。GPU プリセットは `randomsampling_rate: 1.0`・`submap_target_num_points: 50000` なので **1 係数あたり約 200 KB**。CPU プリセットの 0.2 なら 1/5。1 万係数で 2 GB — **「最後の 45 秒で急増」「7 GB 水準」の両方と桁が合うのはこの経路だけ**。

### GPU プリセットは上流デフォルトのまま = CPU 版より 5 倍重い

| 設定 | CPU 版 | GPU 版 (修正前) |
|---|---|---|
| `randomsampling_rate` | 0.2 | **1.0** |
| `submap_downsample_resolution` | 0.3 | 0.1 |
| `create_between_factors` | true | false |

`config.gpu.json` 自身のコメントにも「GPU 用 3 本は上流デフォルトのまま」と明記してある (2026-10-07 の構成追加時点で既知)。

## 5. 棄却できなかった代替仮説 (critic)

- **(a) VRAM の主消費者が CUDA ではない** — `config_ros.json` の `extension_modules` に `libstandard_viewer.so` が入っており、同一プロセスの OpenGL が同じ 8 GB を奪い合う。→ **§6 の完走実行で後退**: プロセス別記録に現れた GPU 利用者は `glim_rosbag` 単独で、ピークも 4728 MiB に収まった。ただし「前回の実行で RViz2 が開いていたか」は記録がなく、完全には消えていない。
- **(b) メモリ圧と独立のコードバグ** — `gtsam_points/.../integrated_vgicp_derivatives_inliers.cu::update_inliers()` は、確保サイズに**前回 GPU が返した** `num_inliers` を使い、書き込み量は**今回再評価する述語**が決める。ずれれば範囲外書き込み。`new_num_inliers == 0` のとき `cudaMallocAsync(…, 0, …)` は **cudaSuccess でヌルを返す (警告すら出ない)**。作者自身が `constexpr bool validate = false;` と書いて検証を無効化している。→ **§6 で後退**: 消費量を 1/5 にしたら同じデータで同じ区間を通過して落ちなくなった。データ依存のバグなら量に関係なく同じ位置で落ちるはず。ただし「係数が多いほど退化した対応が増える」なら相関するので完全には消えない。
- **(c) sm_52 のみのビルド + PTX JIT** — `/root/gtsam_points/build/CMakeCache.txt` は `CMAKE_CUDA_ARCHITECTURES=native` で、**イメージビルド時に GPU が見えず 52 に解決された**と見られる (SASS は sm_52 のみ、PTX は compute_52 が 21 個)。Pascal では JIT で動いている。公式にサポートされた経路で誤コード生成の既知事例は薄いが、**ビルド設定の事故そのものは実在**し直す価値がある。
- **(d) QEMU ゲスト + パススルーの単発フォルト** — ゲストであることは確定 (`systemd-detect-virt` = qemu)。ただし Xid 31 は**アプリ側起因**を示す種類で、慢性的な MMU フォルトや Xid 79 は無い。→ **§6 で後退** (決定性があった側に傾いた)。GTX 1080 は ECC 非搭載なのでビット反転を検出する手段が無い点だけ残る。

## 6. 対処と結果 (2026-10-08)

**変更は 1 行のみ** (切り分けのため他は固定):

```
ros2_ws_glim/config/config_global_mapping_gpu.json
  "randomsampling_rate": 1.0  →  0.2   (CPU 版と同値)
```

同じ bag・同じコマンドで再実行した結果:

| | 修正前 | 修正後 |
|---|---|---|
| GPU メモリのピーク | 8064 MB (上限到達) | **4728 MB** |
| `glim_mem.log` の警告 (80% 超で記録) | 多数 | **0 行** |
| 結果 | 97% 地点で Abort | **完走** (保存まで) |
| サブマップ | 0 (dump 全損) | **643** |
| 軌跡 | 0 行 | **51366 行** |
| 最終スタンプ | — | 1790999538.99 (bag 終端 1790999539 と一致) |
| CUDA エラー | 連発 | **0 件** |

dump: `bags/glim/2026-10-03_1126_tsukuba_dump/gpu/` (1.8 GB)。所要 45 分。GPU のプロセス別記録では利用者は `glim_rosbag` 単独。

## 7. 残っている確認事項

1. **確保失敗の警告の有無** — 修正前の構成で stderr をファイルに保存して再現し、最初の `cudaErrorIllegalAddress` の直前に `warning: cudaErrorMemoryAllocation` があるか見る。**あれば §3 の経路が確定し「原因 = GPU 係数のメモリ消費」が確定に昇格**。無ければ §5 (b) が本命に戻る。35 分 × 1 本。
2. **sm_61 での再ビルド** — `-DCMAKE_CUDA_ARCHITECTURES=61` で gtsam_points を建て直し、JIT キャッシュ (`/root/.nv/ComputeCache`) を消して同じ bag を流す。§5 (c) の確認。イメージ再ビルドのコストが高く確率は低いので後回し。
3. **GPU プリセット全体の見直し** — 今回下げたのは `randomsampling_rate` だけ。`submap_downsample_resolution` (0.1 vs CPU 0.3) も残っており、**CPU 版との結果比較をするなら条件を揃える作業が別途必要**。

## 8. 併せて記録しておく別件

- **この版の GLIM には GPU メモリの上限制御もオフロードも無い** (`grep offload` で 0 件)。逼迫しても安全に縮退せず、**プロセスごと落ちて途中経過が全損する**。長尺 bag では消費を抑えた設定で臨むこと。
- **dump は終了時にしか書かれない**。処理中に出力先が空なのは正常で、失敗の証拠ではない。逆に、サブマップが 1 つもできる前に止めると保存は成功するのに中身が空になる (実際に 2026-10-07 に 2 回発生)。
- **bag のパスを間違えても GLIM はエラーを出さずに起動する**。bag を 1 つも開かないまま待機状態になる。開始の判定は `[glim] [info] opening <path>` と `detected storage_id=mcap` の 2 行が出るかで行う。
- **`/tmp/glim_submap.log` に `imu_rate stamp does not cover the scan duration range!!` が多発** (imu_rate_end が scan_end より約 0.018 s 手前)。クラッシュの 12 分 40 秒前で止まっており別モジュールなので**本件との関与は傍証以下**。収録側で IMU がスキャン末尾に届かない区間がある、という別の話として扱う。
- **この bag の IMU 予測品質が悪い**。`num_validations=48064` で `IMU better ratios trans=0.23 / vel=0.36` は `2026-09-24_bno086_auto_tare_rotates_motion_outputs.md` の基準 (0.34 / 0.73) より明確に悪い。**「落ちなくなった = 良い地図が出た」ではない**ので、できた dump の品質は別途評価が要る。
