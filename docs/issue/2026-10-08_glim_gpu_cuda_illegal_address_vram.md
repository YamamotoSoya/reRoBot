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

## 6.5 同一 bag での CPU/GPU 比較 (2026-10-08、実測)

同じ bag の CPU dump が **10-03 に既に存在していた** (`bags/glim/2026-10-03_1126_tsukuba_dump/default`、CPU 既定設定、サブマップ 816 個)。GPU dump と同じ指標で測った結果:

| 指標 | CPU 既定 (`default`) | GPU (`gpu`) |
|---|---|---|
| matching cost 因子 (ユニーク対) | 1186 | 14729 |
| うち **長距離 (\|i−j\| > 100)** | **0 対** | **1215 対** |
| 再訪 \|Δz\| 中央値 (traj、xy<1 m・Δt>300 s) | **8.92 m** (n=371) | **0.09 m** (n=146) |
| 再訪 \|Δz\| 中央値 (odom 段) | 13.16 m (n=379) | 1.10 m (n=8) |
| traj の z 範囲 | 0.1 〜 **79.9 m** | −0.8 〜 **35.4 m** |
| odom 段の z 範囲 | 0.1 〜 70.1 m | −5.7 〜 30.7 m |

**CPU 既定では長距離の閉合が 1 対も立っていない** (因子 1186 対はすべて近接 submap 間)。§5 H-A の予測どおりで、09-20 の CPU 実測「周回を結ぶ因子 0 本」とも整合。ユーザの目視 (「行きと帰りが同じ z 平面」) は定量でも再現し、差は目視以上 (8.92 m → 0.09 m)。

⚠️ ただし **odom 段の z 範囲自体が CPU 70 m / GPU 30.7 m と 2 倍以上違う**。閉合の有無だけでなく**前段の推定精度も違う**ので、「閉合が立ったから揃った」と「前段が良いから揃った」が分離できていない。

### CPU 版に GPU と同じパラメータを入れる実験 → **ホスト RAM 不足で失敗**

`config_global_mapping_cpu.json` に `submap_voxelmap_levels: 2` + `submap_voxel_resolution_max: 1.0` (dmin 5 / dmax 20) + `create_between_factors: false`、`config_sub_mapping_cpu.json` に `submap_downsample_resolution: 0.1` + `submap_target_num_points: 50000` を入れて同 bag を実行:

```
[mem] [warning] CPU memory usage: 31987.59 / 32070.71 MB 99.74%
Out of memory: Killed process 1217234 (glim_rosbag) anon-rss:26684480kB
[ros2run]: Killed
```

bag の **68% 地点 (3499/5149 s)** で OOM kill。**dump は保存されず全損** (§8 と同じ事情)。RSS 26.7 GB に対しホスト 32 GB。

**判明したこと**: GPU 版の密度設定は、CPU 版では**主記憶に載りきらない**。GPU 版が 8 GB の VRAM で足りたのは `randomsampling_rate` を 0.2 に下げた後だから。したがって **「GPU は不要で設定の問題」とは言えない** — 密度を上げることと、それを保持できることの組み合わせが要る。

**未実施 (次の実験)**: 密度は CPU 既定のまま、**閉合判定のボクセルだけ**粗くする (`submap_voxelmap_levels` と `submap_voxel_resolution_max` の 2 つだけ)。判定は `voxelmaps.back()` の重なりで決まるので、これだけなら記憶消費はほとんど増えない。→ §7.5

## 7. 残っている確認事項

1. **確保失敗の警告の有無** — 修正前の構成で stderr をファイルに保存して再現し、最初の `cudaErrorIllegalAddress` の直前に `warning: cudaErrorMemoryAllocation` があるか見る。**あれば §3 の経路が確定し「原因 = GPU 係数のメモリ消費」が確定に昇格**。無ければ §5 (b) が本命に戻る。35 分 × 1 本。
2. **sm_61 での再ビルド** — `-DCMAKE_CUDA_ARCHITECTURES=61` で gtsam_points を建て直し、JIT キャッシュ (`/root/.nv/ComputeCache`) を消して同じ bag を流す。§5 (c) の確認。イメージ再ビルドのコストが高く確率は低いので後回し。
3. **GPU プリセット全体の見直し** — 今回下げたのは `randomsampling_rate` だけ。`submap_downsample_resolution` (0.1 vs CPU 0.3) も残っており、**CPU 版との結果比較をするなら条件を揃える作業が別途必要**。

## 7.5 別 PC 向け実験手順 (エージェントへの依頼仕様)

**この節だけ読めば実行できるように書いてある。** このリポジトリの他の節を読む必要はない。

### 目的

同一 bag で「**行きと帰りの z が揃うのは閉合判定のボクセル解像度が原因か**」を判定する。現状、CPU 既定では長距離閉合 0 対・再訪 \|Δz\| 8.92 m、GPU では 1215 対・0.09 m (§6.5)。ただし CPU/GPU は 10 以上の設定が同時に違い、前段の推定精度も違うため、何が効いたか分離できていない。

### 必要な環境

| 項目 | 要件 |
|---|---|
| ホスト RAM | **64 GB 以上を推奨** (32 GB では E2 が OOM kill される実績あり。E1/E3 は 32 GB で足りる見込みだが未確認) |
| ディスク | 100 GB 以上 (bag 48 GB + dump 2 GB × 本数) |
| GPU | **不要** (全 run が CPU 構成)。GPU 機なら E4 も可 |
| Docker | リポジトリ同梱の `glim_env` (profile: `glim`) |

### 準備

```bash
git clone --recursive https://github.com/YamamotoSoya/reRoBot.git && cd reRoBot
docker compose up -d glim          # イメージが無ければ docker compose build glim (約 5 GB)

# bag を Drive から取得 (rclone remote: cit-share-bags。無ければリポジトリ管理者に依頼)
mkdir -p bags/raw && cd bags/raw
rclone copy cit-share-bags:bags/raw/2026-10-03_1126_tsukuba.tar.zst .   # 26.5 GiB
tar -I zstd -xf 2026-10-03_1126_tsukuba.tar.zst && rm 2026-10-03_1126_tsukuba.tar.zst
cd ../..
# 展開後: bags/raw/2026-10-03_1126_tsukuba/{metadata.yaml, *_0.mcap}  (mcap 48 GB、収録 5149 s)
```

### 変更する設定 (ベースは CPU 既定。`ros2_ws_glim/config/`)

**どの run も `config.json` は既定のまま** (= `config_odometry_cpu.json` / `config_sub_mapping_cpu.json` / `config_global_mapping_cpu.json` を読む)。

| run | `config_global_mapping_cpu.json` | `config_sub_mapping_cpu.json` | 狙い |
|---|---|---|---|
| **E1** (本命) | `submap_voxelmap_levels: 1→2`、`submap_voxel_resolution_max: 1.0` / `_dmin: 5.0` / `_dmax: 20.0` を追加 | 変更なし | 閉合判定のものさしだけ粗くする。**これで閉合が立てば原因確定** |
| **E2** (再挑戦) | E1 と同じ + `create_between_factors: true→false` | `submap_downsample_resolution: 0.3→0.1`、`submap_target_num_points: 50000` を追加 | GPU プリセット完全模倣。32 GB では 68% で OOM。64 GB での完走可否を見る |
| **E3** (対照) | 変更なし (既定) | 変更なし | 同一環境でのベースライン再取得。既存 `default` dump と一致するか確認 |
| **E4** (任意、GPU 機のみ) | `create_between_factors: false→true` (GPU 側 `config_global_mapping_gpu.json` を編集し `config.json` を GPU 構成に) | 変更なし | between フラグ単独の寄与を見る (§5 H-D の判別) |

⚠️ **1 run につき表の 1 行だけを適用すること**。複数を混ぜると分離できない (それが今の行き詰まりの原因)。

### 実行

```bash
B=2026-10-03_1126_tsukuba
TAG=e1_coarse_voxel          # run ごとに変える: e1_coarse_voxel / e2_gpu_mimic / e3_baseline / e4_between_on

docker exec -it glim_env /ros_entrypoint.sh \
  ros2 run glim_ros glim_rosbag /workspace/bags/raw/$B \
  --ros-args -p config_path:=/glim_config \
  -p auto_quit:=true \
  -p dump_path:=/workspace/bags/glim/${B}_dump/${TAG} \
  2>&1 | tee ~/glim_${TAG}.log
```

- **所要**: 1 run あたり 30〜60 分 (bag 86 分ぶん)。
- **開始の判定**: `opening /workspace/bags/raw/...` と `detected storage_id=mcap` の 2 行が出ること。出なければパスが違う (bag 名を 2 回書く間違いが頻発)。
- **途中で出力先が空なのは正常**。dump は終了時に一括で書かれる。
- **RAM 監視を別端末で**: `while true; do free -g | sed -n 2p; sleep 30; done | tee ~/mem_${TAG}.log`
- OOM kill されたら `[ros2run]: Killed` で終わり **dump は残らない**。その場合は到達した bag 時刻 (`tee` したログ末尾の `stamp=`) と RSS を記録して報告する。

### 評価 (目視を使わないこと)

dump ができたら、次のスクリプトを `glim_env` 内で実行して 3 指標を出す。

```python
# docker cp して docker exec glim_env python3 /tmp/cmp.py
import numpy as np, re
TAGS = ["default", "e1_coarse_voxel"]          # 比較したい dump のタグを列挙
B = "2026-10-03_1126_tsukuba"
for tag in TAGS:
    b = f"/workspace/bags/glim/{B}_dump/{tag}/"
    out = [tag]
    for name in ("odom_lidar.txt", "traj_lidar.txt"):
        a = np.loadtxt(b + name); t, xyz = a[:, 0], a[:, 1:4]
        s = max(1, len(t) // 3000); t2, p2 = t[::s], xyz[::s]
        d = []
        for i in range(len(t2)):
            m = (np.abs(t2 - t2[i]) > 300) & (np.linalg.norm(p2[:, :2] - p2[i, :2], axis=1) < 1.0)
            if m.any(): d.append(np.abs(p2[m, 2] - p2[i, 2]).min())
        med = np.median(d) if d else float("nan")
        out.append(f"{name.split('_')[0]}: z {xyz[:,2].min():.1f}..{xyz[:,2].max():.1f} n={len(d)} med|dz|={med:.2f}")
    pairs = set()
    for line in open(b + "graph.txt", encoding="utf-8", errors="ignore"):
        m = re.search(r"vgicp\w*\D+(\d+)\D+(\d+)", line)
        if m:
            i, j = int(m.group(1)), int(m.group(2)); pairs.add((min(i, j), max(i, j)))
    out.append(f"因子 {len(pairs)} 対 / |i-j|>100 は {sum(1 for i, j in pairs if j - i > 100)} 対")
    print(" | ".join(out))
```

**報告してほしい値** (run ごと):

| 指標 | 取り方 |
|---|---|
| 長距離閉合の対数 (\|i−j\| > 100) | 上のスクリプト |
| 再訪 \|Δz\| 中央値 (traj / odom 段) | 上のスクリプト |
| z 範囲 (traj / odom 段) | 上のスクリプト |
| ピーク RAM | `mem_*.log` の最大値 |
| 所要時間 | 実行の開始〜終了 |
| 完走したか / 落ちた位置 | ログ末尾 |

### 判定基準 (事前に決めておく)

- **E1 で長距離閉合が 100 対以上立ち、traj 再訪 \|Δz\| 中央値が 1 m 未満**になれば → **「閉合判定のボクセル解像度が原因」が確定**。GPU は不要で、CPU 既定に 2 行足すだけで同じ効果が得られることになる (**この場合 CPU 既定の設定を見直す価値が大きい**)。
- **E1 で閉合が 0〜数十対にとどまる**なら → ボクセル解像度だけでは足りない。GPU 枝が voxelmap に全点を入れる実装差 (`global_mapping.cpp:244-270`) か、サブマップ密度が効いている。E2 の結果と突き合わせる。
- **E3 が既存 `default` と一致しない**なら → 環境差があるので E1/E2 の解釈を保留する。

### 成果物の戻し方

```bash
# dump を Drive へ (1 run 約 2 GB)
rclone copy bags/glim/2026-10-03_1126_tsukuba_dump/<TAG> \
  cit-share-bags:bags/glim/2026-10-03_1126_tsukuba_dump/<TAG> --progress
```

ログ (`~/glim_<TAG>.log`、`~/mem_<TAG>.log`) と上の指標表をあわせて報告すること。**設定変更は差分が分かる形で残す** (変更した json を dump 内の `config/` と一緒に上げれば自動で残る)。

### やらないこと

- **実機・CAN・モータには一切触れない**。bag 再生のみ。
- `config.json` の既定 (CPU 構成) を恒久的に変えない。実験後は元に戻す。
- 複数 run を同時に走らせない (RAM と I/O を食い合う)。

## 8. 併せて記録しておく別件

- **この版の GLIM には GPU メモリの上限制御もオフロードも無い** (`grep offload` で 0 件)。逼迫しても安全に縮退せず、**プロセスごと落ちて途中経過が全損する**。長尺 bag では消費を抑えた設定で臨むこと。
- **dump は終了時にしか書かれない**。処理中に出力先が空なのは正常で、失敗の証拠ではない。逆に、サブマップが 1 つもできる前に止めると保存は成功するのに中身が空になる (実際に 2026-10-07 に 2 回発生)。
- **bag のパスを間違えても GLIM はエラーを出さずに起動する**。bag を 1 つも開かないまま待機状態になる。開始の判定は `[glim] [info] opening <path>` と `detected storage_id=mcap` の 2 行が出るかで行う。
- **`/tmp/glim_submap.log` に `imu_rate stamp does not cover the scan duration range!!` が多発** (imu_rate_end が scan_end より約 0.018 s 手前)。クラッシュの 12 分 40 秒前で止まっており別モジュールなので**本件との関与は傍証以下**。収録側で IMU がスキャン末尾に届かない区間がある、という別の話として扱う。
- **この bag の IMU 予測品質が悪い**。`num_validations=48064` で `IMU better ratios trans=0.23 / vel=0.36` は `2026-09-24_bno086_auto_tare_rotates_motion_outputs.md` の基準 (0.34 / 0.73) より明確に悪い。**「落ちなくなった = 良い地図が出た」ではない**ので、できた dump の品質は別途評価が要る。
