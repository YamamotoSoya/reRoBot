<!-- claude: 2026-10-17 ユーザ依頼でメモ章として作成 (waypoint_editor の起動と操作) -->

# メモ 2026-10-17 — waypoint を打つビューア (waypoint_editor)

2D 地図の上で waypoint を机上で打つ GUI: `tools/waypoint_editor/waypoint_editor.py` (matplotlib 製)。
詳細は [tools/README.md](../../tools/README.md) の waypoint_editor 節。

---

## 1. 立ち上げ方

```
ホスト
 └─ xhost +local:docker          ← GUI をコンテナから出すため (済みなら不要)
     └─ rerobot_env (main コンテナ) 内のシェルで
         └─ python3 /workspace/tools/waypoint_editor/waypoint_editor.py <地図yaml> <保存先yaml> [オプション]
```

```bash
# python3 /workspace/tools/waypoint_editor/waypoint_editor.py \
    /workspace/maps/2d/glim/2026-10-03_1126_tsukuba_composite/nav2/map.yaml \
    /workspace/maps/2d/glim/2026-10-03_1126_tsukuba_composite/waypoints/course.yaml 
```

* **保存先は `<map>/waypoints/course.yaml`** にする — `auto_run.launch.py` はこのパスを決め打ちで読む (無いと waypoint_runner が起動しない)。
* 保存先が既にあれば読み込んで続きから編集できる。
* keepout は `<map>/keep_out/keep_out.yaml` を自動で読む。
* matplotlib (TkAgg) はコンテナにしか無い → ホストでは動かない。
* course.yaml がある地図 (2026-10-17 時点): `2026-09-180915_5goukan_dumpbase` / `2026-10-03_1126_tsukuba_dumpbase` / `2026-10-09_1643_5goukan_chal`

| オプション | 効果 |
|---|---|
| `--traj <dump>/traj_lidar.txt` | 走行軌跡を重ねる (点を置く場所の目安) |
| `--radius 0.7` | 通過判定半径の円 (既定 0.7 m、Nav2 の RemovePassedGoals に合わせる) |
| `--keepout <yaml>` / `--no_keepout` | keepout を指定 / 使わない |
| `--check` | GUI なしで各点の判定表だけ (要確認の点があると exit 1) |
| `--render out.png` | GUI なしで描画を PNG に保存 |

## 2. 操作方法

```
マウス
 ├─ 左クリック (空き)         … 末尾に追加
 ├─ 左ドラッグ (点の上)       … 移動
 ├─ Shift + 左クリック        … 最寄りの区間に挿入
 ├─ 右クリック (点の上)       … 削除
 └─ Ctrl + ドラッグ (点の上)  … 向きを手動指定 (紫の矢印)
キー (点の上で)
 ├─ a … 向きを自動 (次の点の方向) に戻す
 └─ p … 停止点 ON/OFF (赤い四角)
キー (全体)
 ├─ n … 番号振りモード: クリックした順に 0,1,2…
 │      (空きクリック = 新しい点 / 右クリック = 番号を外す / Enter か n = 確定 / Esc = 取消)
 ├─ r … 順序を反転      ├─ o … 通過半径の円の表示切替
 ├─ u … 元に戻す        ├─ s … 保存
 └─ h … ヘルプ表示      └─ q … 終了 (未保存なら警告)
ズーム/パン … ツールバーの虫眼鏡・十字 (そのモード中はクリック編集が効かない)
```

**点の色**

| 色 | 意味 |
|---|---|
| 緑 | 空き |
| 橙 | 未観測 |
| 赤 | 壁 or keepout の上 → 経路計算が失敗して走行全体が止まる。必ずずらす |
| 金の星 | ティーチング点 (`teach_from_traj` 由来)。動かせない、右クリック削除のみ可 |

**注意**

* 途中の点の向きは経路計算に使われない。向きが効くのは**最後の点と停止点だけ**。
* waypoint は地図の座標に紐づく → 地図を作り直したら打ち直し。
* 停止点 (`stop: true`) は waypoint_runner だけが読む。RViz の Nav2 パネルで Start すると素通りする。

## 3. 作った後の流れ

1. (任意) 保存後に `--check` で赤い点が無いか確認。
2. 本番: `ros2 launch rerobot_bringup auto_run.launch.py map_dir:=/workspace/maps/2d/glim/<map>` → 30 s 後に waypoint_runner が course.yaml を読む。
3. パッドの **Start** = 区間開始・再開、**Back** = 一時停止。
