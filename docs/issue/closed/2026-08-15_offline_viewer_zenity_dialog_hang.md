<!-- claude: 調査記録。2026-08-15 セッション。offline_viewer のファイル選択不能・open map 0% 停止の原因特定と修正。 -->
# offline_viewer ファイル選択不能 / open map 0% 停止 — 不可視 zenity ダイアログ待ち

- **ステータス: 解決済み (原因実証 + 修正適用)。修正 = docker-compose glim サービスに `GSK_RENDERER=cairo` 追加**
- 実施: 2026-08-15 (glim_env コンテナ、GLIM 2026-08-12 版 `184e677` / glim_ros2 `4d4ec52`、zenity 4.0.1)
- 関連: `docs/text/glim/06_case_studies.md` 事例C-1 (今回の調査で機構を精密化)

## 症状

1. `ros2 run glim_ros offline_viewer` を GUI で使うと、File → Open New Map を押しても
   フォルダ選択ダイアログが出ない (viewer ごと固まって見える)。
2. コマンドラインでダンプパスを指定 (`offline_viewer /bags/<name>_dump`) すると、
   「Load map」進捗モーダルが 0% のまま永遠に進まない。

## 原因 (実証済み)

2 症状は同根。GLIM のダイアログ (portable-file-dialogs = pfd) は **zenity を子プロセスと
して起動し回答を待つ**が、この環境では zenity のウィンドウが**マップ・フォーカスされて
いるのに中身が 1 ピクセルも描画されず完全に不可視**になる。誰も答えられない → pfd が
永遠にブロック:

- 症状 2: `load_map()` (進捗モーダルの背景スレッド) は dump を読み始める**前に**
  `pfd::message("Do optimization?")` で回答待ちする (glim `src/glim/viewer/offline_viewer.cpp:209`)。
  → 0% は「重い処理」ではなく「見えない質問に誰も答えていない」だけ。
- 症状 1: `Open New Map` → `pfd::select_folder` も同じ zenity。こちらは viewer の
  **UI スレッド自体が `.result()` でブロック**するため viewer ごとフリーズして見える。

### 不可視になる機構

zenity 4.x は GTK4 製で、既定の GSK レンダラが **OpenGL 描画**を要求する。この PC の
GUI は CRD (Chrome Remote Desktop) の仮想 X サーバで **DRI3 が無く**、zenity 起動時に

```
libEGL warning: DRI3 error: Could not get DRI3 device
MESA: error: Failed to attach to x11 shm
```

を出して GL サーフェスへの描画が失敗する (stderr は pfd が `2>/dev/null` に捨てるため
通常は見えない)。ウィンドウ管理上は正常 (`_NET_WM_STATE_MODAL, _NET_WM_STATE_FOCUSED`、
スタッキング最上位) なのに、ピクセルが描かれない。

### 実証手順と結果 (2026-08-15)

1. `offline_viewer <dump>` で 0% 停止を再現 → コンテナ内 `ps -ef` で
   `zenity --question ... 'Do optimization?'` が入力待ちで生存。
2. `xprop -root _NET_CLIENT_LIST_STACKING` → zenity が**最前面かつフォーカス済み**。
3. `xwd -id <zenityウィンドウ>` でウィンドウ内容を直接キャプチャ → **背後の画面が写る
   だけ = 何も描画されていない**ことを確認 (不可視の直接証拠)。
4. zenity を kill → pfd がキャンセル扱いで返り、**その場で 0% モーダルが消え地図の
   読み込み・描画が進行** (0% の原因が回答待ちのみであることの実証)。
5. `GSK_RENDERER=cairo zenity --question ...` で再実行 → タイトル・本文・No/Yes ボタン
   まで**完全に描画される**ことをキャプチャで確認。

### 切り分けで否定した仮説

| 仮説 | 結果 |
|---|---|
| zenity 未インストール (upstream [koide3/glim#36](https://github.com/koide3/glim/issues/36) と同じ) | 否定 — zenity 4.0.1 導入済み・プロセスも起動している |
| ダイアログが巨大 viewer (2560x1440 > 実画面 1920x1006) の背後に隠れる | 否定 — スタッキング最上位・フォーカス済み。viewer は WM (mutter) が画面内に自動最大化しており、ウィンドウサイズ縮小も効果なし |
| dump 内 config の `librviz_viewer.so` が ROS 未初期化でハング (08-12 に疑った説) | 現行版では否定 — GLIM 2026-08-12 版は `*viewer*`/`*monitor*` を名前フィルタでスキップする |
| dump 破損 (values.bin 欠け等) | 否定 — 全 dump に values.bin あり (事例C-2 とは別問題) |

## 修正 (2026-08-15 適用)

`docker-compose.yml` の glim サービス environment に **`GSK_RENDERER=cairo`** を追加。
GTK4 の描画を GL からソフトウェア (cairo) に固定し、DRI3 の無い CRD X サーバでも
ダイアログが描画されるようにする。反映はコンテナ再作成:

```bash
docker compose --profile glim up -d glim
```

以後は素の `ros2 run glim_ros offline_viewer [<dump>]` で「Do optimization?」もフォルダ
選択も見える。

## 修正なしでの回避 (旧環境・参考)

- 非対話の PLY 書き出し: `--export_path <out.ply>` — ダイアログ自体をスキップ
  (`export_map_path` が非空だと `enable_optimization=false` になり質問が出ない)。
- 0% 停止中にどうしても進めたい場合: コンテナ内で `pkill -x zenity` → キャンセル扱いで
  読み込みだけは進む (最適化は無効になる)。Alt+Tab は**効かない** (見えないのは
  重なりではなく描画失敗のため)。

## 教訓

- 「進捗 0% で停止」= 重い処理とは限らない。**子プロセスの入力待ち**を `ps -ef` の
  プロセスツリーで先に確認する。
- 「ダイアログが出ない」には 2 段階ある: (a) プロセスが起動していない (zenity 欠如 =
  upstream #36)、(b) **起動して最前面にいるのに描画されない** (GTK4 GL × リモート X)。
  切り分けは `ps` → `xprop` (スタッキング) → `xwd -id` (実描画) の順が最短。
- pfd 系ツールは zenity の stderr を握りつぶす。「無反応でエラーも出ない」時ほど
  子プロセスを直接単体実行して stderr を見る。
