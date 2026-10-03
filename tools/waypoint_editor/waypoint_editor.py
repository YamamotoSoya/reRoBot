#!/usr/bin/env python3
# claude: 2026-10-02 作成
# 2D 地図 (Nav2 map_server 形式の yaml + pgm/png) の上で waypoint を机上で打ち、
# RViz Nav2 パネルの「Load WPs」で読める YAML に保存する GUI ツール。
# 屋外に出る前に waypoint を用意しておき、当日は Load WPs → Start Nav Through Poses するだけにする。
#
# 出力形式 (nav2_rviz_plugins/src/nav2_panel.cpp jazzy の handleGoalLoader と同じ):
#   waypoints:
#     waypoint0:
#       pose: [x, y, z]
#       orientation: [w, x, y, z]   ← ⚠️ w が先頭 (panel の convert_to_msg がこの順で読む)
# frame は panel 側で "map" 固定。yaw は既定で「次の点の方向」を自動で入れる (最後の点は直前区間の向き)。
# claude: 2026-10-02 点ごとの手動 yaw (Ctrl + ドラッグ) を追加。手動の点には `yaw_manual: true` を付けて保存し、
#   再読込でも手動のまま残す (panel は pose / orientation しか読まないので余分なキーは無視される)。
#   ⚠️ NavigateThroughPoses (既定 BT + NavFn + RPP) で向きが効くのは**最後の点だけ** — 途中の点の
#   向きは経路計算に使われず通過するだけ。最後の点は goal checker の yaw_goal_tolerance まで旋回して止まる。
#
# 点の色: 緑 = 空き / 橙 = 未観測 (地図の灰) / 赤 = 壁 or keepout 上。赤い点は
#   ComputePathThroughPoses が失敗して NavigateThroughPoses 全体が止まる原因になるので避ける。
# 薄い円 = bt_navigator 既定 BT の RemovePassedGoals 半径 (既定 0.7 m、--radius)。
#
# 依存: numpy, matplotlib (TkAgg), PIL, PyYAML。rerobot_env にある (ホストには無い)。
#
# 使い方 (rerobot_env 内。xhost +local:docker 済みであること):
#   python3 /workspace/tools/waypoint_editor/waypoint_editor.py \
#       /workspace/maps/2d/glim/<map>/nav2/my_map.yaml \
#       /workspace/maps/2d/glim/<map>/waypoints/course.yaml
#   (out_yaml が既にあれば読み込んで続きから編集。keepout は <map_dir>/keep_out/keep_out.yaml を自動検出)
#   GUI なしの検査:  ... --check          (各点の判定を表で出すだけ)
#   画像に書き出し:  ... --render out.png (GUI なしで描画を保存)

import argparse
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image

HELP = """\
操作:
  左クリック (空き)      末尾に追加
  左ドラッグ (点の上)    移動
  Shift + 左クリック     最寄りの区間に挿入
  右クリック (点の上)    削除
  Ctrl + ドラッグ (点の上)  向きを手動指定 (マウスの方向を向く。紫の矢印)
  a  (点の上で)          向きを自動 (次の点の方向) に戻す
  u  元に戻す    s  保存    r  順序を反転    o  通過半径の表示切替
  h  この説明    q  終了 (未保存なら警告)
  ズーム/パンはツールバー (虫眼鏡/十字)。そのモード中はクリック編集を無視する。"""


# ----------------------------------------------------------------------------
# 地図
# ----------------------------------------------------------------------------
class GridMap:
    """map_server 形式の yaml + 画像。occ は 0 (空き)〜1 (占有) の占有確率。"""

    def __init__(self, yaml_path):
        meta = yaml.safe_load(open(yaml_path))
        img = meta["image"]
        if not os.path.isabs(img):
            img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img)
        self.path = yaml_path
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        if len(meta["origin"]) > 2 and abs(float(meta["origin"][2])) > 1e-9:
            print(f"⚠ {yaml_path}: origin の yaw ≠ 0 は未対応 (無視して表示する)", file=sys.stderr)
        self.occ_th = float(meta.get("occupied_thresh", 0.65))
        self.free_th = float(meta.get("free_thresh", 0.25))
        pix = np.asarray(Image.open(img).convert("L"), dtype=np.float32) / 255.0
        self.img = pix
        self.occ = pix if int(meta.get("negate", 0)) else 1.0 - pix
        self.h, self.w = pix.shape

    @property
    def extent(self):
        return [self.ox, self.ox + self.w * self.res, self.oy, self.oy + self.h * self.res]

    def cell(self, x, y):
        """map 座標 → 占有値。地図外は None。画像の行 0 が y 最大 (map_server と同じ)。"""
        c = int(math.floor((x - self.ox) / self.res))
        r = self.h - 1 - int(math.floor((y - self.oy) / self.res))
        if 0 <= r < self.h and 0 <= c < self.w:
            return float(self.occ[r, c])
        return None

    def classify(self, x, y):
        v = self.cell(x, y)
        if v is None:
            return "地図外"
        if v > self.occ_th:
            return "壁"
        if v < self.free_th:
            return "空き"
        return "未観測"


def find_keepout(map_yaml):
    """map_dir 規約 (<map_dir>/nav2/*.yaml + <map_dir>/keep_out/keep_out.yaml) から自動検出。"""
    cand = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(map_yaml))),
                        "keep_out", "keep_out.yaml")
    return cand if os.path.exists(cand) else None


# ----------------------------------------------------------------------------
# waypoint 入出力
# ----------------------------------------------------------------------------
def headings(pts, manual=None):
    """各点の yaw。manual[i] が None でなければそれ (手動)、None なら次の点への方向 (自動)。
    自動の最後の点は直前区間の向き、1 点だけなら 0。"""
    n = len(pts)
    yaws = []
    for i in range(n):
        if manual is not None and manual[i] is not None:
            yaws.append(manual[i])
            continue
        if n == 1:
            yaws.append(0.0)
            continue
        a, b = (pts[i], pts[i + 1]) if i + 1 < n else (pts[i - 1], pts[i])
        yaws.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    return yaws


def load_waypoints(path):
    """戻り値 (pts, manual)。manual[i] = 手動 yaw [rad] または None (自動)。
    panel で保存したファイル (yaw_manual 無し) は全点自動として読む。"""
    d = yaml.safe_load(open(path)) or {}
    pts, manual = [], []
    for _, wp in (d.get("waypoints") or {}).items():  # PyYAML は記述順を保つ
        pts.append((float(wp["pose"][0]), float(wp["pose"][1])))
        if wp.get("yaw_manual"):
            w, _, _, z = (float(v) for v in wp["orientation"])  # [w, x, y, z]
            manual.append(2.0 * math.atan2(z, w))
        else:
            manual.append(None)
    return pts, manual


def save_waypoints(path, pts, manual, map_yaml):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    lines = [
        f"# claude: waypoint_editor.py が生成 — map: {os.path.abspath(map_yaml)}",
        "# RViz Nav2 パネル「Load WPs」用。orientation は [w, x, y, z] の順。",
        "waypoints:",
    ]
    for i, ((x, y), yaw) in enumerate(zip(pts, headings(pts, manual))):
        lines += [
            f"  waypoint{i}:",
            f"    pose: [{x:.3f}, {y:.3f}, 0.0]",
            f"    orientation: [{math.cos(yaw / 2):.6f}, 0.0, 0.0, {math.sin(yaw / 2):.6f}]",
        ]
        if manual[i] is not None:
            lines.append("    yaw_manual: true")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    # コンテナ (root) で書いても所有者をホスト側の親ディレクトリに合わせる (他 tools と同じ)
    try:
        st = os.stat(os.path.dirname(os.path.abspath(path)))
        os.chown(path, st.st_uid, st.st_gid)
    except OSError:
        pass


def keepout_hit(ko, x, y):
    return ko is not None and (ko.cell(x, y) or 0.0) > ko.occ_th


def report(gm, ko, pts, manual):
    """各点の判定と区間長を表で出す。戻り値 = 問題のある点の数。yaw の * = 手動指定。"""
    bad = 0
    total = 0.0
    print(f"{'#':>3} {'x':>9} {'y':>9} {'yaw[deg]':>10} {'区間[m]':>8}  判定")
    for i, ((x, y), yaw) in enumerate(zip(pts, headings(pts, manual))):
        seg = math.dist(pts[i - 1], pts[i]) if i else 0.0
        total += seg
        st = gm.classify(x, y)
        if keepout_hit(ko, x, y):
            st += "+keepout"
        mark = "✔" if st == "空き" else ("▲" if st == "未観測" else "✘")
        bad += mark != "✔"
        ym = "*" if manual[i] is not None else " "
        print(f"{i:>3} {x:>9.2f} {y:>9.2f} {math.degrees(yaw):>9.1f}{ym} {seg:>8.2f}  {mark} {st}")
    print(f"計 {len(pts)} 点, 総延長 {total:.1f} m, 要確認 {bad} 点")
    return bad


# ----------------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------------
class Editor:
    PICK_PX = 10  # 点をつかむ判定半径 [画面 px]

    def __init__(self, plt, gm, ko, traj, out, pts, manual, radius):
        self.plt, self.gm, self.ko, self.out, self.radius = plt, gm, ko, out, radius
        self.pts = list(pts)
        self.manual = list(manual)  # pts と同じ長さ。手動 yaw [rad] or None (全編集で同期させる)
        self.undo = []
        self.drag = None
        self.rot = None  # Ctrl + ドラッグで向きを決めている点の index
        self.dirty = False
        self.show_radius = True
        self.artists = []

        for k in ("keymap.save", "keymap.quit", "keymap.back", "keymap.home", "keymap.yscale"):
            plt.rcParams[k] = []  # s / q / 等を本ツールの操作に使うため既定キーを外す
        # 日本語タイトル用のフォールバック (rerobot_env に入っている CJK フォント)
        plt.rcParams["font.family"] = ["DejaVu Sans", "Droid Sans Fallback"]
        global OUTLINE
        from matplotlib import patheffects
        OUTLINE = [patheffects.withStroke(linewidth=2.5, foreground="white")]  # 白地図でも読める番号
        self.fig, self.ax = plt.subplots(figsize=(11, 9))
        self.ax.imshow(gm.img, cmap="gray", vmin=0, vmax=1, extent=gm.extent,
                       origin="upper", interpolation="nearest")
        if ko is not None:
            mask = np.zeros((ko.h, ko.w, 4), np.float32)
            mask[ko.occ > ko.occ_th] = (1.0, 0.0, 0.0, 0.25)
            self.ax.imshow(mask, extent=ko.extent, origin="upper", interpolation="nearest")
        if traj is not None:
            self.ax.plot(traj[:, 0], traj[:, 1], "-", color="tab:cyan", lw=0.8, alpha=0.7,
                         label="traj")
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("x [m] (map)")
        self.ax.set_ylabel("y [m] (map)")
        if self.pts:  # 既存 waypoint があればその範囲に寄せる
            xs, ys = zip(*self.pts)
            m = 10.0
            self.ax.set_xlim(min(xs) - m, max(xs) + m)
            self.ax.set_ylim(min(ys) - m, max(ys) + m)

        c = self.fig.canvas
        c.mpl_connect("button_press_event", self.on_press)
        c.mpl_connect("motion_notify_event", self.on_motion)
        c.mpl_connect("button_release_event", self.on_release)
        c.mpl_connect("key_press_event", self.on_key)
        c.mpl_connect("close_event", self.on_close)
        self.redraw()

    # --- 描画 ---------------------------------------------------------------
    def color(self, x, y):
        st = self.gm.classify(x, y)
        if st in ("壁", "地図外") or keepout_hit(self.ko, x, y):
            return "red"
        return "limegreen" if st == "空き" else "orange"

    def redraw(self):
        for a in self.artists:
            a.remove()
        self.artists = []
        ax = self.ax
        if self.pts:
            xs, ys = zip(*self.pts)
            self.artists += ax.plot(xs, ys, "-", color="tab:blue", lw=1.5)
            yaws = headings(self.pts, self.manual)
            cols = ["magenta" if m is not None else "tab:blue" for m in self.manual]
            self.artists.append(ax.quiver(xs, ys, np.cos(yaws), np.sin(yaws), color=cols,
                                          angles="xy", scale_units="inches", scale=3, width=0.004,
                                          zorder=5))
            self.artists.append(ax.scatter(xs, ys, s=36, zorder=3, edgecolors="k",
                                           c=[self.color(x, y) for x, y in self.pts]))
            for i, (x, y) in enumerate(self.pts):
                self.artists.append(ax.annotate(str(i), (x, y), xytext=(5, 5),
                                                textcoords="offset points", fontsize=8,
                                                color="k", zorder=4, path_effects=OUTLINE))
                if self.show_radius:
                    circ = self.plt.Circle((x, y), self.radius, fill=False, color="tab:blue",
                                           lw=0.6, alpha=0.5)
                    ax.add_patch(circ)
                    self.artists.append(circ)
        total = sum(math.dist(a, b) for a, b in zip(self.pts, self.pts[1:]))
        bad = sum(self.color(x, y) != "limegreen" for x, y in self.pts)
        nman = sum(m is not None for m in self.manual)
        ax.set_title(f"{'* ' if self.dirty else ''}{os.path.basename(self.out)} — "
                     f"{len(self.pts)} 点 / {total:.1f} m / 要確認 {bad} 点 / 向き手動 {nman} 点"
                     f"   (h: 操作説明)")
        self.fig.canvas.draw_idle()

    # --- 編集 ---------------------------------------------------------------
    def push(self):
        self.undo.append((list(self.pts), list(self.manual)))
        self.dirty = True

    def nearest(self, ev):
        """画面上で PICK_PX 以内の最寄り点 index、無ければ None。"""
        if not self.pts:
            return None
        d = np.hypot(*(self.ax.transData.transform(self.pts) - (ev.x, ev.y)).T)
        i = int(np.argmin(d))
        return i if d[i] < self.PICK_PX else None

    def insert_index(self, p):
        """p に最も近い区間 (i, i+1) を探し、挿入位置 i+1 を返す。点が 1 個以下なら末尾。"""
        if len(self.pts) < 2:
            return len(self.pts)
        best, idx = float("inf"), len(self.pts)
        for i, (a, b) in enumerate(zip(self.pts, self.pts[1:])):
            ab = np.subtract(b, a)
            t = np.clip(np.dot(np.subtract(p, a), ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
            d = math.dist(p, np.add(a, t * ab))
            if d < best:
                best, idx = d, i + 1
        return idx

    def toolbar_busy(self):
        tb = getattr(self.fig.canvas, "toolbar", None)
        return tb is not None and str(getattr(tb, "mode", "")) != ""

    def on_press(self, ev):
        if ev.inaxes is not self.ax or self.toolbar_busy():
            return
        i = self.nearest(ev)
        if ev.button == 1 and ev.key == "control":
            if i is not None:
                self.push()
                self.rot = i
            return
        if ev.button == 1:
            if i is not None and ev.key != "shift":
                self.push()
                self.drag = i
                return
            self.push()
            p = (ev.xdata, ev.ydata)
            if ev.key == "shift":
                j = self.insert_index(p)
                self.pts.insert(j, p)
                self.manual.insert(j, None)
            else:
                self.pts.append(p)
                self.manual.append(None)
            self.redraw()
        elif ev.button == 3 and i is not None:
            self.push()
            del self.pts[i]
            del self.manual[i]
            self.redraw()

    def on_motion(self, ev):
        if ev.inaxes is not self.ax:
            return
        if self.rot is not None:
            x, y = self.pts[self.rot]
            if math.hypot(ev.xdata - x, ev.ydata - y) > 1e-6:
                self.manual[self.rot] = math.atan2(ev.ydata - y, ev.xdata - x)
                self.redraw()
            return
        if self.drag is None:
            return
        self.pts[self.drag] = (ev.xdata, ev.ydata)
        self.redraw()

    def on_release(self, ev):
        self.drag = None
        self.rot = None

    def on_key(self, ev):
        k = ev.key
        if k == "u" and self.undo:
            self.pts, self.manual = self.undo.pop()
            self.dirty = True
        elif k == "s":
            save_waypoints(self.out, self.pts, self.manual, self.gm.path)
            self.dirty = False
            print(f"保存: {self.out}")
            report(self.gm, self.ko, self.pts, self.manual)
        elif k == "a":
            i = self.nearest(ev)
            if i is None or self.manual[i] is None:
                return
            self.push()
            self.manual[i] = None
        elif k == "r" and self.pts:
            self.push()
            self.pts.reverse()
            self.manual.reverse()  # 手動の向きは絶対方位のまま (逆走でも同じ向きを向かせる)
        elif k == "o":
            self.show_radius = not self.show_radius
        elif k == "h":
            print(HELP)
        elif k == "q":
            if self.dirty:
                print("⚠ 未保存の変更あり。s で保存してから q (破棄して閉じるならウィンドウの × )")
                return
            self.plt.close(self.fig)
            return
        else:
            return
        self.redraw()

    def on_close(self, ev):
        if self.dirty:
            print(f"⚠ 未保存のまま閉じた ({len(self.pts)} 点は {self.out} に書かれていない)")


def load_traj(path):
    """GLIM traj_lidar.txt 等: 1 行 = 't x y z ...' (空白区切り)。x, y だけ使う。"""
    a = np.loadtxt(path, comments="#", ndmin=2)
    return a[:, 1:3]


def main():
    ap = argparse.ArgumentParser(description="2D 地図上で waypoint を打ち Nav2 パネル用 YAML に保存",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=HELP)
    ap.add_argument("map_yaml", help="map_server 形式の地図 yaml (nav2/my_map.yaml 等)")
    ap.add_argument("out_yaml", help="waypoint の保存先 (既存なら読み込んで編集)")
    ap.add_argument("--keepout", help="keepout マスク yaml (既定: <map_dir>/keep_out/keep_out.yaml を自動検出)")
    ap.add_argument("--no_keepout", action="store_true", help="keepout を表示・判定しない")
    ap.add_argument("--traj", help="走行軌跡 (GLIM traj_lidar.txt 等) を重ねて表示")
    ap.add_argument("--radius", type=float, default=0.7,
                    help="表示する通過判定半径 [m] (bt_navigator 既定 BT の RemovePassedGoals = 0.7)")
    ap.add_argument("--check", action="store_true", help="GUI を出さず各点の判定表だけ出す")
    ap.add_argument("--render", metavar="PNG", help="GUI を出さず描画を PNG に保存")
    args = ap.parse_args()

    gm = GridMap(args.map_yaml)
    ko_path = None if args.no_keepout else (args.keepout or find_keepout(args.map_yaml))
    ko = GridMap(ko_path) if ko_path else None
    pts, manual = load_waypoints(args.out_yaml) if os.path.exists(args.out_yaml) else ([], [])
    print(f"地図: {args.map_yaml} ({gm.w}x{gm.h}, {gm.res} m/px)")
    print(f"keepout: {ko_path or 'なし'}")
    print(f"waypoint: {args.out_yaml} ({'既存 ' + str(len(pts)) + ' 点' if pts else '新規'})")

    if args.check:
        sys.exit(1 if report(gm, ko, pts, manual) else 0)

    import matplotlib
    if args.render:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    traj = load_traj(args.traj) if args.traj else None
    ed = Editor(plt, gm, ko, traj, args.out_yaml, pts, manual, args.radius)
    if args.render:
        ed.fig.savefig(args.render, dpi=150, bbox_inches="tight")
        print(f"描画: {args.render}")
        return
    print(HELP)
    plt.show()


if __name__ == "__main__":
    main()
