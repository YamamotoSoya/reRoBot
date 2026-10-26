#!/usr/bin/env python3
# claude: 2026-10-26 作成
"""R-Fans-16 の生パケットから「櫛状の距離偽点」(0.686 m の倍数ずれ) の率を数える.

背景: docs/issue/2026-10-25_rfans_range_comb_ghost.md (§6, §7)。
data_level 3 (PACKET_USER_SIMPLE_S, 1206 byte, ssFrameLib.h:577-594) のパケットを
ドライバを通さずにデコードし、同じレーザの列方向で
「両隣 2 点ずつが直線に乗るのに、中央だけ k x 171.5 LSB (k=±1..±5) ずれた点」を数える。

入力:
  - pcap (tcpdump -w。pcapng は不可)  … ドライバのデコードを一切通らない最も素の経路
  - rosbag2 ディレクトリ (/rfans_driver/rfans_packets) … rosbag2_py が要る (main コンテナ内)

例:
  python3 rfans_comb_check.py a.pcap b.pcap c.pcap           # 条件ごとの比較表
  python3 rfans_comb_check.py --png out.png a.pcap b.pcap    # オフセット分布の図も出す
  python3 rfans_comb_check.py /workspace/bags/raw/<bag>      # bag の packets トピック
"""
import argparse
import json
import os
import struct
import sys

import numpy as np

PKT_SIZE = 1206
LSB = 0.004  # m
P_LSB = 171.5  # 櫛の間隔 (0.686 m)
GRP = np.dtype([("flag", "<u2"), ("az", "<u2"), ("pts", [("r", "<u2"), ("i", "u1")], 32)])
PKT = np.dtype([("g", GRP, 12), ("gps", "<u4"), ("A", "u1"), ("B", "u1")])
assert PKT.itemsize == PKT_SIZE


def read_pcap(path):
    """(timestamp[s], payload) を 1206 byte の UDP ペイロードだけ返す."""
    with open(path, "rb") as f:
        buf = f.read()
    magic = buf[:4]
    if magic == b"\x0a\x0d\x0d\x0a":
        sys.exit(f"{path}: pcapng は未対応。tcpdump -w (既定の pcap 形式) で取り直すか "
                 f"editcap -F pcap で変換する")
    for endian, mag in (("<", 0xA1B2C3D4), (">", 0xA1B2C3D4), ("<", 0xA1B23C4D), (">", 0xA1B23C4D)):
        if struct.unpack(endian + "I", magic)[0] == mag:
            break
    else:
        sys.exit(f"{path}: pcap ではない (magic {magic.hex()})")
    tdiv = 1e9 if mag == 0xA1B23C4D else 1e6
    link = struct.unpack(endian + "I", buf[20:24])[0]
    l2 = {1: 14, 113: 16, 276: 20, 0: 4, 101: 0}.get(link)
    if l2 is None:
        sys.exit(f"{path}: 未対応のリンク種別 {link}")
    out = []
    pos = 24
    while pos + 16 <= len(buf):
        ts, tf, incl, _ = struct.unpack(endian + "IIII", buf[pos:pos + 16])
        pos += 16
        fr = buf[pos:pos + incl]
        pos += incl
        ip = fr[l2:]
        if len(ip) < 28 or ip[0] >> 4 != 4 or ip[9] != 17:
            continue
        ihl = (ip[0] & 0xF) * 4
        pl = ip[ihl + 8:]
        if len(pl) == PKT_SIZE:
            out.append((ts + tf / tdiv, pl))
    return out


def read_bag(path, topic, start=0.0, duration=None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = rosbag2_py.SequentialReader()
    sid = "mcap" if any(n.endswith(".mcap") for n in os.listdir(path)) else "sqlite3"
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if topic not in types:
        sys.exit(f"{path}: {topic} が無い (packets を録っていない bag)")
    M = get_message(types[topic])
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    t_bag0 = r.get_metadata().starting_time.nanoseconds
    r.seek(int(t_bag0 + start * 1e9))
    out = []
    while r.has_next():
        _, data, ts = r.read_next()
        if duration is not None and ts > t_bag0 + (start + duration) * 1e9:
            break
        m = deserialize_message(data, M)
        # RfansPacket は 1 msg = udp_count 個の連結。Packet[] を持つ型 (RfansScan) にも対応
        for p in (m.packets if hasattr(m, "packets") else [m]):
            b = bytes(p.data)
            sz = getattr(p, "udp_size", 0) or PKT_SIZE
            for k in range(getattr(p, "udp_count", 0) or len(b) // sz):
                out.append((ts * 1e-9, b[k * sz:k * sz + PKT_SIZE]))
    return out


def analyze(pkts, chunk_s=1.0):
    if not pkts:
        return None
    t = np.array([p[0] for p in pkts])
    a = np.frombuffer(b"".join(p[1] for p in pkts), dtype=PKT)
    devid = sorted({int(x) for x in a["B"]} | {int(x) for x in a["A"]})
    R = a["g"]["pts"]["r"].reshape(-1, 32).astype(np.int32)
    I = a["g"]["pts"]["i"].reshape(-1, 32).astype(np.int32)
    az = a["g"]["az"].ravel().astype(np.int32)
    tg = np.repeat(t, 12)
    # 1 group = 2 発光列 (j<16 / j>=16)。時刻順の列に並べ替える (bufferDecode.cpp の timeflag 順)
    cols = np.stack([R[:, :16], R[:, 16:]], 1).reshape(-1, 16)
    icol = np.stack([I[:, :16], I[:, 16:]], 1).reshape(-1, 16)
    tc = np.repeat(tg, 2)
    m2, m1, c, p1, p2 = cols[:-4], cols[1:-3], cols[2:-2], cols[3:-1], cols[4:]
    slope = (2 * (p2 - m2) + (p1 - m1)) / 10.0
    mean = (m2 + m1 + p1 + p2) / 4.0
    res = np.max(np.abs(np.stack([m2 - (mean - 2 * slope), m1 - (mean - slope),
                                  p1 - (mean + slope), p2 - (mean + 2 * slope)])), axis=0)
    # 平滑な近傍 (両隣 2 点ずつが 4 cm 以内で直線、0.5 m 以上) を持つ点だけを母数にする
    valid = (res < 10) & (np.minimum.reduce([m2, m1, p1, p2]) > 125) & (c > 0)
    off = c - mean
    k = np.round(off / P_LSB)
    comb = valid & (np.abs(k) >= 1) & (np.abs(k) <= 5) & (np.abs(off - k * P_LSB) < 15)
    other = valid & (np.abs(off) > 50) & ~comb
    tcc = tc[2:-2]
    # 回転数: 方位角の巻き戻り回数 / 時間
    wraps = int((np.diff(az) < -18000).sum())
    dur = float(t[-1] - t[0])
    # 時間ブロックごとの率 → ばらつき (条件比較の誤差幅)
    blk = ((tcc - t[0]) // chunk_s).astype(int)
    nb = blk.max() + 1
    vb = np.bincount(np.repeat(blk, 16), weights=valid.ravel(), minlength=nb)
    cb = np.bincount(np.repeat(blk, 16), weights=comb.ravel(), minlength=nb)
    ok = vb > 1000
    rates = 100 * cb[ok] / vb[ok]
    tr = mean * LSB
    rng_bins = [(0.5, 4), (4, 8), (8, 15), (15, 30), (30, 200)]
    r = {
        "packets": len(a),
        "duration_s": round(dur, 1),
        "devid_bytes": [hex(d) for d in devid],
        "rps_est": round(wraps / dur, 2) if dur > 0 else None,
        "valid": int(valid.sum()),
        "comb_pct": round(100 * comb.sum() / max(valid.sum(), 1), 3),
        "comb_pct_block_mean": round(float(rates.mean()), 3) if len(rates) else None,
        "comb_pct_block_sd": round(float(rates.std(ddof=1)), 3) if len(rates) > 1 else None,
        "n_blocks": int(len(rates)),
        "other_spike_pct": round(100 * other.sum() / max(valid.sum(), 1), 3),
        "k_hist": {int(kk): int(((k == kk) & comb).sum()) for kk in range(-5, 6) if kk},
        "per_laser_pct": [round(100 * comb[:, l].sum() / max(valid[:, l].sum(), 1), 2) for l in range(16)],
        "per_range_pct": {f"{lo}-{hi}m": round(100 * (comb & (tr >= lo) & (tr < hi)).sum()
                                             / max((valid & (tr >= lo) & (tr < hi)).sum(), 1), 3)
                          for lo, hi in rng_bins},
        "intensity_true_vs_ghost": [round(float(np.median(icol[2:-2][valid & ~comb])), 1) if valid.any() else None,
                                    round(float(np.median(icol[2:-2][comb])), 1) if comb.any() else None],
    }
    return r, off[valid & (np.abs(off) > 50)] * LSB, valid.sum()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="pcap ファイル or rosbag2 ディレクトリ")
    ap.add_argument("--topic", default="/rfans_driver/rfans_packets")
    ap.add_argument("--skip", type=float, default=2.0, help="先頭を捨てる秒数 (回転の立ち上がり除外)")
    ap.add_argument("--bag-start", type=float, default=0.0, help="bag: 開始位置 [bag 先頭からの秒]")
    ap.add_argument("--duration", type=float, help="bag: 読む長さ [s] (既定: 最後まで)")
    ap.add_argument("--png", help="スパイクのオフセット分布を描く出力先")
    ap.add_argument("--json", help="結果を JSON で保存")
    args = ap.parse_args()

    results = {}
    hists = {}
    for path in args.inputs:
        pk = read_bag(path, args.topic, args.bag_start, args.duration) if os.path.isdir(path) else read_pcap(path)
        if pk:
            t0 = pk[0][0]
            pk = [p for p in pk if p[0] - t0 >= args.skip]
        out = analyze(pk)
        name = os.path.basename(os.path.normpath(path))
        if out is None:
            print(f"== {name}: 1206 byte のパケットが 0 個 (data_level 3 以外 / 未送信 / 別 IF)")
            results[name] = None
            continue
        r, offs, nv = out
        results[name] = r
        hists[name] = (offs, nv)
        print(f"== {name}")
        for kk, v in r.items():
            print(f"  {kk:24s} {v}")

    ok = {n: r for n, r in results.items() if r}
    if len(ok) > 1:
        print("\n条件比較 (comb_pct = 櫛の偽点 / 平滑な近傍を持つ点。±は 1 秒ブロックの標準偏差)")
        print(f"  {'input':40s} {'rps':>5s} {'comb%':>7s} {'±sd':>6s} {'other%':>7s}  k=+3 の割合")
        for n, r in ok.items():
            kh = r["k_hist"]
            tot = max(sum(kh.values()), 1)
            print(f"  {n:40s} {r['rps_est']!s:>5s} {r['comb_pct']:7.3f} {r['comb_pct_block_sd']!s:>6s} "
                  f"{r['other_spike_pct']:7.3f}  {100 * kh[3] / tot:.0f}%")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=1, ensure_ascii=False)
    if args.png and hists:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axs = plt.subplots(len(hists), 1, figsize=(12, 2.8 * len(hists)), sharex=True, squeeze=False)
        for ax, (n, (o, nv)) in zip(axs[:, 0], hists.items()):
            ax.hist(o, bins=np.arange(-3.5, 3.51, 0.01), weights=np.full(len(o), 100 / nv))
            for kk in range(-5, 6):
                if kk:
                    ax.axvline(kk * P_LSB * LSB, color="r", alpha=0.3, ls="--")
            ax.set_title(f"{n}  comb {results[n]['comb_pct']:.3f}%  rps {results[n]['rps_est']}")
            ax.set_ylabel("% / 1 cm bin")
        axs[-1, 0].set_xlabel("offset from neighbours [m] (red = k x 0.686 m)")
        fig.tight_layout()
        fig.savefig(args.png, dpi=80)
        print(f"wrote {args.png}")


if __name__ == "__main__":
    main()
