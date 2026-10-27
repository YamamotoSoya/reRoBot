#!/usr/bin/env python3
# claude: 2026-10-26 作成
"""R-Fans の生 UDP を pcap (tcpdump -w と同じ classic pcap, Ethernet) に保存する.

ホストの tcpdump に sudo が使えないときの代替。main コンテナ (root, network_mode host) 内で動かす。
AF_PACKET でフレームをそのまま取るので、ドライバのソケットとは独立 (ドライバ起動中でも録れる)。

例:
  python3 rfans_capture.py -i enx6c1ff7be7a78 -t 45 -o /workspace/bags/rfans_check/<日付>/C1_rps10.pcap
"""
import argparse
import socket
import struct
import time

ETH_P_ALL = 0x0003


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--iface", required=True)
    ap.add_argument("-t", "--seconds", type=float, default=45.0)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--src", default="192.168.0.3", help="送信元 IP (LiDAR)")
    args = ap.parse_args()

    src = socket.inet_aton(args.src)
    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_ALL))
    s.bind((args.iface, 0))
    s.settimeout(0.5)
    n = 0
    sizes = {}
    t_end = time.time() + args.seconds
    with open(args.out, "wb") as f:
        f.write(struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1))
        while time.time() < t_end:
            try:
                fr, addr = s.recvfrom(65535)
            except socket.timeout:
                continue
            if addr[2] == socket.PACKET_OUTGOING:
                continue
            # Ethernet(14) + IPv4 + UDP、送信元 IP だけで絞る
            if len(fr) < 42 or fr[12:14] != b"\x08\x00" or fr[23] != 17 or fr[26:30] != src:
                continue
            t = time.time()
            f.write(struct.pack("<IIII", int(t), int((t % 1) * 1e6), len(fr), len(fr)))
            f.write(fr)
            n += 1
            ihl = (fr[14] & 0xF) * 4
            pl = len(fr) - 14 - ihl - 8
            sizes[pl] = sizes.get(pl, 0) + 1
    print(f"{args.out}: {n} packets ({n / args.seconds:.0f}/s), UDP payload sizes {sizes}")


if __name__ == "__main__":
    main()
