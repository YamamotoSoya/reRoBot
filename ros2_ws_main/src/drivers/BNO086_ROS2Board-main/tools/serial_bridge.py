#!/usr/bin/env python3
"""Expose a local serial port over TCP.

Docker Desktop on macOS and Windows cannot pass a USB device into a
container. Run this on the host holding the board, then point the ROS 2
driver at the bridge:

    # on the macOS host
    python3 tools/serial_bridge.py --port /dev/cu.usbmodem* --listen 5555

    # in the container
    ros2 launch bno086_imu_driver bno086.launch.py \\
        port:=socket://host.docker.internal:5555

Only one client is served at a time; a new connection replaces the old one,
so restarting the node just works. Requires pyserial and nothing else.
"""

from __future__ import annotations

import argparse
import glob
import socket
import sys
import threading
import time

import serial


def resolve_port(pattern: str) -> str | None:
    """Accept a glob so /dev/cu.usbmodem* can be used verbatim."""
    if any(ch in pattern for ch in "*?["):
        matches = sorted(glob.glob(pattern))
        if not matches:
            return None
        if len(matches) > 1:
            print(f"[bridge] {len(matches)} ports match, using {matches[0]}", file=sys.stderr)
        return matches[0]
    return pattern


def open_serial(pattern: str, baud: int, quiet: bool = False) -> serial.Serial:
    """Wait for the board to appear, then open it.

    Pressing S1 (RESET) makes the USB CDC device vanish and come back a
    moment later, so the bridge has to survive the port disappearing rather
    than exiting and taking the ROS session with it.
    """
    warned = quiet
    while True:
        device = resolve_port(pattern)
        if device is not None:
            try:
                ser = serial.Serial(device, baud, timeout=0.1, write_timeout=1.0)
                print(f"[bridge] opened {device}")
                return ser
            except (serial.SerialException, OSError) as exc:
                if not warned:
                    print(f"[bridge] {device} not ready: {exc}", file=sys.stderr)
                    warned = True
        elif not warned:
            print(f"[bridge] waiting for a port matching {pattern!r}", file=sys.stderr)
            warned = True
        time.sleep(0.5)


def pump(src_read, dst_send, stop: threading.Event, label: str, failed: set) -> None:
    while not stop.is_set():
        try:
            data = src_read()
        except Exception as exc:
            print(f"[bridge] {label} read failed: {exc}", file=sys.stderr)
            failed.add(label)
            break
        if data is None:
            break
        if not data:
            continue
        try:
            dst_send(data)
        except Exception as exc:
            print(f"[bridge] {label} write failed: {exc}", file=sys.stderr)
            failed.add(label)
            break
    stop.set()


def serve(ser: serial.Serial, conn: socket.socket) -> bool:
    """Shuttle bytes until either side goes away.

    @return True if the serial port is the side that failed, so the caller
            knows to reopen it rather than just wait for the next client.
    """
    stop = threading.Event()
    failed: set = set()
    conn.settimeout(0.1)

    def read_serial():
        return ser.read(max(1, ser.in_waiting))

    def read_socket():
        try:
            data = conn.recv(4096)
        except socket.timeout:
            return b""
        # An empty result from recv() means the peer closed the connection.
        return data if data else None

    threads = [
        threading.Thread(target=pump,
                         args=(read_serial, conn.sendall, stop, "serial->tcp", failed),
                         daemon=True),
        threading.Thread(target=pump,
                         args=(read_socket, ser.write, stop, "tcp->serial", failed),
                         daemon=True),
    ]
    for t in threads:
        t.start()
    stop.wait()
    for t in threads:
        t.join(timeout=1.0)

    return "serial->tcp" in failed or "tcp->serial" in failed


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/cu.usbmodem*",
                    help="serial device, glob allowed (default: %(default)s)")
    ap.add_argument("--baud", type=int, default=921600,
                    help="baud rate; ignored by USB CDC (default: %(default)s)")
    ap.add_argument("--listen", type=int, default=5555, help="TCP port to listen on")
    ap.add_argument("--bind", default="0.0.0.0", help="address to bind")
    args = ap.parse_args()

    ser = open_serial(args.port, args.baud)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.listen))
    srv.listen(1)
    print(f"[bridge] listening on {args.bind}:{args.listen}")
    print("[bridge] container side: socket://host.docker.internal:%d" % args.listen)

    try:
        while True:
            conn, addr = srv.accept()
            print(f"[bridge] client {addr[0]}:{addr[1]} connected")
            serial_failed = False
            try:
                # Drop anything buffered while nobody was listening, so the
                # client's frame parser starts on a clean stream.
                ser.reset_input_buffer()
                serial_failed = serve(ser, conn)
            except (serial.SerialException, OSError):
                serial_failed = True
            finally:
                conn.close()
                print("[bridge] client disconnected")

            # A session can end because the board went away, not just because
            # the client left. macOS keeps both the device node and is_open
            # alive after a USB reset while every read returns ENXIO, so the
            # pump's verdict is what decides whether to reopen.
            if serial_failed or not ser.is_open or resolve_port(args.port) is None:
                try:
                    ser.close()
                except Exception:
                    pass
                print("[bridge] board disconnected, waiting for it to return")
                ser = open_serial(args.port, args.baud, quiet=True)
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        ser.close()


if __name__ == "__main__":
    main()
