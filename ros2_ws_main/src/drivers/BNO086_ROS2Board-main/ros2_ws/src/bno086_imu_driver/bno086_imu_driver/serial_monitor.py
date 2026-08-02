"""Standalone serial monitor - useful for bringing a board up without ROS.

    ros2 run bno086_imu_driver serial_monitor --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import math
import time

from . import protocol as proto
from .serial_link import SerialLink


def main() -> None:
    ap = argparse.ArgumentParser(description="Dump frames from the BNO086 ROS2 board")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--rate", type=float, default=100.0, help="IMU rate to request, Hz")
    ap.add_argument("--raw", action="store_true", help="print every frame instead of 1 Hz summary")
    args = ap.parse_args()

    state = {"imu": 0, "mag": 0, "last": None, "status": None,
             "trace": None, "scan": None, "t0": time.time()}

    def on_frame(msg_id: int, payload: bytes) -> None:
        if msg_id == proto.MSG_IMU:
            s = proto.decode_imu(payload)
            state["imu"] += 1
            state["last"] = s
            if args.raw:
                x, y, z, w = s.quat
                print(
                    f"IMU t={s.device_us/1e6:9.3f}s "
                    f"q=({x:+.4f},{y:+.4f},{z:+.4f},{w:+.4f}) "
                    f"g=({s.gyro[0]:+.3f},{s.gyro[1]:+.3f},{s.gyro[2]:+.3f}) rad/s "
                    f"a=({s.accel[0]:+.2f},{s.accel[1]:+.2f},{s.accel[2]:+.2f}) m/s2 "
                    f"acc={s.quat_status}"
                )
        elif msg_id == proto.MSG_MAG:
            m = proto.decode_mag(payload)
            state["mag"] += 1
            if args.raw:
                print(f"MAG {m.field[0]:+.1f} {m.field[1]:+.1f} {m.field[2]:+.1f} uT acc={m.status}")
        elif msg_id == proto.MSG_STATUS:
            st = proto.decode_status(payload)
            state["status"] = st
            if args.raw:
                print(
                    f"STATUS proto=v{st.proto_version} ready={st.imu_ready} "
                    f"resets={st.reset_count} attempts={st.reset_attempts} "
                    f"spi_timeouts={st.spi_timeouts} packets={st.packets_seen} "
                    f"int={'LOW' if st.int_asserted else 'high'} [{st.stage_text}]"
                )
        elif msg_id == proto.MSG_TRACE:
            state["trace"] = proto.decode_trace(payload)
        elif msg_id == proto.MSG_SCAN:
            state["scan"] = proto.decode_scan(payload)

    link = SerialLink(args.port, args.baud, on_frame=on_frame,
                      on_error=lambda e: print(f"serial error: {e}"))
    link.open()
    print(f"listening on {args.port}")

    interval = int(round(1e6 / args.rate)) if args.rate > 0 else 0
    link.write(proto.encode_set_rate(interval, 40000))

    try:
        while True:
            time.sleep(1.0)
            if args.raw:
                continue
            s = state["last"]
            st = state["status"]
            imu_hz = state["imu"]
            state["imu"] = 0
            mag_hz = state["mag"]
            state["mag"] = 0
            if s is None:
                if st is not None:
                    # The board is alive but the IMU is not streaming; say why.
                    print(
                        f"IMU未応答: {st.stage_text} | 試行={st.reset_attempts} "
                        f"SPIタイムアウト={st.spi_timeouts} 受信パケット={st.packets_seen} "
                        f"INT={'LOW(assert)' if st.int_asserted else 'HIGH(idle)'}"
                    )
                    for e in (state["trace"] or []):
                        print(f"    受信SHTP: ch={e.channel} ({e.channel_text}) "
                              f"len={e.length} 先頭={e.data.hex(' ')}")
                    if state["scan"]:
                        print("    SPIモード走査:")
                        for e in state["scan"]:
                            mark = "  <== 応答あり" if e.got_reset_msg else ""
                            print(f"      mode={e.mode} {e.clock_khz:>4} kHz  "
                                  f"有効パケット={e.valid_packets} "
                                  f"reset_complete={'YES' if e.got_reset_msg else 'no'}{mark}")
                else:
                    print(f"基板から応答なし (crc_err={link.crc_errors} resync={link.resyncs})")
                continue
            x, y, z, w = s.quat
            # Yaw/pitch/roll purely for a human-readable sanity check.
            yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
            roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
            resets = st.reset_count if st else "?"
            print(
                f"{imu_hz:4d} Hz imu / {mag_hz:3d} Hz mag  "
                f"rpy=({roll:+7.2f},{pitch:+7.2f},{yaw:+7.2f})deg  "
                f"acc={s.quat_status} resets={resets} "
                f"crc_err={link.crc_errors} resync={link.resyncs}"
            )
    except KeyboardInterrupt:
        pass
    finally:
        link.close()


if __name__ == "__main__":
    main()
