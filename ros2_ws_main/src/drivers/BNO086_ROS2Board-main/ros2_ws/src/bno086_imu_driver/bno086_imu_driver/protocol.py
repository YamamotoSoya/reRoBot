"""Wire protocol shared with the STM32 firmware.

Frame layout (little endian):

    0xAA 0x55 | id:u8 | len:u8 | payload[len] | crc16:u16

The CRC is CRC-16/CCITT-FALSE over id, len and payload.

The board sends raw BNO086 fixed-point values so the MCU never has to touch
floating point; the Q-format scaling is applied here.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, Optional

SYNC0 = 0xAA
SYNC1 = 0x55

PROTO_VERSION = 2

# device -> host
MSG_IMU = 0x01
MSG_MAG = 0x02
MSG_STATUS = 0x03
MSG_TRACE = 0x04
MSG_SCAN = 0x05
MSG_PINS = 0x06
MSG_CMDRESP = 0x07

# host -> device
MSG_SET_RATE = 0x80
MSG_TARE = 0x81
MSG_SAVE_DCD = 0x82
MSG_RESET_IMU = 0x83
MSG_PING = 0x84

STATUS_IMU_READY = 1 << 0
STATUS_INT_ASSERTED = 1 << 1

# How far the firmware's last BNO086 reset attempt got.
STAGE_NONE = 0
STAGE_NO_INT = 1
STAGE_SPI_SILENT = 2
STAGE_NO_RESET_MSG = 3
STAGE_READY = 4

STAGE_NAMES = {
    STAGE_NONE: "not attempted",
    STAGE_NO_INT: "H_INTN never asserted (IMU silent - check power/reset/straps)",
    STAGE_SPI_SILENT: "INT asserted but SPI returned no packet",
    STAGE_NO_RESET_MSG: "packets received but no reset-complete",
    STAGE_READY: "reset complete",
}

FLAG_QUAT_VALID = 1 << 0
FLAG_GYRO_VALID = 1 << 1
FLAG_ACCEL_VALID = 1 << 2

# BNO086 Q-point scale factors (SH-2 reference manual, section 6.5)
QUAT_SCALE = 2.0**-14          # unit quaternion
QUAT_ACC_SCALE = 2.0**-12      # radians
GYRO_SCALE = 2.0**-9           # rad/s
ACCEL_SCALE = 2.0**-8          # m/s^2
MAG_SCALE = 2.0**-4            # microtesla

# Upper bound used to reject a corrupt length byte during resynchronisation.
# It must exceed the largest frame the firmware can emit - the bring-up SCAN
# frame is 65 bytes, which a 64 byte limit would silently discard.
MAX_PAYLOAD = 128


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). crc16(b"123456789") == 0x29B1."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_id: int, payload: bytes = b"") -> bytes:
    """Build a complete frame ready to write to the serial port."""
    if len(payload) > 255:
        raise ValueError("payload too long")
    body = bytes([msg_id, len(payload)]) + payload
    return bytes([SYNC0, SYNC1]) + body + struct.pack("<H", crc16_ccitt(body))


# Which rotation vector the board should stream.
RV_MAGNETIC = 0   # fuses the magnetometer: absolute heading, but disturbed by
                  # steel and motors
RV_GAME = 1       # gyro + accel only: yaw starts at 0 each boot, drifts slowly,
                  # immune to magnetic disturbance


def encode_set_rate(imu_interval_us: int, mag_interval_us: int,
                    rotation_vector: int = RV_MAGNETIC) -> bytes:
    return encode_frame(MSG_SET_RATE, struct.pack(
        "<IIB", imu_interval_us, mag_interval_us, rotation_vector))


# Axis bitmap for encode_tare(). Zeroing yaw alone keeps roll and pitch
# referenced to gravity, which is usually what a robot wants.
TARE_AXIS_X = 0x01
TARE_AXIS_Y = 0x02
TARE_AXIS_Z = 0x04
TARE_AXIS_ALL = 0x07

# SH-2 command ids, as echoed back in a command response.
SH2_CMD_TARE = 0x03
SH2_CMD_SAVE_DCD = 0x06


def encode_tare(persist: bool = False, axes: int = TARE_AXIS_ALL) -> bytes:
    return encode_frame(MSG_TARE, bytes([1 if persist else 0, axes & TARE_AXIS_ALL]))


@dataclass
class CommandResponse:
    """The device's answer to an SH-2 command request."""

    valid: bool
    command: int
    seq: int
    r: tuple

    @property
    def ok(self) -> bool:
        """R0 == 0 means the command succeeded."""
        return self.valid and self.r[0] == 0

    @property
    def text(self) -> str:
        if not self.valid:
            return "no command response yet"
        name = {SH2_CMD_TARE: "tare", SH2_CMD_SAVE_DCD: "save DCD"}.get(
            self.command, f"cmd 0x{self.command:02X}")
        return f"{name}: {'OK' if self.ok else 'FAILED'} (R={list(self.r)})"


def decode_cmdresp(payload: bytes) -> CommandResponse:
    if len(payload) < 7:
        raise ValueError(f"CMDRESP payload too short: {len(payload)}")
    return CommandResponse(
        valid=bool(payload[0]),
        command=payload[1],
        seq=payload[2],
        r=tuple(payload[3:7]),
    )


@dataclass
class ImuSample:
    """One HL_MSG_IMU frame, scaled into SI units."""

    device_us: int
    quat: tuple            # (x, y, z, w) - reordered from the sensor's i,j,k,real
    quat_accuracy_rad: float
    gyro: tuple            # rad/s
    accel: tuple           # m/s^2
    quat_status: int       # 0 unreliable .. 3 high
    gyro_status: int
    accel_status: int
    quat_valid: bool
    gyro_valid: bool
    accel_valid: bool


@dataclass
class MagSample:
    device_us: int
    field: tuple           # microtesla
    status: int


@dataclass
class StatusSample:
    device_us: int
    reset_count: int
    imu_ready: bool
    proto_version: int
    frames: int
    # Bring-up diagnostics (protocol v2 and later; zero on older firmware).
    int_asserted: bool = False
    reset_attempts: int = 0
    spi_timeouts: int = 0
    packets_seen: int = 0
    last_stage: int = STAGE_NONE

    @property
    def stage_text(self) -> str:
        return STAGE_NAMES.get(self.last_stage, f"unknown ({self.last_stage})")


def decode_imu(payload: bytes) -> ImuSample:
    if len(payload) < 28:
        raise ValueError(f"IMU payload too short: {len(payload)}")
    (
        t_us,
        qi, qj, qk, qr, q_acc,
        gx, gy, gz,
        ax, ay, az,
        status, flags,
    ) = struct.unpack("<I11hBB", payload[:28])

    return ImuSample(
        device_us=t_us,
        # ROS geometry_msgs/Quaternion is (x, y, z, w); the sensor sends i,j,k,real.
        quat=(qi * QUAT_SCALE, qj * QUAT_SCALE, qk * QUAT_SCALE, qr * QUAT_SCALE),
        quat_accuracy_rad=q_acc * QUAT_ACC_SCALE,
        gyro=(gx * GYRO_SCALE, gy * GYRO_SCALE, gz * GYRO_SCALE),
        accel=(ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE),
        quat_status=status & 0x03,
        gyro_status=(status >> 2) & 0x03,
        accel_status=(status >> 4) & 0x03,
        quat_valid=bool(flags & FLAG_QUAT_VALID),
        gyro_valid=bool(flags & FLAG_GYRO_VALID),
        accel_valid=bool(flags & FLAG_ACCEL_VALID),
    )


def decode_mag(payload: bytes) -> MagSample:
    if len(payload) < 12:
        raise ValueError(f"MAG payload too short: {len(payload)}")
    t_us, mx, my, mz, status, _pad = struct.unpack("<I3hBB", payload[:12])
    return MagSample(
        device_us=t_us,
        field=(mx * MAG_SCALE, my * MAG_SCALE, mz * MAG_SCALE),
        status=status & 0x03,
    )


def decode_status(payload: bytes) -> StatusSample:
    if len(payload) < 12:
        raise ValueError(f"STATUS payload too short: {len(payload)}")
    t_us, resets, flags, ver, frames = struct.unpack("<IHBBI", payload[:12])
    status = StatusSample(
        device_us=t_us,
        reset_count=resets,
        imu_ready=bool(flags & STATUS_IMU_READY),
        proto_version=ver,
        frames=frames,
        int_asserted=bool(flags & STATUS_INT_ASSERTED),
    )

    # v2 appended the bring-up counters; stay readable against a v1 board.
    if len(payload) >= 20:
        (status.reset_attempts, status.spi_timeouts, status.packets_seen,
         status.last_stage, _reserved) = struct.unpack("<HHHBB", payload[12:20])

    return status


SHTP_CHANNEL_NAMES = {
    0: "command/advertisement",
    1: "executable",
    2: "control",
    3: "input-normal",
    4: "input-wake",
    5: "input-gyro",
}


@dataclass
class TraceEntry:
    """One SHTP packet the firmware saw during a bring-up attempt."""

    channel: int
    length: int
    data: bytes

    @property
    def channel_text(self) -> str:
        return SHTP_CHANNEL_NAMES.get(self.channel, f"invalid({self.channel})")


@dataclass
class ScanEntry:
    """Result of one SPI mode/clock combination tried during bring-up."""

    mode: int
    clock_khz: int
    valid_packets: int
    got_reset_msg: bool
    first_channel: int
    first_length: int


def decode_trace(payload: bytes) -> list:
    if not payload:
        raise ValueError("empty TRACE payload")
    count = payload[0]
    entries = []
    for i in range(count):
        off = 1 + i * 7
        if off + 7 > len(payload):
            break
        channel = payload[off]
        (length,) = struct.unpack_from("<H", payload, off + 1)
        entries.append(TraceEntry(channel, length, payload[off + 3:off + 7]))
    return entries


@dataclass
class PinProbe:
    """Levels read back from MISO / H_INTN under opposite internal biases.

    A line that follows our own bias in both directions is floating: nothing
    on the other end is driving it. MISO is sampled both with CS released
    (where an SPI slave must be high-Z) and with CS asserted (where it must
    drive), which separates "not selected" from "framing is wrong".
    """

    miso_idle_pullup: bool
    miso_idle_pulldown: bool
    miso_sel_pullup: bool
    miso_sel_pulldown: bool
    int_with_pullup: bool
    int_with_pulldown: bool

    @staticmethod
    def _verdict(with_pullup: bool, with_pulldown: bool) -> str:
        if with_pullup and not with_pulldown:
            return "floating (nothing driving it)"
        if not with_pullup and not with_pulldown:
            return "driven LOW"
        if with_pullup and with_pulldown:
            return "driven HIGH"
        return "inconsistent"

    @staticmethod
    def _floating(with_pullup: bool, with_pulldown: bool) -> bool:
        return with_pullup and not with_pulldown

    @property
    def miso_idle_text(self) -> str:
        return self._verdict(self.miso_idle_pullup, self.miso_idle_pulldown)

    @property
    def miso_selected_text(self) -> str:
        return self._verdict(self.miso_sel_pullup, self.miso_sel_pulldown)

    @property
    def int_text(self) -> str:
        return self._verdict(self.int_with_pullup, self.int_with_pulldown)

    @property
    def miso_driven_when_selected(self) -> bool:
        """True when asserting CS makes the BNO086 take over MISO."""
        return not self._floating(self.miso_sel_pullup, self.miso_sel_pulldown)

    @property
    def int_driven(self) -> bool:
        return not self._floating(self.int_with_pullup, self.int_with_pulldown)

    @property
    def diagnosis(self) -> str:
        if self.miso_driven_when_selected:
            return ("BNO086 drives MISO when selected - the bus works, so the "
                    "fault is in the SHTP framing")
        if self.int_driven:
            return ("H_INTN is driven but MISO stays floating when selected - "
                    "the IMU is alive but not answering as an SPI slave "
                    "(protocol strap PS1/PS0, CS net, or MISO joint)")
        return ("neither MISO nor H_INTN is driven - the IMU looks unpowered, "
                "held in reset, or not soldered")


def decode_pins(payload: bytes) -> PinProbe:
    if len(payload) < 6:
        raise ValueError(f"PINS payload too short: {len(payload)}")
    return PinProbe(*[bool(b) for b in payload[:6]])


def decode_scan(payload: bytes) -> list:
    if not payload:
        raise ValueError("empty SCAN payload")
    count = payload[0]
    entries = []
    for i in range(count):
        off = 1 + i * 8
        if off + 8 > len(payload):
            break
        mode = payload[off]
        (clock_khz,) = struct.unpack_from("<H", payload, off + 1)
        valid, got_reset, first_ch = payload[off + 3], payload[off + 4], payload[off + 5]
        (first_len,) = struct.unpack_from("<H", payload, off + 6)
        entries.append(ScanEntry(mode, clock_khz, valid, bool(got_reset), first_ch, first_len))
    return entries


class FrameParser:
    """Incremental byte-stream parser; resynchronises after corruption."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0
        self.resyncs = 0

    def feed(self, data: bytes) -> Iterator[tuple]:
        """Append bytes and yield every complete, CRC-valid (msg_id, payload)."""
        self._buf.extend(data)

        while True:
            frame = self._next_frame()
            if frame is None:
                return
            yield frame

    def _next_frame(self) -> Optional[tuple]:
        buf = self._buf

        while True:
            start = buf.find(bytes([SYNC0, SYNC1]))
            if start < 0:
                # Keep the last byte: it may be the first half of a sync word.
                if len(buf) > 1:
                    del buf[: len(buf) - 1]
                return None
            if start > 0:
                del buf[:start]
                self.resyncs += 1

            if len(buf) < 4:
                return None

            length = buf[3]
            total = 4 + length + 2
            if length > MAX_PAYLOAD:
                # Not a frame we can produce; skip this sync and look again.
                del buf[:2]
                self.resyncs += 1
                continue
            if len(buf) < total:
                return None

            body = bytes(buf[2 : 4 + length])
            (rx_crc,) = struct.unpack("<H", buf[4 + length : total])
            if rx_crc != crc16_ccitt(body):
                self.crc_errors += 1
                del buf[:2]
                continue

            msg_id = buf[2]
            payload = bytes(buf[4 : 4 + length])
            del buf[:total]
            return (msg_id, payload)
