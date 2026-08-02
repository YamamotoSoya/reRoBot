"""Unit tests for the wire protocol, runnable without ROS or hardware."""

import struct

import pytest

from bno086_imu_driver import protocol as proto


def test_crc_reference_vector():
    # The canonical CRC-16/CCITT-FALSE check value.
    assert proto.crc16_ccitt(b"123456789") == 0x29B1


def build_imu_payload(**kw):
    fields = dict(
        t_us=123456, qi=0, qj=0, qk=0, qr=1 << 14, q_acc=0,
        gx=0, gy=0, gz=0, ax=0, ay=0, az=0, status=0, flags=0x07,
    )
    fields.update(kw)
    return struct.pack(
        "<I11hBB",
        fields["t_us"], fields["qi"], fields["qj"], fields["qk"], fields["qr"],
        fields["q_acc"], fields["gx"], fields["gy"], fields["gz"],
        fields["ax"], fields["ay"], fields["az"], fields["status"], fields["flags"],
    )


def test_imu_payload_is_28_bytes():
    assert len(build_imu_payload()) == 28


def test_decode_imu_identity_quaternion():
    s = proto.decode_imu(build_imu_payload())
    assert s.device_us == 123456
    assert s.quat == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert s.quat_valid and s.gyro_valid and s.accel_valid


def test_decode_imu_q_scaling():
    # 1 g on Z in Q8, 1 rad/s on X in Q9.
    s = proto.decode_imu(build_imu_payload(az=int(9.80665 * 256), gx=512))
    assert s.accel[2] == pytest.approx(9.80665, abs=1e-2)
    assert s.gyro[0] == pytest.approx(1.0)


def test_decode_imu_negative_values():
    s = proto.decode_imu(build_imu_payload(ax=-256, qi=-(1 << 13)))
    assert s.accel[0] == pytest.approx(-1.0)
    assert s.quat[0] == pytest.approx(-0.5)


def test_decode_imu_status_and_flag_bits():
    # quat=3, gyro=2, accel=1 packed two bits each; only gyro marked fresh.
    s = proto.decode_imu(build_imu_payload(status=0b011011, flags=proto.FLAG_GYRO_VALID))
    assert (s.quat_status, s.gyro_status, s.accel_status) == (3, 2, 1)
    assert s.gyro_valid
    assert not s.quat_valid and not s.accel_valid


def test_decode_mag_scaling():
    payload = struct.pack("<I3hBB", 42, 16, -32, 0, 3, 0)
    m = proto.decode_mag(payload)
    assert m.field == pytest.approx((1.0, -2.0, 0.0))
    assert m.status == 3


def test_decode_status_v2_with_diagnostics():
    payload = struct.pack(
        "<IHBBIHHHBB", 1000, 7,
        proto.STATUS_IMU_READY | proto.STATUS_INT_ASSERTED,
        proto.PROTO_VERSION, 999,
        3, 1, 42, proto.STAGE_READY, 0,
    )
    st = proto.decode_status(payload)
    assert st.reset_count == 7
    assert st.imu_ready
    assert st.int_asserted
    assert st.proto_version == proto.PROTO_VERSION
    assert st.frames == 999
    assert (st.reset_attempts, st.spi_timeouts, st.packets_seen) == (3, 1, 42)
    assert st.last_stage == proto.STAGE_READY
    assert "reset complete" in st.stage_text


def test_decode_status_accepts_a_short_v1_payload():
    """A v1 board must still be readable rather than raising."""
    payload = struct.pack("<IHBBI", 1000, 0, 0, 1, 5)
    st = proto.decode_status(payload)
    assert st.proto_version == 1
    assert not st.imu_ready
    assert st.last_stage == proto.STAGE_NONE


def test_status_stage_text_explains_a_silent_imu():
    payload = struct.pack("<IHBBIHHHBB", 0, 0, 0, proto.PROTO_VERSION, 0,
                          5, 0, 0, proto.STAGE_NO_INT, 0)
    st = proto.decode_status(payload)
    assert not st.imu_ready
    assert st.reset_attempts == 5
    assert "H_INTN" in st.stage_text


def test_short_payload_raises():
    with pytest.raises(ValueError):
        proto.decode_imu(b"\x00" * 10)


def test_frame_roundtrip():
    payload = build_imu_payload()
    parser = proto.FrameParser()
    frames = list(parser.feed(proto.encode_frame(proto.MSG_IMU, payload)))
    assert frames == [(proto.MSG_IMU, payload)]
    assert parser.crc_errors == 0


def test_parser_handles_byte_at_a_time_delivery():
    """USB CDC can split a frame across reads arbitrarily."""
    stream = proto.encode_frame(proto.MSG_IMU, build_imu_payload())
    parser = proto.FrameParser()
    got = []
    for byte in stream:
        got.extend(parser.feed(bytes([byte])))
    assert len(got) == 1
    assert got[0][0] == proto.MSG_IMU


def test_parser_recovers_after_leading_garbage():
    stream = b"\x00\xFF\xAA garbage" + proto.encode_frame(proto.MSG_STATUS, b"\x01" * 12)
    parser = proto.FrameParser()
    frames = list(parser.feed(stream))
    assert len(frames) == 1
    assert frames[0][0] == proto.MSG_STATUS
    assert parser.resyncs > 0


def test_parser_rejects_corrupted_crc_and_resyncs():
    good = proto.encode_frame(proto.MSG_IMU, build_imu_payload())
    bad = bytearray(good)
    bad[-1] ^= 0xFF                      # break the CRC
    parser = proto.FrameParser()
    frames = list(parser.feed(bytes(bad) + good))
    assert parser.crc_errors == 1
    # The intact frame that follows must still be delivered.
    assert len(frames) == 1
    assert frames[0][0] == proto.MSG_IMU


def test_parser_streams_many_frames():
    payload = build_imu_payload()
    stream = b"".join(proto.encode_frame(proto.MSG_IMU, payload) for _ in range(200))
    parser = proto.FrameParser()
    assert len(list(parser.feed(stream))) == 200


def test_parser_survives_sync_pattern_inside_payload():
    """0xAA 0x55 in the data must not be mistaken for a frame start."""
    payload = build_imu_payload(gx=0x55AA, gy=0x55AA)
    stream = proto.encode_frame(proto.MSG_IMU, payload) * 3
    parser = proto.FrameParser()
    frames = list(parser.feed(stream))
    assert len(frames) == 3
    assert all(f[1] == payload for f in frames)


def test_encode_set_rate_roundtrip():
    frame = proto.encode_set_rate(10000, 40000)
    parser = proto.FrameParser()
    (msg_id, payload), = parser.feed(frame)
    assert msg_id == proto.MSG_SET_RATE
    assert struct.unpack("<IIB", payload)[:2] == (10000, 40000)


def test_encode_tare_persist_flag():
    parser = proto.FrameParser()
    (_, payload), = parser.feed(proto.encode_tare(persist=True))
    assert payload[0] == 1
    (_, payload), = parser.feed(proto.encode_tare(persist=False))
    assert payload[0] == 0


def test_oversized_length_byte_does_not_stall_parser():
    """A corrupt length field must not swallow the following good frame."""
    good = proto.encode_frame(proto.MSG_IMU, build_imu_payload())
    junk = bytes([proto.SYNC0, proto.SYNC1, proto.MSG_IMU, 255])
    parser = proto.FrameParser()
    frames = list(parser.feed(junk + good))
    assert len(frames) == 1


def test_decode_trace_entries():
    payload = bytes([2]) \
        + bytes([1]) + struct.pack("<H", 5) + b"\x01\x00\x00\x00" \
        + bytes([0]) + struct.pack("<H", 276) + b"\x00\x01\x02\x03"
    entries = proto.decode_trace(payload)
    assert len(entries) == 2
    assert entries[0].channel == 1
    assert entries[0].channel_text == "executable"
    assert entries[0].length == 5
    assert entries[0].data[0] == 0x01
    assert entries[1].channel_text == "command/advertisement"


def test_decode_trace_flags_an_impossible_channel():
    """The garbage a floating MISO produces must be labelled as invalid."""
    payload = bytes([1]) + bytes([63]) + struct.pack("<H", 17792) + b"\xff\x0f\xff\xff"
    (entry,) = proto.decode_trace(payload)
    assert "invalid" in entry.channel_text


def test_decode_scan_results():
    payload = bytes([2]) \
        + bytes([3]) + struct.pack("<H", 3000) + bytes([0, 0, 0xFF]) + struct.pack("<H", 0) \
        + bytes([0]) + struct.pack("<H", 750) + bytes([4, 1, 1]) + struct.pack("<H", 5)
    entries = proto.decode_scan(payload)
    assert len(entries) == 2
    assert entries[0].mode == 3 and entries[0].clock_khz == 3000
    assert not entries[0].got_reset_msg
    assert entries[1].mode == 0 and entries[1].clock_khz == 750
    assert entries[1].got_reset_msg and entries[1].valid_packets == 4


def test_decode_scan_truncated_payload_does_not_raise():
    payload = bytes([4]) + bytes([0]) + struct.pack("<H", 3000) + bytes([0, 0, 0])
    assert proto.decode_scan(payload) == []


def test_pin_probe_bus_works_when_cs_makes_miso_driven():
    """CS asserted pulls MISO away from our bias -> the slave is answering."""
    probe = proto.decode_pins(bytes([1, 0, 0, 0, 0, 0]))
    assert probe.miso_driven_when_selected
    assert probe.miso_idle_text.startswith("floating")
    assert probe.miso_selected_text == "driven LOW"
    assert "SHTP framing" in probe.diagnosis


def test_pin_probe_imu_alive_but_not_an_spi_slave():
    """INT driven, MISO floating even with CS asserted."""
    probe = proto.decode_pins(bytes([1, 0, 1, 0, 0, 0]))
    assert not probe.miso_driven_when_selected
    assert probe.int_driven
    assert "not answering as an SPI slave" in probe.diagnosis


def test_pin_probe_nothing_driven_means_imu_absent():
    probe = proto.decode_pins(bytes([1, 0, 1, 0, 1, 0]))
    assert not probe.miso_driven_when_selected
    assert not probe.int_driven
    assert "unpowered" in probe.diagnosis


def test_decode_pins_rejects_short_payload():
    with pytest.raises(ValueError):
        proto.decode_pins(b"\x01\x00")


def test_serial_link_routes_socket_urls_over_tcp():
    """A bridged board (socket://) must be usable in place of a device path."""
    from bno086_imu_driver.serial_link import SerialLink

    link = SerialLink("socket://127.0.0.1:1", 921600)
    # Nothing is listening there. The failure must come from the TCP connect,
    # which proves the URL was handled as a socket rather than as a file path.
    with pytest.raises(Exception) as exc:
        link.open()
    assert "connection refused" in str(exc.value).lower()
    assert not link.is_open


def test_encode_tare_carries_the_axis_bitmap():
    parser = proto.FrameParser()
    (_, payload), = parser.feed(proto.encode_tare(persist=True, axes=proto.TARE_AXIS_Z))
    assert payload == bytes([1, proto.TARE_AXIS_Z])
    (_, payload), = parser.feed(proto.encode_tare())
    assert payload == bytes([0, proto.TARE_AXIS_ALL])


def test_command_response_reports_success_and_failure():
    ok = proto.decode_cmdresp(bytes([1, proto.SH2_CMD_TARE, 7, 0, 0, 0, 0]))
    assert ok.ok and "tare" in ok.text and "OK" in ok.text

    bad = proto.decode_cmdresp(bytes([1, proto.SH2_CMD_TARE, 8, 3, 0, 0, 0]))
    assert not bad.ok and "FAILED" in bad.text

    none = proto.decode_cmdresp(bytes(7))
    assert not none.valid and "no command response" in none.text


def test_set_rate_carries_the_rotation_vector_choice():
    parser = proto.FrameParser()
    (_, payload), = parser.feed(proto.encode_set_rate(10000, 40000, proto.RV_GAME))
    assert struct.unpack("<IIB", payload) == (10000, 40000, proto.RV_GAME)
    # Default stays on the magnetometer-referenced vector.
    (_, payload), = parser.feed(proto.encode_set_rate(10000, 40000))
    assert struct.unpack("<IIB", payload)[2] == proto.RV_MAGNETIC


def test_auto_tare_parameter_survives_yaml_mangling():
    """`-p auto_tare:=off` reaches us as the boolean False, not the string."""
    from bno086_imu_driver.imu_node import Bno086Node

    norm = Bno086Node._normalise_auto_tare
    assert norm(False) == "off"      # YAML turns "off" and "no" into False
    assert norm("off") == "off"
    assert norm(None) == "off"
    assert norm("yaw") == "yaw"
    assert norm("all") == "all"
    assert norm(True) == "all"       # ...and "on"/"yes" into True
    assert norm("nonsense") == "off"
