"""End-to-end test: a simulated firmware byte stream must come out as ROS messages.

A pseudo-terminal stands in for the board's serial port, so the whole chain -
pyserial, the frame parser, the Q-format scaling and the publisher - is
exercised without hardware.
"""

import os
import pty
import select
import struct
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, MagneticField

from bno086_imu_driver import protocol as proto
from bno086_imu_driver.imu_node import Bno086Node


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def serial_pair():
    """A pty; the test writes to the master, the node reads the slave."""
    master, slave = pty.openpty()
    yield master, os.ttyname(slave)
    os.close(master)
    try:
        os.close(slave)
    except OSError:
        pass


def imu_frame(t_us, qi=0, qj=0, qk=0, qr=1 << 14, q_acc=0,
              gx=0, gy=0, gz=0, ax=0, ay=0, az=0, status=0, flags=0x07):
    payload = struct.pack("<I11hBB", t_us, qi, qj, qk, qr, q_acc,
                          gx, gy, gz, ax, ay, az, status, flags)
    return proto.encode_frame(proto.MSG_IMU, payload)


def collect(node, sub_node, messages, count, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline and len(messages) < count:
        rclpy.spin_once(sub_node, timeout_sec=0.05)
        rclpy.spin_once(node, timeout_sec=0.0)
    return messages


def make_node(port, mount_yaw=0.0):
    # Most tests check the sensor-frame path, so the mounting rotation is
    # switched off unless a test is specifically about it.
    return Bno086Node(parameter_overrides=[
        Parameter("port", value=port),
        Parameter("use_device_time", value=True),
        Parameter("mount_yaw_deg", value=mount_yaw),
    ])


def test_imu_frames_become_imu_messages(ros, serial_pair):
    master, slave_name = serial_pair
    node = make_node(slave_name)
    sub_node = rclpy.create_node("test_sub")

    received = []
    sub_node.create_subscription(
        Imu, "/imu/data", received.append, QoSPresetProfiles.SENSOR_DATA.value
    )

    try:
        # 1 g on Z (Q8), 1 rad/s about X (Q9), identity orientation.
        for i in range(5):
            os.write(master, imu_frame(
                t_us=i * 10_000,
                az=int(9.80665 * 256), gx=512, status=0b110011,
            ))
        collect(node, sub_node, received, 5)

        assert len(received) >= 3
        msg = received[0]
        assert msg.header.frame_id == "imu_link"
        assert msg.orientation.w == pytest.approx(1.0)
        assert msg.linear_acceleration.z == pytest.approx(9.80665, abs=1e-2)
        assert msg.angular_velocity.x == pytest.approx(1.0)
        # status 0b110011 -> quat=3, gyro=0b00... check the accuracy mapping
        # produced a finite covariance rather than the -1 "unknown" marker.
        assert msg.orientation_covariance[0] > 0.0
        assert msg.angular_velocity_covariance[0] > 0.0
    finally:
        sub_node.destroy_node()
        node.destroy_node()


def test_invalid_flags_mark_fields_unknown(ros, serial_pair):
    master, slave_name = serial_pair
    node = make_node(slave_name)
    sub_node = rclpy.create_node("test_sub2")

    received = []
    sub_node.create_subscription(
        Imu, "/imu/data", received.append, QoSPresetProfiles.SENSOR_DATA.value
    )

    try:
        # Only the gyroscope is fresh in this frame.
        for i in range(5):
            os.write(master, imu_frame(t_us=i * 10_000, flags=proto.FLAG_GYRO_VALID))
        collect(node, sub_node, received, 5)

        assert received
        msg = received[0]
        # REP-145: a leading -1 means "this field is not supplied".
        assert msg.orientation_covariance[0] == -1.0
        assert msg.linear_acceleration_covariance[0] == -1.0
        assert msg.angular_velocity_covariance[0] > 0.0
    finally:
        sub_node.destroy_node()
        node.destroy_node()


def test_magnetometer_frames_are_converted_to_tesla(ros, serial_pair):
    master, slave_name = serial_pair
    node = make_node(slave_name)
    sub_node = rclpy.create_node("test_sub3")

    received = []
    sub_node.create_subscription(
        MagneticField, "/imu/mag", received.append, QoSPresetProfiles.SENSOR_DATA.value
    )

    try:
        # 16 in Q4 = 1.0 microtesla = 1e-6 T
        payload = struct.pack("<I3hBB", 1000, 16, -32, 0, 3, 0)
        for _ in range(5):
            os.write(master, proto.encode_frame(proto.MSG_MAG, payload))
        collect(node, sub_node, received, 5)

        assert received
        assert received[0].magnetic_field.x == pytest.approx(1e-6)
        assert received[0].magnetic_field.y == pytest.approx(-2e-6)
    finally:
        sub_node.destroy_node()
        node.destroy_node()


def test_stream_with_corruption_still_delivers_good_frames(ros, serial_pair):
    """Dropped USB bytes must cost one sample, not the whole stream."""
    master, slave_name = serial_pair
    node = make_node(slave_name)
    sub_node = rclpy.create_node("test_sub4")

    received = []
    sub_node.create_subscription(
        Imu, "/imu/data", received.append, QoSPresetProfiles.SENSOR_DATA.value
    )

    try:
        good = imu_frame(t_us=0, az=256)
        truncated = good[:10]              # a frame cut in half by a lost read
        os.write(master, truncated + b"\x00\xFF\xAA" + good * 4)
        collect(node, sub_node, received, 4)

        assert len(received) >= 3
        assert received[-1].linear_acceleration.z == pytest.approx(1.0)
    finally:
        sub_node.destroy_node()
        node.destroy_node()


def test_tare_service_writes_a_frame_to_the_port(ros, serial_pair):
    """The service must put a well-formed command frame on the wire."""
    master, slave_name = serial_pair
    node = make_node(slave_name)

    try:
        from std_srvs.srv import Trigger

        response = node._srv_tare(Trigger.Request(), Trigger.Response())
        assert response.success

        time.sleep(0.2)
        data = os.read(master, 4096)

        parser = proto.FrameParser()
        frames = list(parser.feed(data))
        ids = [f[0] for f in frames]
        # The node also pushes a SET_RATE frame when it connects.
        assert proto.MSG_TARE in ids
    finally:
        node.destroy_node()


def test_node_reconnects_after_the_board_disappears(ros, serial_pair):
    """Pressing S1 drops the USB device; the node must come back on its own."""
    master, slave_name = serial_pair
    node = make_node(slave_name)
    sub_node = rclpy.create_node("test_sub5")

    received = []
    sub_node.create_subscription(
        Imu, "/imu/data", received.append, QoSPresetProfiles.SENSOR_DATA.value
    )

    try:
        os.write(master, imu_frame(t_us=0, az=256) * 3)
        collect(node, sub_node, received, 3)
        assert received, "no data before the simulated reset"

        # Tear the port out from under the reader, the way a USB reset does.
        node._link._serial.close()
        deadline = time.time() + 3.0
        while time.time() < deadline and node._link.is_healthy:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not node._link.is_healthy, "the dead port went unnoticed"

        # The reconnect timer should reopen the same pty and resume.
        received.clear()
        deadline = time.time() + 6.0
        while time.time() < deadline and not node._link.is_healthy:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert node._link.is_healthy, "never reconnected"

        os.write(master, imu_frame(t_us=0, az=256) * 3)
        collect(node, sub_node, received, 3, timeout=3.0)
        assert received, "no data after reconnecting"
    finally:
        sub_node.destroy_node()
        node.destroy_node()


def make_node_auto_tare(port, mode="all", delay=0.2):
    return Bno086Node(parameter_overrides=[
        Parameter("port", value=port),
        Parameter("auto_tare", value=mode),
        Parameter("auto_tare_delay", value=delay),
        Parameter("mount_yaw_deg", value=0.0),
    ])


def drain_tare_frames(master):
    """Return the tare frames the node has written to the port so far.

    The read has to be guarded by select(): a pty master with nothing pending
    blocks forever otherwise.
    """
    time.sleep(0.2)
    data = b""
    while select.select([master], [], [], 0)[0]:
        chunk = os.read(master, 8192)
        if not chunk:
            break
        data += chunk
    if not data:
        return []
    parser = proto.FrameParser()
    return [pl for mid, pl in parser.feed(data) if mid == proto.MSG_TARE]


def test_auto_tare_fires_once_the_stream_is_alive(ros, serial_pair):
    master, slave_name = serial_pair
    node = make_node_auto_tare(slave_name, mode="all")

    try:
        # Nothing should be tared while the board is silent.
        deadline = time.time() + 1.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not drain_tare_frames(master), "tared before any data arrived"

        # Once frames flow, the pose must be zeroed.
        for i in range(5):
            os.write(master, imu_frame(t_us=i * 10_000, az=256))
        deadline = time.time() + 3.0
        tares = []
        while time.time() < deadline and not tares:
            rclpy.spin_once(node, timeout_sec=0.05)
            tares = drain_tare_frames(master)

        assert tares, "auto tare never fired"
        assert tares[0][1] == proto.TARE_AXIS_ALL
    finally:
        node.destroy_node()


def test_auto_tare_yaw_mode_only_tares_z(ros, serial_pair):
    master, slave_name = serial_pair
    node = make_node_auto_tare(slave_name, mode="yaw")

    try:
        for i in range(5):
            os.write(master, imu_frame(t_us=i * 10_000, az=256))
        deadline = time.time() + 3.0
        tares = []
        while time.time() < deadline and not tares:
            rclpy.spin_once(node, timeout_sec=0.05)
            tares = drain_tare_frames(master)

        assert tares, "auto tare never fired"
        assert tares[0][1] == proto.TARE_AXIS_Z
    finally:
        node.destroy_node()


def test_auto_tare_off_by_default(ros, serial_pair):
    master, slave_name = serial_pair
    node = make_node(slave_name)

    try:
        for i in range(5):
            os.write(master, imu_frame(t_us=i * 10_000, az=256))
        deadline = time.time() + 2.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not drain_tare_frames(master), "tared even though auto_tare is off"
    finally:
        node.destroy_node()


def test_mount_yaw_180_flips_x_and_y(ros, serial_pair):
    """U4 sits at 180 deg on this PCB, so the board frame negates X and Y."""
    master, slave_name = serial_pair
    node = make_node(slave_name, mount_yaw=180.0)
    sub_node = rclpy.create_node("test_sub_mount")

    received = []
    sub_node.create_subscription(
        Imu, "/imu/data", received.append, QoSPresetProfiles.SENSOR_DATA.value
    )

    try:
        # 1 rad/s about sensor X, 2 m/s^2 along sensor Y, 1 g along sensor Z.
        for i in range(5):
            os.write(master, imu_frame(t_us=i * 10_000, gx=512, ay=512, az=256))
        collect(node, sub_node, received, 5)

        assert received
        msg = received[0]
        assert msg.angular_velocity.x == pytest.approx(-1.0, abs=1e-3)
        assert msg.linear_acceleration.y == pytest.approx(-2.0, abs=1e-3)
        # Z is the rotation axis, so it must be untouched.
        assert msg.linear_acceleration.z == pytest.approx(1.0, abs=1e-3)
    finally:
        sub_node.destroy_node()
        node.destroy_node()


def test_mount_yaw_180_offsets_the_heading(ros, serial_pair):
    """An identity sensor quaternion means the board is yawed 180 deg."""
    master, slave_name = serial_pair
    node = make_node(slave_name, mount_yaw=180.0)
    sub_node = rclpy.create_node("test_sub_mount2")

    received = []
    sub_node.create_subscription(
        Imu, "/imu/data", received.append, QoSPresetProfiles.SENSOR_DATA.value
    )

    try:
        for i in range(5):
            os.write(master, imu_frame(t_us=i * 10_000, az=256))
        collect(node, sub_node, received, 5)

        assert received
        q = received[0].orientation
        # Rz(180) is (0, 0, 1, 0) up to sign.
        assert abs(q.z) == pytest.approx(1.0, abs=1e-3)
        assert abs(q.w) == pytest.approx(0.0, abs=1e-3)
    finally:
        sub_node.destroy_node()
        node.destroy_node()
