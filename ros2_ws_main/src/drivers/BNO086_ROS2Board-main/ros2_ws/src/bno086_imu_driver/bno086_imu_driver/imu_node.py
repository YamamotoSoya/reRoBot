"""ROS 2 driver node for the BNO086_ROS2Board."""

from __future__ import annotations

import math
import threading

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, MagneticField
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from . import protocol as proto
from .device_clock import DeviceClock
from .serial_link import SerialLink

# The BNO086 status byte is 0 (unreliable) .. 3 (high accuracy).
ACCURACY_NAMES = ("unreliable", "low", "medium", "high")


class Bno086Node(Node):
    def __init__(self, **kwargs) -> None:
        # kwargs is forwarded to rclpy's Node so tests can inject
        # parameter_overrides (notably a pty in place of the serial port).
        super().__init__("bno086_imu_driver", **kwargs)

        self._declare_parameters()
        p = self.get_parameter

        self._frame_id = p("frame_id").value
        self._use_device_time = p("use_device_time").value
        self._publish_tf = p("publish_tf").value
        self._parent_frame = p("parent_frame").value
        self._require_calibrated = p("require_calibrated_orientation").value

        gyro_sd = p("angular_velocity_stddev").value
        acc_sd = p("linear_acceleration_stddev").value
        mag_sd = p("magnetic_field_stddev").value
        self._gyro_var = gyro_sd * gyro_sd
        self._accel_var = acc_sd * acc_sd
        self._mag_var = (mag_sd * 1e-6) ** 2
        self._min_orientation_stddev = p("min_orientation_stddev").value

        # Precompute the mounting rotation: q_mount rotates the orientation
        # into the board frame, and (cos, sin) rotate the measured vectors.
        yaw = math.radians(float(p("mount_yaw_deg").value))
        self._mount_qs = math.sin(yaw / 2.0)
        self._mount_qc = math.cos(yaw / 2.0)
        self._mount_cos = math.cos(yaw)
        self._mount_sin = math.sin(yaw)
        self._mount_identity = abs(self._mount_qs) < 1e-12

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self._pub_imu = self.create_publisher(Imu, "imu/data", sensor_qos)
        self._pub_mag = self.create_publisher(MagneticField, "imu/mag", sensor_qos)
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._tf = TransformBroadcaster(self) if self._publish_tf else None

        # Not "_clock": rclpy's Node uses that attribute for its own ROS clock.
        self._dev_clock = DeviceClock()
        self._lock = threading.Lock()
        self._imu_count = 0
        self._mag_count = 0
        self._last_imu_ns = 0
        self._last_status = None
        self._device_resets = 0
        self._last_diag_count = 0

        self._link = SerialLink(
            port=p("port").value,
            baudrate=p("baudrate").value,
            on_frame=self._on_frame,
            on_error=self._on_serial_error,
        )

        self.create_service(Trigger, "~/tare", self._srv_tare)
        self.create_service(Trigger, "~/tare_yaw", self._srv_tare_yaw)
        self.create_service(Trigger, "~/save_calibration", self._srv_save)
        self.create_service(Trigger, "~/reset", self._srv_reset)

        self._auto_tare_mode = self._normalise_auto_tare(p("auto_tare").value)
        self._auto_tare_delay = float(p("auto_tare_delay").value)
        self._auto_tare_deadline = None

        self._reconnect_attempts = 0
        self._connect()

        self.create_timer(1.0, self._on_diagnostics)
        self.create_timer(1.0, self._maintain_connection)
        self.create_timer(0.25, self._check_auto_tare)

    # ------------------------------------------------------------ parameters
    def _declare_parameters(self) -> None:
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("use_device_time", True)
        self.declare_parameter("imu_rate_hz", 100.0)
        self.declare_parameter("mag_rate_hz", 25.0)
        # Datasheet noise densities integrated over the default bandwidth;
        # override per unit once you have characterised yours.
        self.declare_parameter("angular_velocity_stddev", 0.005)     # rad/s
        self.declare_parameter("linear_acceleration_stddev", 0.05)   # m/s^2
        self.declare_parameter("magnetic_field_stddev", 0.5)         # microtesla
        self.declare_parameter("min_orientation_stddev", 0.01)       # rad
        self.declare_parameter("require_calibrated_orientation", False)
        # The magnetometer-referenced rotation vector gives an absolute compass
        # heading, but it is unusable near motors or steel. The game rotation
        # vector drops the magnetometer: yaw starts at 0 every boot and drifts
        # slowly instead.
        self.declare_parameter("use_game_rotation_vector", False)
        # The BNO086 (U4) is placed at 180 degrees on this PCB, so the chip's
        # +X/+Y point along the board's -X/-Y. This rotates everything about Z
        # into the board frame, which is what frame_id then refers to.
        self.declare_parameter("mount_yaw_deg", 180.0)
        # "off" | "yaw" | "all" - zero the orientation automatically once the
        # stream comes up, so a reset makes the board's current pose the
        # origin. "yaw" keeps roll and pitch referenced to gravity.
        #
        # Dynamically typed on purpose: `-p auto_tare:=off` on the command
        # line is parsed as YAML, which turns "off" into the boolean False and
        # would otherwise be rejected as a type mismatch.
        self.declare_parameter(
            "auto_tare", "off",
            ParameterDescriptor(dynamic_typing=True,
                                description="off | yaw | all"))
        # Seconds to let the fusion settle before taring. Taring immediately
        # after a reset would capture an orientation that has not converged.
        self.declare_parameter("auto_tare_delay", 2.0)

    # -------------------------------------------------------------- transport
    def _connect(self) -> bool:
        port = self.get_parameter("port").value
        try:
            self._link.open()
        except Exception as exc:
            self.get_logger().error(f"could not open {port}: {exc}")
            return False

        self.get_logger().info(f"opened {port}")
        self._push_rates()
        self._arm_auto_tare()
        return True

    def _maintain_connection(self) -> None:
        """Reopen the port after the board disappears.

        Pressing S1 (RESET) reboots the MCU, so the USB CDC device drops off
        the bus and comes back a moment later. Without this the node would
        stay silent until it was restarted by hand.
        """
        if self._link.is_healthy:
            self._reconnect_attempts = 0
            return

        self._link.close()
        try:
            self._link.open()
        except Exception as exc:
            self._reconnect_attempts += 1
            # The device node is gone for a second or two after a reset; only
            # start complaining if it stays away.
            if self._reconnect_attempts in (5, 30) or self._reconnect_attempts % 60 == 0:
                self.get_logger().warn(
                    f"still waiting for {self.get_parameter('port').value}"
                    f" ({self._reconnect_attempts}s): {exc}"
                )
            return

        # The board restarted, so its microsecond counter did too. Throwing
        # the old estimate away avoids a huge jump in the published stamps.
        self._dev_clock = DeviceClock()
        self._reconnect_attempts = 0
        self.get_logger().info("reconnected")
        self._push_rates()
        self._arm_auto_tare()

    @staticmethod
    def _normalise_auto_tare(value) -> str:
        """Accept off/yaw/all however YAML mangled it on the command line."""
        if value is None or value is False:
            return "off"
        if value is True:
            return "all"
        text = str(value).strip().lower()
        if text in ("yaw", "z"):
            return "yaw"
        if text in ("all", "true", "xyz"):
            return "all"
        return "off"

    def _arm_auto_tare(self) -> None:
        """Queue a tare for once the stream has settled."""
        if self._auto_tare_mode not in ("yaw", "all"):
            return
        self._auto_tare_deadline = (
            self.get_clock().now().nanoseconds + int(self._auto_tare_delay * 1e9))

    def _check_auto_tare(self) -> None:
        if self._auto_tare_deadline is None:
            return
        now = self.get_clock().now().nanoseconds
        if now < self._auto_tare_deadline:
            return

        with self._lock:
            streaming = self._last_imu_ns != 0 and (now - self._last_imu_ns) < 500_000_000
        if not streaming:
            return           # keep waiting; the board is not sending yet

        axes = proto.TARE_AXIS_Z if self._auto_tare_mode == "yaw" else proto.TARE_AXIS_ALL
        if self._link.write(proto.encode_tare(persist=False, axes=axes)):
            self._auto_tare_deadline = None
            self.get_logger().info(
                f"auto tare ({self._auto_tare_mode}): current pose is now the origin")

    def _push_rates(self) -> None:
        imu_hz = float(self.get_parameter("imu_rate_hz").value)
        mag_hz = float(self.get_parameter("mag_rate_hz").value)
        imu_us = int(round(1e6 / imu_hz)) if imu_hz > 0 else 0
        mag_us = int(round(1e6 / mag_hz)) if mag_hz > 0 else 0
        game = bool(self.get_parameter("use_game_rotation_vector").value)
        rv = proto.RV_GAME if game else proto.RV_MAGNETIC
        if self._link.write(proto.encode_set_rate(imu_us, mag_us, rv)):
            self.get_logger().info(
                f"requested {imu_hz:g} Hz IMU / {mag_hz:g} Hz magnetometer, "
                f"{'game' if game else 'magnetic'} rotation vector"
            )

    def _on_serial_error(self, exc: Exception) -> None:
        self.get_logger().error(f"serial error: {exc}")

    # --------------------------------------------------------------- frames
    def _on_frame(self, msg_id: int, payload: bytes) -> None:
        try:
            if msg_id == proto.MSG_IMU:
                self._handle_imu(proto.decode_imu(payload))
            elif msg_id == proto.MSG_MAG:
                self._handle_mag(proto.decode_mag(payload))
            elif msg_id == proto.MSG_STATUS:
                self._handle_status(proto.decode_status(payload))
            elif msg_id == proto.MSG_CMDRESP:
                self._handle_cmdresp(proto.decode_cmdresp(payload))
        except ValueError as exc:
            self.get_logger().warn(f"malformed frame 0x{msg_id:02X}: {exc}")

    def _to_board(self, quat, *vectors):
        """Rotate a sensor-frame sample into the board frame.

        The orientation is post-multiplied (a change of the body frame), while
        measured vectors are rotated by the inverse - they are expressed in the
        sensor frame and we want them in the board frame.
        """
        if self._mount_identity:
            return quat, vectors

        x, y, z, w = quat
        s, c = self._mount_qs, self._mount_qc
        quat = (x * c + y * s, -x * s + y * c, w * s + z * c, w * c - z * s)

        cs, sn = self._mount_cos, self._mount_sin
        out = tuple((v[0] * cs + v[1] * sn, -v[0] * sn + v[1] * cs, v[2])
                    for v in vectors)
        return quat, out

    def _stamp(self, device_us: int):
        now = self.get_clock().now()
        if not self._use_device_time:
            return now.to_msg()
        ns = self._dev_clock.to_ros_ns(device_us, now.nanoseconds)
        return rclpy.time.Time(nanoseconds=ns).to_msg()

    def _handle_imu(self, s: proto.ImuSample) -> None:
        msg = Imu()
        msg.header.stamp = self._stamp(s.device_us)
        msg.header.frame_id = self._frame_id

        quat, (gyro, accel) = self._to_board(s.quat, s.gyro, s.accel)

        if s.quat_valid and not (self._require_calibrated and s.quat_status == 0):
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat
            # The sensor reports its own orientation uncertainty as an angle;
            # use it directly as an isotropic covariance.
            sd = max(s.quat_accuracy_rad, self._min_orientation_stddev)
            var = sd * sd
            msg.orientation_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
        else:
            # REP-145: leading -1 marks the field as not supplied.
            msg.orientation.w = 1.0
            msg.orientation_covariance = [-1.0] + [0.0] * 8

        if s.gyro_valid:
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
            msg.angular_velocity_covariance = [
                self._gyro_var, 0.0, 0.0, 0.0, self._gyro_var, 0.0, 0.0, 0.0, self._gyro_var
            ]
        else:
            msg.angular_velocity_covariance = [-1.0] + [0.0] * 8

        if s.accel_valid:
            (msg.linear_acceleration.x,
             msg.linear_acceleration.y,
             msg.linear_acceleration.z) = accel
            msg.linear_acceleration_covariance = [
                self._accel_var, 0.0, 0.0, 0.0, self._accel_var, 0.0, 0.0, 0.0, self._accel_var
            ]
        else:
            msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8

        self._pub_imu.publish(msg)

        if self._tf is not None and s.quat_valid:
            t = TransformStamped()
            t.header = msg.header
            t.header.frame_id = self._parent_frame
            t.child_frame_id = self._frame_id
            t.transform.rotation = msg.orientation
            self._tf.sendTransform(t)

        with self._lock:
            self._imu_count += 1
            self._last_imu_ns = self.get_clock().now().nanoseconds
            self._last_quat_status = s.quat_status

    def _handle_mag(self, s: proto.MagSample) -> None:
        msg = MagneticField()
        msg.header.stamp = self._stamp(s.device_us)
        msg.header.frame_id = self._frame_id
        # sensor_msgs/MagneticField is in tesla; the sensor reports microtesla.
        _, (field,) = self._to_board((0.0, 0.0, 0.0, 1.0), s.field)
        msg.magnetic_field.x = field[0] * 1e-6
        msg.magnetic_field.y = field[1] * 1e-6
        msg.magnetic_field.z = field[2] * 1e-6
        msg.magnetic_field_covariance = [
            self._mag_var, 0.0, 0.0, 0.0, self._mag_var, 0.0, 0.0, 0.0, self._mag_var
        ]
        self._pub_mag.publish(msg)
        with self._lock:
            self._mag_count += 1

    def _handle_cmdresp(self, r: proto.CommandResponse) -> None:
        # Only speak up when the answer changes, otherwise this repeats at 1 Hz.
        key = (r.command, r.seq, r.r)
        if key == getattr(self, "_last_cmdresp_key", None):
            return
        self._last_cmdresp_key = key
        if r.ok:
            self.get_logger().info(f"device acknowledged {r.text}")
        else:
            self.get_logger().warn(f"device rejected {r.text}")

    def _handle_status(self, s: proto.StatusSample) -> None:
        if s.proto_version != proto.PROTO_VERSION:
            self.get_logger().warn(
                f"firmware speaks protocol v{s.proto_version}, driver expects "
                f"v{proto.PROTO_VERSION}"
            )
        with self._lock:
            if self._last_status is not None and s.reset_count > self._last_status.reset_count:
                self.get_logger().warn(
                    f"BNO086 reset itself ({s.reset_count} total) - check power integrity"
                )
            self._last_status = s

    # ---------------------------------------------------------------- services
    def _trigger(self, frame: bytes, what: str, response: Trigger.Response) -> Trigger.Response:
        if not self._link.is_open:
            response.success = False
            response.message = "serial port is not open"
            return response
        ok = self._link.write(frame)
        response.success = ok
        response.message = f"{what} sent" if ok else f"failed to send {what}"
        if ok:
            self.get_logger().info(f"{what} requested")
        return response

    def _srv_tare(self, request, response):
        return self._trigger(proto.encode_tare(persist=True), "tare (all axes)", response)

    def _srv_tare_yaw(self, request, response):
        """Zero the heading only, leaving roll and pitch on gravity."""
        return self._trigger(
            proto.encode_tare(persist=True, axes=proto.TARE_AXIS_Z),
            "tare (yaw only)", response)

    def _srv_save(self, request, response):
        return self._trigger(proto.encode_frame(proto.MSG_SAVE_DCD), "calibration save", response)

    def _srv_reset(self, request, response):
        result = self._trigger(proto.encode_frame(proto.MSG_RESET_IMU), "IMU reset", response)
        if result.success:
            # A reset clears any tare, so re-apply it if auto tare is on.
            self._arm_auto_tare()
        return result

    # ------------------------------------------------------------- diagnostics
    def _on_diagnostics(self) -> None:
        with self._lock:
            count = self._imu_count
            mag_count = self._mag_count
            last_ns = self._last_imu_ns
            status = self._last_status
            quat_status = getattr(self, "_last_quat_status", 0)

        rate = count - self._last_diag_count
        self._last_diag_count = count

        st = DiagnosticStatus()
        st.name = "bno086_imu_driver: IMU"
        st.hardware_id = self.get_parameter("port").value

        age_s = (self.get_clock().now().nanoseconds - last_ns) / 1e9 if last_ns else float("inf")

        if not self._link.is_healthy:
            st.level = DiagnosticStatus.ERROR
            st.message = "disconnected - waiting for the board"
        elif age_s > 2.0:
            st.level = DiagnosticStatus.ERROR
            st.message = "no data from board"
        elif quat_status == 0:
            st.level = DiagnosticStatus.WARN
            st.message = "orientation uncalibrated - rotate the board through all axes"
        else:
            st.level = DiagnosticStatus.OK
            st.message = f"streaming ({ACCURACY_NAMES[quat_status]} accuracy)"

        st.values = [
            KeyValue(key="imu_rate_hz", value=str(rate)),
            KeyValue(key="imu_frames", value=str(count)),
            KeyValue(key="mag_frames", value=str(mag_count)),
            KeyValue(key="crc_errors", value=str(self._link.crc_errors)),
            KeyValue(key="resyncs", value=str(self._link.resyncs)),
            KeyValue(key="orientation_accuracy", value=ACCURACY_NAMES[quat_status]),
        ]
        if status is not None:
            st.values.append(KeyValue(key="device_resets", value=str(status.reset_count)))
            st.values.append(KeyValue(key="imu_ready", value=str(status.imu_ready)))

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self._pub_diag.publish(arr)

    def destroy_node(self) -> bool:
        self._link.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Bno086Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
