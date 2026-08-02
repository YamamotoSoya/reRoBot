"""Serial transport for the BNO086 ROS2 board."""

from __future__ import annotations

import threading
from typing import Callable, Optional

import serial

from .protocol import FrameParser


class SerialLink:
    """Background reader that turns a serial stream into decoded frames.

    The board appears as a USB CDC device (/dev/ttyACM*), where the baud rate
    is ignored, or as a plain UART (/dev/ttyUSB*) when the firmware is built
    with HOST_LINK_UART.

    ``port`` also accepts any pyserial URL, notably
    ``socket://<host>:<port>``. That is how the driver reaches a board plugged
    into a machine it is not running on - a macOS host bridging its USB CDC
    port into this container, for instance. See tools/serial_bridge.py.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 921600,
        on_frame: Optional[Callable[[int, bytes], None]] = None,
        on_error: Optional[Callable[[Exception], None]] = None,
    ) -> None:
        self._port_name = port
        self._baudrate = baudrate
        self._on_frame = on_frame
        self._on_error = on_error

        self._serial: Optional[serial.Serial] = None
        self._parser = FrameParser()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._failed = threading.Event()
        self._write_lock = threading.Lock()

    # ------------------------------------------------------------- lifecycle
    def open(self) -> None:
        # serial_for_url() handles both plain device paths and URLs such as
        # socket:// , so the same code path serves local and bridged boards.
        self._serial = serial.serial_for_url(
            self._port_name,
            baudrate=self._baudrate,
            timeout=0.1,
            write_timeout=1.0,
        )
        # Discard whatever accumulated while the port was closed, so the
        # parser does not have to resynchronise through stale bytes.
        self._serial.reset_input_buffer()
        self._stop.clear()
        self._failed.clear()
        self._thread = threading.Thread(target=self._run, name="bno086-serial", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def is_healthy(self) -> bool:
        """False once the reader has given up on the port.

        pyserial keeps reporting is_open after the underlying device has gone
        away - which is exactly what happens when the board is reset with S1
        and the USB device re-enumerates - so the reader thread's verdict is
        what callers should watch.
        """
        return self._serial is not None and not self._failed.is_set()

    # ---------------------------------------------------------------- io
    def write(self, data: bytes) -> bool:
        """Send a pre-encoded frame. Returns False if the port is gone."""
        if self._serial is None:
            return False
        try:
            with self._write_lock:
                self._serial.write(data)
            return True
        except (serial.SerialException, OSError) as exc:
            self._failed.set()
            self._report(exc)
            return False

    @property
    def crc_errors(self) -> int:
        return self._parser.crc_errors

    @property
    def resyncs(self) -> int:
        return self._parser.resyncs

    # -------------------------------------------------------------- internal
    def _report(self, exc: Exception) -> None:
        if self._on_error is not None:
            self._on_error(exc)

    #: A readable fd that yields nothing is normal on a pty and transient on a
    #: real port; only a run of them means the board is really gone.
    MAX_CONSECUTIVE_ERRORS = 20

    def _run(self) -> None:
        errors = 0

        while not self._stop.is_set():
            try:
                # read() blocks up to `timeout`; draining in_waiting first
                # keeps latency low when a burst has already arrived.
                chunk = self._serial.read(max(1, self._serial.in_waiting))
                errors = 0
            except (serial.SerialException, OSError, TypeError) as exc:
                errors += 1
                if errors >= self.MAX_CONSECUTIVE_ERRORS or not self.is_open:
                    self._failed.set()
                    self._report(exc)
                    return
                self._stop.wait(0.05)
                continue

            if not chunk:
                continue

            for msg_id, payload in self._parser.feed(chunk):
                if self._on_frame is not None:
                    self._on_frame(msg_id, payload)
