"""Mapping from the board's free-running clock onto ROS time."""

from __future__ import annotations

from typing import Optional


class DeviceClock:
    """Maps the board's 32-bit microsecond counter onto ROS time.

    The counter wraps every ~71 minutes, so wraps are unrolled here.

    The offset to ROS time is tracked with a minimum filter: of all samples,
    the one that took the least time to arrive is the least biased by
    scheduling and USB latency, so the smallest observed
    ``now - device_time`` is the best estimate. Taking the minimum alone
    would latch onto one lucky early sample forever, so the estimate is also
    allowed to creep upward at a bounded rate. That covers the case where the
    board's crystal runs slow relative to the host clock; the opposite case
    is picked up immediately by the minimum.

    The creep is proportional to elapsed device time rather than to the
    number of samples, so it behaves the same at 10 Hz and at 400 Hz.
    """

    #: Largest drift the estimate can follow, in parts per million. The 8 MHz
    #: crystal and the host clock together stay well inside this.
    MAX_DRIFT_PPM = 100

    def __init__(self) -> None:
        self._last_raw: Optional[int] = None
        self._last_dev_ns = 0
        self._wraps = 0
        self._offset_ns: Optional[int] = None

    def to_ros_ns(self, device_us: int, now_ns: int) -> int:
        """Convert a device timestamp (microseconds) to ROS time (nanoseconds)."""
        if self._last_raw is not None and device_us < self._last_raw:
            self._wraps += 1
        self._last_raw = device_us

        dev_ns = (device_us + (self._wraps << 32)) * 1000
        offset = now_ns - dev_ns

        # "<=" rather than "<": when the two clocks agree exactly the offset
        # must re-latch, otherwise a perfectly stable pair would still creep.
        if self._offset_ns is None or offset <= self._offset_ns:
            self._offset_ns = offset
        else:
            elapsed = max(0, dev_ns - self._last_dev_ns)
            self._offset_ns += (elapsed * self.MAX_DRIFT_PPM) // 1_000_000

        self._last_dev_ns = dev_ns
        return dev_ns + self._offset_ns
