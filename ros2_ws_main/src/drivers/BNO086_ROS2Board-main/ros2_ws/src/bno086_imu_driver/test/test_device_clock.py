"""Tests for the device-clock to ROS-time mapping."""

import pytest

from bno086_imu_driver.device_clock import DeviceClock

SEC = 1_000_000_000
MS = 1_000_000


def test_offset_is_applied_consistently():
    clk = DeviceClock()
    assert clk.to_ros_ns(0, 100 * SEC) == 100 * SEC
    # 1 s later on the device, and 1 s later on the host: same stamp.
    # The bounded upward creep allows a sub-microsecond difference.
    assert clk.to_ros_ns(1_000_000, 101 * SEC) == pytest.approx(101 * SEC, abs=1000)


def test_jitter_does_not_shift_the_estimate_upward():
    """A late-arriving sample must not drag the whole timeline with it."""
    clk = DeviceClock()
    clk.to_ros_ns(0, 100 * SEC)
    # This sample arrived 10 ms late; the stamp should follow the device
    # clock, not the delayed arrival.
    stamp = clk.to_ros_ns(1_000_000, 101 * SEC + 10 * MS)
    assert stamp == pytest.approx(101 * SEC, abs=MS)


def test_minimum_filter_latches_the_lowest_latency_sample():
    clk = DeviceClock()
    clk.to_ros_ns(0, 100 * SEC + 5 * MS)         # 5 ms of latency
    stamp = clk.to_ros_ns(1_000_000, 101 * SEC)  # arrived with none
    assert stamp == 101 * SEC


def test_counter_wrap_is_unrolled():
    clk = DeviceClock()
    near_wrap = 0xFFFF_F000
    base = 4000 * SEC
    clk.to_ros_ns(near_wrap, base)

    # The 32-bit microsecond counter rolls over.
    after = 0x0000_1000
    delta_us = (after + (1 << 32)) - near_wrap
    stamp = clk.to_ros_ns(after, base + delta_us * 1000)

    assert stamp - base == pytest.approx(delta_us * 1000, abs=1000)


def test_stable_clocks_produce_exact_stamps():
    """With no drift and no jitter the minimum filter pins the offset."""
    clk = DeviceClock()
    for i in range(500):
        dev_us = i * 10_000
        host_ns = 50 * SEC + dev_us * 1000
        assert clk.to_ros_ns(dev_us, host_ns) == host_ns


def test_estimate_follows_a_slow_device_crystal():
    """Device clock 50 ppm slow: the true offset grows and must be tracked."""
    clk = DeviceClock()
    drift = 50e-6
    errors = []
    # 100 Hz for 10 minutes.
    for i in range(60_000):
        dev_us = int(i * 10_000 * (1.0 - drift))
        host_ns = 20 * SEC + i * 10 * MS
        errors.append(clk.to_ros_ns(dev_us, host_ns) - host_ns)

    # The creep budget (100 ppm) exceeds the 50 ppm of real drift, so the
    # estimate keeps up instead of falling steadily behind.
    assert abs(errors[-1]) < 2 * MS
    assert abs(errors[-1]) <= abs(errors[len(errors) // 2]) + MS


def test_creep_is_bounded_and_does_not_run_away():
    """With a device clock that is fast, the minimum filter must dominate."""
    clk = DeviceClock()
    drift = -50e-6                       # device runs fast
    for i in range(10_000):
        dev_us = int(i * 10_000 * (1.0 - drift))
        host_ns = 20 * SEC + i * 10 * MS
        stamp = clk.to_ros_ns(dev_us, host_ns)
    # A fast device clock is corrected immediately by the minimum filter.
    assert stamp == pytest.approx(host_ns, abs=MS)


def test_creep_is_independent_of_report_rate():
    """The same wall-clock drift must be tracked at 10 Hz and at 400 Hz."""
    results = []
    for period_us in (100_000, 2_500):    # 10 Hz and 400 Hz
        clk = DeviceClock()
        drift = 50e-6
        n = int(60_000_000 / period_us)   # 60 s of data either way
        for i in range(n):
            dev_us = int(i * period_us * (1.0 - drift))
            host_ns = 20 * SEC + i * period_us * 1000
            err = clk.to_ros_ns(dev_us, host_ns) - host_ns
        results.append(err)

    assert results[0] == pytest.approx(results[1], abs=MS)
