"""Unit tests for QGC-facing MAVLink stream interval specs."""

from __future__ import annotations

import unittest

from qgc_mavlink_streams import (
    DEFAULT_STREAM_HZ,
    QGC_DEFAULT_STREAMS,
    interval_us_for_hz,
    stream_interval_requests,
)


class TestQgcMavlinkStreams(unittest.TestCase):
    def test_default_streams_include_highres_imu_and_optical_flow_rad(self):
        names = {name for _, _, name in QGC_DEFAULT_STREAMS}
        self.assertIn("HIGHRES_IMU", names)
        self.assertIn("OPTICAL_FLOW_RAD", names)
        ids = {msgid for msgid, _, _ in QGC_DEFAULT_STREAMS}
        self.assertIn(105, ids)
        self.assertIn(106, ids)

    def test_interval_us_for_50hz(self):
        self.assertEqual(interval_us_for_hz(50), 20000)

    def test_stream_interval_requests_use_default_hz(self):
        reqs = stream_interval_requests()
        by_id = {msgid: (us, name) for msgid, us, name in reqs}
        self.assertEqual(by_id[105], (interval_us_for_hz(DEFAULT_STREAM_HZ), "HIGHRES_IMU"))
        self.assertEqual(by_id[106], (interval_us_for_hz(DEFAULT_STREAM_HZ), "OPTICAL_FLOW_RAD"))

    def test_stream_interval_requests_custom_hz(self):
        reqs = stream_interval_requests(hz=10)
        by_id = {msgid: us for msgid, us, _ in reqs}
        self.assertEqual(by_id[105], 100000)
        self.assertEqual(by_id[106], 100000)


if __name__ == "__main__":
    unittest.main()
