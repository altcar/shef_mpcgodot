from __future__ import annotations

import math
import struct
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import mpc_controller  # noqa: E402


class PacketTests(unittest.TestCase):
    def test_sensor_layout_round_trip(self) -> None:
        values = [float(index) for index in range(mpc_controller.SENSOR_DOUBLES)]
        frame = mpc_controller.parse_sensor(struct.pack("<698d", *values))

        self.assertEqual(frame.state, tuple(values[0:13]))
        self.assertEqual(frame.lidar, tuple(values[13:373]))
        self.assertEqual(len(frame.lane), 5)
        self.assertEqual(frame.lane[0], tuple(values[373:437]))
        self.assertEqual(frame.lane[4], tuple(values[629:693]))
        self.assertEqual(frame.cte, tuple(values[693:698]))

    def test_rejects_wrong_packet_size(self) -> None:
        with self.assertRaises(ValueError):
            mpc_controller.parse_sensor(b"\x00" * 10)

    def test_matlab_rounding(self) -> None:
        self.assertEqual(mpc_controller.matlab_round(2.4), 2)
        self.assertEqual(mpc_controller.matlab_round(2.5), 3)
        self.assertEqual(mpc_controller.matlab_round(-2.4), -2)
        self.assertEqual(mpc_controller.matlab_round(-2.5), -3)


class NmpcTests(unittest.TestCase):
    def test_zero_error_has_zero_cost(self) -> None:
        cost = mpc_controller.compute_nmpc_cost(
            [0.0] * 5,
            [0.0] * 5,
            mpc_controller.HORIZON_STEP_M / 15.0,
            15.0,
            0.0,
        )
        self.assertEqual(cost, 0.0)

    def test_steering_stays_inside_limits(self) -> None:
        state = [0.0] * 13
        for cte in (
            [0.0] * 5,
            [1.5] * 5,
            [-1.5] * 5,
            [0.5, 1.0, 1.5, 2.0, 2.5],
        ):
            steer = mpc_controller.lane_following_nmpc(state, cte, 15.0, 0.0, optimizer="pgd")
            self.assertTrue(math.isfinite(steer))
            self.assertLessEqual(abs(steer), mpc_controller.MAX_YAW_RATE)

    def test_positive_offset_is_corrected(self) -> None:
        state = [0.0] * 13
        steer = mpc_controller.lane_following_nmpc(
            state, [1.0] * 5, 15.0, 0.0, optimizer="pgd"
        )
        # y must grow towards the reference, so the first yaw rate should be positive.
        self.assertGreater(steer, 0.0)

    def test_self_check(self) -> None:
        self.assertEqual(mpc_controller.self_check(), 0)


if __name__ == "__main__":
    unittest.main()
