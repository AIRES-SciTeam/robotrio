import sys
from pathlib import Path
import unittest
from unittest.mock import Mock

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from safety_commander.telemetry import TelemetryManager


class PX4HeartbeatTests(unittest.TestCase):
    def test_offboard_main_mode_and_other_modes(self):
        fabric = Mock()
        fabric.is_alive.return_value = True
        telemetry = TelemetryManager(fabric)
        for name in ('OFFBOARD', 'LOITER', 'POSCTL', 'MANUAL', 'RTL'):
            base_mode, main_mode, sub_mode = mavutil.px4_map[name]
            for armed in (False, True):
                flags = base_mode | (mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)
                msg = mavutil.mavlink.MAVLink_heartbeat_message(
                    mavutil.mavlink.MAV_TYPE_QUADROTOR,
                    mavutil.mavlink.MAV_AUTOPILOT_PX4,
                    flags, (main_mode << 16) | (sub_mode << 24),
                    mavutil.mavlink.MAV_STATE_ACTIVE, 3,
                )
                telemetry.update(msg)
                state = telemetry.snapshot()
                self.assertEqual(state.flight_mode, name)
                self.assertEqual(state.armed, armed)

    def test_non_px4_heartbeat_does_not_change_state(self):
        fabric = Mock()
        fabric.is_alive.return_value = True
        telemetry = TelemetryManager(fabric)
        msg = mavutil.mavlink.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 29, 6 << 16,
            mavutil.mavlink.MAV_STATE_ACTIVE, 3,
        )
        telemetry.update(msg)
        self.assertIsNone(telemetry.snapshot().flight_mode)
