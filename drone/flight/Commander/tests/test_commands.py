import logging
from pathlib import Path
import sys
import threading
import time
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from Commander import Commander, CommandResult
from FlightControl import FlightControl
from MAVLinkConn import SendingResult
from OffboardControl import OffboardControl, OffboardConfig, TargetResult


class FakeConn:
    def __init__(self):
        self.alive = True
        self.confirm = True
        self.result = SendingResult.SENT
        self.state = SimpleNamespace(armed=False, flight_mode='MANUAL')
        self.calls = []
        self.pending = None
        self.drop_on_wait = False

    def is_available(self):
        return self.alive

    def tm_snapshot(self):
        return self.state

    def mode_mapping(self):
        return dict.fromkeys(('MANUAL', 'LAND', 'TAKEOFF', 'OFFBOARD'), (0, 0, 0))

    def send_command_long(self, command, params):
        self.calls.append(('arm', params[0]))
        self.pending = ('armed', bool(params[0]))
        return self.result

    def set_flight_mode(self, name):
        self.calls.append(('mode', name))
        self.pending = ('flight_mode', name)
        return self.result

    def wait_for_state(self, predicate, timeout):
        if self.drop_on_wait:
            self.alive = False
        elif self.confirm and self.pending:
            setattr(self.state, *self.pending)
            self.pending = None
        else:
            time.sleep(timeout)
        return predicate(self.state)

    def send_manual_control(self, *args):
        self.calls.append(('manual', args))
        return self.result

    def send_local_target(self, *args):
        self.calls.append(('target', args))
        return self.result


class CommandsTest(unittest.TestCase):
    def setUp(self):
        self.conn = FakeConn()
        self.fabric = SimpleNamespace(get_logger=lambda name: logging.getLogger(name))
        self.offboard = OffboardControl(OffboardConfig(100, 0.15), self.conn, self.fabric, lambda: None)
        self.addCleanup(self.offboard.stop)
        self.c = Commander.__new__(Commander)
        self.c._logger = self.fabric.get_logger("Commander")
        self.c._lock = threading.Lock()
        self.c._command_lock = threading.RLock()
        self.c._conn_dead = threading.Event()
        self.c._conn = self.conn
        self.c._flight_ctrl = FlightControl(self.conn, self.fabric)
        self.c._offboard_ctrl = self.offboard

    def test_all_commands_reject_dead_connection(self):
        self.conn.alive = False
        calls = [self.c.arm, self.c.disarm, self.c.takeoff, self.c.land,
                 lambda: self.c.flight_mode('MANUAL'),
                 lambda: self.c.manual_control(0, 0, 0, 0),
                 lambda: self.c.set_target(pos=(0, 0, -1))]
        for call in calls:
            self.assertEqual(call(), CommandResult.NO_CONN)
        self.assertEqual(self.conn.calls, [])

    def test_arm_disarm_confirm_state(self):
        self.assertEqual(self.c.arm(), CommandResult.SUCCESS)
        self.assertTrue(self.conn.state.armed)
        self.assertEqual(self.c.disarm(), CommandResult.SUCCESS)
        self.assertFalse(self.conn.state.armed)

    def test_missing_confirmation(self):
        self.conn.confirm = False
        self.assertEqual(self.c.arm(timeout=0.01), CommandResult.CONFIRMATION_TIMEOUT)

    def test_connection_lost_while_waiting(self):
        self.conn.drop_on_wait = True
        self.assertEqual(self.c.arm(), CommandResult.NO_CONN)

    def test_send_failure_does_not_wait(self):
        self.conn.result = SendingResult.SEND_FAILED
        self.assertEqual(self.c.arm(), CommandResult.SEND_FAILED)
        self.assertFalse(self.conn.state.armed)

    def test_modes_confirm(self):
        self.assertEqual(self.c.takeoff(), CommandResult.SUCCESS)
        self.assertEqual(self.conn.state.flight_mode, 'TAKEOFF')
        self.assertEqual(self.c.land(), CommandResult.SUCCESS)
        self.assertEqual(self.conn.state.flight_mode, 'LAND')

    def test_manual_confirms_mode_before_sticks(self):
        self.conn.state.flight_mode = 'LAND'
        self.assertEqual(self.c.manual_control(1, 2, 3, 4), CommandResult.SENT)
        self.assertEqual([c[0] for c in self.conn.calls], ['mode', 'manual'])

    def test_manual_no_sticks_after_failed_mode(self):
        self.conn.state.flight_mode = 'LAND'
        self.conn.confirm = False
        self.assertEqual(self.c.manual_control(1, 2, 3, 4, timeout=0.01), CommandResult.CONFIRMATION_TIMEOUT)
        self.assertFalse(any(c[0] == 'manual' for c in self.conn.calls))

    def test_invalid_inputs_do_not_send(self):
        self.assertEqual(self.c.flight_mode('BAD'), CommandResult.INVALID_ARGS)
        self.assertEqual(self.c.arm(float('nan')), CommandResult.INVALID_ARGS)
        self.assertEqual(self.c.manual_control(2000, 0, 0, 0), CommandResult.INVALID_ARGS)
        self.assertEqual(self.c.set_target(pos=(0, 1)), CommandResult.INVALID_ARGS)
        self.assertEqual(self.c.set_target(pos=(0, 0, float('nan'))), CommandResult.INVALID_ARGS)
        self.assertEqual(self.conn.calls, [])

    def test_offboard_requires_target(self):
        self.assertEqual(self.c.flight_mode('OFFBOARD'), CommandResult.OFFBOARD_INACTIVE)

    def test_target_stream_and_timeout(self):
        start = time.monotonic()
        self.assertEqual(self.c.set_target(pos=(0, 0, -1)), CommandResult.SENT)
        self.assertGreaterEqual(time.monotonic() - start, 1.1)
        self.assertEqual(self.conn.state.flight_mode, 'OFFBOARD')
        self.assertTrue(self.offboard.is_active())
        deadline = time.monotonic() + 1
        while self.offboard.is_active() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertFalse(self.offboard.is_active())

    def test_failed_start_cancels_stream(self):
        self.assertEqual(self.c.set_target(pos=(0, 0, -1), timeout=0.01),
                         CommandResult.CONFIRMATION_TIMEOUT)
        self.assertFalse(self.offboard.is_active())

    def test_target_send_failure(self):
        self.conn.result = SendingResult.SEND_FAILED
        self.assertEqual(self.c.set_target(pos=(0, 0, -1)), CommandResult.SEND_FAILED)
        self.assertFalse(self.offboard.is_active())

    def test_stop_is_irreversible(self):
        self.offboard.stop()
        self.assertEqual(self.offboard.set_target(pos=(0, 0, -1)), TargetResult.STOPPED)
        self.assertEqual(self.c.set_target(pos=(0, 0, -1)), CommandResult.OFFBOARD_INACTIVE)

    def test_mode_failure_keeps_existing_stream(self):
        self.offboard.set_target(pos=(0, 0, -1))
        self.offboard.activate()
        self.conn.state.flight_mode = 'OFFBOARD'
        self.conn.confirm = False
        self.assertEqual(self.c.land(timeout=0.01), CommandResult.CONFIRMATION_TIMEOUT)
        self.assertTrue(self.offboard.is_active())

    def test_mode_success_deactivates_stream(self):
        self.offboard.set_target(pos=(0, 0, -1))
        self.offboard.activate()
        self.conn.state.flight_mode = 'OFFBOARD'
        self.assertEqual(self.c.land(), CommandResult.SUCCESS)
        self.assertFalse(self.offboard.is_active())

if __name__ == '__main__':
    unittest.main()
