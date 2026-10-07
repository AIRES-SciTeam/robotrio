"""Offline regressions; run: python3 -m unittest discover -s drone/app/tests -v."""
import importlib.util
import logging
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from logger.logger_fabric import LoggerFabric
from safety_commander.commander import Commander, CommanderState, CommandResult
from safety_commander.control_offboard import ControlOffboard, OffboardConfig, OffboardState
from safety_commander.mavlink_conn import MAVLinkConn, MAVLinkConnConfig, ConnState, SendingResult
from safety_commander.telemetry import Telemetry


def fabric():
    result = Mock()
    result.is_alive.return_value = True
    return result


def wait_until(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.005)
    return False


class CommanderTests(unittest.TestCase):
    def setUp(self):
        self.telemetry = Telemetry(armed=False, flight_mode='LOITER',
                                   flight_mode_mapping={'LOITER': 1, 'OFFBOARD': 2})
        self.conn = Mock()
        self.conn.state = ConnState.OK
        self.conn.snapshot_telemetry.side_effect = lambda: self.telemetry
        self.conn.wait_for_telemetry.side_effect = lambda predicate, timeout: predicate(self.telemetry)
        self.conn.send_local_target.return_value = SendingResult.SUCCESS
        self.flight = Mock()
        self.offboard = ControlOffboard(OffboardConfig(), self.conn, fabric())
        with patch('safety_commander.commander.MAVLinkConn', return_value=self.conn), \
             patch('safety_commander.commander.ControlFlight', return_value=self.flight), \
             patch('safety_commander.commander.ControlOffboard', return_value=self.offboard), \
             patch.object(Commander, '_babysit'):
            self.commander = Commander(fabric(), MAVLinkConnConfig(), OffboardConfig())
        self.addCleanup(self.commander.stop)

    def test_arm_disarm_send_and_confirm(self):
        def arm():
            self.telemetry.armed = True
            return SendingResult.SUCCESS
        def disarm():
            self.telemetry.armed = False
            return SendingResult.SUCCESS
        self.flight.arm.side_effect = arm
        self.flight.disarm.side_effect = disarm
        self.assertEqual(self.commander.arm(), CommandResult.SUCCESS)
        self.assertEqual(self.commander.disarm(), CommandResult.SUCCESS)
        self.flight.arm.assert_called_once_with()
        self.flight.disarm.assert_called_once_with()

    def test_toggle_does_not_deadlock(self):
        self.flight.arm.return_value = SendingResult.SUCCESS
        results = []
        thread = threading.Thread(target=lambda: results.append(self.commander.toggle_arming()), daemon=True)
        thread.start()
        thread.join(.5)
        # Avoid a blocking cleanup if this regression reappears.
        if thread.is_alive():
            self._cleanups.clear()
            self.offboard.stop()
        self.assertFalse(thread.is_alive())
        self.flight.arm.assert_called_once_with()
        self.assertEqual(results, [CommandResult.TIMEOUT])

    def test_reconnect_restores_commands(self):
        self.conn.state = ConnState.RECONN
        self.commander._handle_conn()
        self.assertEqual(self.commander.arm(), CommandResult.NO_CONN)
        self.conn.state = ConnState.OK
        self.commander._handle_conn()
        self.assertEqual(self.commander._state, CommanderState.OK)
        self.assertEqual(self.commander._wait_confirmation(lambda t: not t.armed, 1), CommandResult.SUCCESS)

    def test_offboard_entry_and_repeated_target(self):
        def change_mode(mode):
            self.assertGreater(self.conn.send_local_target.call_count, 2)
            self.telemetry.flight_mode = mode
            return SendingResult.SUCCESS
        self.flight.flight_mode.side_effect = change_mode
        self.assertEqual(self.commander.set_offboard_target(pos=(0, 0, -1)), CommandResult.SUCCESS)
        self.assertEqual(self.offboard.state, OffboardState.ACTIVE)
        self.assertEqual(self.commander.set_offboard_target(pos=(1, 0, -1)), CommandResult.SUCCESS)
        self.flight.flight_mode.assert_called_once_with('OFFBOARD')

    def test_offboard_rejected_mode_stops_publication(self):
        self.flight.flight_mode.return_value = SendingResult.FAILURE
        self.assertEqual(self.commander.set_offboard_target(pos=(0, 0, -1)), CommandResult.FAILURE)
        self.assertEqual(self.offboard.state, OffboardState.DEACTIVATING)

    def test_invalid_target_does_not_activate(self):
        self.assertEqual(self.commander.set_offboard_target(pos=(1,)), CommandResult.INVALID)
        self.assertEqual(self.offboard.state, OffboardState.IDLE)
        self.flight.flight_mode.assert_not_called()

    def test_nanny_transitions(self):
        self.offboard.set_target(pos=(0, 0, -1))
        self.offboard.activate()
        self.telemetry.flight_mode = 'OFFBOARD'
        self.commander._handle_offboard()
        self.assertEqual(self.offboard.state, OffboardState.ACTIVE)
        self.offboard.deactivate()
        self.telemetry.flight_mode = 'LOITER'
        self.commander._handle_offboard()
        self.assertEqual(self.offboard.state, OffboardState.IDLE)

    def test_stop_stops_children(self):
        self.commander.stop()
        self.assertEqual(self.commander.arm(), CommandResult.STOPPED)
        self.assertFalse(self.offboard._broadcast_thread.is_alive())
        self.conn.stop.assert_called_once_with()
        self.commander.stop()
        self.conn.stop.assert_called_once_with()

    def test_failed_constructor_stops_connection(self):
        conn = Mock()
        with patch('safety_commander.commander.MAVLinkConn', return_value=conn), \
             patch('safety_commander.commander.ControlOffboard', side_effect=RuntimeError('test')):
            commander = Commander(fabric(), MAVLinkConnConfig(), OffboardConfig())
        self.assertFalse(commander.is_alive())
        conn.stop.assert_called_once_with()


class OffboardTests(unittest.TestCase):
    def make_offboard(self, result=SendingResult.SUCCESS):
        conn = Mock()
        conn.send_local_target.return_value = result
        offboard = ControlOffboard(OffboardConfig(publishing_rate=100, notarget_timeout=.05), conn, fabric())
        self.addCleanup(offboard.stop)
        offboard.set_target(pos=(0, 0, -1))
        offboard.activate()
        return offboard, conn

    def test_timeout_does_not_deadlock_and_stops_sending(self):
        offboard, conn = self.make_offboard()
        offboard.mark_active()
        # Read the raw state in this bounded check so a broken lock cannot hang the suite.
        self.assertTrue(wait_until(lambda: offboard._state == OffboardState.DEACTIVATING))
        count = conn.send_local_target.call_count
        time.sleep(.04)
        self.assertEqual(conn.send_local_target.call_count, count)
        self.assertTrue(offboard._state_lock.acquire(timeout=.1))
        offboard._state_lock.release()
        offboard.mark_idle()
        self.assertEqual(offboard.state, OffboardState.IDLE)

    def test_failed_results_are_counted(self):
        offboard, conn = self.make_offboard(SendingResult.NO_CONN)
        self.assertTrue(wait_until(lambda: offboard._state == OffboardState.DEACTIVATING))
        self.assertTrue(offboard.failed)
        self.assertEqual(conn.send_local_target.call_count, 3)

    def test_exceptions_are_counted(self):
        offboard, conn = self.make_offboard()
        conn.send_local_target.side_effect = RuntimeError('transport failed')
        self.assertTrue(wait_until(lambda: offboard._state == OffboardState.DEACTIVATING))
        self.assertTrue(offboard.failed)


class ConnectionTests(unittest.TestCase):
    def test_initial_attempts_exhausted(self):
        conn = MAVLinkConn(MAVLinkConnConfig(max_reconnects=0), fabric())
        self.assertEqual(conn.state, ConnState.DEAD)
        self.assertFalse(conn.is_alive())
        conn.stop()

    def test_reconnect_exhaustion_inside_telemetry_thread(self):
        conn = MAVLinkConn(MAVLinkConnConfig(max_reconnects=0), fabric())
        conn._stop_event.clear()
        errors = []
        def reconnect():
            try:
                conn._reconnect()
            except Exception as exc:
                errors.append(exc)
        conn._tm_thread = threading.Thread(target=reconnect, daemon=True)
        conn._tm_thread.start()
        conn._tm_thread.join(.5)
        self.assertFalse(conn._tm_thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(conn.state, ConnState.DEAD)

    def test_successful_initial_connection(self):
        transport = Mock()
        transport.target_system = 1
        hb = Mock(autopilot=12, base_mode=0)
        hb.get_srcSystem.return_value = 1
        hb.get_type.return_value = 'HEARTBEAT'
        transport.wait_heartbeat.return_value = hb
        with patch('safety_commander.mavlink_conn.mavutil.mavlink_connection', return_value=transport), \
             patch('safety_commander.telemetry.mavutil.mode_string_v10', return_value='LOITER'), \
             patch.object(MAVLinkConn, '_telemetry_loop'), \
             patch.object(MAVLinkConn, '_heartbeat_loop'):
            conn = MAVLinkConn(MAVLinkConnConfig(max_reconnects=1), fabric())
        self.assertEqual(conn.state, ConnState.OK)
        self.assertEqual(conn.snapshot_telemetry().flight_mode, 'LOITER')
        conn.stop()
        transport.close.assert_called_once_with()


class LoggerTests(unittest.TestCase):
    def test_module_directory_created(self):
        with tempfile.TemporaryDirectory() as directory:
            lf = LoggerFabric('info', Path(directory))
            logger = lf.get_logger('example', module_name='safety_commander')
            try:
                logger.info('test message')
                self.assertIn('test message', (lf.logs_dir / 'safety_commander/example.log').read_text())
            finally:
                for handler in logger.handlers:
                    handler.close()


def load_source(name, relative, stubs):
    path = Path(__file__).resolve().parents[1] / 'src' / relative
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {name: module, **stubs}):
        spec.loader.exec_module(module)
    return module


class CameraAndMissionTests(unittest.TestCase):
    def test_camera_callbacks(self):
        class Node:
            def __init__(self, *args, **kwargs):
                self._logger = object()  # Reserved for the ROS node's own logger.
            def create_subscription(self, *args):
                return object()
            destroy_subscription = Mock()
        class BridgeError(Exception):
            pass
        receiver_module = load_source('_test_image_receiver', 'mission/image_handlers.py', {
            'rclpy': types.ModuleType('rclpy'),
            'rclpy.context': types.SimpleNamespace(Context=object),
            'rclpy.node': types.SimpleNamespace(Node=Node),
            'rclpy.qos': types.SimpleNamespace(qos_profile_sensor_data=object()),
            'sensor_msgs': types.ModuleType('sensor_msgs'),
            'sensor_msgs.msg': types.SimpleNamespace(Image=object, CameraInfo=object),
            'std_msgs': types.ModuleType('std_msgs'),
            'std_msgs.msg': types.SimpleNamespace(Header=object),
            'cv_bridge': types.SimpleNamespace(CvBridge=Mock, CvBridgeError=BridgeError),
        })
        receiver = receiver_module.ImageReceiver(fabric(), Node(), image_topic='image', camerainfo_topic='info')
        frame = object()
        receiver._cv_bridge.imgmsg_to_cv2.return_value = frame
        receiver._image_callback(types.SimpleNamespace(header=types.SimpleNamespace(frame_id='camera')))
        self.assertIs(receiver.get_image(), frame)
        subscription = receiver._camerainfo_sub
        receiver._camerainfo_callback(types.SimpleNamespace(k=[1,0,2,0,3,4,0,0,1], d=[0]*5))
        receiver.node.destroy_subscription.assert_called_once_with(subscription)
        self.assertEqual(receiver.get_camerainfo()[0], (1,3,2,4))
        self.assertIsNone(receiver._camerainfo_sub)

    def test_disarm_respects_state_and_unknown_telemetry(self):
        base = load_source('_test_mission_block', 'mission/mission_block/mission_block.py', {
            'mission.mission_env': types.SimpleNamespace(MissionEnv=object),
        })
        arming = load_source('_test_arming', 'mission/mission_block/arming.py', {
            'mission.mission_block.mission_block': base,
        })
        env = Mock()
        env.commander.armed.return_value = None
        env.commander.disarm.return_value = CommandResult.TIMEOUT
        block = arming.Disarm(env)
        self.assertEqual(block.tick(), base.MBState.PENDING)
        block.cancel()
        self.assertEqual(block.tick(), base.MBState.CANCELLED)
        env.commander.disarm.assert_not_called()
        block.start()
        self.assertEqual(block.tick(), base.MBState.FAILED)
        env.commander.disarm.assert_called_once_with()
        env.commander.armed.return_value = False
        block.start()
        self.assertEqual(block.tick(), base.MBState.SUCCEEDED)


if __name__ == '__main__':
    unittest.main()
