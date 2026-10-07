import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from mission.mission_block import MoveAboveTag, MBState
from safety_commander.commander import CommandResult


class MoveAboveTagTests(unittest.TestCase):
    def setUp(self):
        self.env = SimpleNamespace(
            commander=Mock(), image_receiver=Mock(), tag_detector=Mock(), logger=Mock(),
            camera_rotation=np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.]]),
            camera_position=(.2, 0, .1),
        )
        self.env.commander.get_drone_position.return_value = (10, 20, -3, (1, 0, 0, 0))
        self.env.commander.get_drone_velocity.return_value = (0, 0, 0)
        self.env.commander.set_offboard_target.return_value = CommandResult.SUCCESS
        self.env.image_receiver.get_image.return_value = object()
        self.env.image_receiver.get_camerainfo.return_value = ((500, 500, 320, 240), np.zeros(5))
        self.env.tag_detector.get_tag_position.return_value = (np.array([1., 2., 3.]), np.eye(3))
        self.block = MoveAboveTag(self.env, tag_id=7, tag_size=.2, height=1)
        self.block.start()

    def test_camera_offset_rotation_and_height(self):
        self.assertEqual(self.block.tick(), MBState.RUNNING)
        np.testing.assert_allclose(self.block.pos, (8.2, 21, -.9))
        args = self.env.tag_detector.get_tag_position.call_args.args
        self.assertEqual(args[1], 7)
        self.assertEqual(args[-1], .2)

    def test_yaw_rotates_camera_translation_too(self):
        self.env.commander.get_drone_position.return_value = (10, 20, -3, (2**-.5, 0, 0, 2**-.5))
        self.block.tick()
        np.testing.assert_allclose(self.block.pos, (9, 18.2, -.9))

    def test_loss_resends_world_target_despite_drone_motion(self):
        self.block.tick()
        target = self.block.pos
        self.env.commander.get_drone_position.return_value = (12, 22, -2, (1, 0, 0, 0))
        self.env.image_receiver.get_image.return_value = object()
        self.env.tag_detector.get_tag_position.return_value = None
        self.block.tick()
        self.env.commander.set_offboard_target.assert_called_with(pos=target, yaw_rate=.3)
        self.env.image_receiver.get_image.return_value = None
        self.block.tick()
        self.assertEqual(self.block.pos, target)
        self.env.commander.set_offboard_target.assert_called_with(pos=target, yaw_rate=.3)

    def test_same_frame_is_not_reprojected_and_reacquisition_updates_target(self):
        self.block.tick()
        target = self.block.pos
        self.env.commander.get_drone_position.return_value = (11, 20, -3, (1, 0, 0, 0))
        self.block.tick()
        self.assertEqual(self.block.pos, target)
        self.env.tag_detector.get_tag_position.assert_called_once()
        self.env.image_receiver.get_image.return_value = object()
        self.block.tick()
        np.testing.assert_allclose(self.block.pos, np.array(target) + (1, 0, 0))

    def test_waits_for_first_detection_and_calibration(self):
        self.env.image_receiver.get_camerainfo.return_value = (None, None)
        self.assertEqual(self.block.tick(), MBState.RUNNING)
        self.env.commander.set_offboard_target.assert_called_with(pos=(10, 20, -3), yaw_rate=.3)
        self.env.tag_detector.get_tag_position.assert_not_called()
        self.env.image_receiver.get_camerainfo.return_value = ((500, 500, 320, 240), np.zeros(5))
        self.env.tag_detector.get_tag_position.return_value = None
        self.assertEqual(self.block.tick(), MBState.RUNNING)

    def test_loss_at_target_keeps_searching_until_reacquired(self):
        self.block.tick()
        self.env.commander.get_drone_position.return_value = (*self.block.pos, (1, 0, 0, 0))
        self.env.image_receiver.get_image.return_value = None
        with patch('mission.mission_block.move_to.time.monotonic', return_value=10):
            self.assertEqual(self.block.tick(), MBState.RUNNING)
        with patch('mission.mission_block.move_to.time.monotonic', return_value=11.1):
            self.assertEqual(self.block.tick(), MBState.RUNNING)
        self.env.commander.set_offboard_target.assert_called_with(pos=self.block.pos, yaw_rate=.3)
        self.env.tag_detector.get_tag_position.return_value = (np.array([0., .2, .9]), np.eye(3))
        for timestamp in (12, 13.1):
            self.env.image_receiver.get_image.return_value = object()
            with patch('mission.mission_block.move_to.time.monotonic', return_value=timestamp):
                state = self.block.tick()
        self.assertEqual(state, MBState.SUCCEEDED)
        self.env.commander.set_offboard_target.assert_called_with(pos=self.block.pos, yaw=0.0)

    def test_stale_frame_restarts_search(self):
        with patch('mission.mission_block.move_above_tag.time.monotonic', return_value=10):
            self.block.tick()
        with patch('mission.mission_block.move_above_tag.time.monotonic', return_value=10.6):
            self.assertEqual(self.block.tick(), MBState.RUNNING)
        self.env.commander.set_offboard_target.assert_called_with(pos=self.block.pos, yaw_rate=.3)

    def test_first_detection_stops_search_and_restart_resets_it(self):
        self.env.tag_detector.get_tag_position.return_value = None
        self.block.tick()
        self.env.commander.set_offboard_target.assert_called_with(pos=(10,20,-3), yaw_rate=.3)
        self.env.image_receiver.get_image.return_value = object()
        self.env.tag_detector.get_tag_position.return_value = (np.array([1.,2.,3.]), np.eye(3))
        self.block.tick()
        self.env.commander.set_offboard_target.assert_called_with(pos=self.block.pos, yaw=0.0)
        self.block.cancel()
        self.block.start()
        self.env.image_receiver.get_image.return_value = None
        self.block.tick()
        self.env.commander.set_offboard_target.assert_called_with(pos=(10,20,-3), yaw_rate=.3)

    def test_cancel_restart_and_send_failure(self):
        self.block.tick()
        self.block.cancel()
        self.env.commander.set_offboard_target.reset_mock()
        self.assertEqual(self.block.tick(), MBState.CANCELLED)
        self.env.commander.set_offboard_target.assert_not_called()
        self.block.start()
        self.assertIsNone(self.block.pos)
        self.env.commander.set_offboard_target.return_value = CommandResult.NO_CONN
        self.assertEqual(self.block.tick(), MBState.FAILED)

    def test_invalid_measurement_does_not_replace_target(self):
        self.block.tick()
        target = self.block.pos
        self.env.image_receiver.get_image.return_value = object()
        self.env.tag_detector.get_tag_position.return_value = (np.array([np.nan, 0, 1]), np.eye(3))
        self.block.tick()
        self.assertEqual(self.block.pos, target)

    def test_invalid_height(self):
        for height in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                MoveAboveTag(self.env, 7, .2, height)
