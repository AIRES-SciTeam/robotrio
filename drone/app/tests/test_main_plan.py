import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from mission.mission_controller import MissionController, MissionStep, MissionState
from mission.mission_block import FlightMode, Arm, Takeoff, MoveAboveTag
from safety_commander.commander import CommandResult


class MainPlanTests(unittest.TestCase):
    def test_plan_reaches_tag_then_loiter(self):
        path = Path(__file__).resolve().parents[1] / 'main.py'
        tree = ast.parse(path.read_text())
        constants = {'TAG_SIZE', 'HELIPAD_TAG_ID', 'HELIPAD_TAG_SIZE', 'HEIGHT_ABOVE_TAG'}
        nodes = [n for n in tree.body if (
            isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in constants for t in n.targets)
        ) or (isinstance(n, ast.FunctionDef) and n.name == 'flight_plan')]
        ns = dict(MissionStep=MissionStep, FlightMode=FlightMode, Arm=Arm, Takeoff=Takeoff, MoveAboveTag=MoveAboveTag)
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), ns)
        plan = ns['flight_plan']()
        self.assertEqual([s.block_class for s in plan], [FlightMode, Arm, Takeoff, MoveAboveTag, MoveAboveTag, MoveAboveTag])
        self.assertEqual([s.block_params['tag_id'] for s in plan[3:]], [0, 5, 585])
        self.assertEqual([s.block_params['tag_size'] for s in plan[3:]], [.04, .04, .08])
        self.assertTrue(all(s.block_params['height'] == 1 for s in plan[3:]))
        commander = Mock()
        position = [0., 0., 0.]
        commander.get_drone_position.side_effect = lambda: (*position, (1, 0, 0, 0))
        commander.get_drone_velocity.return_value = (0, 0, 0)
        commander.armed.return_value = False
        commander.arm.return_value = CommandResult.SUCCESS
        commander.set_flight_mode.return_value = CommandResult.SUCCESS
        targets = []
        def send(pos, **kwargs):
            targets.append(pos)
            position[:] = pos
            return CommandResult.SUCCESS
        commander.set_offboard_target.side_effect = send
        env = SimpleNamespace(
            commander=commander, logger=Mock(), image_receiver=Mock(), tag_detector=Mock(),
            camera_rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]]), camera_position=(.1,0,-.22),
        )
        env.image_receiver.get_image.side_effect = object
        env.image_receiver.get_camerainfo.return_value = ((500,500,320,240), np.zeros(5))
        tag_positions = {0: (0,.6,0), 5: (.2,.2,0), 585: (1,1,0)}
        def detect(frame, tag_id, params, dist, size):
            delta = np.array(tag_positions[tag_id]) - position - env.camera_position
            return env.camera_rotation.T @ delta, np.eye(3)
        env.tag_detector.get_tag_position.side_effect = detect
        mission = MissionController(plan, env, end_steps=[MissionStep(FlightMode, {'mode':'LOITER'})])
        mission.start(background=False)
        with patch('mission.mission_block.move_to.time.monotonic', side_effect=range(100)):
            for _ in range(40):
                mission.tick()
                if mission.state == MissionState.COMPLETED:
                    break
        self.assertEqual(mission.state, MissionState.COMPLETED)
        self.assertEqual(targets[0], (0,0,-1))
        np.testing.assert_allclose(position, (1,1,-1))
        detected_ids = [call.args[1] for call in env.tag_detector.get_tag_position.call_args_list]
        self.assertEqual(list(dict.fromkeys(detected_ids)), [0,5,585])
        commander.arm.assert_called_once()
        self.assertEqual(commander.set_flight_mode.call_count, 2)
        mission.stop()
