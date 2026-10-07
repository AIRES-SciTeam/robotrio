import logging
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from mission.mission_controller import FailureAction, MissionController, MissionState, MissionStep
from mission.mission_block.mission_block import MissionBlock, MBState
from mission.mission_env import MissionEnv


class ScriptBlock(MissionBlock):
    def __init__(self, mission_env, name='block', results=(MBState.SUCCEEDED,)):
        super().__init__(name, mission_env)
        self.results = iter(results)
        mission_env.events.append(('create', name, self))

    def start(self):
        super().start()
        self.menv.events.append(('start', self.name))

    def cancel(self):
        super().cancel()
        self.menv.events.append(('cancel', self.name))

    def tick(self):
        self.menv.events.append(('tick', self.name))
        result = next(self.results, MBState.RUNNING)
        if isinstance(result, Exception):
            raise result
        self._state = result
        return result


class ConstructorError(MissionBlock):
    def __init__(self, mission_env):
        raise RuntimeError('constructor')
    def tick(self):
        return MBState.RUNNING


class StartError(ScriptBlock):
    def start(self):
        raise RuntimeError('start')


def step(name, *results, **kwargs):
    return MissionStep(ScriptBlock, {'name': name, 'results': results or (MBState.SUCCEEDED,)}, **kwargs)


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.env = MissionEnv(
            commander=Mock(), logger=Mock(spec=logging.Logger),
            camera_rotation=None, camera_position=None,
            image_receiver=None, tag_detector=None,
        )
        self.env.events = []

    def controller(self, plan=(), **kwargs):
        controller = MissionController(plan, self.env, **kwargs)
        self.addCleanup(controller.stop)
        controller.start(background=False)
        return controller

    def run_until(self, controller, state, limit=30):
        for _ in range(limit):
            if controller.state == state:
                return
            controller.tick()
        self.assertEqual(controller.state, state)

    def ticks(self):
        return [event[1] for event in self.env.events if event[0] == 'tick']

    def test_plan_and_end_chain_before_completed(self):
        c = self.controller([step('a'), step('b')], end_steps=[step('end', MBState.RUNNING, MBState.SUCCEEDED)])
        self.run_until(c, MissionState.FINISHING)
        c.tick()
        self.assertEqual(c.state, MissionState.FINISHING)
        self.run_until(c, MissionState.COMPLETED)
        self.assertEqual(self.ticks(), ['a', 'b', 'end', 'end'])

    def test_empty_plan_runs_end_chain(self):
        c = self.controller(end_steps=[step('end')])
        self.run_until(c, MissionState.COMPLETED)
        self.assertEqual(self.ticks(), ['end'])

    def test_failure_runs_error_chain_and_stays_failed(self):
        c = self.controller([step('bad', MBState.FAILED), step('never')], fail_steps=[step('error')], end_steps=[step('end')])
        self.run_until(c, MissionState.FAILED)
        self.assertEqual(self.ticks(), ['bad', 'error'])
        self.assertIn('FAILED', c.last_error)

    def test_error_chain_failure_does_not_recurse(self):
        c = self.controller([step('bad', MBState.FAILED)], fail_steps=[step('error', MBState.FAILED), step('never')])
        self.run_until(c, MissionState.FAILED)
        for _ in range(3):
            c.tick()
        self.assertEqual(self.ticks(), ['bad', 'error'])

    def test_end_failure_runs_error_chain(self):
        c = self.controller(end_steps=[step('end', MBState.FAILED)], fail_steps=[step('error')])
        self.run_until(c, MissionState.FAILED)
        self.assertEqual(self.ticks(), ['end', 'error'])

    def test_pause_cancels_and_resume_recreates_main_block(self):
        c = self.controller([step('main', MBState.RUNNING)], pause_steps=[step('pause', MBState.RUNNING)])
        c.tick()
        original = c.current_block
        self.assertTrue(c.pause())
        c.tick()
        self.assertEqual(original._state, MBState.CANCELLED)
        self.assertEqual(c.state, MissionState.PAUSED)
        self.assertTrue(c.resume())
        c.tick()
        self.assertIsNot(c.current_block, original)
        self.assertIn(('cancel', 'pause'), self.env.events)
        self.assertEqual(self.ticks(), ['main', 'pause', 'main'])

    def test_finite_pause_chain_waits_for_resume(self):
        c = self.controller([step('main')], pause_steps=[step('p1'), step('p2')])
        c.pause()
        for _ in range(6):
            c.tick()
        self.assertEqual(c.state, MissionState.PAUSED)
        self.assertEqual(self.ticks(), ['p1', 'p2'])
        c.resume()
        self.run_until(c, MissionState.COMPLETED)

    def test_pause_failure_runs_error_chain(self):
        c = self.controller([step('main')], pause_steps=[step('pause', MBState.FAILED)], fail_steps=[step('error')])
        c.pause()
        self.run_until(c, MissionState.FAILED)
        self.assertEqual(self.ticks(), ['pause', 'error'])

    def test_skip_jump_and_previous_cancel_old_block(self):
        c = self.controller([step(str(i), MBState.RUNNING) for i in range(3)])
        c.tick()
        c.skip()
        c.tick()
        c.switch_to(2)
        c.tick()
        c.prev()
        c.tick()
        self.assertEqual(self.ticks(), ['0', '1', '2', '1'])
        self.assertEqual(c.current_index, 1)
        self.assertIn(('cancel', '0'), self.env.events)

    def test_navigation_on_pause_preserves_pause_block(self):
        c = self.controller([step('a'), step('b')], pause_steps=[step('pause', MBState.RUNNING)])
        c.pause()
        c.tick()
        pause_block = c.current_block
        c.skip()
        c.switch_to(0)
        c.next()
        self.assertIs(c.current_block, pause_block)
        self.assertEqual(c.state, MissionState.PAUSED)
        c.resume()
        self.run_until(c, MissionState.COMPLETED)
        self.assertEqual(self.ticks(), ['pause', 'b'])

    def test_skip_last_on_pause_waits_for_resume(self):
        c = self.controller([step('main')], end_steps=[step('end')])
        c.pause()
        c.skip()
        c.tick()
        self.assertEqual(c.state, MissionState.PAUSED)
        c.resume()
        self.run_until(c, MissionState.COMPLETED)
        self.assertEqual(self.ticks(), ['end'])

    def test_explicit_finish_runs_end_chain(self):
        c = self.controller([step('main', MBState.RUNNING)], end_steps=[step('end')])
        c.tick()
        c.finish()
        self.run_until(c, MissionState.COMPLETED)
        self.assertEqual(self.ticks(), ['main', 'end'])

    def test_retry_budget_across_multiple_running_ticks(self):
        c = self.controller([step('retry', MBState.RUNNING, MBState.FAILED,
                                  on_failure=FailureAction.RETRY, max_retries=2, retry_delay=0)])
        self.run_until(c, MissionState.FAILED)
        self.assertEqual(len([e for e in self.env.events if e[0] == 'create']), 3)
        self.assertEqual(len(self.ticks()), 6)

    def test_retry_delay_is_nonblocking_and_can_be_skipped(self):
        c = self.controller([step('retry', MBState.FAILED, on_failure=FailureAction.RETRY,
                                  max_retries=1, retry_delay=100), step('next')])
        with patch('mission.mission_controller.time.monotonic', return_value=10):
            c.tick()
            c.tick()
            self.assertEqual(self.ticks(), ['retry'])
            c.skip()
            c.tick()
        self.assertEqual(self.ticks(), ['retry', 'next'])

    def test_failure_skip_policy(self):
        c = self.controller([step('bad', MBState.FAILED, on_failure=FailureAction.SKIP), step('next')])
        self.run_until(c, MissionState.COMPLETED)
        self.assertEqual(self.ticks(), ['bad', 'next'])

    def test_constructor_start_and_tick_exceptions_run_error_chain(self):
        for specification in (MissionStep(ConstructorError), MissionStep(StartError), step('tick', RuntimeError('tick'))):
            with self.subTest(specification=specification):
                c = self.controller([specification], fail_steps=[step('error')])
                self.run_until(c, MissionState.FAILED)
                self.assertEqual(self.ticks()[-1], 'error')

    def test_stop_cancels_without_running_auxiliary_blocks(self):
        c = self.controller([step('main', MBState.RUNNING)], end_steps=[step('end')], fail_steps=[step('error')])
        c.tick()
        c.stop()
        c.tick()
        self.assertEqual(c.state, MissionState.STOPPED)
        self.assertEqual(self.ticks(), ['main'])
        self.env.commander.stop.assert_not_called()

    def test_background_thread_completes_and_joins(self):
        c = MissionController([step('main')], self.env, tick_period=.001)
        self.addCleanup(c.stop)
        self.assertTrue(c.start())
        self.assertFalse(c.start())
        c._ticker_thread.join(1)
        self.assertFalse(c._ticker_thread.is_alive())
        self.assertEqual(c.state, MissionState.COMPLETED)
        c.stop()
        self.assertEqual(c.state, MissionState.COMPLETED)

    def test_bounds_and_terminal_commands(self):
        c = self.controller([step('main')])
        self.assertFalse(c.prev())
        for index in (-1, 1):
            with self.assertRaises(IndexError):
                c.switch_to(index)
        with self.assertRaises(TypeError):
            c.switch_to(.5)
        self.run_until(c, MissionState.COMPLETED)
        self.assertFalse(c.switch_to(0))
        self.assertFalse(c.skip())
        self.assertFalse(c.pause())
        self.assertFalse(c.resume())

    def test_retry_can_succeed_after_delay(self):
        attempts = []
        class RetryBlock(ScriptBlock):
            def __init__(self, mission_env):
                attempts.append(True)
                result = MBState.FAILED if len(attempts) == 1 else MBState.SUCCEEDED
                super().__init__(mission_env, results=(result,))
        c = self.controller([MissionStep(RetryBlock, on_failure=FailureAction.RETRY,
                                         max_retries=1, retry_delay=5)])
        with patch('mission.mission_controller.time.monotonic', return_value=10):
            c.tick()
        with patch('mission.mission_controller.time.monotonic', return_value=14):
            c.tick()
            self.assertEqual(len(attempts), 1)
        with patch('mission.mission_controller.time.monotonic', return_value=15):
            self.run_until(c, MissionState.COMPLETED)
        self.assertEqual(len(attempts), 2)

    def test_stop_from_block_does_not_join_its_own_thread(self):
        class StopBlock(ScriptBlock):
            def tick(block):
                c.stop()
                return MBState.SUCCEEDED
        c = MissionController([MissionStep(StopBlock)], self.env, tick_period=.001)
        self.addCleanup(c.stop)
        c.start()
        c._ticker_thread.join(1)
        self.assertFalse(c._ticker_thread.is_alive())
        self.assertEqual(c.state, MissionState.STOPPED)
        self.assertEqual(c.current_index, 0)

    def test_validation(self):
        for period in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                MissionController([], self.env, tick_period=period)
        with self.assertRaises(ValueError):
            MissionStep(ScriptBlock, max_retries=-1)
        with self.assertRaises(ValueError):
            MissionStep(ScriptBlock, retry_delay=float('inf'))


if __name__ == '__main__':
    unittest.main()
