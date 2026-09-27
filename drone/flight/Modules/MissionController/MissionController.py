from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Callable
import threading
from typing import Any
from queue import SimpleQueue, Empty
import time

from Modules.MissionController.MissionBlocks import MissionBlock, MissionEnv, MBState
from Modules.SimpleCommander.Commander import Commander
from Modules.Logger.LoggerFabric import LoggerFabric

class FailureAction(Enum):
    SKIP = auto()
    RETRY = auto()
    FAIL = auto()

@dataclass
class MissionStep:
    block_class : type[MissionBlock]
    block_params : dict[str, Any] = field(default_factory=dict)

    on_failure : FailureAction = FailureAction.FAIL
    max_retries : int = 0
    retry_delay : float = 0.5 # s

    block : MissionBlock = field(init=False)

class MissionState(Enum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    COMPLETED = auto()
    FAILED = auto()

class MissionCommand(Enum):
    TOGGLE = auto()
    PAUSE = auto()
    RESUME = auto()
    NEXT = auto()
    PREV = auto()

class DRONE_MissionController:
    def __init__(
        self,
        plan : List[MissionStep],
        commander : Commander,
        logger_fabric : LoggerFabric,
        tick_period : float,
        pause_step : MissionStep | None = None,
        end_step : MissionStep | None = None,
        fail_step : MissionStep | None = None,
        grip_ctrl = None
    ):
        self.plan = plan
        self._logger = logger_fabric.get_logger("MissionController")

        self._menv = MissionEnv(
            commander=commander,
            image_receiver=None,
            tag_detector=None,
            logger=self._logger,
            camera_position=None,
            camera_rotation=None,
            grip_ctrl=grip_ctrl
        )

        self._tick_period = tick_period
        self._stop_event = threading.Event()
        self._ticker_thread = threading.Thread(
            target=self._ticker,
            daemon=True
        )

        self.state = MissionState.IDLE
        
        self._curr_step = None
        self._req_step_idx = 0
        self._init_plan()

    def _init_plan(self):
        for step in self.plan:
            step.block = step.block_class(
                mission_env=self._menv,
                **step.block_params
            )

    def start(self):
        self.state = MissionState.RUNNING
        self._ticker_thread.start()

    # def toggle(self):
    #     self.command_queue.put(MissionCommand.TOGGLE)

    # def pause(self):
    #     self.command_queue.put(MissionCommand.PAUSE)

    # def resume(self):
    #     self.command_queue.put(MissionCommand.RESUME)

    # def next(self):
    #     self.command_queue.put(MissionCommand.NEXT)

    # def prev(self):
    #     self.command_queue.put(MissionCommand.PREV)

    def _complete(self):
        self.state = MissionState.COMPLETED
        self._logger.info("Mission completed")

    def _fail(self):
        self.state = MissionState.FAILED
        self._logger.error("Mission failed")

    # def _request_step(self, index: int) -> None:
    #     index = max(0, min(index, len(self.plan)))
    #     if index == self._req_step_idx:
    #         return
    #     if (
    #         self.state == MissionState.RUNNING
    #         and self.curr_step is not None
    #         and self.curr_step.block._state == MBState.RUNNING
    #     ):
    #         self.curr_step.block.cancel()
    #     self._req_step_idx = index
    #     self.curr_step = None
    #     if index < len(self.plan):
    #         step = self.plan[index]
    #         step.block = step.block_class(
    #             mission_env=self._menv,
    #             **step.block_params,
    #         )

    # def _handle_command(self, command):
    #     match command:
    #         case MissionCommand.TOGGLE:
    #             if self.state != MissionState.RUNNING:
    #                 if not self.plan:
    #                     self._complete()
    #                     return
    #                 if self._req_step_idx >= len(self.plan):
    #                     self._req_step_idx = 0
    #                 step = self.plan[self._req_step_idx]
    #                 step.block = step.block_class(
    #                     mission_env=self._menv,
    #                     **step.block_params,
    #                 )
    #                 self.curr_step = None
    #                 self.state = MissionState.RUNNING
    #             elif self.state in (MissionState.RUNNING, MissionState.PAUSED):
    #                 if (
    #                     self.curr_step is not None
    #                     and self.curr_step.block.status == MBState.RUNNING
    #                 ):
    #                     self.curr_step.block.cancel()
    #                 self.state = MissionState.IDLE
    #         case MissionCommand.PAUSE:
    #             if self.state == MissionState.RUNNING:
    #                 self.state = MissionState.PAUSED
    #         case MissionCommand.RESUME:
    #             if self.state == MissionState.PAUSED:
    #                 self.state = MissionState.RUNNING
    #         case MissionCommand.NEXT:
    #             self._request_step(self._req_step_idx + 1)
    #         case MissionCommand.PREV:
    #             self._request_step(self._req_step_idx - 1)

    # def _process_commands(self):
    #     while True:
    #         if self.state in [MissionState.FAILED, MissionState.COMPLETED]:
    #             return
    #         try:
    #             command = self.command_queue.get_nowait()
    #         except Empty:
    #             return
    #         self._handle_command(command)

    def _handle_step_fail(self):
        match self._curr_step.on_failure:
            case FailureAction.SKIP:
                self._req_step_idx += 1
            case FailureAction.FAIL:
                self._fail()
            case FailureAction.RETRY:
                for _ in range(self._curr_step.max_retries):
                    self._curr_step.block = self._curr_step.block_class(
                        mission_env=self._menv,
                        **self._curr_step.block_params,
                    )
                    self._curr_step.block.start()
                    result = self._curr_step.block.tick()
                    match result:
                        case MBState.RUNNING:
                            return
                        case MBState.SUCCEEDED:
                            self._req_step_idx += 1
                            return
                        case MBState.FAILED:
                            time.sleep(self._curr_step.retry_delay)
                        case MBState.CANCELLED:
                            self._fail()
                            return
                self._fail()

    def _select_step(self):
        if self._req_step_idx == len(self.plan) and self.state == MissionState.RUNNING:
            self._complete()
            return
        match self.state:
            case MissionState.RUNNING:
                self._curr_step = self.plan[self._req_step_idx]
            case MissionState.PAUSED:
                pass
                # self.curr_step = self.pause_step
            case MissionState.COMPLETED:
                self.stop()
                # self.curr_step = self.end_step
            case MissionState.FAILED:
                self.stop()
                # self.curr_step = self.fail_step
        if not self._curr_step.block.is_started():
            self._curr_step.block.start()

    def _handle_regular_step(self, res):
        match res:
            case MBState.SUCCEEDED:
                self._req_step_idx += 1
            case MBState.FAILED:
                self._handle_step_fail()
            case MBState.CANCELLED:
                self._fail()

    # def _handle_pause_step(self, res):
    #     match res:
    #         case MBState.SUCCEEDED:
    #             return
    #         case (
    #             MBState.FAILED
    #             | MBState.CANCELLED
    #         ):
    #             self._fail()

    # def _handle_end_step(self, res):
    #     match res:
    #         case MBState.SUCCEEDED:
    #             self.stop()
    #         case (
    #             MBState.FAILED
    #             | MBState.CANCELLED
    #         ):
    #             self._fail()

    # def _handle_fail_step(self, res):
    #     self.stop()

    def _tick_step(self):
        res = self._curr_step.block.tick()
        if res == MBState.RUNNING:
            return
        match self.state:
            case MissionState.RUNNING:
                self._handle_regular_step(res)
            case MissionState.PAUSED:
                pass
                # self._handle_pause_step(res)
            case MissionState.COMPLETED:
                pass
                # self._handle_end_step(res)
            case MissionState.FAILED:
                pass
                # self._handle_fail_step(res)

    def _ticker(self):
        while not self._stop_event.is_set():
            try:
                # self._process_commands()
                if self.state != MissionState.RUNNING:
                    self._stop_event.wait(self._tick_period)
                    continue
                self._select_step()
                if self.state == MissionState.RUNNING:
                    self._tick_step()
                self._stop_event.wait(self._tick_period)
            except Exception:
                self._menv.logger.exception("Ticker failed")
                self._fail()

    def stop(self):
        self._stop_event.set()

        # self.ros_executor.shutdown()
        # if self.image_thread.is_alive():
        #     self.image_thread.join()
        # self._menv.image_receiver.destroy_node()
        # if self.ros_context.ok():
        #     self.ros_context.shutdown()

        if self._ticker_thread.is_alive() and self._ticker_thread is not threading.current_thread():
            self._ticker_thread.join()
        return
