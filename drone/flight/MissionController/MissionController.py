import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Callable
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
import threading
from typing import Any
from queue import SimpleQueue, Empty
import time

from .MissionBlocks import *
from Commander.MAVLinkCommander import DRONE_MAVLinkCommander
from Utils.Configs import DRONE_ConnConfig


class FailureAction(Enum):
    SKIP = auto()
    RETRY = auto()
    FAIL = auto()

@dataclass
class DRONE_MissionStep:
    block_class : type[DRONE_MissionBlock]
    block_params : dict[str, Any] = field(default_factory=dict)

    on_failure : FailureAction = FailureAction.FAIL
    max_retries : int = 0
    retry_delay : float = 0.5 # s

    block : DRONE_MissionBlock = field(init=False)

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
        plan : List[DRONE_MissionStep],
        commander : DRONE_MAVLinkCommander,
        logger : logging.Logger,
        conn_config : DRONE_ConnConfig,
        tag_family : str,
        camera_rotation : np.ndarray,
        camera_position : np.ndarray,
        tick_period : float, # s
        pause_step : DRONE_MissionStep,
        end_step : DRONE_MissionStep,
        fail_step : DRONE_MissionStep
    ):
        self.plan = plan

        self.ros_context = Context()
        rclpy.init(context=self.ros_context)
        image_reciever = DRONE_ImageReciever(
            conn_config=conn_config,
            logger=logger,
            ros_context=self.ros_context
        )
        self.ros_executor = SingleThreadedExecutor(context=self.ros_context)
        self.ros_executor.add_node(image_reciever)
        tag_detector = DRONE_TagDetector(
            tag_family=tag_family,
            logger=logger
        )

        self.menv = DRONE_MissionEnv(
            commander=commander,
            image_reciever=image_reciever,
            tag_detector=tag_detector,
            logger=logger,
            camera_position=camera_position,
            camera_rotation=camera_rotation
        )
        self.tick_period = tick_period
        self.pause_step = pause_step
        self.end_step = end_step
        self.fail_step = fail_step

        self.stop_event = threading.Event()
        self._stop_lock = threading.Lock()
        self._stopped = False
        self.image_thread = threading.Thread(
            target=self.ros_executor.spin,
            name="DRONE_ImageReciever",
            daemon=True
        )
        self.ticker_thread = threading.Thread(
            target=self._ticker,
            daemon=True
        )

        self.state = MissionState.IDLE
        self.command_queue : SimpleQueue[MissionCommand] = SimpleQueue()
        self.curr_step : DRONE_MissionStep | None = None
        self.req_step_idx = 0

        self._init_plan()

    def _init_plan(self):
        for step in self.plan:
            step.block = step.block_class(
                mission_env=self.menv,
                **step.block_params
            )
        self.pause_step.block = self.pause_step.block_class(
            mission_env=self.menv,
            **self.pause_step.block_params
        )
        self.end_step.block = self.end_step.block_class(
            mission_env=self.menv,
            **self.end_step.block_params
        )
        self.fail_step.block = self.fail_step.block_class(
            mission_env=self.menv,
            **self.fail_step.block_params
        )

    def start(self):
        self.state = MissionState.IDLE
        self.stop_event.clear()
        self.image_thread.start()
        self.ticker_thread.start()

    def toggle(self):
        self.command_queue.put(MissionCommand.TOGGLE)

    def pause(self):
        self.command_queue.put(MissionCommand.PAUSE)

    def resume(self):
        self.command_queue.put(MissionCommand.RESUME)

    def next(self):
        self.command_queue.put(MissionCommand.NEXT)

    def prev(self):
        self.command_queue.put(MissionCommand.PREV)

    def _complete(self):
        self.state = MissionState.COMPLETED

    def _fail(self):
        self.state = MissionState.FAILED

    def _request_step(self, index: int) -> None:
        index = max(0, min(index, len(self.plan)))
        if index == self.req_step_idx:
            return
        if (
            self.state == MissionState.RUNNING
            and self.curr_step is not None
            and self.curr_step.block.status == DRONE_MissionBlockStatus.RUNNING
        ):
            self.curr_step.block.cancel()
        self.req_step_idx = index
        self.curr_step = None
        if index < len(self.plan):
            step = self.plan[index]
            step.block = step.block_class(
                mission_env=self.menv,
                **step.block_params,
            )

    def _handle_command(self, command):
        match command:
            case MissionCommand.TOGGLE:
                if self.state == MissionState.IDLE:
                    if not self.plan:
                        self._complete()
                        return
                    if self.req_step_idx >= len(self.plan):
                        self.req_step_idx = 0
                    step = self.plan[self.req_step_idx]
                    step.block = step.block_class(
                        mission_env=self.menv,
                        **step.block_params,
                    )
                    self.curr_step = None
                    self.state = MissionState.RUNNING
                elif self.state in (MissionState.RUNNING, MissionState.PAUSED):
                    if (
                        self.curr_step is not None
                        and self.curr_step.block.status == DRONE_MissionBlockStatus.RUNNING
                    ):
                        self.curr_step.block.cancel()
                    self.state = MissionState.IDLE
            case MissionCommand.PAUSE:
                if self.state == MissionState.RUNNING:
                    self.state = MissionState.PAUSED
            case MissionCommand.RESUME:
                if self.state == MissionState.PAUSED:
                    self.state = MissionState.RUNNING
            case MissionCommand.NEXT:
                self._request_step(self.req_step_idx + 1)
            case MissionCommand.PREV:
                self._request_step(self.req_step_idx - 1)

    def _process_commands(self):
        while True:
            if self.state in [MissionState.FAILED, MissionState.COMPLETED]:
                return
            try:
                command = self.command_queue.get_nowait()
            except Empty:
                return
            self._handle_command(command)

    def _handle_step_fail(self):
        match self.curr_step.on_failure:
            case FailureAction.SKIP:
                self.req_step_idx += 1
            case FailureAction.FAIL:
                self._fail()
            case FailureAction.RETRY:
                for _ in range(self.curr_step.max_retries):
                    self.curr_step.block = self.curr_step.block_class(
                        mission_env=self.menv,
                        **self.curr_step.block_params,
                    )
                    self.curr_step.block.start()
                    result = self.curr_step.block.tick()
                    match result:
                        case DRONE_MissionBlockStatus.RUNNING:
                            return
                        case DRONE_MissionBlockStatus.SUCCEEDED:
                            self.req_step_idx += 1
                            return
                        case DRONE_MissionBlockStatus.FAILED:
                            time.sleep(self.curr_step.retry_delay)
                        case DRONE_MissionBlockStatus.CANCELLED:
                            self._fail()
                            return
                self._fail()

    def _select_step(self):
        if self.req_step_idx == len(self.plan) and self.state == MissionState.RUNNING:
            self._complete()
        match self.state:
            case MissionState.RUNNING:
                self.curr_step = self.plan[self.req_step_idx]
            case MissionState.PAUSED:
                self.curr_step = self.pause_step
            case MissionState.COMPLETED:
                self.curr_step = self.end_step
            case MissionState.FAILED:
                self.curr_step = self.fail_step
        if not self.curr_step.block.is_stated():
            self.curr_step.block.start()

    def _handle_regular_step(self, res):
        match res:
            case DRONE_MissionBlockStatus.SUCCEEDED:
                self.req_step_idx += 1
            case DRONE_MissionBlockStatus.FAILED:
                self._handle_step_fail()
            case DRONE_MissionBlockStatus.CANCELLED:
                self._fail()

    def _handle_pause_step(self, res):
        match res:
            case DRONE_MissionBlockStatus.SUCCEEDED:
                return
            case (
                DRONE_MissionBlockStatus.FAILED
                | DRONE_MissionBlockStatus.CANCELLED
            ):
                self._fail()

    def _handle_end_step(self, res):
        match res:
            case DRONE_MissionBlockStatus.SUCCEEDED:
                self.stop()
            case (
                DRONE_MissionBlockStatus.FAILED
                | DRONE_MissionBlockStatus.CANCELLED
            ):
                self._fail()

    def _handle_fail_step(self, res):
        self.stop()

    def _tick_step(self):
        res = self.curr_step.block.tick()
        if res == DRONE_MissionBlockStatus.RUNNING:
            return
        match self.state:
            case MissionState.RUNNING:
                self._handle_regular_step(res)
            case MissionState.PAUSED:
                self._handle_pause_step(res)
            case MissionState.COMPLETED:
                self._handle_end_step(res)
            case MissionState.FAILED:
                self._handle_fail_step(res)

    def _ticker(self):
        while not self.stop_event.is_set():
            try:
                self._process_commands()
                if self.state == MissionState.IDLE:
                    self.stop_event.wait(self.tick_period)
                    continue
                self._select_step()
                self._tick_step()
                self.stop_event.wait(self.tick_period)
            except Exception:
                self.menv.logger.exception("DRONE_MissionController: Ticker failed")
                self.stop_event.set()

    def stop(self):
        with self._stop_lock:
            if self._stopped:
                return
            self._stopped = True

        self.stop_event.set()

        self.ros_executor.shutdown()
        if self.image_thread.is_alive():
            self.image_thread.join()
        self.menv.image_reciever.destroy_node()
        if self.ros_context.ok():
            self.ros_context.shutdown()

        if self.ticker_thread.is_alive() and self.ticker_thread is not threading.current_thread():
            self.ticker_thread.join()
