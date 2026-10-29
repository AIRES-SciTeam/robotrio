from enum import Enum, auto
from typing import Dict, Any, List
import threading, time
from contextlib import nullcontext

from .logger_fabric import LoggerFabric, MAIN_LOGGER_NAME
from .commander import Commander
from .image_handlers import ImageReceiver, TagDetector
from .mission_env import MissionEnv
from .mission_units import *


# region MEState
class MEState(Enum):
    PENDING = auto()
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    STOPPED = auto()
# endregion

# region MECommand
class MECommand(Enum):
    NONE = auto()
    STOP = auto()
    PAUSE = auto()
    PLAY = auto()
    NEXT = auto()
    PREV = auto()
# endregion

# region MissionExecutor
class MissionExecutor:
    def __init__(
        self,
        config : Dict[str, Any],
        logger_fabric : LoggerFabric,
        commander : Commander,
        image_receiver : ImageReceiver,
        tag_detectors : Dict[str, TagDetector],
    ):
        self._logger = logger_fabric.get_logger(
            name=f"{MAIN_LOGGER_NAME}.{type(self).__name__}",
            level=config.get("log_level", "info")
        )
        self._logger.info("Initializing")

        self._menv = MissionEnv(
            commander=commander,
            logger=self._logger,
            image_receiver=image_receiver,
            tag_detectors=tag_detectors
        )

        self._stop_event = threading.Event()
        self._stop_event.set()

        self._state = MEState.PENDING
        self._command : MECommand = MECommand.NONE
        self._lock = threading.Lock()

        self._curr_unit_idx : int = 0
        self._plan : List[MissionUnit] | None = None

        self._logger.info("Initialized: state=%s", self._state.name)

    def _interrupt(self):
        try:
            self._plan[self._curr_unit_idx].interrupt()
            self._logger.debug("Interrupt requested: unit=%d (%s)",
                               self._curr_unit_idx + 1, type(self._plan[self._curr_unit_idx]).__name__)
        except Exception as e:
            self._logger.warning("Failed to request interruption: index=%d, state=%s",
                                 self._curr_unit_idx, self._state.name, exc_info=True)

    def reset(self):
        with self._lock:
            if not self._state in (MEState.IDLE, MEState.STOPPED):
                self._logger.warning("Reset rejected: state=%s", self._state.name)
                raise RuntimeError("Can't reset an active or unconfigured mission")
            self._curr_unit_idx = 0
            self._command = MECommand.NONE
            self._state = MEState.IDLE
            self._stop_event.clear()
            self._logger.info("Mission reset: state=IDLE, next unit=1")

    def stop(self):
        with self._lock:
            self._logger.info("Command STOP requested: state=%s, index=%d, replacing=%s",
                              self._state.name, self._curr_unit_idx, self._command.name)
            self._command = MECommand.STOP
            self._interrupt()

    def pause(self):
        with self._lock:
            if self._command == MECommand.STOP:
                self._logger.debug("Command PAUSE ignored: STOP is pending")
                return
            self._logger.info("Command PAUSE requested: state=%s, index=%d, replacing=%s",
                              self._state.name, self._curr_unit_idx, self._command.name)
            self._command = MECommand.PAUSE
            self._interrupt()

    def play(self):
        with self._lock:
            if self._command == MECommand.STOP:
                self._logger.debug("Command PLAY ignored: STOP is pending")
                return
            self._logger.info("Command PLAY requested: state=%s, index=%d, replacing=%s",
                              self._state.name, self._curr_unit_idx, self._command.name)
            self._command = MECommand.PLAY
    
    def next_unit(self):
        with self._lock:
            if self._command == MECommand.STOP:
                self._logger.debug("Command NEXT ignored: STOP is pending")
                return
            self._logger.info("Command NEXT requested: state=%s, index=%d, replacing=%s",
                              self._state.name, self._curr_unit_idx, self._command.name)
            self._command = MECommand.NEXT
            self._interrupt()

    def prev_unit(self):
        with self._lock:
            if self._command == MECommand.STOP:
                self._logger.debug("Command PREV ignored: STOP is pending")
                return
            self._logger.info("Command PREV requested: state=%s, index=%d, replacing=%s",
                              self._state.name, self._curr_unit_idx, self._command.name)
            self._command = MECommand.PREV
            self._interrupt()

    def configure(
        self,
        plan : List[MissionUnit]
    ):
        with self._lock:
            if self._state not in (MEState.PENDING, MEState.IDLE, MEState.STOPPED):
                self._logger.warning("Configure rejected: state=%s", self._state.name)
                raise RuntimeError("Can't configure an active mission")
            self._plan = plan
            self._curr_unit_idx = 0
            self._logger.debug("Preparing plan: units=%d", len(self._plan))
            for unit in self._plan:
                unit.prepare(self._menv)
            self._state = MEState.IDLE
            self._logger.info("Plan loaded: units=%d, state=IDLE", len(self._plan))       
            self._curr_unit_idx = 0
            self._command = MECommand.NONE
            self._state = MEState.IDLE
            self._stop_event.clear()

    def _handle_command(self):
        command = self._command
        previous_state = self._state
        previous_index = self._curr_unit_idx
        match self._command:
            case MECommand.STOP:
                if self._state in (MEState.RUNNING, MEState.PAUSED):
                    self._state = MEState.STOPPED
                    self._stop_event.set()
            case MECommand.PAUSE:
                if self._state == MEState.RUNNING:
                    self._state = MEState.PAUSED
            case MECommand.PLAY:
                if self._state == MEState.PAUSED:
                    self._state = MEState.RUNNING
            case MECommand.NEXT:
                if self._state == MEState.RUNNING:
                    self._curr_unit_idx = min(
                        self._curr_unit_idx + 1,
                        len(self._plan)
                    )
            case MECommand.PREV:
                if self._state == MEState.RUNNING:
                    self._curr_unit_idx = max(
                        self._curr_unit_idx - 1,
                        0
                    )
        if command != MECommand.NONE:
            self._logger.debug(
                "Command %s processed: state=%s -> %s, index=%d -> %d",
                command.name, previous_state.name, self._state.name,
                previous_index, self._curr_unit_idx,
            )
            if self._state != previous_state:
                self._logger.info("Mission state: %s -> %s (command=%s)",
                                  previous_state.name, self._state.name, command.name)
        self._command = MECommand.NONE

    def execute(self):
        with self._lock:
            if self._state != MEState.IDLE:
                self._logger.warning("Execute rejected: state=%s", self._state.name)
                return
            self._state = MEState.RUNNING
            self._logger.info("Mission started: units=%d, index=%d",
                              len(self._plan), self._curr_unit_idx)
        while True:
            with self._lock:
                self._handle_command()
                if self._stop_event.is_set():
                    break
                paused = self._state == MEState.PAUSED
                if not paused:
                    if self._curr_unit_idx >= len(self._plan):
                        self._logger.info("End of plan reached")
                        self._state = MEState.STOPPED
                        self._stop_event.set()
                        break
                    unit = self._plan[self._curr_unit_idx]
                    unit.reset()
                    unit_number = self._curr_unit_idx + 1
                    unit_name = type(unit).__name__
            if paused:
                self._stop_event.wait(0.1)
                continue
            self._logger.info("Unit %d (%s) started", unit_number, unit_name)
            started_at = time.monotonic()
            res = unit.act()
            log_result = self._logger.error if res == MUResult.FAILURE else self._logger.info
            log_result("Unit %d (%s) finished: result=%s, elapsed=%.3fs",
                       unit_number, unit_name, res.name, time.monotonic() - started_at)
            with self._lock:
                if self._command == MECommand.NONE:
                    match res:
                        case MUResult.SUCCESS:
                            self._command = (
                                MECommand.NEXT 
                                if self._curr_unit_idx < len(self._plan) - 1 
                                else MECommand.STOP
                            )
                        case MUResult.FAILURE:
                            self._command = MECommand.STOP
                else:
                    self._logger.debug("Pending command %s takes priority over unit result %s",
                                       self._command.name, res.name)
            wait = self._stop_event.wait(0.1)
            if wait:
                break
        self._logger.info("Mission execution stopped")
# endregion
