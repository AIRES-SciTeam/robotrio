from xgolib import XGO_DOG
from typing import Dict, List, Any, Callable
from dataclasses import dataclass, replace
import threading, time

from .logger_fabric import LoggerFabric, MAIN_LOGGER_NAME

@dataclass
class Telemetry:
    battery : int | None = None
    roll : float | None = None
    pitch : float | None = None
    yaw : float | None = None
    imu : List | None = None
    motor : List | None = None

    last_asked_at : float | None = None

class TelemetryManager:
    def __init__(
        self, 
        config : Dict[str, Any],
        logger_fabric : LoggerFabric
    ):
        self._logger = logger_fabric.get_logger(
            name=f"{MAIN_LOGGER_NAME}.{type(self).__name__}",
            level=config.get("log_level", "info")
        )
        self._journal = logger_fabric.get_logger(
            name=f"telemetry_journal",
            level="trace",
            no_format=True
        )
        self._logger.info("Initializing")

        self._state = Telemetry()
        self._state_cond = threading.Condition()
    
        self._rec_rate = config.get("recording_rate", 5.0)
        self._logger.info("recording_rate = %s", self._rec_rate)        
        self._last_rec = None

        self._logger.info("Initialized")

    def update(self, dog : XGO_DOG):
        asked_at = time.monotonic()
        with self._state_cond:
            self._state.last_asked_at = asked_at
            self._state.battery = dog.read_battery()
            self._state.roll = dog.read_roll()
            self._state.pitch = dog.read_pitch()
            self._state.yaw = dog.read_yaw()
            self._state.imu = dog.read_imu()
            self._state.motor = dog.read_motor()
            self._record()
            self._state_cond.notify_all()

    def _record(self):
        now = time.monotonic()
        if (
            self._last_rec is not None
            and now - self._last_rec < (1 / self._rec_rate)
        ):
            return
        state = self._state
        values = [
            state.last_asked_at,
            state.battery,
            state.roll,
            state.pitch,
            state.yaw,
            *state.imu,
            *state.motor,
        ]
        self._journal.critical("|".join(map(str, values)))
        self._last_rec = now

    def reset(self):
        with self._state_cond:
            self._state = Telemetry()
            self._logger.info("Telemetry reset")
            self._state_cond.notify_all()

    def snapshot(self):
        with self._state_cond:
            return replace(self._state)

    def wait_for(self, predicate : Callable[[Telemetry], bool], timeout: float) -> bool:
        with self._state_cond:
            return self._state_cond.wait_for(
                lambda: predicate(replace(self._state)),
                timeout=timeout
            )
