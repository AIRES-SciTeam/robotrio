from abc import ABC, abstractmethod
from enum import Enum, auto

from mission.mission_env import MissionEnv


class MBState(Enum):
    PENDING = auto()
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()

class MissionBlock(ABC):
    def __init__(
        self,
        name : str,
        mission_env : MissionEnv
    ):
        self.name = name
        self.menv = mission_env
        self._state = MBState.PENDING
        self.menv.logger.info(f"{self.name} Initialized")

    def start(self):
        self.menv.logger.info(f"{self.name} Started")
        self._state = MBState.RUNNING

    def is_started(self):
        return self._state != MBState.PENDING

    def cancel(self):
        self.menv.logger.info(f"{self.name} Cancelled")
        self._state = MBState.CANCELLED

    def _can_tick(self):
        if self._state == MBState.RUNNING:
            return True
        self.menv.logger.warning(f"{self.name} is {self._state}")
        return False

    def _complete(self):
        self.menv.logger.warning(f"{self.name} Succeeded")
        self._state = MBState.SUCCEEDED

    def _fail(self, err : str = "commander fail"):
        self.menv.logger.warning(f"{self.name} Failed: {err}")
        self._state = MBState.FAILED

    @abstractmethod
    def tick(self) -> MBState:
        pass

