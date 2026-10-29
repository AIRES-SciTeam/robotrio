from abc import ABC, abstractmethod
from enum import Enum, auto
import threading
import math

from .mission_env import MissionEnv
from .commander import SendingResult
from .logger_fabric import Logger
from .commander import (
    PACE_LIST, GAIT_LIST
)

# region MUResult
class MUResult(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    STOPPED = auto()
# endregion

# region MissionUnit
class MissionUnit(ABC):
    def __init__(self):
        self._name = type(self).__name__

        self._menv : MissionEnv | None = None

        self._stop_event = threading.Event()

    def prepare(self, menv : MissionEnv):
        self._menv = menv
        self._menv.logger.debug(f"{self._name} prepared") 

    def _check(self):
        if self._menv is None:
            raise RuntimeError(f"{self._name} is not prepared")

    def interrupt(self):
        self._check()
        self._stop_event.set()
        self._menv.logger.warning("%s interrupt requested", self._name)

    def reset(self):
        self._stop_event.clear()

    @abstractmethod
    def act(self) -> MUResult:
        pass
# endregion

# region SetPace
class SetPace(MissionUnit):
    def __init__(
        self,
        pace : str,
    ):
        if not pace in PACE_LIST:
            raise ValueError(f"Invalid pace: {pace!r}")
        self.pace = pace
        super().__init__()

    def act(self) -> MUResult:
        self._check()
        if self._stop_event.is_set():
            return MUResult.STOPPED
        try:
            res = self._menv.commander.set_pace(self.pace)
            if res != SendingResult.SUCCESS:
                return MUResult.FAILURE
        except Exception as e:
            self._menv.logger.error("Error during setting pace: %s", e)
            return MUResult.FAILURE
        return MUResult.SUCCESS
# endregion

# region SetGait
class SetGait(MissionUnit):
    def __init__(
        self,
        gait : str,
    ):
        if not gait in GAIT_LIST:
            raise ValueError(f"Invalid gait: {gait!r}")
        self.gait = gait
        super().__init__()

    def act(self) -> MUResult:
        self._check()
        if self._stop_event.is_set():
            return MUResult.STOPPED
        try:
            res = self._menv.commander.set_gait(self.gait)
            if res != SendingResult.SUCCESS:
                return MUResult.FAILURE
        except Exception as e:
            self._menv.logger.error("Error during setting gait: %s", e)
            return MUResult.FAILURE
        return MUResult.SUCCESS
# endregion

# region FindTag
class FindTag(MissionUnit):
    def __init__(
        self,
        tag: str,  # "family:tag"
        turn_angle: float,
    ):
        if not isinstance(tag, str):
            raise ValueError(f"Invalid tag: {tag!r}; expected family:id")
        try:
            self.family, tag_id = tag.split(":")
            self.tag_id = int(tag_id)
        except ValueError as e:
            raise ValueError(f"Invalid tag: {tag!r}; expected family:id") from e
        if not self.family or self.tag_id < 0:
            raise ValueError(f"Invalid tag: {tag!r}; expected family:id")
        if (
            isinstance(turn_angle, bool)
            or not isinstance(turn_angle, (int, float))
            or not math.isfinite(turn_angle)
            or turn_angle <= 0
        ):
            raise ValueError("turn_angle must be finite and positive")
        self._turn_speed = 10
        self._turn_duration = turn_angle / self._turn_speed
        super().__init__()

    def act(self):
        self._check()
        last_frame_ts = None
        result = MUResult.STOPPED
        try:
            if self._menv.commander.stop_motion() != SendingResult.SUCCESS:
                result = MUResult.FAILURE
                raise RuntimeError("Can't stop motion")
            attempt = 1
            while not self._stop_event.is_set():
                timestamp, frame = self._menv.image_receiver.get_image()
                if frame is None or timestamp == last_frame_ts:
                    self._menv.logger.warning("Current timestamp: %s, last timestamp: %s", timestamp, last_frame_ts)
                    if attempt == 5:
                        raise RuntimeError("No new frame last 0.25 secs")
                    attempt += 1
                    self._stop_event.wait(0.05)
                    continue
                attempt = 1
                last_frame_ts = timestamp
                if not self.family in self._menv.tag_detectors:
                    raise ValueError(f"No tag detector for family: {self.family}")
                detections = self._menv.tag_detectors[self.family].detect(frame)
                for detection in detections:
                    if detection.tag_id == self.tag_id:
                        self._menv.logger.info("Found tag %s", self.tag_id)
                        result = MUResult.SUCCESS
                        break
                if result == MUResult.SUCCESS:
                    break
                if self._stop_event.is_set():
                    result =  MUResult.STOPPED
                    break
                res = self._menv.commander.set_velocity(
                    lin_vel=(0, 0),
                    ang_vel=self._turn_speed,
                    approx_time_limit=self._turn_duration
                )
                if res != SendingResult.SUCCESS:
                    raise RuntimeError(f"Could not start search rotation: {res}")
                if self._stop_event.wait(self._turn_duration * 1.2):
                    result = MUResult.STOPPED
        except Exception as e:
            self._menv.logger.error("Error during tag search: %s", e)
            result = MUResult.FAILURE
        finally:
            try:
                if self._menv.commander.stop_motion() != SendingResult.SUCCESS:
                    result = MUResult.FAILURE
            except Exception as e:
                self._menv.logger.error("Error stopping motion: %s", e)
                result = MUResult.FAILURE
        return result
