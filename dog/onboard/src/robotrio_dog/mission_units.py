from abc import ABC, abstractmethod
from enum import Enum, auto
import threading
import math
import time
import numpy as np

from .mission_env import MissionEnv
from .commander import SendingResult
from .logger_fabric import Logger
from .commander import (
    PACE_LIST, GAIT_LIST, DOG_TURN_ABS_LIMIT, DOG_PITCH_LIMIT
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
        tag: str,  # "family:id"
        clockwise : bool = True
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
        self._turn_speed = -10 if clockwise else 10
        self._turn_duration = 2
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
# engregion

# region Move
class Move(MissionUnit):
    def __init__(
        self,
        x_dist : float,
        y_dist : float
    ):
        self._xdist = x_dist
        self._ydist = y_dist

        self._xspeed = 20
        self._yspeed = 10

        super().__init__()

    def act(self):
        self._check()
        try:
            init_pos = self._menv.commander.get_telemetry().imu
# endregion

# region ReachTag
class ReachTag(MissionUnit):
    def __init__(self, tag: str, tag_size: float, distance: float):
        try:
            self.family, tag_id = tag.split(":")
            self.tag_id = int(tag_id)
        except (AttributeError, ValueError) as e:
            raise ValueError("tag must have the form family:id") from e
        if not self.family or self.tag_id < 0:
            raise ValueError("tag must have a family and nonnegative ID")
        self.tag_size = tag_size
        self.dist = distance
        super().__init__()

    def act(self) -> MUResult:
        self._check()
        commander = self._menv.commander
        result = MUResult.STOPPED
        last_frame_ts = None
        last_frame_at = time.monotonic()
        step_duration = 2.0
        try:
            if commander.stop_motion() != SendingResult.SUCCESS:
                raise RuntimeError("Can't stop before approach")
            # Discard the image available before the initial stop.
            last_frame_ts, _ = self._menv.image_receiver.get_image()
            last_frame_at = time.monotonic()
            detector = self._menv.tag_detectors[self.family]
            while not self._stop_event.is_set():
                timestamp, frame = self._menv.image_receiver.get_image()
                if frame is None or timestamp == last_frame_ts:
                    if time.monotonic() - last_frame_at >= 1:
                        raise TimeoutError("No fresh camera frame for 1 second")
                    self._stop_event.wait(0.05)
                    continue
                last_frame_ts = timestamp
                last_frame_at = time.monotonic()
                tags = [d for d in detector.detect(frame) if d.tag_id == self.tag_id]
                if self._stop_event.is_set():
                    break
                if not tags:
                    self._menv.logger.info("ReachTag: no target after stop; waiting without moving")
                    self._stop_event.wait(0.05)
                    continue
                detection = max(tags, key=lambda d: d.center[1])
                vertical_position = float(detection.center[1]) / frame.shape[0]
                self._menv.logger.info("ReachTag %s:%d: center_y=%.1f%%, stop_at=85%%",
                                       self.family, self.tag_id, vertical_position * 100)
                if vertical_position >= 0.85:
                    result = MUResult.SUCCESS
                    break
                if commander.set_velocity(
                    lin_vel=(25, 0), ang_vel=0, approx_time_limit=step_duration,
                ) != SendingResult.SUCCESS:
                    raise RuntimeError("Can't move forward")
                self._menv.logger.info("ReachTag: forward step for %.1f seconds", step_duration)
                # Do not run detection during the step; cancellation still interrupts it.
                if self._stop_event.wait(step_duration):
                    break
                if commander.stop_motion() != SendingResult.SUCCESS:
                    raise RuntimeError("Can't stop after forward step")
                self._menv.logger.info("ReachTag: step stopped; waiting for fresh frame")
                # Ignore the most recent image from motion. Analyze the next one.
                last_frame_ts, _ = self._menv.image_receiver.get_image()
                last_frame_at = time.monotonic()
        except Exception as e:
            self._menv.logger.exception("ReachTag failed: %s", e)
            result = MUResult.FAILURE
        finally:
            try:
                if commander.stop_motion() != SendingResult.SUCCESS:
                    result = MUResult.FAILURE
            except Exception as e:
                self._menv.logger.error("Error stopping approach: %s", e)
                result = MUResult.FAILURE
        return result
# endregion

# region TurnByAngle
class TurnByAngle(MissionUnit):
    def __init__(
        self, 
        angle: float, 
        speed: float = 20, 
        tolerance: float = 1,
        timeout: float = 90, 
        telemetry_timeout: float = 1,
        yaw_sign: int = 1
    ):
        for name, value in (("angle", angle), ("speed", speed),
                            ("tolerance", tolerance), ("timeout", timeout),
                            ("telemetry_timeout", telemetry_timeout)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
        if not 0 < speed <= DOG_TURN_ABS_LIMIT:
            raise ValueError(f"speed must be in (0, {DOG_TURN_ABS_LIMIT}]")
        if not 0 < tolerance < 180 or timeout <= 0 or telemetry_timeout <= 0:
            raise ValueError("tolerance must be in (0, 180); timeouts must be positive")
        if isinstance(yaw_sign, bool) or yaw_sign not in (-1, 1):
            raise ValueError("yaw_sign must be -1 or 1")
        self.angle = angle
        self.speed = speed
        self.tolerance = tolerance
        self.timeout = timeout
        self.telemetry_timeout = telemetry_timeout
        self.yaw_sign = yaw_sign
        super().__init__()

    def _fresh_yaw(self, after, deadline):
        sample_deadline = min(deadline, time.monotonic() + self.telemetry_timeout)
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= sample_deadline:
                raise TimeoutError("Timed out waiting for fresh yaw telemetry")
            sample = self._menv.commander.get_telemetry()
            now = time.monotonic()
            if (sample.last_asked_at is not None and sample.last_asked_at > after
                    and 0 <= now - sample.last_asked_at <= self.telemetry_timeout
                    and isinstance(sample.yaw, (int, float)) and math.isfinite(sample.yaw)):
                return sample.yaw
            self._stop_event.wait(0.02)
        return None

    def act(self) -> MUResult:
        self._check()
        result = MUResult.STOPPED
        commander = self._menv.commander
        try:
            if commander.stop_motion() != SendingResult.SUCCESS:
                raise RuntimeError("Can't stop motion before turn")
            deadline = time.monotonic() + self.timeout
            previous_yaw = self._fresh_yaw(time.monotonic(), deadline)
            turned = 0.0
            moving = False
            direction = math.copysign(1, self.angle)
            while previous_yaw is not None and not self._stop_event.is_set():
                error = self.angle - turned
                self._menv.logger.debug("TurnByAngle: turned=%.2f, remaining=%.2f", turned, error)
                # Stop at the target or on crossing it; do not reverse to correct overshoot.
                if abs(error) <= self.tolerance or direction * error <= 0:
                    self._menv.logger.info(
                        "TurnByAngle target reached: requested=%.2f, measured=%.2f",
                        self.angle, turned,
                    )
                    result = MUResult.SUCCESS
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("TurnByAngle timed out")
                if not moving:
                    velocity = direction * self.speed * self.yaw_sign
                    if commander.set_velocity(
                        lin_vel=(0, 0), ang_vel=velocity,
                        approx_time_limit=remaining,
                    ) != SendingResult.SUCCESS:
                        raise RuntimeError("Can't start turn")
                    moving = True
                yaw = self._fresh_yaw(time.monotonic(), deadline)
                if yaw is None:
                    break
                turned += (yaw - previous_yaw + 180) % 360 - 180
                previous_yaw = yaw
        except Exception as e:
            self._menv.logger.error("TurnByAngle failed: %s", e)
            result = MUResult.FAILURE
        finally:
            try:
                if commander.stop_motion() != SendingResult.SUCCESS:
                    result = MUResult.FAILURE
            except Exception as e:
                self._menv.logger.error("Error stopping after turn: %s", e)
                result = MUResult.FAILURE
        return result
#endregion
