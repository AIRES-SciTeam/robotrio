from xgolib import XGO_DOG
from typing import Dict, Any
import threading, time
from enum import Enum, auto
from contextlib import nullcontext

from .logger_fabric import LoggerFabric, MAIN_LOGGER_NAME
from .telemetry import TelemetryManager


DOG_VX_ABS_LIMIT = 25
DOG_VY_ABS_LIMIT = 18
DOG_TURN_ABS_LIMIT = 100

DOG_X_TRANS_LIMIT = 25
DOG_Y_TRANS_LIMIT = 18
DOG_HEIGHT_LIMIT = (60, 110)
DOG_ROLL_LIMIT = 20
DOG_PITCH_LIMIT = 22
DOG_YAW_LIMIT = 16

GAIT_LIST = ["slow_trot", "trot", "walk", "high_walk"] 
PACE_LIST = ["slow", "normal", "high"]

class SendingResult(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    INVALID = auto()

class Commander:
    # region Class methods
    def __init__(
        self,
        config : Dict[str, Any],
        telemetry_config : Dict[str, Any],
        logger_fabric : LoggerFabric,
    ):
        self._logger = logger_fabric.get_logger(
            name=f"{MAIN_LOGGER_NAME}.{type(self).__name__}",
            level=config.get("log_level", "info")
        )
        self._logger.info("Initializing")

        self._dog_lock = threading.Lock()
        try:
            self._dog = XGO_DOG(
                port=config.get("port", "/dev/ttyAMA0"),
                baud=config.get("baud", 115200) 
            )
        except Exception as e:
            self._logger.critical("Initialization failed: %s", e)
            raise ConnectionError(f"Initialization failed: {e}")
        if self._dog.version == "Null":
            self._dog.ser.close()
            self._logger.critical("Initialization failed: no response from XGO")
            raise ConnectionError("Initialization failed: no response from XGO")

        self._stop_event = threading.Event()

        self._motion_time_limit = None
        
        self._telemetry = TelemetryManager(
            config=telemetry_config,
            logger_fabric=logger_fabric
        )
        self._tm_rate = config.get("telemetry_asking_rate", 5.0)
        self._tm_thread = threading.Thread(target=self._tm_loop, daemon=True)
        self._tm_thread.start()

        self._logger.info("Initialized")

    def close(self):
        self._logger.debug("Stopping")
        if self._stop_event.is_set():
            self._logger.debug("Already stopped")
            return
        self.stop_motion()
        self._stop_event.set()
        self._logger.debug("_stop_event set by close")
        with self._dog_lock:
            try:
                self._dog.stop()
                self._logger.debug("Dog stopped")
            finally:
                self._dog.ser.close()
                self._logger.debug("Dog connection closed")
        if (
            self._tm_thread.is_alive()
            and self._tm_thread is not threading.current_thread()
        ):
            self._tm_thread.join()
            self._logger.debug("_tm_thread joined")
        self._logger.info("Stopped")

    def _tm_loop(self):
        self._logger.info("_tm_thread started")
        while not self._stop_event.is_set():
            with self._dog_lock:
                if (
                    self._motion_time_limit is not None
                    and self._motion_time_limit < time.monotonic()
                ):
                    self.stop_motion(lock=False)
                if self._stop_event.is_set():
                    break
                try:
                    self._telemetry.update(self._dog)
                except Exception as e:
                    self._logger.error("Error while updating telemetry: %s", e)
            self._stop_event.wait(1 / self._tm_rate)
        self._logger.info("_tm_thread stopped")

    # endregion

    # region Dog control

    def stop_motion(self, lock=True) -> SendingResult:
        with self._dog_lock if lock else nullcontext():
            try:
                self._dog.stop()
                self._motion_time_limit = None
                self._logger.debug("Motion stopped")
            except Exception as e:
                self._logger.error("Error while stopping motion: %s", e)
                return SendingResult.FAILURE
            return SendingResult.SUCCESS

    def reset_pose(self) -> SendingResult:
        with self._dog_lock:
            try:
                self._dog.reset()
                self._motion_time_limit = None
                self._logger.debug("Reset pose command sent")
            except Exception as e:
                self._logger.error("Error while sending the reset pose command: %s", e)
                return SendingResult.FAILURE
            return SendingResult.SUCCESS

    def set_velocity(
        self, 
        lin_vel : tuple[float | int | None, 2] = (None, None), 
        ang_vel : float | int | None = None, 
        approx_time_limit : float | None = 10.0,
        relative : bool = False
    ) -> SendingResult:
        if (not(
            (res := self._valid_lin_vel(lin_vel, relative))[0]
            and (res := self._valid_ang_vel(ang_vel, relative))[0] 
            and (res := self._valid_approx_time_limit(approx_time_limit))[0]
        )):
            self._logger.warning("Invalid move parameter: %s", res[1])
            return SendingResult.INVALID
        self._motion_time_limit = time.monotonic() + approx_time_limit
        with self._dog_lock:
            try:
                if lin_vel[0] is not None:
                    self._dog.move_x((lin_vel[0] / 100 * DOG_VX_ABS_LIMIT) if relative else lin_vel[0])
                if lin_vel[1] is not None:
                    self._dog.move_y((lin_vel[1] / 100 * DOG_VY_ABS_LIMIT) if relative else lin_vel[1])
                if ang_vel is not None:
                    self._dog.turn((ang_vel / 100 * DOG_TURN_ABS_LIMIT) if relative else ang_vel)
                self._logger.debug("New velocity set: lin_vel = %s, ang_vel = %s", lin_vel, ang_vel)
            except Exception as e:
                self._logger.error("Error while sending move commands: %s", e)
                return SendingResult.FAILURE
            return SendingResult.SUCCESS

    def set_body_position(
        self, 
        translation : tuple[float | int | None, 3] = (None, None, None),
        rotation : tuple[float | int | None, 3] = (None, None, None),
        relative : bool = False
    ) -> SendingResult:
        if (not(
            (res := self._valid_translation(translation, relative))[0]
            and (res := self._valid_rotation(rotation, relative))[0]
        )):
            self._logger.warning("Invalid body position: %s", res[1])
            return SendingResult.INVALID
        with self._dog_lock:
            try:
                if translation[0] is not None:
                    self._dog.translation("x", translation[0] * (DOG_X_TRANS_LIMIT / 100 if relative else 1))
                if translation[1] is not None:
                    self._dog.translation("y", translation[1] * (DOG_Y_TRANS_LIMIT / 100 if relative else 1))
                if translation[2] is not None:
                    self._dog.translation("z", 
                        translation[2] 
                        * ((DOG_HEIGHT_LIMIT[1] - DOG_HEIGHT_LIMIT[0]) / 100 if relative else 1) 
                        + DOG_HEIGHT_LIMIT[0] if relative else 0
                    )
                if rotation[0] is not None:
                    self._dog.attitude("r", rotation[0] * (DOG_ROLL_LIMIT / 100 if relative else 1))
                if rotation[1] is not None:
                    self._dog.attitude("p", rotation[1] * (DOG_PITCH_LIMIT / 100 if relative else 1))
                if rotation[2] is not None:
                    self._dog.attitude("y", rotation[2] * (DOG_YAW_LIMIT / 100 if relative else 1))
                self._logger.debug("New body position set: translation = %s, rotation = %s", translation, rotation)
            except Exception as e:
                self._logger.error("Error while sending body position commands: %s", e)
                return SendingResult.FAILURE
            return SendingResult.SUCCESS

    def set_gait(self, gait : str | int = "trot"):
        if not (res := self._valid_gait(gait))[0]:
            self._logger.warning("Invalid gait: %s", res[1])
            return SendingResult.INVALID
        with self._dog_lock:
            try:
                self._dog.gait_type(gait.lower() if isinstance(gait, str) else GAIT_LIST[gait])
                self._logger.debug("New gait set")
            except Exception as e:
                self._logger.error("Error while setting the new gait: %s", e)
                return SendingResult.FAILURE
            return SendingResult.SUCCESS

    def set_pace(self, pace: str | int = "slow") -> SendingResult:
        if not (res := self._valid_pace(pace))[0]:
            self._logger.warning("Invalid pace: %s", res[1])
            return SendingResult.INVALID
        with self._dog_lock:
            try:
                self._dog.pace(pace.lower() if isinstance(pace, str) else PACE_LIST[pace])
            except Exception as e:
                self._logger.error("Error while setting pace: %s", e)
                return SendingResult.FAILURE
        return SendingResult.SUCCESS

    def set_motor_angle(self) -> SendingResult:
        self._logger.error("Method set_motor_angle is not implemented yet")
        return SendingResult.FAILURE

    def control_leg(self) -> SendingResult:
        self._logger.error("Method control_leg is not implemented yet")
        return SendingResult.FAILURE

    # TODO: validation
    def control_arm(self, cartesian : tuple[float | int, 2] | None = None, polar : float | None = None, claw : int | None = None):
        if (
            (res := self._valid_cartesian(cartesian))[0]
            and (res := self._valid_polar(polar))[0]
            and (res := self._valid_claw(claw))[0]
        ):
            self._logger.warning("Invalid arm control parameter: %s", res[1])
            return SendingResult.INVALID
        with self._dog_lock:
            try:
                if cartesian is not None:
                    self._dog.arm(*cartesian)
                if polar is not None:
                    self._dog.arm_polar(polar)
                if claw is not None:
                    self._dog.claw(claw)
                self._logger.debug("New arm pose set: cartesian = %s, polar = %s, claw = %s", cartesian, polar, claw)
            except Exception as e:
                self._logger.error("Error during arm control: %s", e)
                return SendingResult.FAILURE
            return SendingResult.SUCCESS

    # endregion
    
    # region Validators

    @staticmethod
    def _valid_lin_vel(lin_vel : tuple[float | int | None, ...], relative : bool) -> tuple[bool, str]:
        if not isinstance(lin_vel, tuple) or len(lin_vel) != 2:
            return False, "lin_vel must be a tuple of size 2"
        if not all(isinstance(x, float | int | type(None)) for x in lin_vel):
            return False, "lin_vel values must be of type float, int, or None"
        if not relative:
            return (
                (abs(lin_vel[0] if lin_vel[0] is not None else 0) <= DOG_VX_ABS_LIMIT) 
                and (abs(lin_vel[1] if lin_vel[1] is not None else 0) <= DOG_VY_ABS_LIMIT)
            ), f"|lin_vel[0]| (X) must be <= {DOG_VX_ABS_LIMIT} and |lin_vel[1]| (Y) must be <= {DOG_VY_ABS_LIMIT}"
        return (
            all(abs(x if x is not None else 0) <= 100 for x in lin_vel), 
            "lin_vel values must be percentages between -100 and 100 in relative mode"
        )
    
    @staticmethod
    def _valid_ang_vel(ang_vel : float | int | None, relative : bool) -> tuple[bool, str]:
        if not isinstance(ang_vel, float | int | type(None)):
            return False, "ang_vel must be of type float, int, or None"
        if not relative:
            return (abs(ang_vel if ang_vel is not None else 0) <= DOG_TURN_ABS_LIMIT), f"|ang_vel| must be <= {DOG_TURN_ABS_LIMIT}"
        return (abs(ang_vel) if ang_vel is not None else 0) <= 100, "ang_vel must be a percentage between -100 and 100 in relative mode"

    @staticmethod
    def _valid_approx_time_limit(approx_time_limit : float | None):
        if approx_time_limit is None:
            return True, None
        return isinstance(approx_time_limit, float | int) and approx_time_limit >= 0, "approx_time_limit must be a non-negative number or None"

    @staticmethod
    def _valid_translation(translation : tuple[float | int | None, 3], relative : bool) -> tuple[bool, str]:
        if not isinstance(translation, tuple) or len(translation) != 3:
            return False, "translation must be a tuple of size 3"
        if not all(isinstance(x, float | int | type(None)) for x in translation):
            return False, "translation values must be of type float, int, or None"
        if not relative:
            return (
                abs(translation[0] if translation[0] is not None else 0) <= DOG_X_TRANS_LIMIT
                and abs(translation[1] if translation[1] is not None else 0) <= DOG_Y_TRANS_LIMIT 
                and DOG_HEIGHT_LIMIT[0] <= (translation[2] if translation[2] is not None else 0) <= DOG_HEIGHT_LIMIT[1]
            ), (f"|translation[0]| (X) must be <= {DOG_X_TRANS_LIMIT} and "
                f"|translation[1]| (Y) must be <= {DOG_Y_TRANS_LIMIT} and " 
                f"|translation[2]| (Z) must be in {DOG_HEIGHT_LIMIT}"
            )
        return (
            all(abs(x if x is not None else 0) <=100 for x in translation), 
            "translation values must be percentages between -100 and 100 in relative mode"
        )

    @staticmethod
    def _valid_rotation(rotation : tuple[float | int | None, 3], relative : bool) -> tuple[bool, str]:
        if not isinstance(rotation, tuple) or len(rotation) != 3:
            return False, "rotation must be a tuple of size 3"
        if not all(isinstance(x, float | int | type(None)) for x in rotation):
            return False, "rotation values must be of type float, int, or None"
        if not relative:
            return (
                abs(rotation[0] if rotation[0] is not None else 0) <= DOG_ROLL_LIMIT
                and abs(rotation[1] if rotation[1] is not None else 0) <= DOG_PITCH_LIMIT 
                and abs(rotation[2] if rotation[2] is not None else 0) <= DOG_YAW_LIMIT 
            ), (f"|rotation[0]| (X) must be <= {DOG_ROLL_LIMIT} and "
                f"|rotation[1]| (Y) must be <= {DOG_PITCH_LIMIT} and " 
                f"|rotation[2]| (Z) must be <= {DOG_YAW_LIMIT}"
            )
        return (
            all(abs(x if x is not None else 0) <= 100 for x in rotation), 
            "rotation values must be percentages between -100 and 100 in relative mode"
        )

    @staticmethod 
    def _valid_gait(gait : str | int) -> tuple[bool, str]:
        if isinstance(gait, str):
            gait = gait.lower()
        return (
            isinstance(gait, str | int) and (
                gait in GAIT_LIST
                or 0 <= gait < len(GAIT_LIST)
            )
        ), f"gait must be a string in {GAIT_LIST} or an integer between 0 and {len(GAIT_LIST) - 1}"

    @staticmethod 
    def _valid_pace(pace : str | int) -> tuple[bool, str]:
        if isinstance(pace, str):
            pace = pace.lower()
        return (
            isinstance(pace, str | int) and (
                pace in PACE_LIST
                or 0 <= pace < len(PACE_LIST)
            )
        ), f"gait must be a string in {PACE_LIST} or an integer between 0 and {len(PACE_LIST) - 1}"

    @staticmethod
    def _valid_cartesian(cartesian):
        # TODO
        return True, None

    @staticmethod
    def _valid_polar(polar):
        # TODO
        return True, None

    @staticmethod
    def _valid_claw(claw):
        # TODO
        return True, None

    # endregion
