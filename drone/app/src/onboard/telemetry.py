from pymavlink import mavutil
import threading, time
from dataclasses import dataclass, replace
from typing import Callable, Dict, Any

from onboard.logger_fabric import LoggerFabric


@dataclass
class Telemetry:
    armed: bool | None = None
    landed: bool | None = None

    flight_mode: str | None = None
    flight_mode_mapping: dict[int, str] | None = None

    x: float | None = None                                  # позиция, м, локальная NED
    y: float | None = None
    z: float | None = None                                  # вниз — положительное направление
    q : tuple[float, float, float, float] | None = None     # Кватернион ориентации дрона
    vx: float | None = None                                 # скорость, м/с, локальная NED
    vy: float | None = None
    vz: float | None = None

    last_heartbeat_at: float | None = None      # время получения последнего heartbeat, time.monotonic()
    last_position_at: float | None = None       # время получения последнего положения, time.monotonic()
    last_attitude_at: float | None = None       # время получения последнего кватерниона, time.monotonic()
    last_landed_at : float | None = None 		# время последнего преземления, time.monotonic()

class TelemetryManager:
    def __init_(
        self,
        logger_fabric : LoggerFabric,
        config : Dict[str, Any]
    ):
        self._logger = logger_fabric.get_logger(
            name=f"onboard.{type(self).__name__}",
            level=config.get("log_level", "info")
        )
        self._journal = logger_fabric.get_logger(
            name=f"telemetry_journal",
            level="trace"
        )
        self._logger.info("Initializing")

        self._state = Telemetry()
        self._state_cond = threading.Condition()
        
        self._rec_rate = config.get("recording_rate", 5.0)
        self._logger.info("recording_rate = %s", self._rec_rate)        
        self._last_rec = None

        self._logger.info("Initialized")

    def _handle_heartbeat(self, msg, received_at):
        if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            self._logger.debug("Not PX4 heartbeat")
            return
        self._state.last_heartbeat_at = received_at
        
        prev_armed = self._state.armed
        self._state.armed = bool(
            msg.base_mode 
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        if self._state_armed != prev_armed:
            self._logger.info("PX4 %s", 'armed' if self._state.armed else 'disarmed')

        prev_mode = self._state.flight_mode
        self._state.flight_mode = mavutil.mode_string_v10(msg)
        if (
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            and ((msg.custom_mode >> 16) & 0xFF) == mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD
        ):
            self._state.flight_mode = "OFFBOARD"
        if self._state.flight_mode != prev_mode:
            self._logger.info("PX4 flight mode: %s", self._state.flight_mode)

        self._logger.debug("Armed: %s, flight_mode: %s", self._state.armed, self._state.flight_mode)

    def _handle_extended_sys_state(self, msg, received_at):
        self._state.last_landed_at = received_at
        if msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED:
            self._state.landed = None
        else:
            self._state.landed = msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
        self._logger.debug("Landed: %s", self._state.landed)

    def _handle_local_position_ned(self, msg, received_at):
        self._state.last_position_at = received_at
        self._state.x = msg.x
        self._state.y = msg.y
        self._state.z = msg.z
        self._state.vx = msg.vx
        self._state.vy = msg.vy
        self._state.vz = msg.vz
        self._logger.debug("Position = (%s, %s, %s), velocity = (%s, %s, %s)", self._state.x, self._state.y, self._state.z, self._state.vx, self._state.vy, self._state.vz)

    def _handle_attitude_quaternion(self, msg, received_at):
        self._state.last_attitude_at = received_at
        self._state.q = (msg.q1, msg.q2, msg.q3, msg.q4)
        self._logger.debug("Att. quat. = (%s)", self._state.q)

    def _record(self, use_timer=True):
        if (
            use_timer and self._last_rec is not None
            and time.monotonic() - self._last_rec < (1 / self._rec_rate)
        ): 
            return
        self._last_rec = time.monotonic()
        state = self._state
        self._journal.critical(
            f"{state.last_heartbeat_at}|{state.last_attitude_at}|{state.last_landed_at}|{state.last_position_at}|"
            f"{state.armed}|{state.landed}|{state.flight_mode}|{state.x}|{state.y}|{state.z}|{state.vx}|{state.vy}|{state.vz}|{state.q}"
        )

    def update(self, msg):
        received_at = time.monotonic()
        self._logger.debug("Received msg %s at %s", msg.get_type(), received_at)
        with self._state_cond:
            match msg.get_type():
                case "HEARTBEAT":
                    self._handle_heartbeat(msg, received_at)
                case "LOCAL_POSITION_NED":
                    self._handle_local_position_ned(msg, received_at)
                case "ATTITUDE_QUATERNION":
                    self._handle_attitude_quaternion(msg, received_at)
                case "EXTEMDED_SYS_STATE":
                    self._handle_extended_sys_state(msg, received_at)
                case _:
                    return
            self._record()
            self._state_cond.notify_all()

    def set_mode_mapping(self, mapping):
        with self._state_cond:
            self._state.flight_mode_mapping = mapping
            self._state_cond.notify_all()

    def snapshot(self):
        with self._state_cond:
            return replace(self._state)

    def reset(self):
        with self._state_cond:
            self._state = Telemetry()
            self._state_cond.notify_all()

    def wait_for(self, predicate: Callable[[Telemetry], bool], timeout: float) -> bool:
        with self._state_cond:
            return self._state_cond.wait_for(
                lambda: predicate(replace(self._state)),
                timeout=timeout
            )
   