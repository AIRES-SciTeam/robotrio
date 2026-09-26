from pymavlink import mavutil
from dataclasses import dataclass, replace
import time
import threading
from typing import Callable

from Logger.LoggerFabric import LoggerFabric


@dataclass
class DroneState:
    flight_mode : str | None = None     # режим полёта
    armed : bool | None = None          # взведённость 
    landed : bool | None = None

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


class DroneTelemetry:
    def __init__(
        self, 
        logger_fabric : LoggerFabric
	):
        self._logger = logger_fabric.get_logger("drone_telemetry")
        self._state = DroneState()
        self._cond = threading.Condition()
        self._logger.info("Initialized")

    def update(self, msg):
        recieved_at = time.monotonic()
        self._logger.debug('%s: recieved msg %s', recieved_at, msg.get_type())

        with self._cond:
            match msg.get_type():
                case "HEARTBEAT":
                    if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
                        return
                    self._state.last_heartbeat_at = recieved_at

                    prev_armed = self._state.armed
                    self._state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if self._state.armed != prev_armed:
                        self._logger.info('PX4 %s', 'armed' if self._state.armed else 'disarm')

                    prev_mode = self._state.flight_mode
                    self._state.flight_mode = mavutil.mode_string_v10(msg)
                    if self._state.flight_mode != prev_mode:
                        self._logger.info('PX4 flight mode: %s', self._state.flight_mode)
                case "LOCAL_POSITION_NED":
                    self._state.last_position_at = recieved_at

                    self._state.x = msg.x
                    self._state.y = msg.y
                    self._state.z = msg.z

                    self._state.vx = msg.vx
                    self._state.vy = msg.vy
                    self._state.vz = msg.vz
                                        
                    self._logger.debug('Drone pos=(%s, %s, %s), vel=(%s, %s, %s)', msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
                case "ATTITUDE_QUATERNION":
                    self._state.last_attitude_at = recieved_at

                    self._state.q = (msg.q1, msg.q2, msg.q3, msg.q4)

                    self._logger.debug('Drone q=(%s)', self._state.q)
                case "EXTENDED_SYS_STATE":
                    self._state.last_landed_at = recieved_at

                    if msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED:
                        self._state.landed = None
                    else:
                        self._state.landed = msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

                    self._logger.debug('Drone landed: %s', self._state.landed)
                case _:
                    return

            self._cond.notify_all()

    def snapshot(self):
        with self._cond:
            return replace(self._state)

    def reset(self):
        with self._cond:
            self._state = DroneState()
            self._cond.notify_all()

    def wait_for(self, predicate : Callable[[DroneState], bool], timeout : float) -> bool:
        with self._cond:
            return self._cond.wait_for(
                lambda: predicate(replace(self._state)),
                timeout=timeout
            )
