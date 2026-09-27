from pymavlink import mavutil
from dataclasses import dataclass
import time
import math
import threading
from enum import Enum, auto
from typing import Callable

from Modules.SimpleCommander.MAVLinkConn import MAVLinkConn, SendingResult
from Modules.Logger.LoggerFabric import LoggerFabric


class TargetResult(Enum):
    SUCCESS = auto()
    INVALID_ARGS = auto()
    NO_CONN = auto()
    MODE_FAIL = auto()
    STOPPED = auto()

class OffboardState(Enum):
    IDLE = auto()
    STARTING = auto()
    ACTIVE = auto()
    STOPPING = auto()

@dataclass
class OffboardConfig:
    publish_rate : float = 50        # Hz 
    notarget_timeout : float = 2.0   # s

class OffboardControl:
    def __init__(
        self,
        offboard_config : OffboardConfig,
        conn : MAVLinkConn,
        logger_fabric : LoggerFabric,
        on_deactivate: Callable[[], None] | None = None,
    ):
        self._logger = logger_fabric.get_logger("offboard_control")
        self.conn = conn
        self.offboard_config = offboard_config
        self.on_deactivate = on_deactivate

        self._target_lock = threading.Lock()
        self._last_target_at = None
        self._target = None
        self._mask = None

        self._state = OffboardState.IDLE
        self._state_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._spin_thread = threading.Thread(
            target=self._spin,
            daemon=True
        )
        self._spin_thread.start()

        self._logger.info("Initialized")

    def is_active(self):
        with self._state_lock:
            return not self._stop_event.is_set() and self._state in (OffboardState.STARTING, OffboardState.ACTIVE)

    @property
    def state(self):
        with self._state_lock:
            return self._state

    def activate(self):
        with self._state_lock:
            if self._stop_event.is_set():
                return OffboardState.IDLE
            with self._target_lock:
                if self._target is None:
                    return OffboardState.IDLE
            if self._state == OffboardState.IDLE:
                self._state = OffboardState.STARTING
            return self._state

    def mark_active(self) -> bool:
        with self._state_lock:
            if self._stop_event.is_set() or self._state not in (OffboardState.STARTING, OffboardState.ACTIVE):
                return False
            if self._state != OffboardState.ACTIVE:
                with self._target_lock:
                    self._last_target_at = time.monotonic()
            self._state = OffboardState.ACTIVE
            return True

    def deactivate(self, inner: bool = False):
        with self._state_lock:
            changed = (
                not self._stop_event.is_set()
                and self._state in (OffboardState.STARTING, OffboardState.ACTIVE)
            )
            if changed:
                self._state = OffboardState.STOPPING
            state = self._state
        if inner and changed and self.on_deactivate is not None:
            self.on_deactivate()
        return state

    def mark_idle(self):
        with self._state_lock:
            self._state = OffboardState.IDLE
        return OffboardState.IDLE

    def stop(self):
        self._stop_event.set()
        if (
            self._spin_thread is not None
            and self._spin_thread.is_alive()
            and self._spin_thread is not threading.current_thread()
        ):
            self._spin_thread.join()
        self._logger.info("Stopped")
        return

    def _form_mask(self, use_pos : bool = True, ignore_yaw : bool = True, ignore_yaw_rate : bool = True):
        self._logger.log(5, "Forming mask")
        self._mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        )
        if use_pos:
            self._mask |= (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            )
        else:
            self._mask |= (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            )
        if ignore_yaw:
            self._mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        if ignore_yaw_rate:
            self._mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        self._logger.log(5, "Mask: %s", self._mask)

    def set_target(
        self, 
        pos : tuple[float, ...] | None = None,
        vel : tuple[float, ...] | None = None,
        yaw : float | None = None,
        yaw_rate : float | None = None
    ):
        if self._stop_event.is_set():
            return TargetResult.STOPPED
        def valid_number(value):
            return isinstance(value, (int, float)) and math.isfinite(value)
        if (any(v is not None and (not isinstance(v, (tuple, list)) or len(v) != 3
                                   or not all(valid_number(x) for x in v)) for v in (pos, vel))
                or any(v is not None and not valid_number(v) for v in (yaw, yaw_rate))
                or (pos is not None and vel is not None)):
            return TargetResult.INVALID_ARGS
        use_pos = False
        if pos is not None:
            use_pos = True
        else:
            self._logger.log(5, "Pos is None, use (0, 0, 0)")
            pos = (0, 0, 0)
        
        if vel is None:
            self._logger.log(5, "Vel is None, use (0, 0, 0)")
            vel = (0, 0, 0)
        
        ignore_yaw = False
        if yaw is None:
            self._logger.log(5, "Yaw is None, use 0")
            ignore_yaw = True
            yaw = 0
        
        ignore_yaw_rate = False
        if yaw_rate is None:
            self._logger.log(5, "Yaw_rate is None, use 0")
            ignore_yaw_rate = True
            yaw_rate = 0
        
        with self._target_lock:
            self._form_mask(use_pos, ignore_yaw, ignore_yaw_rate)
            self._target = (*pos, *vel, 0, 0, 0, yaw, yaw_rate)
            self._last_target_at = time.monotonic()
            self._logger.log(5, "New target: %s, at: %s", self._target, self._last_target_at)
        return TargetResult.SUCCESS

    def _spin(self):
        self._logger.info("Spin thread started")
        while not self._stop_event.is_set():
            timed_out = False
            with self._state_lock:
                if self._state != OffboardState.IDLE:
                    with self._target_lock:
                        mask, target = self._mask, self._target
                        last_target_at = self._last_target_at
                    timed_out = (
                        self._state == OffboardState.ACTIVE
                        and (target is None or last_target_at is None
                             or time.monotonic() - last_target_at >= self.offboard_config.notarget_timeout)
                    )
                    if not timed_out:
                        self.conn.send_local_target(mask, target)
            if timed_out:
                self._logger.warning("Target timeout; requesting deactivation")
                self.deactivate(inner=True)
            self._stop_event.wait(1 / self.offboard_config.publish_rate)
