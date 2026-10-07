from pymavlink import mavutil
import threading, time, math
from enum import Enum, auto
from dataclasses import dataclass
from typing import Callable

from logger.logger_fabric import LoggerFabric
from safety_commander.mavlink_conn import MAVLinkConn, SendingResult


MODULE_NAME = "safety_commander"

class TargetResult(Enum):
    SUCCESS = auto()
    NO_CONN = auto()
    FAILURE = auto()
    INVALID = auto()
    STOPPED = auto()

class OffboardState(Enum):
    IDLE = auto()
    ACTIVATING = auto()
    ACTIVE = auto()
    DEACTIVATING = auto()
    DEAD = auto()

@dataclass
class OffboardConfig:
    publishing_rate : float = 50
    notarget_timeout : float = 2.0

    def __post_init__(self):
        if self.publishing_rate <= 0:
            raise ValueError("publishing_rate must be positive")
        if self.notarget_timeout <= 0:
            raise ValueError("notarget_timeout must be positive")

class ControlOffboard:
    def __init__(
        self,
        config : OffboardConfig,
        conn : MAVLinkConn,
        logger_fabric : LoggerFabric,
    ):
        if not logger_fabric.is_alive():
            raise RuntimeError("LoggerFabric is stopped")
        self._logger = logger_fabric.get_logger(f"{type(self).__name__}", module_name=MODULE_NAME)
        self._logger.info("Initializing")
        self.conn = conn
        self.config = config

        self._failed = False
        self._fail_num = 0

        self._target_lock = threading.Lock()
        self._last_target_at = None
        self._target = None
        self._mask = None

        self._state_lock = threading.Lock()
        self._state = OffboardState.IDLE

        self._stop_event = threading.Event()
        self._broadcast_thread = threading.Thread(target=self._broadcast, daemon=True)
        self._broadcast_thread.start()

        self._logger.info("Initialized")

    def is_active(self):
        with self._state_lock:
            return self._state in [
                OffboardState.ACTIVATING, OffboardState.ACTIVE, OffboardState.DEACTIVATING
            ]

    def is_alive(self):
        return not self._stop_event.is_set()

    def stop(self):
        with self._state_lock:
            self._stop_event.set()
            self._state = OffboardState.DEAD
        if (
            self._broadcast_thread is not None
            and self._broadcast_thread.is_alive()
            and self._broadcast_thread is not threading.current_thread()
        ):
            self._broadcast_thread.join()
        self._logger.info("ControlOffboard stopped")

    @property
    def state(self):
        with self._state_lock:
            return self._state

    @property
    def failed(self):
        with self._state_lock:
            return self._failed

    def activate(self):
        with self._state_lock:
            if self._stop_event.is_set():
                return self._state
            with self._target_lock:
                if self._target is None:
                    return OffboardState.IDLE
            if self._state == OffboardState.IDLE:
                self._state = OffboardState.ACTIVATING
                self._failed = False
                self._fail_num = 0
            return self._state

    def deactivate(self):
        with self._state_lock:
            if self._stop_event.is_set():
                return self._state
            changed = self._state in [OffboardState.ACTIVATING, OffboardState.ACTIVE]
            if changed:
                self._state = OffboardState.DEACTIVATING
            state = self._state
        return state

    def mark_active(self):
        with self._state_lock:
            if self._stop_event.is_set() or self._state not in [OffboardState.ACTIVATING, OffboardState.ACTIVE]:
                return self._state
            if self._state != OffboardState.ACTIVE:
                with self._target_lock:
                    self._last_target_at = time.monotonic()
            self._state = OffboardState.ACTIVE
        return self._state

    def mark_idle(self):
        with self._state_lock:
            if self._stop_event.is_set():
                return self._state
            self._state = OffboardState.IDLE
        return self._state

    def _prepare_msg_mask(
        self,
        use_pos : bool = True,
        ignore_yaw : bool = True,
        ignore_yaw_rate : bool = True
    ):
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
        self._logger.log(5, "Message mask: %s", self._mask)

    def set_target(
        self,
        pos : tuple[float, ...] | None = None,
        vel : tuple[float, ...] | None = None,
        yaw : float | None = None,
        yaw_rate : float | None = None
    ):
        if self._stop_event.is_set():
            return TargetResult.STOPPED
        if (
            any(
                v is not None 
                and  (
                    not isinstance(v, (tuple, list)) 
                    or len(v) != 3
                    or not all(
                        isinstance(x, (int, float)) 
                        and math.isfinite(x)
                        for x in v
                    )
                ) 
                for v in (pos, vel)
            )
            or any (
                v is not None
                and not (
                    isinstance(v, (int, float)) 
                    and math.isfinite(v)
                )
                for v in (yaw, yaw_rate)
            )
            or (
                pos is not None and vel is not None
            )
        ):
            return TargetResult.INVALID
        
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
            self._prepare_msg_mask(use_pos, ignore_yaw, ignore_yaw_rate)
            self._target = (*pos, *vel, 0, 0, 0, yaw, yaw_rate)
            self._last_target_at = time.monotonic()
            self._logger.debug("New target: %s, at: %s", self._target, self._last_target_at)
        return TargetResult.SUCCESS

    def _broadcast(self):
        self._logger.info("Broadcasting started")
        while not self._stop_event.is_set():
            with self._state_lock:
                if self._state in (OffboardState.ACTIVATING, OffboardState.ACTIVE):
                    with self._target_lock:
                        mask, target = self._mask, self._target
                        last_target = self._last_target_at
                    timed_out = (
                        self._state == OffboardState.ACTIVE
                        and (last_target is None
                             or time.monotonic() - last_target >= self.config.notarget_timeout)
                    )
                    if target is None or timed_out:
                        self._logger.warning("Target timeout")
                        # The lock is already held; do not call deactivate() here.
                        self._state = OffboardState.DEACTIVATING
                    else:
                        try:
                            result = self.conn.send_local_target(mask, target)
                        except Exception:
                            self._logger.exception("Sending target failed")
                            result = SendingResult.FAILURE
                        if result == SendingResult.SUCCESS:
                            self._fail_num = 0
                        else:
                            self._fail_num += 1
                            self._logger.warning("Sending target failed: %s", result)
                            if self._fail_num >= 3:
                                self._failed = True
                                self._state = OffboardState.DEACTIVATING
            self._stop_event.wait(1 / self.config.publishing_rate)
