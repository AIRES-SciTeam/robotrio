import math
import threading
import time
from enum import Enum, auto
from typing import Callable

from Modules.Logger.LoggerFabric import LoggerFabric
from Modules.Commander.MAVLinkConn import MAVLinkConn, MAVLinkConnConfig, SendingResult
from Modules.Commander.FlightControl import FlightControl
from Modules.Commander.OffboardControl import OffboardControl, OffboardConfig, OffboardState, TargetResult


class CommandResult(Enum):
    SUCCESS = auto()
    SENT = auto()
    NO_CONN = auto()
    INVALID_ARGS = auto()
    SEND_FAILED = auto()
    CONFIRMATION_TIMEOUT = auto()
    OFFBOARD_INACTIVE = auto()
    WRONG_MODE = auto()

class Commander:
    def __init__(
        self,
        logger_fabric: LoggerFabric,
        conn_config: MAVLinkConnConfig,
        offboard_config: OffboardConfig,
        callback: Callable[[Exception], None],
        offboard_exit_mode: str = "MANUAL",
    ):
        self._logger = logger_fabric.get_logger("Commander")
        self.callback = callback
        self._offboard_exit_mode = offboard_exit_mode.upper()
        if self._offboard_exit_mode == "OFFBOARD":
            raise ValueError("Offboard exit mode must differ from OFFBOARD")
        self._command_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._exit_requested = threading.Event()
        self._exit_thread = None
        self._conn = None
        self._offboard_ctrl = None
        try:
            self._conn = MAVLinkConn(logger_fabric, conn_config, self._on_error)
            if self._stop_event.is_set():
                self._conn.stop()
                return
            self._flight_ctrl = FlightControl(self._conn, logger_fabric)
            self._offboard_ctrl = OffboardControl(
                offboard_config, self._conn, logger_fabric, self._on_error,
                on_deactivate=self._exit_requested.set,
            )
            self._exit_thread = threading.Thread(target=self._exit_offboard, daemon=True)
            self._exit_thread.start()
        except Exception:
            self.stop()
            raise
        self._logger.info("Initialized")

    def _on_error(self, error: Exception):
        self._stop_event.set()
        self._exit_requested.set()
        self._logger.error("Controller failed: %s", error)
        self.callback(error)

    def stop(self):
        self._stop_event.set()
        self._exit_requested.set()
        if self._offboard_ctrl is not None:
            self._offboard_ctrl.stop()
        if self._conn is not None:
            self._conn.stop()
        if self._exit_thread is not None and self._exit_thread is not threading.current_thread():
            self._exit_thread.join()

    def _exit_offboard(self):
        while True:
            self._exit_requested.wait()
            self._exit_requested.clear()
            if self._stop_event.is_set():
                return
            with self._command_lock:
                if self._offboard_ctrl.state != OffboardState.STOPPING:
                    continue
                result = self.flight_mode(self._offboard_exit_mode)
            if result != CommandResult.SUCCESS and not self._stop_event.is_set():
                self._logger.error("Offboard exit failed: %s", result.name)
                self.callback(RuntimeError(f"Offboard exit failed: {result.name}"))

    @staticmethod
    def _sending_result(result: SendingResult) -> CommandResult:
        return {
            SendingResult.SENT: CommandResult.SENT,
            SendingResult.NO_CONN: CommandResult.NO_CONN,
            SendingResult.INV_ARGS: CommandResult.INVALID_ARGS,
            SendingResult.SEND_FAILED: CommandResult.SEND_FAILED,
        }[result]

    @staticmethod
    def _valid_timeout(timeout):
        return isinstance(timeout, (int, float)) and math.isfinite(timeout) and timeout >= 0

    def _available(self):
        return not self._stop_event.is_set() and self._conn.is_available()

    def _wait_confirmed(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while self._available():
            if predicate(self._conn.tm_snapshot()):
                return CommandResult.SUCCESS
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return CommandResult.CONFIRMATION_TIMEOUT
            self._conn.wait_for_state(predicate, min(remaining, 0.05))
        return CommandResult.NO_CONN

    def _arming(self, armed, timeout):
        with self._command_lock:
            if not self._available():
                return CommandResult.NO_CONN
            if not self._valid_timeout(timeout):
                return CommandResult.INVALID_ARGS
            self._logger.info("Requesting armed=%s", armed)
            predicate = lambda state: state.armed is armed
            if not predicate(self._conn.tm_snapshot()):
                send = self._flight_ctrl.arm if armed else self._flight_ctrl.disarm
                result = self._sending_result(send())
                if result != CommandResult.SENT:
                    return result
            return self._wait_confirmed(predicate, timeout)

    def arm(self, timeout=2.0):
        return self._arming(True, timeout)

    def disarm(self, timeout=2.0):
        return self._arming(False, timeout)

    def toggle_arming(self, timeout=2.0):
        with self._command_lock:
            if not self._available():
                return CommandResult.NO_CONN
            armed = self._conn.tm_snapshot().armed
            if armed is None:
                return CommandResult.NO_CONN
            return self._arming(not armed, timeout)

    def _prepare_offboard(self, deadline):
        if self._offboard_ctrl.activate() not in (OffboardState.STARTING, OffboardState.ACTIVE):
            return CommandResult.OFFBOARD_INACTIVE
        ready_at = time.monotonic() + 1.1
        if self._conn.tm_snapshot().flight_mode == "OFFBOARD":
            ready_at = time.monotonic()
        while self._available():
            if not self._offboard_ctrl.is_active():
                return CommandResult.OFFBOARD_INACTIVE
            if time.monotonic() >= ready_at:
                return CommandResult.SUCCESS
            if time.monotonic() >= deadline:
                return CommandResult.CONFIRMATION_TIMEOUT
            self._stop_event.wait(min(0.02, max(0, deadline - time.monotonic())))
        return CommandResult.NO_CONN

    def flight_mode(self, name: str, timeout=3.0):
        with self._command_lock:
            if not self._available():
                return CommandResult.NO_CONN
            if not isinstance(name, str) or not self._valid_timeout(timeout):
                return CommandResult.INVALID_ARGS
            name = name.upper()
            mapping = self._conn.mode_mapping()
            if mapping is None:
                return CommandResult.NO_CONN
            if name not in mapping:
                return CommandResult.INVALID_ARGS
            deadline = time.monotonic() + timeout
            offboard = self._offboard_ctrl
            if name == "OFFBOARD":
                result = self._prepare_offboard(deadline)
                if result != CommandResult.SUCCESS:
                    if self._conn.tm_snapshot().flight_mode != "OFFBOARD":
                        offboard.mark_idle()
                    return result
            elif offboard.state != OffboardState.IDLE:
                offboard.deactivate()
            self._logger.info("Requesting mode %s", name)
            predicate = lambda state: state.flight_mode == name
            result = CommandResult.SENT
            if not predicate(self._conn.tm_snapshot()):
                result = self._sending_result(self._flight_ctrl.flight_mode(name))
            if result == CommandResult.SENT:
                result = self._wait_confirmed(predicate, max(0, deadline - time.monotonic()))
            if result == CommandResult.SUCCESS:
                if name == "OFFBOARD":
                    if not offboard.mark_active():
                        return CommandResult.OFFBOARD_INACTIVE
                else:
                    offboard.mark_idle()
            elif name == "OFFBOARD" and self._conn.tm_snapshot().flight_mode != "OFFBOARD":
                offboard.mark_idle()
            return result

    def takeoff(self, timeout=3.0):
        return self.flight_mode("TAKEOFF", timeout)

    def land(self, timeout=3.0):
        return self.flight_mode("LAND", timeout)

    def manual_mode(self, timeout=3.0):
        return self.flight_mode("MANUAL", timeout)

    def manual_control(self, roll, pitch, yaw, throttle):
        with self._command_lock:
            if not self._available():
                return CommandResult.NO_CONN
            if (not all(isinstance(v, int) for v in (roll, pitch, yaw, throttle))
                    or any(abs(v) > 1000 for v in (roll, pitch, yaw))
                    or not 0 <= throttle <= 1000):
                return CommandResult.INVALID_ARGS
            if self._conn.tm_snapshot().flight_mode != "MANUAL":
                return CommandResult.WRONG_MODE
            return self._sending_result(self._flight_ctrl.manual_control(roll, pitch, yaw, throttle))

    def set_target(self, pos=None, vel=None, yaw=None, yaw_rate=None, timeout=3.0):
        with self._command_lock:
            if not self._available():
                return CommandResult.NO_CONN
            if not self._valid_timeout(timeout):
                return CommandResult.INVALID_ARGS
            result = self._offboard_ctrl.set_target(pos, vel, yaw, yaw_rate)
            if result != TargetResult.SUCCESS:
                return (CommandResult.INVALID_ARGS if result == TargetResult.INVALID_ARGS
                        else CommandResult.OFFBOARD_INACTIVE)
            return self.flight_mode("OFFBOARD", timeout)
