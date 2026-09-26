import logging
import threading
import time
import math
from enum import Enum, auto
from typing import Callable

from Logger.LoggerFabric import LoggerFabric
from MAVLinkConn import MAVLinkConn, MAVLinkConnConfig, SendingResult
from FlightControl import FlightControl
from OffboardControl import OffboardControl, OffboardConfig, OffboardState, TargetResult


class CommandResult(Enum):
    SUCCESS = auto()  # Requested state confirmed by telemetry.
    SENT = auto()     # Setpoint sent; no per-setpoint acknowledgement exists.
    NO_CONN = auto()
    INVALID_ARGS = auto()
    SEND_FAILED = auto()
    CONFIRMATION_TIMEOUT = auto()
    OFFBOARD_INACTIVE = auto()

class Commander:
    def __init__(
        self,
        logger_fabric : LoggerFabric,
        conn_config : MAVLinkConnConfig,
        offboard_config : OffboardConfig,
        callback : Callable
    ):
        self._logger = logger_fabric.get_logger("Commander")
        self.logger_fabric = logger_fabric
        self.conn_config = conn_config
        self.offboard_config = offboard_config
        self.callback = callback

        self._command_lock = threading.RLock()
        self._lock = threading.Lock()
        self._rebuild_lock = threading.Lock()
        self._conn_dead = threading.Event()
        self._conn_dead.set()
        self._generation = None

        self._conn = None
        self._flight_ctrl = None
        self._offboard_ctrl = None
        
        self.rebuild_conn()
        
        self._logger.info("Initialized")

    def rebuild_conn(self) -> bool:
        if not self._rebuild_lock.acquire(blocking=False):
            self._logger.debug("Connection rebuild already in progress")
            return False
        conn = None
        offboard = None
        published = False
        try:
            with self._lock:
                if not self._conn_dead.is_set():
                    self._logger.debug("Connection rebuild skipped: components are alive")
                    return True
                generation = object()
                failed = threading.Event()
                self._generation = generation

            self._logger.info("Rebuilding connection and controllers")
            conn = MAVLinkConn(
                logger_fabric=self.logger_fabric,
                conn_config=self.conn_config,
                callback=lambda: self._conn_died(generation, failed),
            )
            if failed.is_set():
                self._logger.warning("Connection rebuild failed during connection creation")
                return False
            flight = FlightControl(conn=conn, logger_fabric=self.logger_fabric)
            offboard = OffboardControl(
                offboard_config=self.offboard_config,
                conn=conn,
                logger_fabric=self.logger_fabric,
                callback=lambda: self._offboard_deactivated(generation),
            )
            with self._lock:
                if failed.is_set():
                    self._logger.warning("Connection died while creating controllers")
                    return False
                self._conn = conn
                self._flight_ctrl = flight
                self._offboard_ctrl = offboard
                self._conn_dead.clear()
                published = True
            self._logger.info("Connection and controllers rebuilt")
            return True
        except Exception:
            self._logger.exception("Connection rebuild failed")
            raise
        finally:
            try:
                if not published:
                    if offboard is not None:
                        offboard.stop()
                    if conn is not None:
                        conn.stop()
            finally:
                self._rebuild_lock.release()

    def _conn_died(self, generation, failed):
        with self._lock:
            failed.set()
            if generation is not self._generation or self._conn is None:
                return
            offboard = self._offboard_ctrl
            self._conn = None
            self._flight_ctrl = None
            self._offboard_ctrl = None
        self._logger.warning("Connection died; stopping controllers")
        if offboard is not None:
            offboard.stop()
        with self._lock:
            self._conn_dead.set()
        self._logger.info("Connection components cleared")
        self.callback()

    def _offboard_deactivated(self, generation):
        with self._lock:
            if generation is not self._generation or self._conn is None:
                return
        self._logger.info("Offboard deactivated")

    def _log_result(self, command: str, result: CommandResult,
                    success_level: int = logging.DEBUG) -> CommandResult:
        level = success_level
        if result == CommandResult.SEND_FAILED:
            level = logging.ERROR
        elif result not in (CommandResult.SUCCESS, CommandResult.SENT):
            level = logging.WARNING
        self._logger.log(level, "%s result: %s", command, result.name)
        return result

    @staticmethod
    def _sending_result(result: SendingResult) -> CommandResult:
        return {
            SendingResult.SENT: CommandResult.SENT,
            SendingResult.NO_CONN: CommandResult.NO_CONN,
            SendingResult.INV_ARGS: CommandResult.INVALID_ARGS,
            SendingResult.SEND_FAILED: CommandResult.SEND_FAILED,
        }[result]

    @staticmethod
    def _valid_timeout(timeout: float) -> bool:
        return isinstance(timeout, (int, float)) and math.isfinite(timeout) and timeout >= 0

    def _components(self):
        with self._lock:
            conn, flight, offboard = self._conn, self._flight_ctrl, self._offboard_ctrl
            if self._conn_dead.is_set() or conn is None or flight is None:
                return None
        if not conn.is_available():
            return None
        return conn, flight, offboard

    def _alive(self, conn) -> bool:
        with self._lock:
            current = self._conn is conn and not self._conn_dead.is_set()
        return current and conn.is_available()

    def _wait_confirmed(self, conn, predicate, timeout: float) -> CommandResult:
        self._logger.debug("Waiting for state confirmation, timeout=%s s", timeout)
        deadline = time.monotonic() + timeout
        while self._alive(conn):
            if predicate(conn.tm_snapshot()):
                return CommandResult.SUCCESS if self._alive(conn) else CommandResult.NO_CONN
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return CommandResult.CONFIRMATION_TIMEOUT
            conn.wait_for_state(predicate, min(remaining, 0.05))
        return CommandResult.NO_CONN

    def _arming(self, armed: bool, timeout: float) -> CommandResult:
        with self._command_lock:
            parts = self._components()
            if parts is None:
                return CommandResult.NO_CONN
            if not self._valid_timeout(timeout):
                return CommandResult.INVALID_ARGS
            conn, flight, _ = parts
            predicate = lambda state: state.armed is armed
            if predicate(conn.tm_snapshot()):
                return self._wait_confirmed(conn, predicate, 0)
            result = self._sending_result(flight.arm() if armed else flight.disarm())
            if result != CommandResult.SENT:
                return result
            return self._wait_confirmed(conn, predicate, timeout)

    def arm(self, timeout: float = 2.0) -> CommandResult:
        self._logger.info("Arm requested, timeout=%s s", timeout)
        return self._log_result("arm", (self._arming(True, timeout)), logging.INFO)

    def disarm(self, timeout: float = 2.0) -> CommandResult:
        self._logger.info("Disarm requested, timeout=%s s", timeout)
        return self._log_result("disarm", (self._arming(False, timeout)), logging.INFO)

    def _prepare_offboard(self, conn, offboard, deadline) -> CommandResult:
        if offboard is None or offboard.activate() == OffboardState.IDLE:
            return CommandResult.OFFBOARD_INACTIVE
        while self._alive(conn):
            active, started, last_sent, _, error = offboard.stream_status()
            if error is not None:
                return self._sending_result(error)
            if not active:
                return CommandResult.OFFBOARD_INACTIVE
            # PX4 requires more than one second of setpoints before entry.
            if started is not None and last_sent - started >= 1.1:
                return CommandResult.SUCCESS
            if time.monotonic() >= deadline:
                return CommandResult.CONFIRMATION_TIMEOUT
            time.sleep(min(0.02, max(0, deadline - time.monotonic())))
        return CommandResult.NO_CONN

    @staticmethod
    def _cancel_offboard_start(conn, offboard):
        if offboard is not None and conn.tm_snapshot().flight_mode != "OFFBOARD":
            offboard.deactivate()

    def flight_mode(self, name: str, timeout: float = 3.0) -> CommandResult:
        self._logger.debug("Flight mode requested: %r, timeout=%s s", name, timeout)
        with self._command_lock:
            parts = self._components()
            if parts is None:
                return self._log_result(f"flight_mode({name!r})", (CommandResult.NO_CONN), logging.DEBUG)
            if not isinstance(name, str) or not self._valid_timeout(timeout):
                return self._log_result(f"flight_mode({name!r})", (CommandResult.INVALID_ARGS), logging.DEBUG)
            conn, flight, offboard = parts
            name = name.upper()
            mapping = conn.mode_mapping()
            if mapping is None:
                return self._log_result(f"flight_mode({name!r})", (CommandResult.NO_CONN), logging.DEBUG)
            if name not in mapping:
                return self._log_result(f"flight_mode({name!r})", (CommandResult.INVALID_ARGS), logging.DEBUG)
            deadline = time.monotonic() + timeout
            if name == "OFFBOARD":
                result = self._prepare_offboard(conn, offboard, deadline)
                if result != CommandResult.SUCCESS:
                    self._cancel_offboard_start(conn, offboard)
                    return self._log_result(f"flight_mode({name!r})", (result), logging.DEBUG)
            predicate = lambda state: state.flight_mode == name
            if not predicate(conn.tm_snapshot()):
                self._logger.info("Requesting mode transition to %s", name)
                result = self._sending_result(flight.flight_mode(name))
                if result != CommandResult.SENT:
                    if name == "OFFBOARD":
                        self._cancel_offboard_start(conn, offboard)
                    return self._log_result(f"flight_mode({name!r})", (result), logging.DEBUG)
            result = self._wait_confirmed(conn, predicate, max(0, deadline - time.monotonic()))
            if result != CommandResult.SUCCESS and name == "OFFBOARD":
                self._cancel_offboard_start(conn, offboard)
            if result == CommandResult.SUCCESS and offboard is not None:
                if name == "OFFBOARD":
                    if not offboard.mark_active():
                        return self._log_result(f"flight_mode({name!r})", (CommandResult.OFFBOARD_INACTIVE), logging.DEBUG)
                else:
                    # Keep the stream alive until leaving Offboard is confirmed.
                    offboard.deactivate()
            return self._log_result(f"flight_mode({name!r})", (result), logging.DEBUG)

    def takeoff(self, timeout: float = 3.0) -> CommandResult:
        self._logger.info("Takeoff requested, timeout=%s s", timeout)
        return self._log_result("takeoff", (self.flight_mode("TAKEOFF", timeout)), logging.INFO)

    def land(self, timeout: float = 3.0) -> CommandResult:
        self._logger.info("Land requested, timeout=%s s", timeout)
        return self._log_result("land", (self.flight_mode("LAND", timeout)), logging.INFO)

    def manual_control(self, roll: int, pitch: int, yaw: int, throttle: int,
                       timeout: float = 3.0) -> CommandResult:
        self._logger.log(5, "Manual control requested: roll=%r pitch=%r yaw=%r throttle=%r timeout=%r",
                         roll, pitch, yaw, throttle, timeout)
        with self._command_lock:
            parts = self._components()
            if parts is None:
                return self._log_result("manual_control", (CommandResult.NO_CONN), 5)
            values = (roll, pitch, yaw, throttle)
            if (not all(isinstance(v, int) for v in values)
                    or any(abs(v) > 1000 for v in values[:3])
                    or not 0 <= throttle <= 1000 or not self._valid_timeout(timeout)):
                return self._log_result("manual_control", (CommandResult.INVALID_ARGS), 5)
            conn, flight, _ = parts
            result = self.flight_mode("MANUAL", timeout)
            if result != CommandResult.SUCCESS:
                return self._log_result("manual_control", (result), 5)
            if not self._alive(conn):
                return self._log_result("manual_control", (CommandResult.NO_CONN), 5)
            result = self._sending_result(flight.manual_control(*values))
            return self._log_result("manual_control", (result if self._alive(conn) else CommandResult.NO_CONN), 5)

    def set_target(self, pos: tuple[float, ...] | None = None,
                   vel: tuple[float, ...] | None = None,
                   yaw: float | None = None, yaw_rate: float | None = None,
                   timeout: float = 3.0) -> CommandResult:
        self._logger.log(5, "Target requested: pos=%r vel=%r yaw=%r yaw_rate=%r timeout=%r",
                         pos, vel, yaw, yaw_rate, timeout)
        with self._command_lock:
            parts = self._components()
            if parts is None:
                return self._log_result("set_target", (CommandResult.NO_CONN), 5)
            if not self._valid_timeout(timeout):
                return self._log_result("set_target", (CommandResult.INVALID_ARGS), 5)
            conn, _, offboard = parts
            if offboard is None:
                return self._log_result("set_target", (CommandResult.OFFBOARD_INACTIVE), 5)
            result = offboard.set_target(pos, vel, yaw, yaw_rate)
            if result != TargetResult.SUCCESS:
                return (self._log_result("set_target", (CommandResult.INVALID_ARGS if result == TargetResult.INVALID_ARGS
                        else CommandResult.OFFBOARD_INACTIVE), 5))
            revision = offboard.target_revision()
            deadline = time.monotonic() + timeout
            result = self.flight_mode("OFFBOARD", timeout)
            if result != CommandResult.SUCCESS:
                return self._log_result("set_target", (result), 5)
            while self._alive(conn):
                active, _, _, sent_revision, error = offboard.stream_status()
                if error is not None:
                    return self._log_result("set_target", (self._sending_result(error)), 5)
                if not active:
                    return self._log_result("set_target", (CommandResult.OFFBOARD_INACTIVE), 5)
                if sent_revision >= revision:
                    return self._log_result("set_target", (CommandResult.SENT), 5)
                if time.monotonic() >= deadline:
                    return self._log_result("set_target", (CommandResult.CONFIRMATION_TIMEOUT), 5)
                time.sleep(min(0.02, max(0, deadline - time.monotonic())))
            return self._log_result("set_target", (CommandResult.NO_CONN), 5)
