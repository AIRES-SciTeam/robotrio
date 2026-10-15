from pymavlink import mavutil
import threading, time
from enum import Enum, auto
from dataclasses import dataclass
from typing import Callable

from logger.logger_fabric import LoggerFabric
from safety_commander.telemetry import TelemetryManager, Telemetry


MODULE_NAME = "safety_commander"

@dataclass
class MAVLinkConnConfig:
    address: str = "udpin:127.0.0.1:14551"
    source_system: int = 255

    gcs_hb_rate : float = 1.0           # Hz
    px4_hb_timeout : float = 5.0        # s
    recv_match_timeout : float = 1.0    # s

    max_reconnects : int | None = None

    def __post_init__(self):
        if self.gcs_hb_rate <= 0:
            raise ValueError("gcs_hb_rate must be positive")
        if self.px4_hb_timeout <= 0:
            raise ValueError("px4_hb_timeout must be positive")
        if self.recv_match_timeout <= 0:
            raise ValueError("recv_match_timeout must be positive")
        if self.max_reconnects is not None and self.max_reconnects < 0:
            raise ValueError("max_reconnects must be non-negative or None")

class SendingResult(Enum):
    SUCCESS = auto()
    NO_CONN = auto()
    FAILURE = auto()
    INVALID = auto()

class ConnState(Enum):
    OK = auto()
    RECONN = auto()
    DEAD = auto()

class MAVLinkConn:
    def __init__(
        self,
        config : MAVLinkConnConfig,
        logger_fabric : LoggerFabric
    ):
        if not logger_fabric.is_alive():
            raise RuntimeError("LoggerFabric is stopped")
        self._logger = logger_fabric.get_logger(f"{type(self).__name__}", module_name=MODULE_NAME)
        self._logger.info("Initializing")
        self.config = config
        
        self._telemetry = TelemetryManager(logger_fabric)

        self._state = ConnState.RECONN
        self._state_lock = threading.Lock()

        self._stop_event = threading.Event()
        
        self._conn_lock = threading.Lock()
        self._conn : mavutil.mavlink_connection | None = None

        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._tm_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._reconnect()
        if self._stop_event.is_set():
            return
        self._hb_thread.start()
        self._tm_thread.start()

        self._logger.info("Initialized")

    def _reconnect(self):
        with self._state_lock:
            if self._stop_event.is_set():
                return
            self._state = ConnState.RECONN
        with self._conn_lock:
            if self._conn is not None:
                self._conn.close()
                self._conn = None
        attempt = 1
        max_attempts = self.config.max_reconnects if self.config.max_reconnects is not None else float('inf') 
        while not self._stop_event.is_set() and attempt <= max_attempts:
            self._telemetry.reset()
            self._logger.info("Connecting to MAVLink at %s (attempt %d / %s)", self.config.address, attempt, max_attempts if max_attempts != float('inf') else "∞")
            conn = None
            hb = None
            try:
                conn = mavutil.mavlink_connection(
                    self.config.address,
                    source_system=self.config.source_system
                )
                hb = conn.wait_heartbeat(timeout=self.config.px4_hb_timeout)
            except Exception as e:
                self._logger.warning("Failed to connect to MAVLink: %s", e)
                if conn is not None:
                    conn.close()
                self._stop_event.wait(1)
                attempt += 1
                continue
            if (
                hb is not None
                and hb.get_srcSystem() == conn.target_system
                and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
            ):
                self._telemetry.update(hb)
                self._telemetry.set_mode_mapping(conn.mode_mapping())
                with self._conn_lock:
                    if self._stop_event.is_set():
                        conn.close()
                        return
                    self._conn = conn
                with self._state_lock:
                    if not self._stop_event.is_set():
                        self._state = ConnState.OK
                self._logger.info("Connected to MAVLink at %s", self.config.address)
                return
            else:
                self._logger.warning("Received invalid heartbeat from MAVLink: %s", hb)
                if conn is not None:
                    conn.close()
                self._stop_event.wait(1)
                attempt += 1
                continue
        if self._stop_event.is_set():
            return
        self._logger.error("Failed to connect to MAVLink after %d attempts", attempt - 1)
        self.stop()

    def _telemetry_loop(self):
        self._logger.info("Telemetry loop started")
        while not self._stop_event.is_set():
            with self._conn_lock:
                conn = self._conn          

            last_hb = self._telemetry.snapshot().last_heartbeat_at
            if last_hb is not None and time.monotonic() - last_hb > self.config.px4_hb_timeout:
                self._logger.warning("No MAVLink connection, attempting to reconnect")
                self._reconnect()
                continue

            try:
                msg = conn.recv_match(blocking=True, timeout=self.config.recv_match_timeout)
            except Exception as e:
                self._logger.warning("Error receiving MAVLink message: %s", e)
                continue

            if (
                msg is not None
                and msg.get_srcSystem() == conn.target_system
                and not self._stop_event.is_set()
            ):
                self._telemetry.update(msg)

    def _heartbeat_loop(self):
        self._logger.info("Heartbeat loop started")
        while not self._stop_event.is_set():
            with self._conn_lock:
                conn = self._conn
            if conn is None:
                self._stop_event.wait(1.0 / self.config.gcs_hb_rate)
                continue
            try:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0, 0
                )
            except Exception as e:
                self._logger.warning("Error sending MAVLink heartbeat: %s", e)
            self._stop_event.wait(1.0 / self.config.gcs_hb_rate)

    @property
    def state(self):
        with self._state_lock:
            return self._state
        
    def is_alive(self) -> bool:
        return not self._stop_event.is_set() and self._conn is not None

    def snapshot_telemetry(self) -> Telemetry:
        return self._telemetry.snapshot()

    def wait_for_telemetry(self, predicate : Callable[[Telemetry], bool], timeout : float) -> bool:
        return self._telemetry.wait_for(predicate, timeout)

    def stop(self):
        if self._stop_event.is_set():
            return
        with self._state_lock:
            self._state = ConnState.DEAD
            self._stop_event.set()
        with self._conn_lock:
            if self._conn is not None:
                self._conn.close()
                self._conn = None
        for thread in (self._hb_thread, self._tm_thread):
            if thread.is_alive() and thread is not threading.current_thread():
                thread.join()
        self._logger.info("MAVLinkConn stopped")

    def send_flight_mode(
        self, 
        mode : str
    ) -> SendingResult:
        with self._conn_lock:
            conn = self._conn
        if self._stop_event.is_set() or conn is None:
            return SendingResult.NO_CONN
        try:
            conn.set_mode(mode)
            return SendingResult.SUCCESS
        except Exception as e:
            self._logger.warning("Error sending MAVLink flight mode: %s", e)
            return SendingResult.FAILURE

    def send_command_long(
        self,
        command : int,
        params: tuple[float, ...],
        confirmation : int = 0
    ) -> SendingResult:
        with self._conn_lock:
            conn = self._conn
        if self._stop_event.is_set() or conn is None:
            return SendingResult.NO_CONN
        try:
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                command, confirmation, *params
            )
            return SendingResult.SUCCESS
        except Exception as e:
            self._logger.warning("Error sending MAVLink command_long: %s", e)
            return SendingResult.FAILURE

    def send_manual_control(
        self,
        roll : int,
        pitch : int,
        yaw : int,
        throttle : int
    ) -> SendingResult:
        with self._conn_lock:
            conn = self._conn
        if self._stop_event.is_set() or conn is None:
            return SendingResult.NO_CONN
        try:
            conn.mav.manual_control_send(
                conn.target_system,
                pitch, roll, throttle, yaw, 0
            )
            return SendingResult.SUCCESS
        except Exception as e:
            self._logger.warning("Error sending MAVLink manual_control: %s", e)
            return SendingResult.FAILURE

    def send_local_target(
        self,
        mask : int,
        target : tuple[float, ...]
    ) -> SendingResult:
        with self._conn_lock:
            conn = self._conn
        if self._stop_event.is_set() or conn is None:
            return SendingResult.NO_CONN
        try:
            conn.mav.set_position_target_local_ned_send(
                0, conn.target_system, conn.target_component, 
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, *target
            )
            return SendingResult.SUCCESS
        except Exception as e:
            self._logger.warning("Error sending MAVLink set_position_target_local_ned: %s", e)
            return SendingResult.FAILURE
        
