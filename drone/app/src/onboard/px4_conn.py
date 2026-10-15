from pymavlink import mavutil
import threading, time, json
from enum import Enum, auto
from typing import Dict, Any

from onboard.logger_fabric import LoggerFabric
from onboard.telemetry import TelemetryManager


class SendingResult(Enum):
    SUCCESS = auto()
    NO_CONN = auto()
    FAILURE = auto()
    INVALID = auto()

class ConnState(Enum):
    OK = auto()
    RECONN = auto()
    DEAD = auto()

class PX4Conn:
    def __init__(
        self,
        config : Dict[str, Any],
        logger_fabric : LoggerFabric
    ):
        self._logger = logger_fabric.get_logger(
            name=f"onboard.{type(self).__name__}",
            level=config.get("log_level", "info")
        )

        self._logger.info("Initializing")

        self._px4_address= config.get("address", "udpin:127.0.0.1:14551")
        self._logger.info("address = %s", self._px4_address)
        self._source_system = config.get("source_system", 255)
        self._logger.info("source_system = %s", self._source_system)
        self._hb_rate = config.get("heartbeat_rate", 1.0)
        self._logger.info("heartbeat_rate = %s", self._hb_rate)        
        self._hb_timeout = config.get("heartbeat_timeout", 5.0)
        self._logger.info("heartbeat_timeout = %s", self._hb_timeout)    
        self._rm_timeout = config.get("recv_match_timeout", 1.0)
        self._logger.info("recv_match_timeout = %s", self._rm_timeout)    
        self._max_reconnects = config.get("max_reconnects", None) 
        self._logger.info("max_reconnects = %s", self._max_reconnects)    

        self._stop_event = threading.Event()

        self._state = ConnState.RECONN
        self._state_lock = threading.Lock()

        self._conn : mavutil.mavlink_connection | None = None
        self._conn_lock = threading.Lock()

        self._telemetry = TelemetryManager({"log_level": config.get("telemetry_log_level", "info")})

        self._hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._tm_thread = threading.Thread(target=self._tm_loop, daemon=True)

        self._reconn()
        if self._stop_event.is_set():
            self._logger.critical("Initialization failed: stop_event is set")
            return
        
        self._hb_thread.start()
        self._tm_thread.start()

        self._logger.info("Initialized")

    def stop(self):
        self._logger.debug("Stopping")
        if self._stop_event.is_set():
            self._logger.debug("Is already stopped")
            return
        self._set_state(ConnState.DEAD)
        self._stop_event.set()
        self._logger.debug("Stop_event is set by stop")
        self._close_conn()
        for t in (self._hb_thread,):
            if t.is_alive() and t is not threading.current_thread():
                t.join()
                self._logger.debug("HB_thread joined")
        self._logger.info("Stopped")

    def _set_state(self, new_state):
        self._logger.info("Setting new state: %s", new_state)
        with self._state_lock:
            self._logger.debug("State_lock locked by _set_state")
            self._state = new_state
            self._logger.info("New state: %s", self._state)
        self._logger.debug("State_lock unlocked by _set_state")

    def _close_conn(self):
        self._logger.debug("Closing connection")
        with self._conn_lock:
            self._logger.debug("Conn_lock locked by _close_conn")
            if self._conn is not None:
                self._conn.close()
                self._conn = None 
                self._logger.debug("Connection closed")
            self._logger.debug("Connection already is none")
        self._logger.debug("Conn_lock unlocked by _close_conn")

    def _reconn_attempt(self):
        self._logger.debug("Reconnection attempt")
        conn = None
        hb = None
        try:
            conn = mavutil.mavlink_connection(
                self._px4_address,
                source_system=self._source_system
            )
            hb = conn.wait_heartbeat(timeout=2*self._hb_timeout)
        except Exception as e:
            self._logger.error("Connection failed: %s", e)
            if conn is not None:
                conn.close()
                return False
        if (
            hb is not None
            and hb.get_srcSystem() == conn.target_system
            and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
        ):
            with self._conn_lock:
                self._logger.debug("Conn_lock locked by _reconn_attempt")
                if self._stop_event.is_set():
                    self._logger.error("Connection failed: stop_event is set")
                    conn.close()
                    return False
                self._conn = conn
            self._logger.debug("Conn_lock unlocked by _reconn_attempt")
            self._set_state(ConnState.OK)
            self._logger.info("Connected to MAVLink at %s", self._px4_address)
            return True
        else:
            if hb is None:
                self._logger.error("Connection failed: timeout")
            else:
                self._logger.error("Connection failed: invalid heartbeat: %s", hb)
            if conn is not None:
                conn.close()
            return False
        
    def _reconn(self):
        if self._stop_event.is_set():
            self._logger.error("Stop event is set")
            return
        self._set_state(ConnState.RECONN)
        self._close_conn()
        
        attempt = 1
        max_attempts = self._max_reconnects if self._max_reconnects is not None else float("inf")
        while (
            not self._stop_event.is_set()
            and attempt <= max_attempts
        ):
            self._logger.info("Connecting to MAVLink at %s (attempt %d / %s)", self._px4_address, attempt, max_attempts if max_attempts != float('inf') else "∞")
            res = self._reconn_attempt()
            if res:
                return
            attempt += 1
            self._stop_event.wait(1)
        if self._stop_event.is_set():
            return
        self._logger.critical("Failed to connect to MAVLink after %d attempts", attempt - 1)
        self.stop()

    def _hb_loop(self):
        self._logger.info("HB_thread started")
        while not self._stop_event.is_set():
            with self._conn_lock:
                self._logger.trace("Conn_lock locked by _hb_loop")
                conn = self._conn
            self._logger.trace("Conn_lock unlocked by _hb_loop")
            if conn is None:
                self._stop_event.wait(1.0 / self._hb_thread)
                continue
            try:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTPILOT_INVALID,
                    0, 0, 0, 0
                )
                self._logger.trace("HB sent successfully")
            except Exception as e:
                self._logger.warning("Error sending MAVLink heartbeat: %s", e)
            self._stop_event.wait(1.0 / self._hb_rate)

    def _tm_loop(self):
        self._logger.info("TM_thread started")
        while not self._stop_event.is_set():
            with self._conn_lock:
                self._logger.trace("Conn_lock locked by _tm_loop")
                conn = self._conn
            self._logger.trace("Conn_lock unlocked by _tm_loop")
            last_hb = self._telemetry.snapshot().last_heartbeat_at
            if last_hb is not None and time.monotonic() - last_hb > self._hb_timeout

