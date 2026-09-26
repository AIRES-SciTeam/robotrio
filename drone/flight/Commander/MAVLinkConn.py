from pymavlink import mavutil
from dataclasses import dataclass
import time
import threading
from enum import Enum, auto
from typing import Callable

from Logger.LoggerFabric import LoggerFabric
from DroneTelemetry import DroneTelemetry


@dataclass
class MAVLinkConnConfig:
    address : str = "udpin:127.0.0.1:14541"
    source_system : int = 255
    
    own_heartbeat_rate : float = 1          # Hz
    px4_heartbeat_timeout : float = 5.0     # s
    recv_match_timeout : float = 0.2        # s

    max_reconnects : int | None = None

class SendingResult(Enum):
    SENT = auto()
    NO_CONN = auto()
    SEND_FAILED = auto()
    INV_ARGS  = auto()

class MAVLinkConn:
    def __init__(
        self,
        logger_fabric : LoggerFabric,
        conn_config : MAVLinkConnConfig,
        callback : Callable
    ):
        self.conn_config = conn_config
        self.callback = callback
        self._logger = logger_fabric.get_logger("MAVLinkConn")

        self._telemetry = DroneTelemetry(logger_fabric)

        self._stop_event = threading.Event()
        
        self._conn_lock = threading.Lock()
        self._conn : mavutil.mavfile | None = None
        self._hb_thread = None
        self._tm_thread = None
        self._reconnect()
        if self._stop_event.is_set():
            return

        self._hb_thread = threading.Thread(
            target=self._heartbeat,
            daemon=True
        )
        self._hb_thread.start()

        self._tm_thread = threading.Thread(
            target=self._upd_telemetry,
            daemon=True
        )
        self._tm_thread.start()

        self._logger.info("Initialized")

    def is_available(self) -> bool:
        with self._conn_lock:
            if self._stop_event.is_set() or self._conn is None:
                return False
            last_hb = self._telemetry.snapshot().last_heartbeat_at
            return (last_hb is not None and
                    time.monotonic() - last_hb <= self.conn_config.px4_heartbeat_timeout)

    def tm_snapshot(self):
        return self._telemetry.snapshot()

    def mode_mapping(self):
        with self._conn_lock:
            if self._conn is None:
                return None
            mapping = self._conn.mode_mapping()
            return mapping.copy() if mapping is not None else None

    def wait_for_state(self, predicate, timeout : float):
        return self._telemetry.wait_for(predicate, timeout)

    def stop(self, inner : bool = False):
        with self._conn_lock:
            first_stop = not self._stop_event.is_set()
            self._stop_event.set()
            if self._conn is not None:
                self._conn.close()
                self._conn = None
        if inner:
            if first_stop:
                self.callback()
            return
        for t in [self._tm_thread, self._hb_thread]:
            if (
                t is not None
                and t.is_alive()
                and t is not threading.current_thread()
            ):
                t.join()
        self._logger.info("Stopped.")

    def _reconnect(self):
        with self._conn_lock:
            if self._conn is not None:
                self._conn.close()
                self._conn = None
        try_n = 1                
        while (
            not self._stop_event.is_set()
            and (
                self.conn_config.max_reconnects is None
                or try_n <= self.conn_config.max_reconnects
            )
        ):
            self._telemetry.reset()
            self._logger.debug('Connecting at %s, ss = %s, try_n = %s / %s', self.conn_config.address, self.conn_config.source_system, try_n, self.conn_config.max_reconnects)
            
            conn = mavutil.mavlink_connection(
                self.conn_config.address, 
                source_system=self.conn_config.source_system
            )
            
            hb = None
            try:
                hb = conn.wait_heartbeat(timeout=self.conn_config.px4_heartbeat_timeout)
            except Exception as e:
                self._logger.error('Heartbeat waiting exception: %s', e)
            if self._stop_event.is_set():
                conn.close()
                return
            
            if (
                hb is not None 
                and hb.get_srcSystem() == conn.target_system
                and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
            ):
                self._logger.info("Connected")
                self._telemetry.update(hb)
                with self._conn_lock:
                    if self._stop_event.is_set():
                        conn.close()
                        return
                    self._conn = conn
                return
            else:
                conn.close()
                self._logger.warning("No PX4 heartbeat")
            
            try_n += 1

        if self._stop_event.is_set():
            return
            
        self._logger.error("Connection failed")
        self.stop(inner = True)
        return

    def _heartbeat(self):
        self._logger.info("Heartbeat thread started")
        while not self._stop_event.wait(1 / self.conn_config.own_heartbeat_rate):
            with self._conn_lock:
                if self._conn is None:
                    self._logger.log(5, "HBT: Connection is None")
                    continue
                self._logger.log(5, "Heartbeat sent")
                try:
                    self._conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0
                    )
                except Exception as e:
                    self._logger.error("Heartbeat sending error: %s", e)
                    break
        if not self._stop_event.is_set():
            self.stop(inner=True)

    def _upd_telemetry(self):
        self._logger.info("Telemetry thread started")
        while not self._stop_event.is_set():
            last_hb = self._telemetry.snapshot().last_heartbeat_at
            if last_hb is not None and time.monotonic() - last_hb <= self.conn_config.px4_heartbeat_timeout:
                with self._conn_lock:
                    conn = self._conn
                if conn is None:
                    return
                try:
                    msg = conn.recv_match(blocking=True, timeout=self.conn_config.recv_match_timeout)    
                except Exception as e:
                    self._logger.error('recv_match error: %s', e)
                    continue
                if (
                    msg is not None
                    and not self._stop_event.is_set()
                    and msg.get_srcSystem() == conn.target_system
                ):
                    self._telemetry.update(msg)
            else:
                self._logger.warning('No heartbeat more than %s s. Reconnecting', self.conn_config.px4_heartbeat_timeout)
                self._reconnect()

    def set_flight_mode(self, name : str) -> SendingResult:
        with self._conn_lock:
            if self._stop_event.is_set() or self._conn is None:
                self._logger.warning('Flight mode setting: connection is None')
                return SendingResult.NO_CONN
            self._logger.info('Requesiong flight mode: %s', name)
            try:
                self._conn.set_mode(name)
            except Exception as e:
                self._logger.error("Flight mode setting error: %s", e)
                return SendingResult.SEND_FAILED
            self._logger.info('Flight mode sent: %s', name)
            return SendingResult.SENT

    def send_command_long(
        self, 
        command : int,
        params : tuple[float,...],
        confirmation : int = 0
    ) -> SendingResult:
        with self._conn_lock:
            if self._stop_event.is_set() or self._conn is None:
                self._logger.warning('Command long sending: connection is None')
                return SendingResult.NO_CONN
            try:
                self._conn.mav.command_long_send(
                    self._conn.target_system, self._conn.target_component,
                    command, confirmation, *params
                )
            except Exception as e:
                self._logger.error("Command long sending error: %s", e)
                return SendingResult.SEND_FAILED
            self._logger.debug('Command long sent: (%s, %s, %s)', command, confirmation, params)
            return SendingResult.SENT

    def send_manual_control(
        self,
        roll : int,     # [-1000, 1000]
        pitch : int,    # [-1000, 1000]
        yaw : int,      # [-1000, 1000]
        throttle : int  # [0, 1000]
    ) -> SendingResult:
        with self._conn_lock:
            if self._stop_event.is_set() or self._conn is None:
                self._logger.warning('Manual control sending: connection is None')
                return SendingResult.NO_CONN
            try:
                self._conn.mav.manual_control_send(
                    self._conn.target_system,
                    pitch, roll, throttle, yaw, 0 
                )
            except Exception as e:
                self._logger.error("Manual control sending error: %s", e)
                return SendingResult.SEND_FAILED
            self._logger.log(5, "Manual control sent: roll=%s, pitch=%s, yaw=%s, throttle=%s", roll, pitch, yaw, throttle)
            return SendingResult.SENT

    def send_local_target(
        self, 
        mask : int, 
        target : tuple[float, ...]
    ) -> SendingResult:
        with self._conn_lock:
            if self._stop_event.is_set() or self._conn is None:
                self._logger.warning("Local target sending: connection is None")
                return SendingResult.NO_CONN
            try:
                self._conn.mav.set_position_target_local_ned_send(
                    0, self._conn.target_system, self._conn.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, *target
                )
            except Exception as e:
                self._logger.error("Local target sending error: %s", e)
                return SendingResult.SEND_FAILED
            self._logger.log(5, "Local target sent: mask=%s, target=%s", mask, target)
            return SendingResult.SENT
