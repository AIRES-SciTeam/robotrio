from pymavlink import mavutil
import threading, time
from enum import Enum, auto

from logger.logger_fabric import LoggerFabric
from safety_commander.control_flight import ControlFlight
from safety_commander.control_offboard import ControlOffboard, OffboardConfig, TargetResult, OffboardState
from safety_commander.mavlink_conn import MAVLinkConn, MAVLinkConnConfig, ConnState, SendingResult
from safety_commander.telemetry import Telemetry


MODULE_NAME = "safety_commander"

class CommandResult(Enum):
    SUCCESS = auto()
    NO_CONN = auto()
    FAILURE = auto()
    INVALID = auto()
    LAZY_OK = auto()
    TIMEOUT = auto()
    WR_MODE = auto()
    STOPPED = auto()

    @staticmethod
    def from_sending_result(res):
        return {
            SendingResult.SUCCESS : CommandResult.SUCCESS,
            SendingResult.NO_CONN : CommandResult.NO_CONN,
            SendingResult.FAILURE : CommandResult.FAILURE,
            SendingResult.INVALID : CommandResult.INVALID
        }[res]

class CommanderState(Enum):
    OK = auto()
    RECONN = auto()
    NO_CONN = auto()
    NO_OFFBOARD = auto()
    DEAD = auto()

class Commander:
    def __init__(
        self,
        logger_fabric : LoggerFabric,
        conn_config : MAVLinkConnConfig,
        offboard_config : OffboardConfig,
    ):
        if not logger_fabric.is_alive():
            raise RuntimeError("LoggerFabric is stopped")
        self._logger = logger_fabric.get_logger(f"{type(self).__name__}", module_name=MODULE_NAME)
        self._logger.info("Initializing")

        self._stop_event = threading.Event()
    
        self._nanny = threading.Thread(target=self._babysit, daemon=True)
        self._state = CommanderState.OK
        self._state_lock = threading.Lock()

        self._command_lock = threading.RLock()
        self._conn = None
        self._control_flight = None
        self._control_offboard = None

        try:
            self._conn = MAVLinkConn(conn_config, logger_fabric)
            if not self._conn.is_alive():
                raise TimeoutError("No connection")
            self._control_flight = ControlFlight(self._conn, logger_fabric)
            self._control_offboard = ControlOffboard(offboard_config, self._conn, logger_fabric)
            if not self._control_offboard.is_alive():
                raise TimeoutError("No Offboard controller")
            self._nanny.start()
        except Exception as e:
            self._logger.error(f"Failed to initialize Commader: {e}")
            self.stop()
            return
        
        self._logger.info("Initialized")

    def stop(self):
        if self._stop_event.is_set():
            return
        with self._command_lock:
            with self._state_lock:
                self._state = CommanderState.DEAD
            self._stop_event.set()
        if self._nanny.is_alive() and self._nanny is not threading.current_thread():
            self._nanny.join()
        if self._control_offboard is not None:
            self._control_offboard.stop()
        if self._conn is not None:
            self._conn.stop()

    def _handle_conn(self):
        conn_state = self._conn.state
        with self._state_lock:
            if self._state == CommanderState.DEAD:
                return
            if conn_state == ConnState.RECONN:
                self._state = CommanderState.RECONN
            elif conn_state == ConnState.DEAD:
                self._state = CommanderState.NO_CONN
            elif self._state in (CommanderState.RECONN, CommanderState.NO_CONN):
                self._state = CommanderState.OK
        if conn_state != ConnState.OK and self._control_offboard is not None:
            self._control_offboard.deactivate()

    def _handle_offboard(self):
        match self._control_offboard.state:
            case OffboardState.DEAD:
                self._logger.error("Control Offboard is dead")
                with self._state_lock:
                    if self._state == CommanderState.OK:
                        self._state = CommanderState.NO_OFFBOARD
            case OffboardState.ACTIVATING:
                self._logger.log(5, "Offboard State: ACTIVATING")
                if self._conn.snapshot_telemetry().flight_mode == "OFFBOARD":
                    self._control_offboard.mark_active()
            case OffboardState.DEACTIVATING:
                self._logger.log(5, "Offboard State: DEACTIVATING")
                if self._conn.snapshot_telemetry().flight_mode != "OFFBOARD":
                    self._control_offboard.mark_idle()
            case OffboardState.ACTIVE:
                if self._conn.snapshot_telemetry().flight_mode != "OFFBOARD":
                    self._control_offboard.mark_idle()
            case OffboardState.IDLE:
                self._logger.log(5, "Offboard State: IDLE")
                return

    def _babysit(self):
        self._logger.info("Nanny is on guard")
        while not self._stop_event.is_set():
            if self._conn is not None:
                self._handle_conn()
            if self._control_offboard is not None:
                self._handle_offboard()
            self._stop_event.wait(0.2)

    def _can_send(self):
        with self._state_lock:
            return self._state in (CommanderState.OK, CommanderState.NO_OFFBOARD)

    def is_alive(self):
        with self._state_lock:
            return self._state != CommanderState.DEAD

    def _wait_confirmation(self, predicate, timeout=None):
        if timeout is None:
            return CommandResult.LAZY_OK
        with self._state_lock:
           state = self._state
        if state in (CommanderState.OK, CommanderState.NO_OFFBOARD):
            if self._conn.wait_for_telemetry(predicate, timeout):
                return CommandResult.SUCCESS
        return CommandResult.TIMEOUT
        
    def _do_arming(self, armed, timeout=2.0):
        self._logger.info("Requesting armed = %s", armed)
        if not self.is_alive():
            return CommandResult.STOPPED
        if not self._can_send():
            return CommandResult.NO_CONN
        with self._command_lock:
            predicate = lambda state: state.armed is armed
            if not predicate(self._conn.snapshot_telemetry()):
                send = self._control_flight.arm if armed else self._control_flight.disarm
                res = CommandResult.from_sending_result(send())
                if res != CommandResult.SUCCESS:
                    return res
            return self._wait_confirmation(predicate, timeout)

    def _snapshot_telemetry(self):
        if self._conn is None:
            return Telemetry()
        return self._conn.snapshot_telemetry()

    def armed(self):
        return self._snapshot_telemetry().armed

    def is_landed(self):
        return self._snapshot_telemetry().landed

    def get_drone_position(self):
        state = self._snapshot_telemetry()
        return state.x, state.y, state.z, state.q

    def get_drone_velocity(self):
        state = self._snapshot_telemetry()
        return state.vx, state.vy, state.vz

    def arm(self):
        return self._do_arming(True)

    def disarm(self):
        return self._do_arming(False)

    def toggle_arming(self):
        if not self.is_alive():
            return CommandResult.STOPPED
        if not self._can_send():
            return CommandResult.NO_CONN
        with self._command_lock:
            if self._conn.snapshot_telemetry().armed:
                return self.disarm()
            return self.arm()
        
    toogle_arming = toggle_arming

    def set_flight_mode(self, name : str, timeout=3.0):
        self._logger.info("Requesting flight mode = %s", name)
        if not self.is_alive():
            return CommandResult.STOPPED
        if not self._can_send():
            return CommandResult.NO_CONN
        if (name := name.upper()) == "OFFBOARD":
            return CommandResult.INVALID
        with self._command_lock:
            mapping = self._conn.snapshot_telemetry().flight_mode_mapping
            if mapping is None:
                return CommandResult.NO_CONN
            if name not in mapping:
                return CommandResult.INVALID
            self._control_offboard.deactivate()
            predicate = lambda state: state.flight_mode == name
            res = CommandResult.SUCCESS
            if not predicate(self._conn.snapshot_telemetry()):
                res = CommandResult.from_sending_result(self._control_flight.flight_mode(name))
            if res == CommandResult.SUCCESS and (timeout is None or timeout >= 0):
                res = self._wait_confirmation(predicate, timeout)
            return res

    def loiter(self):
        return self.set_flight_mode("LOITER")

    def takeoff(self):
        return self.set_flight_mode("TAKEOFF")

    def land(self):
        return self.set_flight_mode("LAND")

    def manual_control(self, roll, pitch, yaw, throttle):
        if not self.is_alive():
            return CommandResult.STOPPED
        if not self._can_send():
            return CommandResult.NO_CONN
        # Keep streaming input while arm/mode commands wait for telemetry.
        if self._conn.snapshot_telemetry().flight_mode not in ["LOITER", "MANUAL", "POSCTL"]:
            return CommandResult.WR_MODE
        if (res := self._control_flight.manual_control(roll, pitch, yaw, throttle)) == SendingResult.SUCCESS:
            return CommandResult.LAZY_OK
        return CommandResult.from_sending_result(res)

    def set_offboard_target(self, pos=None, vel=None, yaw=None, yaw_rate=None):
        if not self.is_alive():
            return CommandResult.STOPPED
        if not self._can_send():
            return CommandResult.NO_CONN
        if self._control_offboard.state == OffboardState.DEAD:
            return CommandResult.STOPPED
        with self._command_lock:
            res = self._control_offboard.set_target(pos, vel, yaw, yaw_rate)
            if res != TargetResult.SUCCESS:
                return {
                    TargetResult.INVALID: CommandResult.INVALID,
                    TargetResult.STOPPED: CommandResult.STOPPED,
                    TargetResult.NO_CONN: CommandResult.NO_CONN,
                    TargetResult.FAILURE: CommandResult.FAILURE,
                }[res]
            state = self._control_offboard.activate()
            if state == OffboardState.DEAD:
                return CommandResult.STOPPED
            if state == OffboardState.DEACTIVATING:
                return CommandResult.WR_MODE
            if state == OffboardState.ACTIVE:
                return CommandResult.SUCCESS
            # Stream setpoints before requesting PX4's OFFBOARD mode.
            if self._stop_event.wait(1.1):
                return CommandResult.STOPPED
            if self._control_offboard.state == OffboardState.ACTIVE:
                return CommandResult.SUCCESS
            if self._control_offboard.state != OffboardState.ACTIVATING:
                return CommandResult.FAILURE
            res = CommandResult.from_sending_result(self._control_flight.flight_mode("OFFBOARD"))
            if res == CommandResult.SUCCESS:
                res = self._wait_confirmation(lambda state: state.flight_mode == "OFFBOARD", timeout=2.0)
            if res == CommandResult.SUCCESS:
                if self._control_offboard.mark_active() != OffboardState.ACTIVE:
                    return CommandResult.FAILURE
            else:
                self._control_offboard.deactivate()
            return res
