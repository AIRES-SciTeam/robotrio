from pymavlink import mavutil
from enum import Enum, auto

from MAVLinkConn import MAVLinkConn, SendingResult
from Logger.LoggerFabric import LoggerFabric


class FlightControl:
    def __init__(
        self,
        conn : MAVLinkConn,
        logger_fabric : LoggerFabric
    ):
        self._logger = logger_fabric.get_logger("FlightControl")
        self.conn = conn 
        self._logger.info("Initialized")

    def arm(self):
        self._logger.info("Arming")
        res = self.conn.send_command_long(
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            params=(1, 0, 0, 0, 0, 0, 0)
        )
        self._logger.debug("Arming result: %s", res.name)
        return res

    def disarm(self):
        self._logger.info("Disarming")
        res = self.conn.send_command_long(
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            params=(0, 0, 0, 0, 0, 0, 0)
        )
        self._logger.debug("Disarming result: %s", res.name)
        return res

    def flight_mode(self, name : str):
        name = name.upper()
        mode_mapping = self.conn.mode_mapping()
        if mode_mapping is None:
            return SendingResult.NO_CONN
        if name not in mode_mapping:
            return SendingResult.INV_ARGS
        self._logger.info("Setting flight mode: %s", name)
        res = self.conn.set_flight_mode(name)
        self._logger.debug("Setting flight mode result: %s", res.name)
        return res
        
    def manual_control(
        self,
        roll : int,     # [-1000, 1000]
        pitch : int,    # [-1000, 1000]
        yaw : int,      # [-1000, 1000]
        throttle : int  # [0, 1000]
    ):
        if (
            roll < -1000 or roll > 1000
            or pitch < -1000 or pitch > 1000
            or yaw < -1000 or yaw > 1000
            or throttle < 0 or throttle > 1000
        ):
            self._logger.error("Invalid manual control: roll=%s, pitch=%s, yaw=%r, throttle=%s", roll, pitch, yaw, throttle)
            return SendingResult.INV_ARGS
        self._logger.log(5, "Manual Control: roll=%s, pitch=%s, yaw=%s, throttle=%s", roll, pitch, yaw, throttle)
        res = self.conn.send_manual_control(roll, pitch, yaw, throttle)
        self._logger.log(5, "Manual Control result: %s", res.name)
        return res
