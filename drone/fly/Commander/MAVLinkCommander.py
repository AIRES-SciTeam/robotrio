from pymavlink import mavutil
import logging

from drone.control.Utils.Configs import DRONE_ConnConfig, DRONE_FlyCommand


class DRONE_MAVLinkCommander:
    def __init__(
        self, 
        conn_config : DRONE_ConnConfig,
        logger : logging.Logger
    ):
        self.conn = mavutil.mavlink_connection(
            f"{conn_config.type}:{conn_config.ip}:{conn_config.port}",
            input=False, source_system=255
        )
        self.logger = logger

        self.logger.debug("DRONE_MAVLinkConn: MAVLink connection started")

    def arm(self):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 
            1, # 1 - arm, 0 - disarm
            0, 
            0, 0, 0, 0, 0
        )
        self.logger.info("DRONE_MAVLinkConn: Arming command sent")

    def send_MC(self, flycommand : DRONE_FlyCommand):
        self.conn.mav.manual_control_send(
            self.conn.target_system,
            flycommand.roll,
            flycommand.pitch,
            flycommand.throttle,
            flycommand.yaw,
            0
        )
        self.logger.debug("DRONE_MAVLinkConn: Manual Control command sent")
        self.logger.debug(f"DRONE_MAVLinkConn: Command: roll - {flycommand.roll}, pitch - {flycommand.pitch}, yaw - {flycommand.yaw}, throttle - {flycommand.throttle}")

    def heartbeat(self):
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        self.logger.debug("DRONE_MAVLinkConn: Heartbeat sent")
