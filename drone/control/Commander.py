from abc import ABC, abstractmethod
import logging

from Gripper import DRONE_Gripper
from MAVLinkConn import DRONE_MAVLinkConn
from utils import DRONE_FlyCommand


class DRONE_Commander(ABC):
    def __init__(
        self,
        conn : DRONE_MAVLinkConn,
        gripper : DRONE_Gripper, 
        logger : logging.Logger
    ):
        self.logger = logger
        self.conn = conn
        self.gripper = gripper

        self.logger.debug("DRONE_Commander: Initializing commander...")

    def _arm(self):
        self.logger.debug("DRONE_Commander: sending arming command")
        self.conn.arm()

    def _send_flycommand(self, flycommand : DRONE_FlyCommand):
        self.logger.debug("DRONE_Commander: sending flycommand")
        self.conn.send_MC(flycommand)

    def _grip(self):
        self.logger.debug("DRONE_Commander: sending grip command")
        self.gripper.toggle()

    def _send_heartbeat(self):
        self.logger.debug("DRONE_Commander: sending heartbeat")
        self.conn.heartbeat()

    @abstractmethod
    def run(self):
        pass
