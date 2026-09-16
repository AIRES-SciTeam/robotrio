from abc import ABC, abstractmethod
import logging

from drone.control.GripContorller.GripController import DRONE_GripController
from drone.control.MAVLinkCommander import DRONE_MAVLinkCommander
from drone.control.Utils.Configs import DRONE_FlyCommand


class DRONE_ManualController(ABC):
    def __init__(
        self,
        com : DRONE_MAVLinkCommander,
        gripper : DRONE_GripController, 
        logger : logging.Logger
    ):
        self.logger = logger
        self.com = com
        self.gripper = gripper

        self.logger.debug("DRONE_Commander: Initializing commander...")

    def _arm(self):
        self.logger.debug("DRONE_Commander: sending arming command")
        self.com.arm()

    def _send_flycommand(self, flycommand : DRONE_FlyCommand):
        self.logger.debug("DRONE_Commander: sending flycommand")
        self.com.send_MC(flycommand)

    def _grip(self):
        self.logger.debug("DRONE_Commander: sending grip command")
        self.gripper.toggle()

    def _send_heartbeat(self):
        self.logger.debug("DRONE_Commander: sending heartbeat")
        self.com.heartbeat()

    @abstractmethod
    def run(self):
        pass
