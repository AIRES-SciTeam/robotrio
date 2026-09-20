import pygame
import numpy as np
import logging

from Commander.MAVLinkCommander import DRONE_MAVLinkCommander
from ManualController.ManualController import DRONE_ManualController
from GripController.GripController import DRONE_GripController
from Utils.Configs import DRONE_FlyCommand


class DRONE_KeyboardController(DRONE_ManualController):
    def __init__(
        self,
        com : DRONE_MAVLinkCommander,
        gripper : DRONE_GripController,
        logger : logging.Logger,
    ):
        super().__init__(
            com=com,
            gripper=gripper,
            logger=logger
        )

        pygame.init()
        pygame.display.set_mode((640, 240))
        pygame.display.set_caption("DRONE_KeyboardController")
        pygame.key.set_repeat()

        self.sensitivity = 1.0
        self.running = False

        self._help_prompt()

    def _help_prompt(self):
        print(
            "   A, D: Control Roll\n"
            "   W, S: Control Pitch\n"
            "   Q, E: Control Yaw\n"
            "   Space: Up Throttle\n"
            "   L-Shift: Down Throttle\n"
            "   R: Arm\n"
            "   1-9, 0: Sensitivity 10-100%\n"
            "   F: Grip/Release\n"
            "   Esc: Exit"
        )

    def _get_axis(self, keys, pos, neg):
        return int(keys[pos]) - int(keys[neg])

    def _get_attitude(self, keys):
        scale = int(1000 * self.sensitivity)
        roll = scale * self._get_axis(keys, pygame.K_w, pygame.K_s)
        pitch = scale * self._get_axis(keys, pygame.K_d, pygame.K_a)
        yaw = scale * self._get_axis(keys, pygame.K_e, pygame.K_q)

        return roll, pitch, yaw

    def _get_throttle(self, keys):
        scale = int(500 * self.sensitivity)
        throttle = 500 + scale * self._get_axis(keys, pygame.K_SPACE, pygame.K_LSHIFT)

        return throttle

    def _handle_button(self, key):
        match key:
            case pygame.K_r: 
                self._arm()
            case pygame.K_0:
                self.sensitivity = 1.0
            case pygame.K_1:
                self.sensitivity = 0.1
            case pygame.K_2:
                self.sensitivity = 0.2
            case pygame.K_3:
                self.sensitivity = 0.3
            case pygame.K_4:
                self.sensitivity = 0.4
            case pygame.K_5:
                self.sensitivity = 0.5
            case pygame.K_6:
                self.sensitivity = 0.6
            case pygame.K_7:
                self.sensitivity = 0.7
            case pygame.K_8:
                self.sensitivity = 0.8
            case pygame.K_9:
                self.sensitivity = 0.9
            case pygame.K_f:
                self._grip()
            case pygame.K_ESCAPE:
                self.running = False

    def run(self):
        clock = pygame.time.Clock()
        self.running = True
        try: 
            while self.running:
                pygame.event.pump()

                keys = pygame.key.get_pressed()
                roll, pitch, yaw = self._get_attitude(keys)
                throttle = self._get_throttle(keys)
  
                flycommand = DRONE_FlyCommand(roll, pitch, yaw, throttle)

                self._send_flycommand(flycommand)

                for event in pygame.event.get():  
                    if event.type == pygame.KEYDOWN:
                        self._handle_button(event.key)

                self._send_heartbeat()

                clock.tick(50)

        except KeyboardInterrupt:
            self.running = False
            self.logger.warning("Bridge stopped by KeyboardInterrupt")
        finally:
            pygame.quit()

