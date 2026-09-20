import pygame
import numpy as np
import logging

from Commander.MAVLinkCommander import DRONE_MAVLinkCommander
from ManualController.ManualController import DRONE_ManualController
from GripController.GripController import DRONE_GripController
from Utils.Configs import DRONE_FlyCommand


class DRONE_GamepadController(DRONE_ManualController):
    def __init__(
        self,
        com : DRONE_MAVLinkCommander,
        gripper : DRONE_GripController,
        logger : logging.Logger,
        deadzone = 0.1
    ):
        super().__init__(
            com=com, 
            gripper=gripper,
            logger=logger
        )

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.logger.error("Failed: no gamepad found")
            raise RuntimeError("Failed: no gamepad found")

        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.logger.debug(f"Gamepad: {self.js.get_name()}")

        self.deadzone = deadzone
        self.slow_multiplier = 1.0
        self.running = False

        self.logger.debug("Gamepad Commander initialized")

        self._help_prompt()
        
    def _help_prompt(self):
        print(
            "   Left Stick: Control Roll and Pitch\n",
            "   Right Stick: Control Yaw\n",
            "   Right Trigger: Up Throttle\n",
            "   Left Trigger: Down Throttle\n",
            "   Button A: Arm\n",
            "   Button B: Toggle Slow Mode\n",
            "   Button X: Grip/Release\n",
            "   Button Start: Exit"
        )

    def _normalize_axis(self, value, invert=False, name=None):
        res = 0
        if np.abs(value) >= self.deadzone:
            res = int((value if not invert else -value) * 1000)
        self.logger.debug(f"Normalizing {name}-axis, value={value}, result={res}")
        return res

    def _normalize_throttle(self, value, p=2.0, name=None):
        res = 0
        value = max(min(value, 1), -1)
        if value + 1 > self.deadzone:
            res = int(500 * (1 + np.sign(value) * (np.abs(value) ** p)))
        self.logger.debug(f"Normalizing {name}-throttle, value={value}, result={res}")
        return res

    def _get_attitude(self):
        roll = self.slow_multiplier * self.js.get_axis(1)
        pitch = self.slow_multiplier * self.js.get_axis(0)
        yaw = self.slow_multiplier * self.js.get_axis(2)

        self.logger.debug(f"Get attitude: roll - {roll}, pitch - {pitch}, yaw - {yaw}")

        roll = self._normalize_axis(roll, invert=True, name="roll")
        pitch = self._normalize_axis(pitch, name="pitch")
        yaw = self._normalize_axis(yaw, name="yaw")

        return roll, pitch, yaw

    def _get_throttle(self):
        up = self.slow_multiplier * self.js.get_axis(5)
        down = self.slow_multiplier * self.js.get_axis(4)

        self.logger.debug(f"Get throttle: up - {up}, down - {down}")

        up = self._normalize_throttle(up)
        down = self._normalize_throttle(down)

        throttle = 500 + (up - down) // 2
        self.logger.debug(f"Get throttle: throttle - {throttle}")

        return throttle

    def _handle_button(self, button):
        match button:
            case 0: # XBox A
                self._arm()
            case 1: # XBox B
                self.slow_multiplier = 1.5 - self.slow_multiplier
                self.logger.info(f"Slow mode is {"on" if self.slow_multiplier == 0.5 else "off"}")
            case 2: # XBox X
                self._grip()
            case 7: # XBox Start
                self.running = False

    def run(self):
        clock = pygame.time.Clock()
        self.running = True
        try: 
            while self.running:
                pygame.event.pump()

                roll, pitch, yaw = self._get_attitude()
                throttle = self._get_throttle()

                flycommand = DRONE_FlyCommand(roll, pitch, yaw, throttle)

                self._send_flycommand(flycommand)

                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        button = event.button
                        self._handle_button(button)

                self._send_heartbeat()

                clock.tick(50)

        except KeyboardInterrupt:
            self.running = False
            self.logger.warning("Bridge stopped by KeyboardInterrupt")
        finally:
            pygame.quit()
