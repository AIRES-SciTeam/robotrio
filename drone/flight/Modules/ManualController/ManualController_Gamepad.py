import pygame
import numpy as np
import threading

from Modules.Commander.Commander import Commander
from Modules.ManualController.ManualController import ManualController
from Modules.GripController.GripController import GripController
from Modules.Logger.LoggerFabric import LoggerFabric


class GamepadController(ManualController):
    def __init__(
        self,
        commander: Commander,
        grip_ctrl: GripController,
        logger_fabric: LoggerFabric,
        rate : int = 50,
        deadzone: float = 0.1,
    ) -> None:
        super().__init__(
            commander=commander,
            grip_ctrl=grip_ctrl,
            logger_fabric=logger_fabric,
            rate=rate,
            type="Gamepad"
        )

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self._logger.error("No gamepad found")
            raise RuntimeError("Failed: no gamepad found")

        self._js = pygame.joystick.Joystick(0)
        self._js.init()
        self._logger.debug(f"Gamepad: {self._js.get_name()}")

        self._deadzone = deadzone
        self._multiplier = 1.0

        self._help_prompt()
        self._thread.start()

        self._logger.info(f"Initialized")
        
    def _help_prompt(self) -> None:
        print(
            "   Left Stick: Control Roll and Pitch\n",
            "   Right Stick: Control Yaw\n",
            "   Right Trigger: Up Throttle\n",
            "   Left Trigger: Down Throttle\n",
            "   Button A: Arm\n",
            "   Button Y: Start/Stop Mission\n",
            "   Button B: Toggle Slow Mode\n",
            "   Button X: Grip/Release\n",
            "   D-pad Up/Down: Pause/Resume Mission\n",
            "   D-pad Left/Right: Previous/Next Mission Step\n",
            "   Button Start: Exit"
        )

    def _normalize_axis(self, value: float, invert: bool = False) -> int:
        res = 0
        if np.abs(value) >= self._deadzone:
            res = int((value if not invert else -value) * 1000)
        return res

    def _normalize_throttle(self, value: float, p: float = 2.0) -> int:
        res = 0
        value = max(min(value, 1), -1)
        if value + 1 > self._deadzone:
            res = int(500 * (1 + np.sign(value) * (np.abs(value) ** p)))
        return res

    def _get_attitude(self) -> tuple[int, int, int]:
        roll = self._multiplier * self._js.get_axis(0)
        pitch = self._multiplier * self._js.get_axis(1)
        yaw = self._multiplier * self._js.get_axis(2)

        roll = self._normalize_axis(roll)
        pitch = self._normalize_axis(pitch, invert=True)
        yaw = self._normalize_axis(yaw)

        return roll, pitch, yaw

    def _get_throttle(self) -> int:
        up = self._multiplier * self._js.get_axis(5)
        down = self._multiplier * self._js.get_axis(4)

        up = self._normalize_throttle(up)
        down = self._normalize_throttle(down)

        throttle = 500 + (up - down) // 2
        return throttle

    def _handle_button(self, button: int) -> None:
        match button:
            case 0: # XBox A
                self._toggle_arming()
            case 1: # XBox B
                self._multiplier = 1.5 - self._multiplier
                self._logger.info(f"Gamepad slow mode is {'on' if self._multiplier == 0.5 else 'off'}")
            case 2: # XBox X
                self._toggle_gripping()
            case 7: # XBox Start
                self._logger.info("Gamepad exit requested")
                self.running = False

    def _handle_pygame_event(self):
        pygame.event.pump()
        roll, pitch, yaw = self._get_attitude()
        throttle = self._get_throttle()
        self._logger.log(5, 
            f"Manual control: "
            f"roll={roll}, pitch={pitch}, yaw={yaw}, throttle={throttle}"
        )
        self._manual_control(roll, pitch, yaw, throttle)
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                button = event.button
                self._handle_button(button)

    def _spin(self) -> None:
        clock = pygame.time.Clock()
        self._logger.info(f"Gamepad control started at {self.rate} Hz")
        while not self._stop_event.is_set():
            self._handle_pygame_event()
            clock.tick(self.rate)
             
    def stop(self):
        super().stop()
        pygame.quit()
        self._logger.info("Stopped")
