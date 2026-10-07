import pygame
from pygame._sdl2 import controller as sdl_controller
import numpy as np
import time

from logger.logger_fabric import LoggerFabric
from manual_controller.manual_controller import ManualController
from grip_controller.grip_controller import GripController
from safety_commander.commander import Commander


MODULE_NAME = "manual_controller"

class GamepadController(ManualController):
    def __init__(
        self,
        commander: Commander,
        grip_controller: GripController,
        logger_fabric: LoggerFabric,
        rate : int = 50,
        deadzone: float = 0.1,
    ) -> None:
        super().__init__(
            commander=commander,
            grip_controller=grip_controller,
            logger_fabric=logger_fabric,
            rate=rate,
        )

        pygame.init()
        pygame.joystick.init()
        self._js = None
        self._instance_id = None
        self._next_connect_at = 0.0
        self._dpad_buttons = {}
        self._sdl_controller = None
        sdl_controller.init()

        self._deadzone = deadzone
        self._sensitivity = 1.0

        self._help_prompt()
        self._thread.start()

        self._logger.info(f"Initialized")

    def _disconnect(self) -> None:
        was_connected = self._js is not None
        for device in (self._sdl_controller, self._js):
            if device is not None:
                try:
                    device.quit()
                except pygame.error:
                    pass
        self._js = None
        self._sdl_controller = None
        self._instance_id = None
        self._dpad_buttons = {}
        if was_connected:
            self._logger.warning("Gamepad disconnected; manual input stopped, waiting for reconnect")

    def _try_connect(self) -> None:
        self._next_connect_at = time.monotonic() + 1.0
        # SDL may miss hotplug notifications in a container. Re-enumerate
        # devices while disconnected instead of polling its stale device list.
        if self._js is not None:
            return
        sdl_controller.quit()
        pygame.joystick.quit()
        pygame.joystick.init()
        sdl_controller.init()
        for index in range(pygame.joystick.get_count()):
            try:
                self._js = pygame.joystick.Joystick(index)
                self._js.init()
                if self._js.get_numaxes() < 6:
                    self._disconnect()
                    continue
                self._instance_id = self._js.get_instance_id()
                if sdl_controller.is_controller(index):
                    self._sdl_controller = sdl_controller.Controller(index)
                    mapping = self._sdl_controller.get_mapping()
                    for name, direction in {
                        "dpleft": (-1, 0), "dpright": (1, 0),
                        "dpdown": (0, -1), "dpup": (0, 1),
                    }.items():
                        binding = mapping.get(name, "")
                        if binding.startswith("b") and binding[1:].isdigit():
                            self._dpad_buttons[int(binding[1:])] = direction
                self._logger.info("Gamepad connected: %s (instance %s)", self._js.get_name(), self._instance_id)
                return
            except pygame.error:
                self._disconnect()
        
    def _help_prompt(self) -> None:
        print(
            "   Left Stick: Control Roll and Pitch\n",
            "   Right Stick: Control Yaw\n",
            "   Right Trigger: Up Throttle\n",
            "   Left Trigger: Down Throttle\n",
            "   Button A: Arm\n",
            "   D-pad Left/Right: Sensitivity -/+10%\n",
            "   D-pad Down/Up: Sensitivity 10%/100%\n",
            "   Button X: Grip/Release\n",
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

    def _effective_sensitivity(self) -> float:
        return 0.13 if self._sensitivity == 0.1 else self._sensitivity

    def _get_attitude(self) -> tuple[int, int, int]:
        roll = self._js.get_axis(0)
        pitch = self._js.get_axis(1)
        yaw = self._js.get_axis(2)

        roll = self._normalize_axis(roll)
        pitch = self._normalize_axis(pitch, invert=True)
        yaw = self._normalize_axis(yaw)

        return tuple(int(value * self._effective_sensitivity()) for value in (roll, pitch, yaw))

    def _get_throttle(self) -> int:
        up = self._js.get_axis(5)
        down = self._js.get_axis(4)

        up = self._normalize_throttle(up)
        down = self._normalize_throttle(down)

        throttle = 500 + int((up - down) * self._effective_sensitivity() / 2)
        return throttle

    def _handle_button(self, button: int) -> None:
        if button in self._dpad_buttons:
            self._handle_hat(self._dpad_buttons[button])
            return
        match button:
            case 0: # XBox A
                self._toggle_arming()
            case 2: # XBox X
                self._toggle_gripping()

    def _handle_hat(self, value: tuple[int, int]) -> None:
        horizontal, vertical = value
        previous_sensitivity = self._sensitivity
        if vertical:
            self._sensitivity = 1.0 if vertical > 0 else 0.1
        elif horizontal:
            self._sensitivity = round(
                max(0.1, min(1.0, self._sensitivity + horizontal * 0.1)), 1
            )
        if self._sensitivity != previous_sensitivity:
            self._logger.info(
                f"Gamepad sensitivity set to {self._sensitivity:.0%}"
            )

    def _handle_pygame_event(self):
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.JOYDEVICEREMOVED and event.instance_id == self._instance_id:
                self._disconnect()
            elif event.type == pygame.JOYDEVICEADDED:
                self._next_connect_at = 0.0
        if self._js is None:
            if time.monotonic() >= self._next_connect_at:
                self._try_connect()
            return
        for event in events:
            if getattr(event, "instance_id", None) != self._instance_id:
                continue
            if event.type == pygame.JOYBUTTONDOWN:
                self._handle_button(event.button)
            elif event.type == pygame.JOYHATMOTION and event.hat == 0:
                self._handle_hat(event.value)
        roll, pitch, yaw = self._get_attitude()
        throttle = self._get_throttle()
        self._logger.log(5, 
            f"Manual control: "
            f"roll={roll}, pitch={pitch}, yaw={yaw}, throttle={throttle}"
        )
        self._manual_control(roll, pitch, yaw, throttle)

    def _spin(self) -> None:
        clock = pygame.time.Clock()
        self._logger.info(f"Gamepad control started at {self.rate} Hz")
        while not self._stop_event.is_set():
            try:
                self._handle_pygame_event()
            except pygame.error as exc:
                self._logger.warning("Gamepad read failed: %s", exc)
                self._disconnect()
                self._next_connect_at = time.monotonic() + 1.0
            clock.tick(self.rate)
             
    def stop(self):
        super().stop()
        self._disconnect()
        pygame.quit()
        self._logger.info("Stopped")
