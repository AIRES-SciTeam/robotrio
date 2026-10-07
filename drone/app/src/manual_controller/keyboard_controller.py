import pygame

from logger.logger_fabric import LoggerFabric
from manual_controller.manual_controller import ManualController
from grip_controller.grip_controller import GripController
from safety_commander.commander import Commander


MODULE_NAME = "manual_controller"


class KeyboardController(ManualController):
    def __init__(
        self,
        commander: Commander,
        grip_controller: GripController,
        logger_fabric: LoggerFabric,
        rate: int = 50,
    ) -> None:
        super().__init__(
            commander=commander,
            grip_controller=grip_controller,
            logger_fabric=logger_fabric,
            rate=rate,
        )

        pygame.init()
        pygame.display.set_mode((640, 240))
        pygame.display.set_caption("KeyboardController")
        pygame.key.set_repeat()

        self._sensitivity = 1.0

        self._help_prompt()
        self._thread.start()

        self._logger.info("Initialized")

    def _help_prompt(self) -> None:
        print(
            "   A, D: Control Roll\n"
            "   W, S: Control Pitch\n"
            "   Q, E: Control Yaw\n"
            "   Space: Up Throttle\n"
            "   Left Shift: Down Throttle\n"
            "   R: Toggle Arming\n"
            "   1-9: Sensitivity 10%-90%, 0: 100%\n"
            "   F: Grip/Release\n"
        )

    def _effective_sensitivity(self) -> float:
        # The displayed 10% setting must clear PX4's 10% deadzone.
        return 0.13 if self._sensitivity == 0.1 else self._sensitivity

    def _get_attitude(self, keys) -> tuple[int, int, int]:
        scale = int(1000 * self._effective_sensitivity())
        roll = scale * (int(keys[pygame.K_d]) - int(keys[pygame.K_a]))
        pitch = scale * (int(keys[pygame.K_w]) - int(keys[pygame.K_s]))
        yaw = scale * (int(keys[pygame.K_e]) - int(keys[pygame.K_q]))
        return roll, pitch, yaw

    def _get_throttle(self, keys) -> int:
        scale = int(500 * self._effective_sensitivity())
        up = int(keys[pygame.K_SPACE])
        down = int(keys[pygame.K_LSHIFT])
        return 500 + scale * (up - down)

    def _handle_button(self, key: int) -> None:
        previous_sensitivity = self._sensitivity
        match key:
            case pygame.K_r:
                self._toggle_arming()
            case pygame.K_0:
                self._sensitivity = 1.0
            case _ if pygame.K_1 <= key <= pygame.K_9:
                self._sensitivity = (key - pygame.K_0) / 10
            case pygame.K_f:
                self._toggle_gripping()
        if self._sensitivity != previous_sensitivity:
            self._logger.info(
                f"Keyboard sensitivity set to {self._sensitivity:.0%}"
            )

    def _handle_pygame_event(self):
        pygame.event.pump()
        keys = pygame.key.get_pressed()
        roll, pitch, yaw = self._get_attitude(keys)
        throttle = self._get_throttle(keys)
        self._logger.log(
            5,
            f"Manual control: "
            f"roll={roll}, pitch={pitch}, yaw={yaw}, throttle={throttle}",
        )
        self._manual_control(roll, pitch, yaw, throttle)
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                self._handle_button(event.key)
            elif event.type == pygame.QUIT:
                self._stop_event.set()

    def _spin(self) -> None:
        clock = pygame.time.Clock()
        self._logger.info(f"Keyboard control started at {self.rate} Hz")
        while not self._stop_event.is_set():
            self._handle_pygame_event()
            clock.tick(self.rate)

    def stop(self):
        super().stop()
        pygame.quit()
        self._logger.info("Stopped")
