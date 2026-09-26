import pygame
import numpy as np
import logging
from typing import TYPE_CHECKING

from Commander.MAVLinkCommander import DRONE_MAVLinkCommander
from ManualController.ManualController import DRONE_ManualController
from GripController.GripController import DRONE_GripController

if TYPE_CHECKING:
    from MissionController.MissionController import DRONE_MissionController


class DRONE_GamepadController(DRONE_ManualController):
    """
    Ручное управление дроном с геймпада через pygame.
    Ответственность:
        * Преобразование осей и кнопок геймпада в команды ручного управления.
        * Передача команд включения/выключения двигателей и управления захватом.
    Интерфейс:
        1. __init__(com, gripper, logger, mission_controller, rate, deadzone) --
           подключает геймпад и принимает контроллер миссии для её управления.
           deadzone -- порог игнорирования отклонений осей.
        2. run() -- считывает геймпад и отправляет manual_control с частотой rate.
           Start завершает цикл и закрывает pygame.
    Кнопки и оси геймпада Xbox:
        * Левый стик вверх/вниз -- тангаж
        * Левый стик влево/вправо -- крен
        * Правый стик влево/вправо -- рысканье
        * Правый/левый триггер -- увеличение/уменьшение тяги
        * A -- включение/выключение двигателей
        * X -- захват/освобождение груза
        * B -- включение/выключение медленного режима
        * Y -- запуск/остановка миссии
        * Крестовина вверх/вниз -- пауза/продолжение миссии
        * Крестовина влево/вправо -- предыдущий/следующий блок миссии
        * Start -- выход
    """

    def __init__(
        self,
        com: DRONE_MAVLinkCommander,
        gripper: DRONE_GripController,
        logger: logging.Logger,
        mission_controller: "DRONE_MissionController | None" = None,
        rate : int = 50,
        deadzone: float = 0.1,
    ) -> None:
        super().__init__(
            com=com,
            gripper=gripper,
            logger=logger,
            rate=rate
        )

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.logger.error("DRONE_ManualController: No gamepad found")
            raise RuntimeError("Failed: no gamepad found")

        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.logger.debug(f"DRONE_ManualController: Gamepad: {self.js.get_name()}")

        self.deadzone = deadzone
        self.slow_multiplier = 1.0
        self.running = False

        self.logger.info(f"DRONE_ManualController: Initialized gamepad controller at {self.rate} Hz")

        self._help_prompt()
        
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

    def _mission_action(self, action: str) -> None:
        if self.mission_controller is None:
            self.logger.warning(
                f"DRONE_ManualController: Mission action ignored without MissionController: {action}"
            )
            return
        getattr(self.mission_controller, action)()

    def _normalize_axis(self, value: float, invert: bool = False) -> int:
        res = 0
        if np.abs(value) >= self.deadzone:
            res = int((value if not invert else -value) * 1000)
        return res

    def _normalize_throttle(self, value: float, p: float = 2.0) -> int:
        res = 0
        value = max(min(value, 1), -1)
        if value + 1 > self.deadzone:
            res = int(500 * (1 + np.sign(value) * (np.abs(value) ** p)))
        return res

    def _get_attitude(self) -> tuple[int, int, int]:
        roll = self.slow_multiplier * self.js.get_axis(0)
        pitch = self.slow_multiplier * self.js.get_axis(1)
        yaw = self.slow_multiplier * self.js.get_axis(2)

        roll = self._normalize_axis(roll)
        pitch = self._normalize_axis(pitch, invert=True)
        yaw = self._normalize_axis(yaw)

        return roll, pitch, yaw

    def _get_throttle(self) -> int:
        up = self.slow_multiplier * self.js.get_axis(5)
        down = self.slow_multiplier * self.js.get_axis(4)

        up = self._normalize_throttle(up)
        down = self._normalize_throttle(down)

        throttle = 500 + (up - down) // 2
        return throttle

    def _handle_button(self, button: int) -> None:
        match button:
            case 0: # XBox A
                self.logger.info("DRONE_ManualController: Gamepad arming toggle requested")
                self.toggle_arming()
            case 1: # XBox B
                self.slow_multiplier = 1.5 - self.slow_multiplier
                self.logger.info(f"DRONE_ManualController: Gamepad slow mode is {'on' if self.slow_multiplier == 0.5 else 'off'}")
            case 2: # XBox X
                self.logger.info("DRONE_ManualController: Gamepad gripper toggle requested")
                self.toggle_gripping()
            case 3: # XBox Y
                self.logger.info("DRONE_ManualController: Gamepad mission start/stop requested")
                self._mission_action("toggle")
            case 7: # XBox Start
                self.logger.info("DRONE_ManualController: Gamepad exit requested")
                self.running = False

    def _handle_hat(self, value: tuple[int, int]) -> None:
        match value:
            case (0, 1):
                self.logger.info("DRONE_ManualController: Gamepad mission pause requested")
                self._mission_action("pause")
            case (0, -1):
                self.logger.info("DRONE_ManualController: Gamepad mission resume requested")
                self._mission_action("resume")
            case (-1, 0):
                self.logger.info("DRONE_ManualController: Gamepad previous mission step requested")
                self._mission_action("prev")
            case (1, 0):
                self.logger.info("DRONE_ManualController: Gamepad next mission step requested")
                self._mission_action("next")

    def run(self) -> None:
        clock = pygame.time.Clock()
        self.running = True
        self.logger.info(f"DRONE_ManualController: Gamepad control started at {self.rate} Hz")
        try: 
            while self.running:
                pygame.event.pump()

                roll, pitch, yaw = self._get_attitude()
                throttle = self._get_throttle()

                self.logger.debug(
                    f"DRONE_ManualController: Manual control: "
                    f"roll={roll}, pitch={pitch}, yaw={yaw}, throttle={throttle}"
                )
                self.manual_control(roll, pitch, yaw, throttle)

                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        button = event.button
                        self._handle_button(button)
                    elif event.type == pygame.JOYHATMOTION:
                        self.logger.debug(
                            f"DRONE_ManualController: Gamepad hat value={event.value}"
                        )
                        self._handle_hat(event.value)

                clock.tick(self.rate)

        except KeyboardInterrupt:
            self.running = False
            self.logger.info("DRONE_ManualController: Gamepad control interrupted")
        finally:
            self.commander.stop()
            pygame.quit()
            self.logger.info("DRONE_ManualController: Gamepad control stopped")


if __name__ == "__main__":
    from Utils.Configs import DRONE_ModelConfig, DRONE_TagConfig

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler()],
    )
    logger = logging.getLogger("DRONE_ManualController")
    commander: DRONE_MAVLinkCommander | None = None

    try:
        commander = DRONE_MAVLinkCommander(
            conn_address="udpin:127.0.0.1:14541",
            logger=logger,
        )
        gripper = DRONE_GripController(
            model_config=DRONE_ModelConfig(world="scene", model="x500", cargo="goods"),
            tag_config=DRONE_TagConfig(family="tag36h11", list=["00", "01", "02", "03", "04", "05"]),
            logger=logger,
            grip_distance=0.6,
        )
        DRONE_GamepadController(com=commander, gripper=gripper, logger=logger).run()
    finally:
        if commander is not None and commander.conn is not None:
            commander.stop()
