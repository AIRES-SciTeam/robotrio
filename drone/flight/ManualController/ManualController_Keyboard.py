import pygame
import logging

from Commander.MAVLinkCommander import DRONE_MAVLinkCommander
from ManualController.ManualController import DRONE_ManualController
from GripController.GripController import DRONE_GripController


class DRONE_KeyboardController(DRONE_ManualController):
    """
    Ручное управление дроном с клавиатуры через окно pygame.
    Ответственность:
        * Преобразование нажатых клавиш в команды ручного управления.
        * Передача команд включения/выключения двигателей и управления захватом.
    Интерфейс:
        1. __init__(com, gripper, logger, rate) -- создаёт окно управления.
        2. run() -- считывает клавиатуру и отправляет manual_control с частотой 50 Гц.
           Esc завершает цикл и закрывает pygame.
    Клавиши:
        * W/S -- тангаж
        * A/D -- крен
        * Q/E -- рысканье
        * Space/Left Shift -- увеличение/уменьшение тяги
        * R -- включение/выключение двигателей
        * F -- захват/освобождение груза
        * 1-9/0 -- чувствительность управления 10-90/100%
        * Esc -- выход
    """

    def __init__(
        self,
        com: DRONE_MAVLinkCommander,
        gripper: DRONE_GripController,
        logger: logging.Logger,
        rate : int = 50
    ) -> None:
        super().__init__(
            com=com,
            gripper=gripper,
            logger=logger,
            rate=rate
        )

        pygame.init()
        pygame.display.set_mode((640, 240))
        pygame.display.set_caption("DRONE_KeyboardController")
        pygame.key.set_repeat()

        self.sensitivity = 1.0
        self.running = False

        self.logger.info(f"DRONE_ManualController: Initialized keyboard controller at {self.rate} Hz")
        self._help_prompt()

    def _help_prompt(self) -> None:
        print(
            "   A, D: Control Roll\n"
            "   W, S: Control Pitch\n"
            "   Q, E: Control Yaw\n"
            "   Space: Up Throttle\n"
            "   L-Shift: Down Throttle\n"
            "   R: Arm\n"
            "   T: Disarm\n"
            "   1-9, 0: Sensitivity 10-100%\n"
            "   F: Grip/Release\n"
            "   Esc: Exit"
        )

    def _get_axis(self, keys, pos: int, neg: int) -> int:
        return int(keys[pos]) - int(keys[neg])

    def _get_attitude(self, keys) -> tuple[int, int, int]:
        scale = int(1000 * self.sensitivity)
        roll = scale * self._get_axis(keys, pygame.K_d, pygame.K_a)
        pitch = scale * self._get_axis(keys, pygame.K_w, pygame.K_s)
        yaw = scale * self._get_axis(keys, pygame.K_e, pygame.K_q)

        return roll, pitch, yaw

    def _get_throttle(self, keys) -> int:
        scale = int(500 * self.sensitivity)
        throttle = 500 + scale * self._get_axis(keys, pygame.K_SPACE, pygame.K_LSHIFT)

        return throttle

    def _handle_button(self, key: int) -> None:
        previous_sensitivity = self.sensitivity
        match key:
            case pygame.K_r:
                self.logger.info("DRONE_ManualController: Keyboard arming toggle requested")
                self.toggle_arming()
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
                self.logger.info("DRONE_ManualController: Keyboard gripper toggle requested")
                self.toggle_gripping()
            case pygame.K_ESCAPE:
                self.logger.info("DRONE_ManualController: Keyboard exit requested")
                self.running = False
        if self.sensitivity != previous_sensitivity:
            self.logger.info(f"DRONE_ManualController: Keyboard sensitivity set to {self.sensitivity:.0%}")

    def run(self) -> None:
        clock = pygame.time.Clock()
        self.running = True
        self.logger.info(f"DRONE_ManualController: Keyboard control started at {self.rate} Hz")
        try: 
            while self.running:
                pygame.event.pump()

                keys = pygame.key.get_pressed()
                roll, pitch, yaw = self._get_attitude(keys)
                throttle = self._get_throttle(keys)

                self.logger.debug(
                    f"DRONE_ManualController: Manual control: "
                    f"roll={roll}, pitch={pitch}, yaw={yaw}, throttle={throttle}"
                )
                self.manual_control(roll, pitch, yaw, throttle)

                for event in pygame.event.get():  
                    if event.type == pygame.KEYDOWN:
                        self._handle_button(event.key)

                clock.tick(self.rate)

        except KeyboardInterrupt:
            self.running = False
            self.logger.info("DRONE_ManualController: Keyboard control interrupted")
        finally:
            self.commander.stop()
            pygame.quit()
            self.logger.info("DRONE_ManualController: Keyboard control stopped")


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
        DRONE_KeyboardController(com=commander, gripper=gripper, logger=logger).run()
    finally:
        if commander is not None and commander.conn is not None:
            commander.stop()
