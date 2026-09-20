from abc import ABC, abstractmethod
import logging

from GripController.GripController import DRONE_GripController
from Commander.MAVLinkCommander import DRONE_MAVLinkCommander


class DRONE_ManualController(ABC):
    """
    Базовый класс для источников ручного управления дроном.
    Ответственность:
        * Передача команд взведения, снятия с охраны и ручного управления через Commander.
        * Хранение ссылки на контроллер захвата груза.
    Интерфейс:
        1. Инициализация __init__. Аргументы:
            * com : DRONE_MAVLinkCommander -- соединение и команды PX4.
            * gripper : DRONE_GripController -- управление захватом груза.
            * logger : logging.Logger -- объект для записи сообщений в лог.
            * rate : int = 50 -- частота публикации команд ручного управления, в Гц
        2. Метод toogle_arming() -- проверяет состояние взведения через
           Commander.is_armed(). Если дрон взведён, вызывает disarm();
           иначе вызывает arm(). Если состояние неизвестно (None), записывает
           предупреждение и также вызывает arm(). Результат команды не возвращает.
        3. Метод manual_control(roll, pitch, yaw, throttle) -- передаёт
           параметры ручного управления в Commander.manual_control().
           Аргументы: roll, pitch, yaw от -1000 до 1000; throttle от 0 до 1000.
           Подтверждения доставки нет; частоту отправки задаёт вызывающий класс.
        4. Метод toggle_gripping() -- переключает захват через Gripper.toggle().
           Дополнительного подтверждения состояния захвата не ожидает.
        5. Метод run() -- абстрактный метод запуска цикла ввода в конкретном
           контроллере.
    Отправку heartbeat выполняет DRONE_MAVLinkCommander. 
    По контракту всегда шлёт команды ручного управления.
    """

    def __init__(
        self,
        com: DRONE_MAVLinkCommander,
        gripper: DRONE_GripController,
        logger: logging.Logger,
        rate : int = 50
    ):
        self.logger = logger
        self.commander = com
        self.gripper = gripper

        self.rate = rate

        self.logger.info("DRONE_ManualController: Initialized")

    def toggle_arming(self):
        armed = self.commander.is_armed()
        if armed is None:
            self.logger.warning("DRONE_ManualController: Drone is unreachable")
        if armed:
            self.logger.info("DRONE_ManualController: Disarming command sent")    
            self.commander.disarm()
        else:
            self.logger.info("DRONE_ManualController: Arming command sent")    
            self.commander.arm()

    def manual_control(self, roll, pitch, yaw, throttle):
        self.commander.manual_control(roll, pitch, yaw, throttle)

    def toggle_gripping(self) -> None:
        self.logger.info("DRONE_ManualController: Gripping command sent")    
        self.gripper.toggle()

    @abstractmethod
    def run(self) -> None:
        pass
