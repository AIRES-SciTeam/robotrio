from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
import numpy as np
import logging
import time
import subprocess

from .TagDetector import DRONE_TagDetector
from .ImageReciever import DRONE_ImageReciever
from Commander.MAVLinkCommander import DRONE_MAVLinkCommander


@dataclass
class DRONE_MissionEnv:
    """
    Общие зависимости и параметры камеры для блоков миссии.
    Аргументы:
        * commander -- общий MAVLinkCommander.
        * image_reciever -- источник изображения и калибровки камеры.
        * tag_detector -- детектор и оценка позы тега.
        * logger -- журнал выполнения блоков.
        * camera_rotation -- матрица 3x3, переводящая векторы из оптической
          системы камеры (вправо, вниз, вперёд) в корпусную FRD (вперёд, вправо, вниз).
        * camera_position -- положение камеры относительно начала корпуса, м, FRD.
    Параметры крепления преобразуются в numpy.ndarray и проверяются при создании.
    Некорректные размеры, нечисловые значения или неверная матрица вращения
    вызывают ValueError.
    """
    commander : DRONE_MAVLinkCommander
    image_reciever : DRONE_ImageReciever
    tag_detector : DRONE_TagDetector
    logger : logging.Logger
    camera_rotation : np.ndarray # optical camera frame -> FRD
    camera_position : tuple[float, float, float] # m, FRD

    def __post_init__(self):
        self.camera_rotation = np.array(self.camera_rotation, dtype=float)
        self.camera_position = np.array(self.camera_position, dtype=float)
        if self.camera_position.shape != (3,):
            raise ValueError("camera_position must contain three coordinates")
        if self.camera_rotation.shape != (3, 3):
            raise ValueError("camera_rotation must be a 3x3 rotation matrix")
        if not np.isfinite(self.camera_position).all() or not np.isfinite(self.camera_rotation).all():
            raise ValueError("camera transform must be finite")
        if not np.allclose(self.camera_rotation.T @ self.camera_rotation, np.eye(3)) or not np.isclose(np.linalg.det(self.camera_rotation), 1):
            raise ValueError("camera_rotation must be a proper rotation matrix")

class DRONE_MissionBlockStatus(Enum):
    """
    Состояние блока: PENDING -- до запуска, RUNNING -- выполняется,
    SUCCEEDED -- цель достигнута, FAILED -- ошибка, CANCELLED -- отменён.
    """
    PENDING = auto()
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()

class DRONE_MissionBlock(ABC):
    """
    Базовый класс действия миссии.
    Аргументы:
        * name : str -- имя блока в журнале.
        * mission_env : DRONE_MissionEnv -- общие зависимости блока.
    Интерфейс:
        * start() -- переводит блок в RUNNING; наследники сбрасывают свои таймеры.
        * tick() -- выполняет шаг и возвращает DRONE_MissionBlockStatus.
          Вне RUNNING конкретные блоки возвращают статус без отправки команд.
        * cancel() -- переводит блок в CANCELLED.
        * _complete() / _fail(err) -- фиксируют успех или ошибку в статусе и журнале.
    Контроллер миссии регулярно вызывает tick() и назначает следующее действие.
    Изменение статуса само по себе не отменяет последнюю команду Commander.
    Вызовы Commander внутри tick() могут ожидать подтверждения от PX4.
    """
    def __init__(
        self,
        name : str,
        mission_env : DRONE_MissionEnv
    ):
        self.name = name
        self.env = mission_env
        self.status = DRONE_MissionBlockStatus.PENDING

    def start(self):
        self.env.logger.info(f"DRONE_MissionBlock: {self.name} Started")
        self.status = DRONE_MissionBlockStatus.RUNNING

    def is_stated(self):
        return self.status != DRONE_MissionBlockStatus.PENDING

    def cancel(self):
        self.env.logger.info(f"DRONE_MissionBlock: {self.name} Cancelled")
        self.status = DRONE_MissionBlockStatus.CANCELLED

    def _can_tick(self):
        if self.status == DRONE_MissionBlockStatus.RUNNING:
            return True
        self.env.logger.warning(f"DRONE_MissionBlock: {self.name} is {self.status}")
        return False

    def _complete(self):
        self.env.logger.warning(f"DRONE_MissionBlock: {self.name} Succeeded")
        self.status = DRONE_MissionBlockStatus.SUCCEEDED

    def _fail(self, err : str = "commander fail"):
        self.env.logger.warning(f"DRONE_MissionBlock: {self.name} Failed: {err}")
        self.status = DRONE_MissionBlockStatus.FAILED

    @abstractmethod
    def tick(self) -> DRONE_MissionBlockStatus:
        pass

class Hold(DRONE_MissionBlock):
    """
    Задавать нулевую линейную скорость в течение указанного времени.
    Аргументы:
        * mission_env -- окружение миссии.
        * timeout : float | None = None -- длительность в секундах;
          None означает выполнение до отмены.
    Таймер начинается с первого tick() и сбрасывается при start().
    По истечении времени -- SUCCEEDED, при отказе set_target() -- FAILED.
    Фиксированная точка и курс не задаются.
    """
    def __init__(
        self,
        mission_env,
        timeout : float | None = None # s
    ):
        super().__init__("Hold", mission_env)
        self.timeout = timeout
        self.start_time = None

    def start(self):
        self.start_time = None
        super().start()

    def tick(self):
        if self._can_tick():
            if self.timeout is None:
                if not self.env.commander.set_target():
                    self._fail()
                return self.status
            if self.start_time is None:
                self.start_time = time.monotonic()
            if time.monotonic() - self.start_time <= self.timeout:
                if not self.env.commander.set_target():
                    self._fail()
                return self.status
            self._complete()
        return self.status

class Arm(DRONE_MissionBlock):
    """
    Взвести моторы через Commander.arm().
    Аргумент mission_env -- окружение миссии.
    Первый выполняемый tick() ожидает подтверждения до 2 с и возвращает
    SUCCEEDED при успехе либо FAILED при отказе. Повторов внутри блока нет.
    """
    def __init__(
        self, 
        mission_env
    ):
        super().__init__("Arm", mission_env)

    def tick(self):
        if self._can_tick():
            if self.env.commander.arm():
                self._complete()
            else:
                self._fail()
        return self.status

class Disarm(DRONE_MissionBlock):
    """
    Выключить моторы через Commander.disarm().
    Аргумент mission_env -- окружение миссии.
    Первый выполняемый tick() ожидает подтверждения до 2 с и возвращает
    SUCCEEDED при успехе либо FAILED при отказе. Повторов внутри блока нет.
    """
    def __init__(
        self, 
        mission_env
    ):
        super().__init__("Disarm", mission_env)

    def tick(self):
        if self._can_tick():
            if self.env.commander.disarm():
                self._complete()
            else:
                self._fail()
        return self.status

class Manual(DRONE_MissionBlock):
    """
    Передать управление в режим MANUAL.
    Аргумент mission_env -- окружение миссии.
    Через Commander.manual() останавливает поток Offboard и ожидает подтверждения
    режима. При подтверждении -- SUCCEEDED, при отказе -- FAILED.
    Команды стиков отправляет отдельный ручной контроллер.
    """
    def __init__(self, mission_env):
        super().__init__("Manual", mission_env)

    def tick(self):
        if self._can_tick():
            if self.env.commander.manual():
                self._complete()
            else:
                self._fail()
        return self.status

class ReachAltitude(DRONE_MissionBlock):
    """
    Достичь высоты относительно начала локальной системы NED.
    Аргументы:
        * mission_env -- окружение миссии.
        * altitude : float -- высота в метрах; целевая координата Z = -altitude.
        * eps : float = 0.15 -- допуск по высоте в метрах.
    X и Y фиксируются по первому доступному положению после start().
    При ошибке высоты не больше eps -- SUCCEEDED; при неизвестном положении
    или отказе команды -- FAILED. Выдержка в допуске и таймаут не предусмотрены.
    """
    def __init__(
        self, 
        mission_env,
        altitude : float, # m
        eps : float = 0.15 # m
    ):
        super().__init__("ReachAltitude", mission_env)
        self.altitude = altitude
        self.eps = eps
        self.hold_xy = None

    def start(self):
        self.hold_xy = None
        super().start()

    def tick(self):
        if self._can_tick():
            x, y, z, _ = self.env.commander.get_drone_position()
            if any(value is None for value in (x, y, z)):
                self._fail(f"unknown drone position: {(x, y, z)}")
                return self.status
            if self.hold_xy is None:
                self.hold_xy = (x, y)
            if abs(z + self.altitude) <= self.eps:
                self._complete()
            else:
                if not self.env.commander.set_target(pos=(*self.hold_xy, -self.altitude)):
                    self._fail()
        return self.status

class Takeoff(ReachAltitude):
    """
    Набрать высоту 3 м относительно начала локальной NED.
    Аргумент mission_env -- окружение миссии.
    Использует ReachAltitude с допуском 0.15 м и фиксацией X, Y.
    Самостоятельно моторы не взводит; перед блоком требуется Arm.
    """
    def __init__(self, mission_env):
        super().__init__(
            mission_env=mission_env,
            altitude=3
        )
        self.name = "Takeoff"

class ReachCourse(DRONE_MissionBlock):
    """
    Достичь абсолютного курса в NED при нулевой линейной скорости.
    Аргументы:
        * mission_env -- окружение миссии.
        * yaw : float -- целевой курс в радианах.
        * eps : float = 0.15 -- угловой допуск в радианах.
    Курс вычисляется из кватерниона (w, x, y, z). Ошибка учитывает переход через ±π.
    При попадании в допуск -- SUCCEEDED; при отсутствии ориентации или отказе
    команды -- FAILED. Выдержка в допуске и таймаут не предусмотрены.
    """
    def __init__(
        self, 
        mission_env,
        yaw : float, # rad
        eps : float = 0.15 # rad
    ):
        super().__init__("ReachCourse", mission_env)
        self.yaw = yaw
        self.eps = eps

    def tick(self):
        if self._can_tick():
            _, _, _, q = self.env.commander.get_drone_position()
            if q is None:
                self._fail(f"unknown drone orientation: {q}")
                return self.status
            w, x, y, z = q
            yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            delta = self.yaw - yaw
            error = np.arctan2(np.sin(delta), np.cos(delta))
            if abs(error) <= self.eps:
                self._complete()
            else:
                if not self.env.commander.set_target(yaw=self.yaw):
                    self._fail()
        return self.status

class MoveTo(DRONE_MissionBlock):
    """
    Перейти в точку локальной NED (север, восток, вниз).
    Аргументы:
        * mission_env -- окружение миссии.
        * pos : tuple[float, float, float] -- целевые координаты, м.
        * eps : float = 0.15 -- допустимое расстояние до цели, м.
    При расстоянии не больше eps -- SUCCEEDED; при неизвестном положении или
    отказе команды -- FAILED. Курс, выдержка в допуске и таймаут не задаются.
    """
    def __init__(
        self,
        mission_env,
        pos : tuple[float, float, float], # m, NED
        eps : float = 0.15 # m
    ):
        super().__init__("MoveTo", mission_env)
        self.pos = pos
        self.eps = eps

    def tick(self):
        if self._can_tick():
            x, y, z, _ = self.env.commander.get_drone_position()
            if any(value is None for value in (x, y, z)):
                self._fail(f"unknown drone position: {(x, y, z)}")
            elif np.linalg.norm(np.array((x, y, z)) - np.array(self.pos)) <= self.eps:
                self._complete()
            elif not self.env.commander.set_target(pos=self.pos):
                self._fail()
        return self.status

class MoveVelocity(DRONE_MissionBlock):
    """
    Непрерывно задавать скорость до отмены контроллером миссии.
    Аргументы:
        * mission_env -- окружение миссии.
        * vel : tuple[float, float, float] -- скорость в локальной NED, м/с.
        * yaw : float | None = None -- абсолютный курс NED, рад.
        * yaw_rate : float | None = None -- угловая скорость вокруг Z, рад/с.
    None отключает соответствующее поле поворота. Если заданы оба, активны оба.
    Успешный tick() оставляет RUNNING; отказ set_target() переводит в FAILED.
    Самостоятельного завершения по времени или расстоянию нет.
    """
    def __init__(
        self,
        mission_env,
        vel : tuple[float, float, float], # m/s, NED
        yaw : float | None = None, # rad
        yaw_rate : float | None = None # rad/s
    ):
        super().__init__("MoveVelocity", mission_env)
        self.vel = vel
        self.yaw = yaw
        self.yaw_rate = yaw_rate

    def tick(self):
        if self._can_tick():
            if not self.env.commander.set_target(vel=self.vel, yaw=self.yaw, yaw_rate=self.yaw_rate):
                self._fail()
        return self.status

class Spin(MoveVelocity):
    """
    Вращаться с нулевой линейной скоростью до отмены.
    Аргументы:
        * mission_env -- окружение миссии.
        * yaw_rate : float = 0.2 -- угловая скорость, рад/с.
    Частный случай MoveVelocity. Положительная скорость в NED соответствует
    вращению по часовой стрелке при взгляде сверху. При отказе команды -- FAILED.
    """
    def __init__(
        self,
        mission_env,
        yaw_rate : float = 0.2 # rad/s
    ):
        super().__init__(mission_env, vel=(0, 0, 0), yaw_rate=yaw_rate)
        self.name = "Spin"

class Land(DRONE_MissionBlock):
    """
    Выполнить посадку и дождаться подтверждения приземления.
    Аргументы:
        * mission_env -- окружение миссии.
        * timeout : float = 60 -- время ожидания посадки, с, с первого tick().
    Один раз вызывает Commander.land(), который останавливает Offboard
    и включает LAND. Возвращает SUCCEEDED только при is_landed() is True.
    Отказ включения LAND или истечение таймаута приводят к FAILED.
    start() сбрасывает таймер. Отдельную команду disarm() блок не отправляет.
    """
    def __init__(
        self,
        mission_env,
        timeout : float = 60 # s
    ):
        super().__init__("Land", mission_env)
        self.timeout = timeout
        self.start_time = None

    def start(self):
        self.start_time = None
        super().start()

    def tick(self):
        if self._can_tick():
            if self.start_time is None:
                self.start_time = time.monotonic()
                if not self.env.commander.land():
                    self._fail()
                    return self.status
            if self.env.commander.is_landed() is True:
                self._complete()
            elif time.monotonic() - self.start_time > self.timeout:
                self._fail("landing timeout")
        return self.status

class ReachTag(DRONE_MissionBlock):
    """
    Занять положение начала координат корпуса относительно тега.
    Аргументы:
        * mission_env -- окружение с калибровкой крепления камеры.
        * tag_id : int -- ID искомого тега в семействе детектора.
        * tag_size : float -- размер стороны тега, м.
        * offset : tuple[float, float, float] -- желаемое положение корпуса
          относительно центра тега, м, в системе тега из TagDetector.
        * yaw : float | None = None -- абсолютный курс NED, рад;
          None отключает управление курсом при подлёте.
        * eps : float = 0.15 -- допуск расстояния до цели, м.
        * yaw_eps : float = 0.15 -- допуск курса, рад, если yaw задан.
        * search_yaw_rate : float = 0.2 -- ненулевая скорость поиска, рад/с.
        * search_timeout : float = 30 -- время одного непрерывного поиска, с.
    Переводит цель из системы тега через камеру и корпус в локальную NED.
    Крепление камеры берётся из camera_rotation и camera_position окружения.
    При отсутствии позы тега вращается с нулевой линейной скоростью и остаётся
    RUNNING. Обнаружение сбрасывает таймер поиска и возобновляет подлёт.
    Каждая новая потеря запускает таймер заново; start() также его сбрасывает.
    При попадании в позиционный и заданный угловой допуски отправляет цель
    и возвращает SUCCEEDED. Выдержка в допуске не предусмотрена.
    Таймаут поиска, отсутствие данных камеры или позы дрона, неверная поза дрона
    и отказ команды приводят к FAILED. Неверные параметры конструктора
    вызывают ValueError. Общего таймаута подлёта нет.
    """
    def __init__(
        self,
        mission_env,
        tag_id : int,
        tag_size : float, # m
        offset : tuple[float, float, float], # m, tag frame
        yaw : float | None = None, # rad, NED
        eps : float = 0.15, # m
        yaw_eps : float = 0.15, # rad
        search_yaw_rate : float = 0.2, # rad/s
        search_timeout : float = 30 # s
    ):
        super().__init__("ReachTag", mission_env)
        self.tag_id = tag_id
        self.tag_size = tag_size
        self.offset = np.array(offset, dtype=float)
        self.yaw = yaw
        self.eps = eps
        self.yaw_eps = yaw_eps
        self.search_yaw_rate = search_yaw_rate
        self.search_timeout = search_timeout
        self.search_start_time = None
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
            raise ValueError("offset must contain three finite coordinates")
        if not np.isfinite(tag_size) or tag_size <= 0 or not np.isfinite(eps) or eps <= 0:
            raise ValueError("tag_size and eps must be positive and finite")
        if not np.isfinite(yaw_eps) or yaw_eps <= 0 or (yaw is not None and not np.isfinite(yaw)):
            raise ValueError("yaw must be finite and yaw_eps must be positive and finite")
        if not np.isfinite(search_yaw_rate) or search_yaw_rate == 0:
            raise ValueError("search_yaw_rate must be finite and nonzero")
        if not np.isfinite(search_timeout) or search_timeout <= 0:
            raise ValueError("search_timeout must be positive and finite")

    def start(self):
        self.search_start_time = None
        super().start()

    def tick(self):
        if self._can_tick():
            frame = self.env.image_reciever.get_image()
            camera_params, dist_coeffs = self.env.image_reciever.get_camerainfo()
            x, y, z, q = self.env.commander.get_drone_position()
            if frame is None or camera_params is None:
                self._fail("camera unavailable")
                return self.status
            if any(value is None for value in (x, y, z, q)):
                self._fail("unknown drone pose")
                return self.status
            position = np.array((x, y, z), dtype=float)
            quaternion = np.array(q, dtype=float)
            if quaternion.shape != (4,) or not np.isfinite(quaternion).all() or not np.isfinite(position).all() or np.linalg.norm(quaternion) == 0:
                self._fail("invalid drone pose")
                return self.status
            pose = self.env.tag_detector.get_tag_position(
                frame, self.tag_id, camera_params, dist_coeffs, self.tag_size
            )
            if pose is None:
                now = time.monotonic()
                if self.search_start_time is None:
                    self.search_start_time = now
                if now - self.search_start_time >= self.search_timeout:
                    self._fail(f"tag {self.tag_id} search timeout")
                elif not self.env.commander.set_target(yaw_rate=self.search_yaw_rate):
                    self._fail()
                return self.status
            self.search_start_time = None
            tag_position, tag_rotation = pose
            w, qx, qy, qz = quaternion / np.linalg.norm(quaternion)
            rotation = np.array([
                [1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - w*qz), 2 * (qx*qz + w*qy)],
                [2 * (qx*qy + w*qz), 1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz - w*qx)],
                [2 * (qx*qz - w*qy), 2 * (qy*qz + w*qx), 1 - 2 * (qx*qx + qy*qy)]
            ])
            error_camera = tag_position + tag_rotation @ self.offset
            error_body = self.env.camera_position + self.env.camera_rotation @ error_camera
            target = position + rotation @ error_body
            yaw_reached = True
            if self.yaw is not None:
                current_yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
                delta = self.yaw - current_yaw
                yaw_reached = abs(np.arctan2(np.sin(delta), np.cos(delta))) <= self.yaw_eps
            if np.linalg.norm(error_body) <= self.eps and yaw_reached:
                if self.env.commander.set_target(pos=tuple(target), yaw=self.yaw):
                    self._complete()
                else:
                    self._fail()
            elif not self.env.commander.set_target(pos=tuple(target), yaw=self.yaw):
                self._fail()
        return self.status
