from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
import numpy as np
import time
from logging import Logger

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from Modules.GripController.GripController import GripController
    from Modules.MissionController.ImageReciever import ImageReceiver
    from Modules.MissionController.TagDetector import TagDetector
from Modules.Logger.LoggerFabric import LoggerFabric
from Modules.SimpleCommander.Commander import Commander, CommandResult


@dataclass
class MissionEnv:
    commander : Commander
    image_receiver : ImageReceiver
    tag_detector : TagDetector
    logger : Logger
    camera_rotation : np.ndarray # optical camera frame -> FRD
    camera_position : tuple[float, float, float] # m, FRD
    grip_ctrl : GripController | None = None

    # def __post_init__(self):
    #     self.camera_rotation = np.array(self.camera_rotation, dtype=float)
    #     self.camera_position = np.array(self.camera_position, dtype=float)
    #     if self.camera_position.shape != (3,):
    #         raise ValueError("camera_position must contain three coordinates")
    #     if self.camera_rotation.shape != (3, 3):
    #         raise ValueError("camera_rotation must be a 3x3 rotation matrix")
    #     if not np.isfinite(self.camera_position).all() or not np.isfinite(self.camera_rotation).all():
    #         raise ValueError("camera transform must be finite")
    #     if not np.allclose(self.camera_rotation.T @ self.camera_rotation, np.eye(3)) or not np.isclose(np.linalg.det(self.camera_rotation), 1):
    #         raise ValueError("camera_rotation must be a proper rotation matrix")

class MBState(Enum):
    PENDING = auto()
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()

class MissionBlock(ABC):
    def __init__(
        self,
        name : str,
        mission_env : MissionEnv
    ):
        self.name = name
        self.menv = mission_env
        self._state = MBState.PENDING
        self.menv.logger.info(f"{self.name} Initialized")

    def start(self):
        self.menv.logger.info(f"{self.name} Started")
        self._state = MBState.RUNNING

    def is_started(self):
        return self._state != MBState.PENDING

    def cancel(self):
        self.menv.logger.info(f"{self.name} Cancelled")
        self._state = MBState.CANCELLED

    def _can_tick(self):
        if self._state == MBState.RUNNING:
            return True
        self.menv.logger.warning(f"{self.name} is {self._state}")
        return False

    def _complete(self):
        self.menv.logger.warning(f"{self.name} Succeeded")
        self._state = MBState.SUCCEEDED

    def _fail(self, err : str = "commander fail"):
        self.menv.logger.warning(f"{self.name} Failed: {err}")
        self._state = MBState.FAILED

    @abstractmethod
    def tick(self) -> MBState:
        pass

class Arm(MissionBlock):
    def __init__(
        self, 
        mission_env
    ):
        super().__init__("Arm", mission_env)

    def tick(self):
        if self._can_tick():
            if self.menv.commander.armed():
                self._complete()
                return self._state
            res = self.menv.commander.arm()
            if res == CommandResult.SUCCESS:
                self._complete()
            else:
                self.menv.logger.warning("Arm: %s; retrying next tick", res.name)
        return self._state

class Grip(MissionBlock):
    def __init__(self, mission_env, tag: str):
        super().__init__("Grip", mission_env)
        self.tag = tag
        self._sent = False
        self._hold_pos = None

    def start(self):
        self._sent = False
        self._hold_pos = None
        super().start()

    def _request(self):
        return self.menv.grip_ctrl.grip(self.tag, confirm=False)

    def _confirmed(self):
        return self.menv.grip_ctrl.is_attached(self.tag)

    def tick(self):
        if not self._can_tick():
            return self._state
        if self.menv.grip_ctrl is None:
            self._fail("GripController unavailable")
            return self._state
        if self._hold_pos is None:
            x, y, z, _ = self.menv.commander.get_drone_position()
            if any(v is None for v in (x, y, z)):
                self._fail("Unknown drone position")
                return self._state
            self._hold_pos = (x, y, z)
        res = self.menv.commander.set_target(pos=self._hold_pos)
        if res != CommandResult.SUCCESS:
            self._fail(res.name)
            return self._state
        if not self._sent:
            if not self._request():
                self._fail("Gripper command rejected")
                return self._state
            self._sent = True
        if self._confirmed():
            self._complete()
        return self._state


class Release(Grip):
    def __init__(self, mission_env):
        super().__init__(mission_env, tag=None)
        self.name = "Release"

    def _request(self):
        return self.menv.grip_ctrl.release(confirm=False)

    def _confirmed(self):
        return not self.menv.grip_ctrl.is_busy()


class Disarm(MissionBlock):
    def __init__(
        self, 
        mission_env
    ):
        super().__init__("Disarm", mission_env)

    def tick(self):
        if not self.menv.commander.armed():
            self._complete()
        if self._can_tick():
            res = self.menv.commander.disarm()
            if res == CommandResult.SUCCESS:
                self._complete()
            else:
                self._fail(f"{res}")
        return self._state

class ReachAltitude(MissionBlock):
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
            x, y, z, _ = self.menv.commander.get_drone_position()
            if any(value is None for value in (x, y, z)):
                self._fail(f"Unknown drone position: {(x, y, z)}")
                return self._state
            if self.hold_xy is None:
                self.hold_xy = (x, y)
            if abs(z + self.altitude) <= self.eps:
                self._complete()
            else:
                res = self.menv.commander.set_target(pos=(*self.hold_xy, -self.altitude))
                if res != CommandResult.SUCCESS:
                    self._fail(f"{res.name}")
        return self._state

class Takeoff(ReachAltitude):
    def __init__(self, mission_env):
        super().__init__(
            mission_env=mission_env,
            altitude=1
        )
        self.name = "Takeoff"

class ReachCourse(MissionBlock):
    def __init__(
        self, 
        mission_env,
        yaw : float, # rad
        eps : float = 0.15, # rad
        pos : tuple[float, float, float] | None = None,
        max_yaw_rate : float = 0.3 # rad/s, target slew rate
    ):
        super().__init__("ReachCourse", mission_env)
        self.yaw = yaw
        self.pos = pos
        self.eps = eps
        if not np.isfinite(max_yaw_rate) or max_yaw_rate <= 0:
            raise ValueError("max_yaw_rate must be positive and finite")
        self.max_yaw_rate = max_yaw_rate
        self._command_yaw = None
        self._last_tick_at = None

    def start(self):
        self._command_yaw = None
        self._last_tick_at = None
        super().start()

    def tick(self):
        if self._can_tick():
            _, _, _, q = self.menv.commander.get_drone_position()
            if q is None:
                self._fail(f"unknown drone orientation: {q}")
                return self._state
            w, x, y, z = q
            yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            delta = self.yaw - yaw
            error = np.arctan2(np.sin(delta), np.cos(delta))
            now = time.monotonic()
            if self._command_yaw is None:
                self._command_yaw = float(yaw)
            else:
                delta = self.yaw - self._command_yaw
                remaining = np.arctan2(np.sin(delta), np.cos(delta))
                step = self.max_yaw_rate * (now - self._last_tick_at)
                next_yaw = self._command_yaw + np.clip(remaining, -step, step)
                self._command_yaw = float(np.arctan2(np.sin(next_yaw), np.cos(next_yaw)))
            self._last_tick_at = now
            res = self.menv.commander.set_target(pos=self.pos, yaw=self._command_yaw)
            if res != CommandResult.SUCCESS:
                self._fail(res.name)
            else:
                remaining = self.yaw - self._command_yaw
                target_error = np.arctan2(np.sin(remaining), np.cos(remaining))
                if abs(target_error) < 1e-6 and abs(error) <= self.eps:
                    self._complete()
        return self._state

class FlightMode(MissionBlock):
    def __init__(
        self,
        mission_env,
        mode : str,
        timeout : float = 60 # s
    ):
        super().__init__(f"Flight mode {mode}", mission_env)
        self.timeout = timeout
        self.mode = mode

    def tick(self):
        if self._can_tick():
            res = self.menv.commander.flight_mode(self.mode, self.timeout)
            if res == CommandResult.SUCCESS:
                self._complete()
            else:
                self._fail(res.name)
        return self._state

class Land(MissionBlock):
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
                res = self.menv.commander.land()
                if res != CommandResult.SUCCESS:
                    self._fail(f"{res.name}")
                    return self._state
            if self.menv.commander.is_landed() is True:
                self._complete()
            elif time.monotonic() - self.start_time > self.timeout:
                self._fail("landing timeout")
        return self._state

class Hold(MissionBlock):
    def __init__(
        self,
        mission_env,
        timeout : float | None = None, # s
        pos : tuple[float, float, float] | None = None
    ):
        super().__init__("Hold", mission_env)
        self.timeout = timeout
        self.pos = pos
        self.start_time = None

    def start(self):
        self.start_time = None
        super().start()

    def tick(self):
        if self._can_tick():
            if self.timeout is None:
                res = self.menv.commander.set_target(pos=self.pos)
                if res != CommandResult.SUCCESS:
                    self._fail(f"{res.name}")
                return self._state
            if self.start_time is None:
                self.start_time = time.monotonic()
            if time.monotonic() - self.start_time <= self.timeout:
                res = self.menv.commander.set_target(pos=self.pos)
                if res != CommandResult.SUCCESS:
                    self._fail(f"{res.name}")
                return self._state
            self._complete()
        return self._state


class MoveTo(MissionBlock):
    def __init__(
        self,
        mission_env,
        pos : tuple[float, float, float], # m, NED
        eps : float = 0.05, # m
        speed_eps : float = 0.05, # m/s
        settle_time : float = 2.0, # s
        yaw : float | None = None, # rad, NED
        max_yaw_rate : float = 0.3, # rad/s, target slew rate
        yaw_eps : float = 0.15 # rad
    ):
        super().__init__("MoveTo", mission_env)
        self.pos = pos
        self.yaw = yaw
        self.max_yaw_rate = max_yaw_rate
        self.yaw_eps = yaw_eps
        if not np.isfinite(max_yaw_rate) or max_yaw_rate <= 0:
            raise ValueError("max_yaw_rate must be positive and finite")
        self._command_yaw = None
        self._last_tick_at = None
        self.eps = eps
        self.speed_eps = speed_eps
        self.settle_time = settle_time
        self._settled_since = None

    def start(self):
        self._command_yaw = None
        self._last_tick_at = None
        self._settled_since = None
        super().start()

    def tick(self):
        if self._can_tick():
            x, y, z, q = self.menv.commander.get_drone_position()
            self.menv.logger.debug("Position = %s, destination = %s", (x, y, z), self.pos)
            if any(value is None for value in (x, y, z)):
                self._fail(f"unknown drone position: {(x, y, z)}")
                return self._state
            now = time.monotonic()
            yaw_reached = True
            if self.yaw is not None:
                if q is None:
                    self._fail("Unknown drone orientation")
                    return self._state
                w, qx, qy, qz = q
                current_yaw = np.arctan2(2*(w*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
                if self._command_yaw is None:
                    self._command_yaw = float(current_yaw)
                else:
                    delta = self.yaw - self._command_yaw
                    remaining = np.arctan2(np.sin(delta), np.cos(delta))
                    step = self.max_yaw_rate * (now - self._last_tick_at)
                    angle = self._command_yaw + np.clip(remaining, -step, step)
                    self._command_yaw = float(np.arctan2(np.sin(angle), np.cos(angle)))
                self._last_tick_at = now
                error = self.yaw - current_yaw
                remaining = self.yaw - self._command_yaw
                yaw_reached = (
                    abs(np.arctan2(np.sin(error), np.cos(error))) <= self.yaw_eps
                    and abs(np.arctan2(np.sin(remaining), np.cos(remaining))) < 1e-6
                )
            res = self.menv.commander.set_target(pos=self.pos, yaw=self._command_yaw)
            if res != CommandResult.SUCCESS:
                self._fail(f"{res}")
                return self._state
            velocity = self.menv.commander.get_drone_velocity()
            distance = np.linalg.norm(np.array((x, y, z)) - np.array(self.pos))
            speed = None if any(v is None for v in velocity) else np.linalg.norm(velocity)
            if distance <= self.eps and speed is not None and speed <= self.speed_eps and yaw_reached:
                now = time.monotonic()
                if self._settled_since is None:
                    self._settled_since = now
                    self.menv.logger.debug("MoveTo settling: distance=%.3f, speed=%.3f", distance, speed)
                if now - self._settled_since >= self.settle_time:
                    self._complete()
            else:
                self._settled_since = None
        return self._state

# class MoveVelocity(DRONE_MissionBlock):
#     """
#     Непрерывно задавать скорость до отмены контроллером миссии.
#     Аргументы:
#         * mission_env -- окружение миссии.
#         * vel : tuple[float, float, float] -- скорость в локальной NED, м/с.
#         * yaw : float | None = None -- абсолютный курс NED, рад.
#         * yaw_rate : float | None = None -- угловая скорость вокруг Z, рад/с.
#     None отключает соответствующее поле поворота. Если заданы оба, активны оба.
#     Успешный tick() оставляет RUNNING; отказ set_target() переводит в FAILED.
#     Самостоятельного завершения по времени или расстоянию нет.
#     """
#     def __init__(
#         self,
#         mission_env,
#         vel : tuple[float, float, float], # m/s, NED
#         yaw : float | None = None, # rad
#         yaw_rate : float | None = None # rad/s
#     ):
#         super().__init__("MoveVelocity", mission_env)
#         self.vel = vel
#         self.yaw = yaw
#         self.yaw_rate = yaw_rate

#     def tick(self):
#         if self._can_tick():
#             if not self.env.commander.set_target(vel=self.vel, yaw=self.yaw, yaw_rate=self.yaw_rate):
#                 self._fail()
#         return self.status

# class Spin(MoveVelocity):
#     """
#     Вращаться с нулевой линейной скоростью до отмены.
#     Аргументы:
#         * mission_env -- окружение миссии.
#         * yaw_rate : float = 0.2 -- угловая скорость, рад/с.
#     Частный случай MoveVelocity. Положительная скорость в NED соответствует
#     вращению по часовой стрелке при взгляде сверху. При отказе команды -- FAILED.
#     """
#     def __init__(
#         self,
#         mission_env,
#         yaw_rate : float = 0.2 # rad/s
#     ):
#         super().__init__(mission_env, vel=(0, 0, 0), yaw_rate=yaw_rate)
#         self.name = "Spin"



# class ReachTag(DRONE_MissionBlock):
#     """
#     Занять положение начала координат корпуса относительно тега.
#     Аргументы:
#         * mission_env -- окружение с калибровкой крепления камеры.
#         * tag_id : int -- ID искомого тега в семействе детектора.
#         * tag_size : float -- размер стороны тега, м.
#         * offset : tuple[float, float, float] -- желаемое положение корпуса
#           относительно центра тега, м, в системе тега из TagDetector.
#         * yaw : float | None = None -- абсолютный курс NED, рад;
#           None отключает управление курсом при подлёте.
#         * eps : float = 0.15 -- допуск расстояния до цели, м.
#         * yaw_eps : float = 0.15 -- допуск курса, рад, если yaw задан.
#         * search_yaw_rate : float = 0.2 -- ненулевая скорость поиска, рад/с.
#         * search_timeout : float = 30 -- время одного непрерывного поиска, с.
#     Переводит цель из системы тега через камеру и корпус в локальную NED.
#     Крепление камеры берётся из camera_rotation и camera_position окружения.
#     При отсутствии позы тега вращается с нулевой линейной скоростью и остаётся
#     RUNNING. Обнаружение сбрасывает таймер поиска и возобновляет подлёт.
#     Каждая новая потеря запускает таймер заново; start() также его сбрасывает.
#     При попадании в позиционный и заданный угловой допуски отправляет цель
#     и возвращает SUCCEEDED. Выдержка в допуске не предусмотрена.
#     Таймаут поиска, отсутствие данных камеры или позы дрона, неверная поза дрона
#     и отказ команды приводят к FAILED. Неверные параметры конструктора
#     вызывают ValueError. Общего таймаута подлёта нет.
#     """
#     def __init__(
#         self,
#         mission_env,
#         tag_id : int,
#         tag_size : float, # m
#         offset : tuple[float, float, float], # m, tag frame
#         yaw : float | None = None, # rad, NED
#         eps : float = 0.15, # m
#         yaw_eps : float = 0.15, # rad
#         search_yaw_rate : float = 0.2, # rad/s
#         search_timeout : float = 30 # s
#     ):
#         super().__init__("ReachTag", mission_env)
#         self.tag_id = tag_id
#         self.tag_size = tag_size
#         self.offset = np.array(offset, dtype=float)
#         self.yaw = yaw
#         self.eps = eps
#         self.yaw_eps = yaw_eps
#         self.search_yaw_rate = search_yaw_rate
#         self.search_timeout = search_timeout
#         self.search_start_time = None
#         if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
#             raise ValueError("offset must contain three finite coordinates")
#         if not np.isfinite(tag_size) or tag_size <= 0 or not np.isfinite(eps) or eps <= 0:
#             raise ValueError("tag_size and eps must be positive and finite")
#         if not np.isfinite(yaw_eps) or yaw_eps <= 0 or (yaw is not None and not np.isfinite(yaw)):
#             raise ValueError("yaw must be finite and yaw_eps must be positive and finite")
#         if not np.isfinite(search_yaw_rate) or search_yaw_rate == 0:
#             raise ValueError("search_yaw_rate must be finite and nonzero")
#         if not np.isfinite(search_timeout) or search_timeout <= 0:
#             raise ValueError("search_timeout must be positive and finite")

#     def start(self):
#         self.search_start_time = None
#         super().start()

#     def tick(self):
#         if self._can_tick():
#             frame = self.env.image_reciever.get_image()
#             camera_params, dist_coeffs = self.env.image_reciever.get_camerainfo()
#             x, y, z, q = self.env.commander.get_drone_position()
#             if frame is None or camera_params is None:
#                 self._fail("camera unavailable")
#                 return self.status
#             if any(value is None for value in (x, y, z, q)):
#                 self._fail("unknown drone pose")
#                 return self.status
#             position = np.array((x, y, z), dtype=float)
#             quaternion = np.array(q, dtype=float)
#             if quaternion.shape != (4,) or not np.isfinite(quaternion).all() or not np.isfinite(position).all() or np.linalg.norm(quaternion) == 0:
#                 self._fail("invalid drone pose")
#                 return self.status
#             pose = self.env.tag_detector.get_tag_position(
#                 frame, self.tag_id, camera_params, dist_coeffs, self.tag_size
#             )
#             if pose is None:
#                 now = time.monotonic()
#                 if self.search_start_time is None:
#                     self.search_start_time = now
#                 if now - self.search_start_time >= self.search_timeout:
#                     self._fail(f"tag {self.tag_id} search timeout")
#                 elif not self.env.commander.set_target(yaw_rate=self.search_yaw_rate):
#                     self._fail()
#                 return self.status
#             self.search_start_time = None
#             tag_position, tag_rotation = pose
#             w, qx, qy, qz = quaternion / np.linalg.norm(quaternion)
#             rotation = np.array([
#                 [1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - w*qz), 2 * (qx*qz + w*qy)],
#                 [2 * (qx*qy + w*qz), 1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz - w*qx)],
#                 [2 * (qx*qz - w*qy), 2 * (qy*qz + w*qx), 1 - 2 * (qx*qx + qy*qy)]
#             ])
#             error_camera = tag_position + tag_rotation @ self.offset
#             error_body = self.env.camera_position + self.env.camera_rotation @ error_camera
#             target = position + rotation @ error_body
#             yaw_reached = True
#             if self.yaw is not None:
#                 current_yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
#                 delta = self.yaw - current_yaw
#                 yaw_reached = abs(np.arctan2(np.sin(delta), np.cos(delta))) <= self.yaw_eps
#             if np.linalg.norm(error_body) <= self.eps and yaw_reached:
#                 if self.env.commander.set_target(pos=tuple(target), yaw=self.yaw):
#                     self._complete()
#                 else:
#                     self._fail()
#             elif not self.env.commander.set_target(pos=tuple(target), yaw=self.yaw):
#                 self._fail()
#         return self.status
