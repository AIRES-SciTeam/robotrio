from __future__ import annotations

import time
import numpy as np
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from logging import Logger
    from xgolib import XGO_DOG
    from ..src.runner.camera_handler import CameraHandler


BODY_Y_LIMIT = 18.0
SIDE_STEP_SPEED = 5
SIDE_STEP_SECONDS = 1.0
MAX_SIDE_STEPS = 8
TAG_SIZES = {"tag36h11:0": 27.0, "tag36h11:1": 10.0}

R = np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0],
])


def select_tag(poses):
    """
    Выбирает тег, нормаль которого наиболее параллельна оси Y камеры.
    Принимает словарь детекций CameraHandler. Без подходящей позы возвращает None.
    """
    best_pose = None
    best_score = -1.0
    for pose in poses.values():
        if pose.pose_R is None or pose.pose_t is None:
            continue
        rotation = np.asarray(pose.pose_R)
        position = np.asarray(pose.pose_t).reshape(-1)
        if (rotation.shape != (3, 3) or position.size != 3
                or not np.isfinite(rotation).all()
                or not np.isfinite(position).all() or position[2] <= 0):
            continue
        tag_z = rotation[:, 2]
        length = float(np.linalg.norm(tag_z))
        if length == 0:
            continue
        score = abs(float(tag_z[1])) / length
        if score > best_score:
            best_pose, best_score = pose, score
    return best_pose


def _observe_target(camera : CameraHandler, *, tag_id=None):
    """
    Возвращает выбранный тег семейства tag36h11 с позицией в миллиметрах.
    Размеры известных тегов заданы в TAG_SIZES.
    """
    tags = TAG_SIZES
    if tag_id is not None:
        key = f"tag36h11:{tag_id}"
        if key not in tags:
            return None
        tags = {key: tags[key]}
    return select_tag(camera.get_poses(tags) or {})


def find_and_align_tag(logger : Logger, dog : XGO_DOG, camera : CameraHandler, *, tag_id=0):
    """
    Ищет тег и поворачивает робота до отклонения не более пяти градусов.
    При tag_id=None выбирает цель по ориентации. Возвращает найденную детекцию.
    Поиск продолжается до нахождения цели или прерывания теста.
    """
    if tag_id is not None and f"tag36h11:{tag_id}" not in TAG_SIZES:
        logger.error("Unknown target tag size: tag36h11:%s", tag_id)
        return None
    positive_turn_speed = 7
    negative_turn_speed = -6
    turn_seconds = 1.5
    angle_tolerance = np.deg2rad(5)
    search_direction = -1

    try:
        dog.pace("slow")
        dog.attitude("p", 10)
        dog.translation("z", 80)
        dog.stop()
        time.sleep(1)
        while True:
            target = _observe_target(camera, tag_id=tag_id)
            if target is None:
                logger.info("Target tag not visible; searching")
            else:
                position = np.asarray(target.pose_t).reshape(3) @ R.T
                angle = float(np.arctan2(position[1], position[0]))
                logger.info("Tag %s: angle=%.2f deg, center=%s",
                            target.tag_id, np.rad2deg(angle), target.center)
                if abs(angle) <= angle_tolerance:
                    return target
                search_direction = 1 if angle > 0 else -1

            turn_speed = positive_turn_speed if search_direction > 0 else negative_turn_speed
            logger.info("Turn step: speed=%s, duration=%.2fs", turn_speed, turn_seconds)
            try:
                dog.turn(turn_speed)
                time.sleep(turn_seconds)
            finally:
                dog.stop()
            time.sleep(1)
    finally:
        dog.stop()


def approach_tag(logger : Logger, dog : XGO_DOG, camera : CameraHandler, *, tag_id=None):
    """
    Подходит к тегу до расстояния 70–100 мм вдоль продольной оси камеры.
    Сохраняет угловое отклонение в пределах пяти градусов.
    Возвращает детекцию цели или None при невозможности начать поиск.
    """
    move_speed = 6
    min_step_seconds = 1.5
    max_step_seconds = 6.0
    min_step_distance_mm = 170.0
    doubling_distance_mm = 170.0
    min_x_mm, max_x_mm = 70, 100
    angle_tolerance = np.deg2rad(5)

    try:
        if find_and_align_tag(logger, dog, camera, tag_id=tag_id) is None:
            return None
        while True:
            target = _observe_target(camera, tag_id=tag_id)
            if target is None:
                dog.stop()
                if find_and_align_tag(logger, dog, camera, tag_id=tag_id) is None:
                    return None
                continue

            position = np.asarray(target.pose_t).reshape(3) @ R.T
            angle = float(np.arctan2(position[1], position[0]))
            if abs(angle) > angle_tolerance:
                dog.stop()
                if find_and_align_tag(logger, dog, camera, tag_id=tag_id) is None:
                    return None
                continue

            x_mm = float(position[0])
            logger.info("Tag %s: X=%.1f mm, angle=%.2f deg",
                        target.tag_id, x_mm, np.rad2deg(angle))
            if min_x_mm <= x_mm <= max_x_mm:
                return target

            speed = move_speed if x_mm > max_x_mm else -move_speed
            exponent = np.clip(
                (x_mm - min_step_distance_mm) / doubling_distance_mm,
                0.0, np.log2(max_step_seconds / min_step_seconds),
            )
            step_seconds = float(min_step_seconds * 2.0 ** exponent)
            logger.info("X step: speed=%s, duration=%.2fs", speed, step_seconds)
            try:
                dog.move_x(speed, step_seconds)
            finally:
                dog.stop()
            time.sleep(1)
    finally:
        dog.stop()


def align_y(logger : Logger, dog : XGO_DOG, camera : CameraHandler, *, tag_id=None):
    """
    Центрирует тег по горизонтали с точностью 1 мм смещением корпуса и шагами.
    Возвращает True при успехе, False при потере цели или исчерпании попыток.
    """
    body_y = 0.0
    side_steps = 0
    side_step_pending = False
    try:
        dog.stop()
        dog.translation("y", body_y)
        dog.translation("z", 60)
        time.sleep(2)
        for _ in range(30):
            pose = _observe_target(camera, tag_id=tag_id)
            if pose is None:
                logger.warning("No target tag with a valid pose visible")
                return False
            position = np.asarray(pose.pose_t).reshape(3)
            x_offset = float(position[0])
            logger.info("Current offset (camera X): %.2f mm; body Y: %.2f mm",
                        x_offset, body_y)
            if abs(x_offset) <= 1:
                logger.info("Reached offset: %.2f mm", x_offset)
                return True
            required_y = body_y - x_offset
            if side_step_pending or abs(required_y) > BODY_Y_LIMIT:
                if side_steps >= MAX_SIDE_STEPS:
                    logger.error("Side step limit reached before centering tag")
                    return False
                if body_y != 0:
                    side_step_pending = True
                    dog.translation("y", 0)
                    body_y = 0.0
                    time.sleep(2)
                    continue
                speed = float(np.copysign(SIDE_STEP_SPEED, required_y))
                logger.info("Side step: speed=%g, duration=%.2fs", speed, SIDE_STEP_SECONDS)
                try:
                    dog.move_y(speed, SIDE_STEP_SECONDS)
                finally:
                    dog.stop()
                side_steps += 1
                side_step_pending = False
                time.sleep(2)
                continue
            step = float(np.clip(-0.5 * x_offset, -3, 3))
            new_y = float(np.clip(body_y + step, -BODY_Y_LIMIT, BODY_Y_LIMIT))
            if new_y == body_y:
                logger.error("Body Y limit reached before centering tag")
                return False
            dog.translation("y", new_y)
            body_y = new_y
            time.sleep(2)
        logger.error("Could not center tag in 30 observations")
        return False
    finally:
        dog.stop()


def grip(logger : Logger, dog : XGO_DOG, camera : CameraHandler, *, tag_id=None):
    """
    Захватывает объект по позе тега с использованием калибровки манипулятора.
    Координаты и размеры тегов заданы в мм. Возвращает признак успеха.
    """
    M = np.array([
        [0.61951821, -0.00891379],
        [-0.1621911, 0.76268587]
    ])
    b = np.array([75.08762923, 22.50635963])
    pose = _observe_target(camera, tag_id=tag_id)
    if pose is None:
        logger.warning("No target tag with a valid pose visible")
        return False
    position = np.asarray(pose.pose_t).reshape(3)
    position = (position @ R.T)[[0, 2]]
    x, z = position @ M + b
    logger.info("Target position: %s mm; arm target: X=%.2f, Z=%.2f", position, x, z)
    dog.claw(30)
    time.sleep(1)
    dog.arm(float(x), float(z))
    time.sleep(1)
    dog.claw(255)
    time.sleep(1)
    dog.arm(0, 100)
    return True


def main(logger : Logger, dog : XGO_DOG, camera : CameraHandler):
    """
    Запускает подход, боковое выравнивание и захват через команду run test.
    После подхода сохраняет выбранный ID для выравнивания и захвата.
    Подключением и итоговым сбросом робота управляет TestRunner.
    """
    target = approach_tag(logger, dog, camera)
    if target is None:
        return
    if not align_y(logger, dog, camera, tag_id=target.tag_id):
        return
    grip(logger, dog, camera, tag_id=target.tag_id)
