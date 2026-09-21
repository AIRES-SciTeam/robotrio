import cv2
from pupil_apriltags import Detector
import numpy as np
import logging


class DRONE_TagDetector:
    """
    Обнаружение AprilTag и оценка положения тега относительно камеры.
    Инициализация __init__. Аргументы:
        * tag_family : str -- семейство тегов для pupil_apriltags.Detector.
        * logger : logging.Logger -- объект журнала.
    Создаёт один детектор и использует его при последующих вызовах.
    Интерфейс:
        * detect(frame) -- принимает изображение BGR, преобразует его в серое
          и возвращает список Detection. Если тегов нет, возвращает [].
          Оценка позы в pupil_apriltags отключена; порядок списка не задаётся.
        * estimate_pose(detection, camera_params, dist_coeffs, tag_size):
            * detection -- объект Detection с corners формы (4, 2).
            * camera_params -- (fx, fy, cx, cy), параметры камеры в пикселях.
            * dist_coeffs -- коэффициенты дисторсии для OpenCV; None означает
              отсутствие дисторсии.
            * tag_size -- размер стороны тега в метрах.
          Оценивает позу методом solvePnP с SOLVEPNP_IPPE_SQUARE.
          Возвращает (position, rotation): numpy.ndarray формы (3,) в метрах
          и матрицу вращения формы (3, 3).
          Преобразование: point_camera = rotation @ point_tag + position.
          Начало системы тега находится в центре его квадрата; координаты углов
          задаются в плоскости Z=0 в порядке (-h,h), (h,h), (h,-h), (-h,-h),
          где h = tag_size / 2. Оптические оси камеры: X вправо, Y вниз, Z вперёд.
          Возвращает None, если решение не найдено, векторы содержат нечисловые
          или бесконечные значения либо центр тега находится при Z <= 0.
        * get_tag_position(frame, tag_id, camera_params, dist_coeffs, tag_size) --
          обнаруживает теги и оценивает позу первого Detection с указанным ID.
          Возвращает (position, rotation) либо None при отсутствии тега
          или отклонённом решении оценки позы.
    Класс не переводит позу в систему корпуса или NED и не сглаживает измерения.
    Некорректные входные данные могут вызвать исключения OpenCV или Python;
    они не преобразуются в None.
    """
    def __init__(
        self, 
        tag_family : str,
        logger : logging.Logger
    ):
        self.logger = logger
        self.detector = Detector(families = tag_family)

    def detect(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray_frame,
            estimate_tag_pose = False
        )
        return detections

    def estimate_pose(self, detection, camera_params, dist_coeffs, tag_size):
        corners = detection.corners

        fx, fy, cx, cy = camera_params
        camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float64)

        half = tag_size / 2
        object_points = np.array([
            [-half, half, 0],
            [half, half, 0],
            [half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float64)

        image_points = np.ascontiguousarray(corners, dtype=np.float64).reshape(4, 2)

        success, rvec, tvec = cv2.solvePnP(
            objectPoints=object_points,
            imagePoints=image_points,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success or not np.isfinite(tvec).all() or not np.isfinite(rvec).all():
            return None
        if tvec[2, 0] <= 0:
            return None

        position = tvec.reshape(3)
        rotation, _ = cv2.Rodrigues(rvec)

        return position, rotation

    def get_tag_position(self, frame, tag_id, camera_params, dist_coeffs, tag_size):
        detections = self.detect(frame)

        det_idx : int | None = None
        for idx, detection in enumerate(detections):
            if detection.tag_id == tag_id:
                det_idx = idx
                break
        else:
            return None

        return self.estimate_pose(detections[det_idx], camera_params, dist_coeffs, tag_size)
