import rclpy
from rclpy.context import  Context
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import logging
import numpy as np
import time

from Utils.Configs import DRONE_ConnConfig


class DRONE_ImageReciever(Node):
    """
    ROS 2 узел для получения изображения и калибровки камеры.
    Инициализация __init__. Аргументы:
        * conn_config : DRONE_ConnConfig -- настройки топиков:
          image_topic для Image, camerainfo_topic для CameraInfo.
        * logger : logging.Logger -- журнал ошибок преобразования изображения.
    Создаёт подписки с qos_profile_sensor_data. Для приёма сообщений вызывающая
    сторона должна инициализировать rclpy и выполнять узел через executor/spin.
    Собственный поток обработки ROS класс не запускает.

    Интерфейс:
        * get_image() -- возвращает последний кадр BGR8 как numpy.ndarray
          формы (height, width, 3) либо None до первого успешного приёма.
          Возвращается сохранённый массив без копирования и проверки давности.
        * get_camerainfo() -- возвращает (camera_params, dist_coeffs).
          camera_params -- (fx, fy, cx, cy) из матрицы K, в пикселях;
          dist_coeffs -- numpy.ndarray коэффициентов дисторсии типа float64.
          До получения CameraInfo возвращает (None, None).

    Обработка сообщений:
        * _image_callback() -- преобразует Image через CvBridge в BGR8.
          При CvBridgeError пишет ошибку в журнал и сохраняет предыдущий кадр.
        * _camerainfo_callback() -- сохраняет первую калибровку и удаляет
          подписку CameraInfo; последующие изменения калибровки не принимаются.
    Освобождение узла через destroy_node() организует владелец объекта.
    """
    def __init__(
        self, 
        conn_config : DRONE_ConnConfig, 
        logger : logging.Logger,
        ros_context : Context | None = None
    ):
        super().__init__("DRONE_ImageReciever", context=ros_context)
        self.logger = logger
        self.cv_bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, conn_config.image_topic,
            self._image_callback,
            qos_profile_sensor_data
        )
        self.camerainfo_sub = self.create_subscription(
            CameraInfo, conn_config.camerainfo_topic,
            self._camerainfo_callback,
            qos_profile_sensor_data
        )

        self.camera_params = None 
        self.dist_coeffs = None

        self.frame = None

    def _image_callback(self, msg : Image):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.logger.error(f"Image error: {e}")
            return
        self.frame = frame

    def _camerainfo_callback(self, msg : CameraInfo):
        self.camera_params = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.destroy_subscription(self.camerainfo_sub)
        self.camerainfo_sub = None

    def get_image(self):
        return self.frame

    def get_camerainfo(self):
        return self.camera_params, self.dist_coeffs
    