import rclpy
from rclpy.context import  Context
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from dataclasses import dataclass
import threading

from Modules.Logger.LoggerFabric import LoggerFabric


@dataclass
class ROSConfig:
    image_topic : str
    camerainfo_topic  : str

class ImageReceiver(Node):
    def __init__(
        self, 
        ros_config : ROSConfig, 
        logger_fabric : LoggerFabric,
        ros_context : Context | None = None
    ):
        super().__init__("ImageReciever", context=ros_context)
        self._logger = logger_fabric.get_logger("ImageReceiver")
        self._cv_bridge = CvBridge()

        self._image_lock = threading.Lock()
        self._image_sub = self.create_subscription(
            Image, ros_config.image_topic,
            self._image_callback,
            qos_profile_sensor_data
        )
        self._camerainfo_sub = self.create_subscription(
            CameraInfo, ros_config.camerainfo_topic,
            self._camerainfo_callback,
            qos_profile_sensor_data
        )

        self._camera_params = None 
        self._dist_coeffs = None

        self._frame = None
        
        self._logger.info("Initialized")

    def _image_callback(self, msg : Image):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._logger.log(5, "%s: Frame received", time.monotonic())
        except CvBridgeError as e:
            self._logger.error(f"Image error: {e}")
            return
        with self._image_lock:
            self._frame = frame

    def _camerainfo_callback(self, msg : CameraInfo):
        self._camera_params = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)
        self._logger.info("CameraInfo: %s, %s", self._camera_params, self._dist_coeffs)
        self.destroy_subscription(self.camerainfo_sub)
        self._camerainfo_sub = None

    def get_image(self):
        with self._image_lock:
            return self._frame

    def get_camerainfo(self):
        return self._camera_params, self._dist_coeffs
    