import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import logging
import numpy as np
import time

from Utils.Configs import DRONE_ConnConfig


class DRONE_ImageReciever(Node):
    def __init__(
        self, 
        conn_config : DRONE_ConnConfig, 
        logger : logging.Logger
    ):
        super().__init__("DRONE_ImageReciever")
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
    