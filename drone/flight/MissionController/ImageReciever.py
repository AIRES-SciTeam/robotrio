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
        self.dist_model = None
        self.frame = None
        self.timestamp = None

        self.fps = 0.0
        self.fps_start = None
        self.fps_intervals = 0
        self.last_recieved_at = None

        self.recieving_lag = None

    def _image_callback(self, msg : Image):
        now = time.monotonic()
        self.last_recieved_at = now

        if self.fps_start is None:
            self.fps_start = now
        else:
            self.fps_intervals += 1
            elapsed = now - self.fps_start

            if elapsed >= 1.0:
                self.fps = self.fps_intervals / elapsed
                self.fps_start = now
                self.fps_intervals = 0

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.logger.error(f"Image error: {e}")
            return

        self.frame = frame
        self.timestamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        self.recieving_lag = time.monotonic() - now 

    def _camerainfo_callback(self, msg : CameraInfo):
        self.camera_params = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.dist_model = msg.distortion_model
        self.destroy_subscription(self.camerainfo_sub)
        self.camerainfo_sub = None

    def get_image(self):
        return self.timestamp, self.frame

    def get_camerainfo(self):
        return self.camera_params, self.dist_coeffs, self.dist_model

    def get_fps(self, timeout=1.0):
        if self.last_recieved_at is None:
            return 0.0, self.recieving_lag
        if time.monotonic() - self.last_recieved_at > timeout:
            return 0.0, self.recieving_lag
        return self.fps, self.recieving_lag
