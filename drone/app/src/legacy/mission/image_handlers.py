import rclpy
from std_msgs.msg import Header
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy
import numpy as np
import threading, time

from logger.logger_fabric import LoggerFabric


MODULE_NAME = "mission"

class ImageReceiver:
    def __init__(
        self,
        logger_fabric : LoggerFabric,
        node : Node,
        *,
        image_topic : str,
        camerainfo_topic : str
    ):
        if not logger_fabric.is_alive():
            raise RuntimeError("LoggerFabric is stopped")
        self._logger = logger_fabric.get_logger(f"{type(self).__name__}", module_name=MODULE_NAME)
        self._logger.info("Initializing")
        self.node = node
        self._cv_bridge = CvBridge()

        self._image_lock = threading.Lock()
        self._frame = None
        self._header = None
        self._image_sub = self.node.create_subscription(
            Image, image_topic,
            self._image_callback,
            qos_profile_sensor_data
        )
        self._camerainfo_sub = self.node.create_subscription(
            CameraInfo, camerainfo_topic,
            self._camerainfo_callback,
            qos_profile_sensor_data
        )

        self._camera_params = None 
        self._dist_coeffs = None
        
        self._logger.info("Initialized")

    def _image_callback(self, msg : Image):
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._logger.log(5, "%s: Frame received", time.monotonic())
        except CvBridgeError as e:
            self._logger.error(f"Image error: {e}")
            return
        with self._image_lock:
            self._frame = frame
            self._header = deepcopy(msg.header)

    def _camerainfo_callback(self, msg : CameraInfo):
        self._camera_params = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)
        self._logger.info("CameraInfo: %s, %s", self._camera_params, self._dist_coeffs)
        self.node.destroy_subscription(self._camerainfo_sub)
        self._camerainfo_sub = None

    def get_image(self, *, with_header: bool = False):
        with self._image_lock:
            if with_header:
                return self._frame, deepcopy(self._header)
            return self._frame

    def get_camerainfo(self):
        return self._camera_params, self._dist_coeffs

class ImagePublisher:
    def __init__(
        self,
        logger_fabric : LoggerFabric,
        node : Node,
        *,
        topic : str
    ):
        if not logger_fabric.is_alive():
            raise RuntimeError("LoggerFabric is stopped")
        self._logger = logger_fabric.get_logger(f"{type(self).__name__}", module_name=MODULE_NAME)
        self._logger.info("Initializing")
        self.node = node
        self._cv_bridge = CvBridge()
        self._pub = self.node.create_publisher(
            Image, topic, 
            qos_profile_sensor_data
        )
        self._logger.info("Initialized")

    def pub(self, frame, header: Header | None = None):
        try:
            msg = self._cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            if header is not None:
                msg.header = deepcopy(header)
            else:
                msg.header.stamp = self.node.get_clock().now().to_msg()
            self._pub.publish(msg)
        except CvBridgeError as e:
            self._logger.error("Image conversion failed: %s", e)
            