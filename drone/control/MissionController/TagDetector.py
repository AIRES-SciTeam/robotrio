from rclpy.node import Node # type: ignore
from rclpy.qos import qos_profile_sensor_data # type: ignore
from sensor_msgs.msg import Image # type: ignore
from cv_bridge import CvBridge, CvBridgeError # type: ignore
import cv2 # type: ignore
import numpy as np
import logging
from pupil_apriltags import Detector # type: ignore

from drone.control.Utils.Configs import DRONE_TagConfig


class DRONE_TagDetector(Node):
    def __init__(
        self,
        tag_config : DRONE_TagConfig,
        logger : logging.Logger
    ):
        super().__init__("DRONE_ImageGetter")
        try:
            self.bridge = CvBridge()
            self.logger = logger

            self.detector = Detector(families = tag_config.family) 

            self.sub = self.create_subscription(
                Image, "/x500/down_cam/image",
                self._callback, qos_profile_sensor_data
            )
            self.pub = self.create_publisher(Image, "/x500/down_cam/image_tag", 10)

            self.logger.info("DRONE_TagDetector: Initialized")
        except Exception:
            self.destroy_node()
            raise

    def _callback(self, msg : Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.logger.error(f"CvBridgeError: {e}")
            return

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = self._detect_tags(frame_gray)

        highlight = self._highlight_tags(frame, tags)

        output = self.bridge.cv2_to_imgmsg(highlight, encoding="bgr8")
        output.header = msg.header
        self.pub.publish(output)

    def _detect_tags(self, frame_gray):
        return self.detector.detect(frame_gray)
        
    def _highlight_tags(self, frame, tags):
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)

        for tag in tags:
            corners = np.rint(tag.corners).astype(np.int32)
            cv2.fillConvexPoly(mask, corners, 255)
        
        highlight = cv2.bitwise_and(frame, frame, mask=mask)
        
        return highlight
