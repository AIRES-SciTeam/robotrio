import cv2
from pupil_apriltags import Detector
import numpy as np
import logging

from drone.flight.Utils.Configs import DRONE_TagConfig


class DRONE_TagHandler:
    def __init__(
        self, 
        tag_config : DRONE_TagConfig,
        logger : logging.Logger
    ):
        self.tag_config = tag_config
        self.logger = logger
        
        self.detector = Detector(families = tag_config.family)

        self.detections = dict()

    def detect(self, timestamp, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.detections[timestamp] = self.detector.detect(
            gray_frame,
            estimate_tag_pose = False
        )

        return [tag.tag_id for tag in self.detections[timestamp]]
    
    def estimate_pose(self, timestamp, tag_id, camera_params, dist_coeffs, tag_size):
        if tag_size <= 0:
            self.logger.error(f"Wrong tag size: {tag_size}")
            raise ValueError(f"Wrong tag size: {tag_size}")
        
        detection = self.detections[timestamp].get(tag_id)
        if detection is None:
            return None

        _, corners = detection
        fx, fy, cx, cy = camera_params

        camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1] 
        ], dtype=np.float64)

        half_tag_size = tag_size / 2
        object_points = np.array([
            [-half_tag_size, half_tag_size, 0],
            [half_tag_size, half_tag_size, 0],
            [half_tag_size, -half_tag_size, 0],
            [-half_tag_size, -half_tag_size, 0]
        ], dtype=np.float64)

        image_points = np.ascontiguousarray(corners, dtype=np.float64).reshape(4, 2)

        success, rvec, tvec = cv2.solvePnP(
            objectPoints=object_points,
            imagePoints=image_points,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success or not np.isfinite(tvec).all():
            return None

        if tvec[2, 0] <= 0:
            return None

        position = tvec.reshape(3)
        rotation, _ = cv2.Rodrigues(rvec)

        return position, rotation
